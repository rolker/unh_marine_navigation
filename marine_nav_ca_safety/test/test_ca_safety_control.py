"""
Exercise the CA-safety marine_control device-control channel (#140 / ADR-0003).

`CaSafetyNode` is a plain rclcpp::Node, so the ControlServer is live as soon as
the node starts — no lifecycle transition needed. This test asserts the wiring
added when CA-safety became a marine_control adopter:

  - The node advertises a ControlSet on `~/control/state` containing all 15 live
    tunables, each in its declared UI group.
  - An in-range change on `~/control/change` is applied to the bound parameter
    and confirmed by the next state echo.
  - The node's existing parameter validation still guards the channel: a
    non-positive value is rejected, and a change that would violate the
    slowdown_min <= slowdown_max ordering is rejected. In both cases the echoed
    value is held — the fire-and-forget channel cannot push an invalid tuning
    into the live safety brake (ADR-0003 D8.3).

The numbered scenarios describe coverage, not execution order: unittest runs the
methods alphabetically and each is self-contained.
"""

import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from marine_control_interfaces.msg import ControlItem, ControlSet, ControlValue
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

NAMESPACE = '/test_ca'
NODE_NAME = 'ca_safety'
STATE_TOPIC = f'{NAMESPACE}/{NODE_NAME}/control/state'
CHANGE_TOPIC = f'{NAMESPACE}/{NODE_NAME}/control/change'

EXPECTED_DEVICE_NAME = 'Collision-Avoidance Safety'

# All 15 live tunables and the UI group each is bound under.
EXPECTED_GROUPS = {
    'ttc_time_constant': 'slowdown',
    'slowdown_min_length': 'slowdown',
    'slowdown_max_length': 'slowdown',
    'slowdown_speed_floor': 'slowdown',
    'slowdown_width': 'slowdown',
    'stop_length': 'stop',
    'stop_width': 'stop',
    'stop_speed_eps': 'stop',
    'reverse_speed': 'reverse',
    'reverse_distance': 'reverse',
    'reverse_duration': 'reverse',
    'reverse_clear_debounce': 'reverse',
    'cancel_yaw_during_reverse': 'reverse',
    'source_timeout': 'source',
    'publish_visualization': 'visualization',
}

# Heartbeat is 1 Hz; allow slack for discovery.
TEST_TIMEOUT_SECONDS = 25.0

# marine_control state QoS: RELIABLE + VOLATILE (ADR-0003 D5).
_CONTROL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


@pytest.mark.launch_test
def generate_test_description():
    node = Node(
        package='marine_nav_ca_safety',
        executable='ca_safety_node',
        name=NODE_NAME,
        namespace=NAMESPACE,
        output='screen',
    )
    return LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestCaSafetyControlChannel(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('ca_safety_control_test')
        cls.latest = None

        def on_state(msg: ControlSet):
            cls.latest = msg

        cls.sub = cls.node.create_subscription(
            ControlSet, STATE_TOPIC, on_state, _CONTROL_QOS)
        cls.change_pub = cls.node.create_publisher(
            ControlValue, CHANGE_TOPIC, _CONTROL_QOS)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout=TEST_TIMEOUT_SECONDS):
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def _item(self, name):
        if self.latest is None:
            return None
        for item in self.latest.items:
            if item.name == name:
                return item
        return None

    def _value(self, name):
        item = self._item(name)
        return None if item is None else float(item.value)

    def _send_change(self, name, value):
        msg = ControlValue()
        msg.header = Header()
        msg.name = name
        msg.value = repr(float(value))
        self.change_pub.publish(msg)

    def _drive_to(self, name, value):
        def predicate():
            self._send_change(name, value)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            current = self._value(name)
            return current is not None and abs(current - value) < 1e-6
        return predicate

    def test_state_advertises_all_live_tunables(self):
        self.assertTrue(
            self._spin_until(lambda: self.latest is not None),
            f'no ControlSet received on {STATE_TOPIC} within '
            f'{TEST_TIMEOUT_SECONDS}s; the ControlServer may not have been '
            f'constructed in the node constructor.')

        self.assertEqual(self.latest.device_name, EXPECTED_DEVICE_NAME)

        advertised = {item.name: item for item in self.latest.items}
        self.assertEqual(
            set(advertised), set(EXPECTED_GROUPS),
            'advertised controls do not match the 15 expected live tunables.')
        for name, group in EXPECTED_GROUPS.items():
            self.assertEqual(
                advertised[name].group, group,
                f'control {name!r} bound under group {advertised[name].group!r}, '
                f'expected {group!r}.')
        # The two bools should render as BOOL, the rest as FLOAT.
        self.assertEqual(advertised['publish_visualization'].type, ControlItem.TYPE_BOOL)
        self.assertEqual(advertised['cancel_yaw_during_reverse'].type, ControlItem.TYPE_BOOL)
        self.assertEqual(advertised['stop_length'].type, ControlItem.TYPE_FLOAT)
        self.assertEqual(advertised['stop_length'].units, 'm')

    def test_in_range_change_is_applied(self):
        # stop_length default is 5.0; drive it to 8.0.
        self.assertTrue(
            self._spin_until(self._drive_to('stop_length', 8.0)),
            'in-range change to stop_length was not reflected in the state echo; '
            'the change channel did not apply the bound parameter.')

    def test_non_positive_change_is_rejected(self):
        # Establish a known-good baseline (also proves the channel is live here).
        self.assertTrue(self._spin_until(self._drive_to('stop_length', 6.0)))

        # Request a non-positive value, then a valid sentinel. RELIABLE ordered
        # delivery means a landed sentinel proves the bad request was delivered
        # and processed (rejected), not dropped. Track the min echoed value: a
        # rejected <= 0 request must never move stop_length to or below zero.
        min_seen = 6.0
        self._send_change('stop_length', -1.0)
        self._send_change('stop_length', 7.0)

        def sentinel_applied():
            nonlocal min_seen
            rclpy.spin_once(self.node, timeout_sec=0.1)
            current = self._value('stop_length')
            if current is None:
                return False
            min_seen = min(min_seen, current)
            return abs(current - 7.0) < 1e-6

        self.assertTrue(
            self._spin_until(sentinel_applied),
            'sentinel change after the non-positive request never landed; cannot '
            'confirm the bad request was delivered rather than dropped.')
        self.assertGreater(
            min_seen, 0.0,
            'stop_length dropped to <= 0; a non-positive operator value reached '
            'the live safety brake (the onSetParameters > 0 guard was bypassed).')

    def test_slowdown_range_ordering_is_enforced(self):
        # Defaults: slowdown_min_length=5, slowdown_max_length=25. Establish the
        # baseline, then request a min above the max (must be rejected by the
        # cross-field ordering check), then a valid sentinel min.
        self.assertTrue(self._spin_until(self._drive_to('slowdown_min_length', 5.0)))

        max_seen = 5.0
        self._send_change('slowdown_min_length', 30.0)   # > slowdown_max_length (25)
        self._send_change('slowdown_min_length', 6.0)    # valid sentinel

        def sentinel_applied():
            nonlocal max_seen
            rclpy.spin_once(self.node, timeout_sec=0.1)
            current = self._value('slowdown_min_length')
            if current is None:
                return False
            max_seen = max(max_seen, current)
            return abs(current - 6.0) < 1e-6

        self.assertTrue(
            self._spin_until(sentinel_applied),
            'sentinel change after the out-of-order request never landed; cannot '
            'confirm the bad request was delivered rather than dropped.')
        max_len = self._value('slowdown_max_length')
        self.assertIsNotNone(max_len)
        self.assertLessEqual(
            max_seen, max_len,
            f'slowdown_min_length rose to {max_seen} above slowdown_max_length '
            f'{max_len}; the min <= max ordering guard was bypassed over the '
            f'marine_control channel.')


@launch_testing.post_shutdown_test()
class TestProcessShutdown(unittest.TestCase):

    def test_clean_exit(self, proc_info):
        pass
