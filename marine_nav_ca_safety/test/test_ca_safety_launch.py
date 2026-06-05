"""
Cross-process integration test for the CA safety node.

Validates what in-process unit tests cannot: QoS interop over real DDS
(best-effort reflex cloud + reliable cmd_vel), the TF transform of a real
PointCloud2 into the base frame, the braking reaction when an obstacle sits in
the stop box, and that the node is the sole publisher on the output topic.
"""

import time
import unittest

from geometry_msgs.msg import TwistStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


@pytest.mark.launch_test
def generate_test_description():
    ca_safety = launch_ros.actions.Node(
        package='marine_nav_ca_safety',
        executable='ca_safety_node',
        name='ca_safety',
        parameters=[{'source_timeout': 2.0}],
    )
    # Identity transform base_link <- cloud_frame so the published cloud lands in
    # the base frame unchanged.
    static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'cloud_frame'],
    )
    return (
        launch.LaunchDescription([
            ca_safety,
            static_tf,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestCaSafetyNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('ca_safety_integration_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_brakes_on_obstacle_in_stop_box(self):
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        cmd_pub = self.node.create_publisher(
            TwistStamped, 'cmd_vel_smoothed', reliable)
        cloud_pub = self.node.create_publisher(
            PointCloud2, 'collision_monitor/pointcloud', best_effort)

        outputs = []
        states = []
        self.node.create_subscription(
            TwistStamped, 'piloting_mode/autonomous/cmd_vel',
            lambda m: outputs.append(m), reliable)
        self.node.create_subscription(
            CollisionMonitorState, 'collision_monitor_state',
            lambda m: states.append(m), reliable)

        # Let discovery + static TF settle.
        self._spin(1.0)

        braked = False
        stop_state = False
        deadline = time.time() + 10.0
        while time.time() < deadline and not (braked and stop_state):
            cloud = point_cloud2.create_cloud_xyz32(
                _header(self.node, 'cloud_frame'),
                [(3.0, 0.0, 0.0)])  # inside the 5 m x 4 m stop box
            cloud_pub.publish(cloud)

            cmd = TwistStamped()
            cmd.twist.linear.x = 1.0
            cmd_pub.publish(cmd)

            self._spin(0.2)
            # Forward command must be braked (reverse or zero) once the obstacle
            # is seen — proving cloud QoS interop + TF + modulation across processes.
            if outputs and outputs[-1].twist.linear.x <= 0.0:
                braked = True
            # The viz timer must report STOP so CAMP fills the stop box.
            if any(s.action_type == CollisionMonitorState.STOP for s in states):
                stop_state = True

        self.assertTrue(braked, 'node did not brake with an obstacle in the stop box')
        self.assertTrue(stop_state, 'no STOP CollisionMonitorState published for CAMP fill')

    def test_closed_loop_stop_with_odom(self):
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        cmd_pub = self.node.create_publisher(TwistStamped, 'cmd_vel_smoothed', reliable)
        cloud_pub = self.node.create_publisher(
            PointCloud2, 'collision_monitor/pointcloud', best_effort)
        odom_pub = self.node.create_publisher(Odometry, 'odom', reliable)
        outputs = []
        self.node.create_subscription(
            TwistStamped, 'piloting_mode/autonomous/cmd_vel',
            lambda m: outputs.append(m), reliable)
        self._spin(1.0)

        def publish(odom_speed):
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                _header(self.node, 'cloud_frame'), [(3.0, 0.0, 0.0)]))
            cmd = TwistStamped()
            cmd.twist.linear.x = 1.0
            cmd_pub.publish(cmd)
            odom = Odometry()
            odom.twist.twist.linear.x = odom_speed
            odom_pub.publish(odom)

        # Moving forward in the stop box: should command reverse thrust to brake.
        reversed_seen = False
        deadline = time.time() + 8.0
        while time.time() < deadline and not reversed_seen:
            publish(1.0)
            self._spin(0.2)
            if outputs and outputs[-1].twist.linear.x < 0.0:
                reversed_seen = True
        self.assertTrue(reversed_seen, 'expected reverse thrust moving forward in stop box')

        # Now report stopped: should hold zero, not keep reversing.
        outputs.clear()
        held_zero = False
        deadline = time.time() + 8.0
        while time.time() < deadline and not held_zero:
            publish(0.0)
            self._spin(0.2)
            if outputs and outputs[-1].twist.linear.x == 0.0:
                held_zero = True
        self.assertTrue(held_zero, 'expected hold-zero once odom reports the boat stopped')

    def test_brakes_with_nan_odom(self):
        # A NaN odom speed must NOT make the node read "stopped" and hold zero in
        # the stop box; the non-finite sample is dropped → odom goes stale →
        # reverse brake falls back to the duration backstop. Output must be <= 0.
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        cmd_pub = self.node.create_publisher(TwistStamped, 'cmd_vel_smoothed', reliable)
        cloud_pub = self.node.create_publisher(
            PointCloud2, 'collision_monitor/pointcloud', best_effort)
        odom_pub = self.node.create_publisher(Odometry, 'odom', reliable)
        outputs = []
        self.node.create_subscription(
            TwistStamped, 'piloting_mode/autonomous/cmd_vel',
            lambda m: outputs.append(m), reliable)
        self._spin(1.0)

        braked = False
        deadline = time.time() + 10.0
        while time.time() < deadline and not braked:
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                _header(self.node, 'cloud_frame'), [(3.0, 0.0, 0.0)]))
            cmd = TwistStamped()
            cmd.twist.linear.x = 1.0
            cmd_pub.publish(cmd)
            odom = Odometry()
            odom.twist.twist.linear.x = float('nan')
            odom_pub.publish(odom)
            self._spin(0.2)
            if outputs and outputs[-1].twist.linear.x <= 0.0:
                braked = True
        self.assertTrue(braked, 'NaN odom must not suppress braking in the stop box')

    def test_single_output_publisher(self):
        self._spin(1.0)
        count = self.node.count_publishers('/piloting_mode/autonomous/cmd_vel')
        self.assertEqual(count, 1, 'expected exactly one publisher on the helm output topic')


def _header(node, frame_id):
    header = Header()
    header.stamp = node.get_clock().now().to_msg()
    header.frame_id = frame_id
    return header


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
