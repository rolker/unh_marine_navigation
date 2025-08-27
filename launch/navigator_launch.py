from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LifecycleTransition
from launch_ros.actions import Node

from lifecycle_msgs.msg import Transition

def generate_launch_description():
    map_frame = LaunchConfiguration('map_frame')

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map'
    )

    return LaunchDescription([
        map_frame_arg,
        LifecycleNode(
            package='project11_navigation',
            executable='navigator',
            name='navigator',
            namespace='',
            remappings=[
                ('cmd_vel', 'piloting_mode/autonomous/cmd_vel'),
                ('enable', 'piloting_mode/autonomous/active'),
                # ('/tf', 'tf'),
                # ('/tf_static', 'tf_static')
            ],
            respawn=True,
            respawn_delay=5,
            emulate_tty=True,
        ),
        LifecycleTransition(
            lifecycle_node_names=(
                PythonExpression(
                    expression = [
                        '"',
                        LaunchConfiguration("ros_namespace", default=''),
                        '" + "/navigator"'
                    ],
                ),
            ),
            transition_ids=(
                Transition.TRANSITION_CONFIGURE,
                Transition.TRANSITION_ACTIVATE,
            )
        ),
        # occupancy_vector_map_from_geo converts a geo referenced occupancy grid to a vector map.
        Node(
            package='project11_navigation',
            executable='occupancy_vector_map_from_geo',
            name='occupancy_vector_map_from_geo',
            remappings=[
                ('input', 'project11/avoidance_map'),
                ('output', 'project11/avoidance_map_local')
            ],
            parameters=[{'frame_id': map_frame}],
            emulate_tty=True
        ),
        # occupancy_grid_from_vector_map converts a vector map to an occupancy grid.
        Node(
            package='project11_navigation',
            executable='occupancy_grid_from_vector_map',
            name='occupancy_grid_from_vector_map',
            remappings=[
                ('input', 'project11/avoidance_map_local'),
                ('output', 'project11/avoidance_grid')
            ],
            emulate_tty=True
        ),

    ])


# <launch>

#   <node pkg="manda_coverage" type="manda_coverage_node" name="manda_coverage">
#     <param name="soundings_topic" value="sensors/mbes/soundings"/>
#   </node>

#   <rosparam file="$(find project11_navigation)/config/default_stack.yaml" command="load" ns="navigator"/>

# </launch>
