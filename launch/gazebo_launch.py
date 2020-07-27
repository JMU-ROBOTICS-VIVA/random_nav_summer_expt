import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import launch_ros.actions


def generate_launch_description():
    tb_prefix = get_package_share_directory('turtlebot3_navigation')
    # python launch file for turtle bot 3 doesnt exist
    localizationLaunch = IncludeLaunchDescription(PythonLaunchDescriptionSource([tb_prefix, '/launch/acml_launch.py']), [
        ('initial_pose_x', 0.0),
        ('initial_pose_y', 0.0),
        ('initial_pose_z', 0.0),
    ])
    moveBaseLaunch = IncludeLaunchDescription(PythonLaunchDescriptionSource([tb_prefix, '/launch/move_base_launch.py']), [
        ('move_base/DWAPlannerROS/max_rot_vel', 1.0),
        ('move_base/DWAPlannerROS/acc_lim_theta', 10.0),
    ])
    return LaunchDescription([
        Node(
            package="map_server",
            node_executable="map_server",
            node_name="map_server",
            output="screen",
            parameters=[
                {"map_file": os.environ['TURTLE_GAZEBO_MAP_FILE']}
            ]
        ),
        localizationLaunch,
        moveBaseLaunch
    ])


if __name__ == "__main__":
    print(generate_launch_description().entities)
