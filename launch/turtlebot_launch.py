import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import launch_ros.actions


def generate_launch_description():

    this_prefix = get_package_share_directory('random_nav')
    bricks_map = os.path.join(this_prefix, 'maps', 'bricks_map.yaml')

    # python launch file for turtle bot 3 doesnt exist
    tbb_prefix = get_package_share_directory('turtlebot3_bringup')
    minimalLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tbb_prefix, '/launch/minmal_launch.py']))

    tbrviz_prefix = get_package_share_directory('turtlebot3_rviz_launchers')
    viewNavigationLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tbrviz_prefix, '/launch/view_navigation_launch.py']))

    tbn_prefix = get_package_share_directory('turtlebot3_navigation')
    amclLaunch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [tbn, '/launch/view_navigation_launch.py']), [
        ('map_file', bricks_map),
    ])

    return LaunchDescription([
        minimalLaunch,
        viewNavigationLaunch,
        amclLaunch
    ])


if __name__ == "__main__":
    print(generate_launch_description().entities)
