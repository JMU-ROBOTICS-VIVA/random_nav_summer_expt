import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import launch_ros.actions


def generate_launch_description():

    this_prefix = get_package_share_directory('random_nav')
    bricks_world = os.path.join(this_prefix, 'worlds', 'bricks.world')
    bricks_map = os.path.join(this_prefix, 'maps', 'bricks_map.yaml')

    # python launch file for turtle bot 3 doesnt exist
    tb_prefix = get_package_share_directory('turtlebot3_gazebo')
    worldLaunch = IncludeLaunchDescription(PythonLaunchDescriptionSource([tb_prefix, '/launch/turtlebot_world_launch.py']), [
        ('world_file', bricks_world),
    ])

    tbviz_prefix = get_package_share_directory('turtlebot3_rviz_launchers')
    viewNavLaunch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [tbviz_prefix, '/launch/view_navigation_launch.py']))

    moveBaseLaunch = IncludeLaunchDescription(PythonLaunchDescriptionSource([this_prefix, '/launch/gazebo_launch.py']), [
        ('map_file', bricks_map),
    ])

    return LaunchDescription([
        worldLaunch,
        viewNavLaunch,
        moveBaseLaunch
    ])


if __name__ == "__main__":
    print(generate_launch_description().entities)
