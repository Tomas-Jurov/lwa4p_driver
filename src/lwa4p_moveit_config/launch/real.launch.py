import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    drv = get_package_share_directory("lwa4p_driver")
    here = get_package_share_directory("lwa4p_moveit_config")

    def include(path, **args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                        launch_arguments=args.items())

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("can_channel", default_value="can0"),
        GroupAction(scoped=True, actions=[
            include(os.path.join(drv, "launch", "bringup.launch.py"),
                    rviz="false", can_channel=LaunchConfiguration("can_channel"))]),
        include(os.path.join(here, "launch", "move_group.launch.py")),
        GroupAction(condition=IfCondition(LaunchConfiguration("rviz")), actions=[
            include(os.path.join(here, "launch", "moveit_rviz.launch.py"))]),
    ])
