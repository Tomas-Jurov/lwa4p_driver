import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory("lwa4p_description")
    drv = get_package_share_directory("lwa4p_driver")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(desc, "urdf", "lwa4p.urdf.xacro")]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("can_channel", default_value="can0"),
        DeclareLaunchArgument("gripper", default_value="true"),
        DeclareLaunchArgument("gripper_channel", default_value="can1"),
        Node(package="lwa4p_driver", executable="driver", name="lwa4p_driver", output="screen",
             parameters=[os.path.join(drv, "config", "driver.yaml"),
                         {"can_channel": LaunchConfiguration("can_channel")}]),
        Node(package="lwa4p_driver", executable="gripper", name="gripper", output="screen",
             condition=IfCondition(LaunchConfiguration("gripper")),
             parameters=[os.path.join(drv, "config", "gripper.yaml"),
                         {"can_channel": LaunchConfiguration("gripper_channel")}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(desc, "rviz", "lwa4p.rviz")]),
    ])
