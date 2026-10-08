import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory("lwa4p_description")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(desc, "urdf", "lwa4p.urdf.xacro")]), value_type=str)
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher", executable="joint_state_publisher"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(desc, "rviz", "lwa4p.rviz")]),
    ])
