"""
ROS side of the Isaac Sim backend: TF, controllers, MoveIt and RViz.

Start Isaac Sim first (isaac_sim/run.sh); the controller manager runs inside it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

PACKAGE_NAME = 'manipulation_isaac_sim'
CONTROLLERS = ('joint_state_broadcaster', 'arm_controller', 'gripper_controller')
# Isaac Sim takes minutes to start the first time (shader compilation, URDF import).
CONTROLLER_MANAGER_TIMEOUT_S = '600'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE_NAME)
    moveit_config = (
        MoveItConfigsBuilder('lwa4p', package_name='lwa4p_moveit_config')
        .robot_description(file_path=os.path.join(share, 'urdf', 'lwa4p_isaac.urdf.xacro'))
        .trajectory_execution(file_path=os.path.join(share, 'config', 'moveit_controllers.yaml'))
        .to_moveit_configs())
    sim_time = {'use_sim_time': True}

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[moveit_config.robot_description, sim_time])
    spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[*CONTROLLERS, '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', CONTROLLER_MANAGER_TIMEOUT_S],
        parameters=[sim_time])
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[moveit_config.to_dict(), sim_time])
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(
            get_package_share_directory('lwa4p_moveit_config'), 'config', 'moveit.rviz')],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits, sim_time])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='RViz with MoveIt'),
        robot_state_publisher, spawner, move_group, rviz,
    ])
