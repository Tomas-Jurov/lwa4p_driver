"""
ROS side of the Isaac Sim backend: TF, controllers, MoveIt and RViz.

Start Isaac Sim first (isaac_sim/run.sh); the controller manager runs inside it.
world:=<name> must match isaac_sim/run.sh --world; config/worlds.yaml says what MoveIt
sees in each world.
"""

import os
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from manipulation_isaac_sim import isaac_urdf
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

PACKAGE_NAME = 'manipulation_isaac_sim'
CONTROLLERS = ('joint_state_broadcaster', 'arm_controller', 'gripper_controller')
# Isaac Sim takes minutes to start the first time (shader compilation, URDF import).
CONTROLLER_MANAGER_TIMEOUT_S = '600'


def descriptions_for(world: dict, urdf_xml: str, srdf_xml: str) -> tuple[str, str]:
    """Return the URDF and SRDF MoveIt and TF use in this world."""
    root = ElementTree.fromstring(urdf_xml)
    if not world['workspace_obstacles'] and not isaac_urdf.remove_workspace_obstacles(root):
        raise RuntimeError('the robot description has no workspace obstacles to remove')
    if world.get('sdf_obstacles'):
        sdf_path = os.path.join(get_package_share_directory('lwa4p_gazebo'),
                                world['sdf_obstacles'])
        with open(sdf_path, encoding='utf-8') as file:
            added = isaac_urdf.add_sdf_obstacles(root, file.read())
        if not added:
            raise RuntimeError(f'{sdf_path} has no static models')
        srdf_xml = isaac_urdf.disable_obstacle_collisions(srdf_xml, root)
    return ElementTree.tostring(root, encoding='unicode'), srdf_xml


def parameter_text(parameters: dict, name: str) -> str:
    value = parameters[name]
    return str(value.value if hasattr(value, 'value') else value)


def launch_nodes(context):
    share = get_package_share_directory(PACKAGE_NAME)
    with open(os.path.join(share, 'config', 'worlds.yaml'), encoding='utf-8') as file:
        worlds = yaml.safe_load(file)
    world_name = LaunchConfiguration('world').perform(context)
    if world_name not in worlds:
        raise RuntimeError(f'unknown world {world_name!r}; choose one of: {", ".join(worlds)}')

    moveit_config = (
        MoveItConfigsBuilder('lwa4p', package_name='lwa4p_moveit_config')
        .robot_description(file_path=os.path.join(share, 'urdf', 'lwa4p_isaac.urdf.xacro'))
        .trajectory_execution(file_path=os.path.join(share, 'config', 'moveit_controllers.yaml'))
        .to_moveit_configs())
    urdf_xml, srdf_xml = descriptions_for(
        worlds[world_name],
        parameter_text(moveit_config.robot_description, 'robot_description'),
        parameter_text(moveit_config.robot_description_semantic, 'robot_description_semantic'))
    moveit_config.robot_description = {'robot_description': urdf_xml}
    moveit_config.robot_description_semantic = {'robot_description_semantic': srdf_xml}
    sim_time = {'use_sim_time': True}

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[moveit_config.robot_description, sim_time])
    # No parameters: the spawner passes its own --params-file on to every controller it
    # loads, and launch writes that file to this machine's /tmp, which the controller
    # manager inside Isaac Sim cannot read when the ROS side runs in a container.
    spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[*CONTROLLERS, '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', CONTROLLER_MANAGER_TIMEOUT_S])
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

    return [robot_state_publisher, spawner, move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='lab',
                              description='the world isaac_sim/run.sh --world runs'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz with MoveIt'),
        OpaqueFunction(function=launch_nodes),
    ])
