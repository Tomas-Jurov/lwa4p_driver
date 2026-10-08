import os
import xml.dom.minidom

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

GAZEBO_SYS = "/usr/share/gazebo-11"


def urdf_without_comments(path):
    def strip(node):
        for child in list(node.childNodes):
            if child.nodeType == xml.dom.minidom.Node.COMMENT_NODE:
                node.removeChild(child)
            else:
                strip(child)
    doc = xacro.process_file(path)
    strip(doc)
    return doc.toxml()


def generate_launch_description():
    sim = get_package_share_directory("lwa4p_gazebo")
    desc = get_package_share_directory("lwa4p_description")

    moveit_config = (
        MoveItConfigsBuilder("lwa4p", package_name="lwa4p_moveit_config")
        .trajectory_execution(file_path=os.path.join(sim, "config", "moveit_controllers.yaml"))
        .to_moveit_configs())
    moveit_config.robot_description = {
        "robot_description": urdf_without_comments(os.path.join(sim, "urdf", "lwa4p_gazebo.urdf.xacro"))}
    sim_time = {"use_sim_time": True}

    gzserver = ExecuteProcess(output="screen", cmd=[
        "gzserver", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so",
        LaunchConfiguration("world")])
    gzclient = ExecuteProcess(cmd=["gzclient"], output="log",
                              condition=IfCondition(LaunchConfiguration("gui")))

    # RViz/MoveIt see the measured gripper opening (see finger_mimic.py)
    measured = [("joint_states", "joint_states_measured")]
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[moveit_config.robot_description, sim_time], remappings=measured)
    spawn = Node(package="gazebo_ros", executable="spawn_entity.py", output="screen",
                 arguments=["-topic", "robot_description", "-entity", "lwa4p"])

    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "-c", "/controller_manager"])

    controllers = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[
        spawner("joint_state_broadcaster"), spawner("arm_controller"),
        spawner("gripper_controller"), spawner("finger_controller")]))
    finger_mimic = Node(package="lwa4p_gazebo", executable="finger_mimic.py", parameters=[sim_time])

    move_group = Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                      parameters=[moveit_config.to_dict(), sim_time], remappings=measured)
    rviz = Node(package="rviz2", executable="rviz2", output="log",
                condition=IfCondition(LaunchConfiguration("rviz")),
                arguments=["-d", os.path.join(sim, "config", "sim.rviz")],
                parameters=[moveit_config.robot_description,
                            moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics,
                            moveit_config.planning_pipelines,
                            moveit_config.joint_limits, sim_time])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo window"),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz with MoveIt"),
        DeclareLaunchArgument("world", default_value=os.path.join(sim, "worlds", "lwa4p_table.world")),
        AppendEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.dirname(desc)),
        AppendEnvironmentVariable("GAZEBO_PLUGIN_PATH", os.path.join(get_package_prefix("gazebo_ros"), "lib")),
        gzserver, gzclient, rsp, spawn, controllers, finger_mimic, move_group, rviz,
    ])
