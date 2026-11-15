"""
ROS 2 side of the simulation: /clock and the in-process controller manager.

The controller manager is NVIDIA's isaacsim.ros2.control: a standard ros2_control
controller_manager inside Isaac Sim, stepped with the physics. Its robot description is
built from the USD stage; the ROS graph keeps the one from robot_state_publisher.
"""

import omni.graph.core as og
from simulation import control_tree_fix
import usdrt.Sdf

GRAPH_PATH = '/World/RosGraph'
ROS_EXTENSIONS = ('isaacsim.ros2.core', 'isaacsim.ros2.bridge', 'isaacsim.ros2.control')


def enable_ros_extensions(app_utils) -> None:
    for extension in ROS_EXTENSIONS:
        app_utils.enable_extension(extension)
    control_extension = app_utils.get_extension_dict('isaacsim.ros2.control') or {}
    control_tree_fix.install(str(control_extension.get('package', {}).get('version', '?')))


def create_ros_graph(robot_path: str, controller_config: str) -> None:
    """Publish /clock and start the controller manager on the first physics tick."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('OnPlaybackTick', 'omni.graph.action.OnPlaybackTick'),
                ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('PublishClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
                ('ControllerManager', 'isaacsim.ros2.control.ROS2ControlManager'),
            ],
            keys.CONNECT: [
                ('OnPlaybackTick.outputs:tick', 'PublishClock.inputs:execIn'),
                ('ReadSimTime.outputs:simulationTime', 'PublishClock.inputs:timeStamp'),
                ('OnPlaybackTick.outputs:tick', 'ControllerManager.inputs:execIn'),
            ],
            keys.SET_VALUES: [
                ('ControllerManager.inputs:targetPrim', [usdrt.Sdf.Path(robot_path)]),
                ('ControllerManager.inputs:controllerConfig', controller_config),
                ('ControllerManager.inputs:namespace', ''),
                ('ControllerManager.inputs:publishRobotDescription', False),
                ('ControllerManager.inputs:useSimTime', True),
            ],
        },
    )
