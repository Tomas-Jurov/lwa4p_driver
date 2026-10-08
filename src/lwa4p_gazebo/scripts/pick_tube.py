#!/usr/bin/env python3
import argparse
import sys

import rclpy

from simple_moveit import SimpleMoveIt

STAND_TOP = 0.10
TUBE_HEIGHT = 0.10


def tube_z(robot):
    try:
        from gazebo_msgs.srv import GetEntityState
    except ImportError:
        return None
    c = robot.create_client(GetEntityState, "/gazebo/get_entity_state")
    if not c.wait_for_service(timeout_sec=1.0):
        return None
    f = c.call_async(GetEntityState.Request(name="test_tube", reference_frame="world"))
    rclpy.spin_until_future_complete(robot, f, timeout_sec=2.0)
    r = f.result()
    return r.state.pose.position.z if r and r.success else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--x", type=float, default=0.45, help="tube x [m]")
    ap.add_argument("--y", type=float, default=0.0, help="tube y [m]")
    ap.add_argument("--place-y", type=float, default=-0.10, help="where to put it down (y)")
    ap.add_argument("--grip", type=float, default=0.013,
                    help="gap to close to [m]; a bit less than the 15 mm tube = squeeze")
    a = ap.parse_args()

    rclpy.init()
    robot = SimpleMoveIt("pick_tube")
    grasp_z = STAND_TOP + TUBE_HEIGHT - 0.03
    above_z = grasp_z + 0.12

    def step(ok):
        if not ok:
            robot.get_logger().error("step failed - stopping")
            robot.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    robot.add_box("stand", (0.3, 0.4, 0.16), (0.5, 0.0, 0.02))
    robot.remove("test_tube")       # bin_pick.py adds the tube as an obstacle
    step(robot.gripper(0.085))
    step(robot.named("home"))
    step(robot.pose(a.x, a.y, above_z))
    step(robot.pose(a.x, a.y, grasp_z))
    step(robot.gripper(a.grip))
    step(robot.pose(a.x, a.y, above_z))
    z = tube_z(robot)
    if z is not None:
        lifted = z > STAND_TOP + TUBE_HEIGHT / 2 + 0.05
        robot.get_logger().info(f"tube centre at z = {z:.3f} m -> {'LIFTED' if lifted else 'NOT lifted'}")

    step(robot.pose(a.x, a.place_y, above_z))
    step(robot.pose(a.x, a.place_y, grasp_z + 0.005))
    step(robot.gripper(0.085))
    step(robot.pose(a.x, a.place_y, above_z))
    step(robot.named("home"))
    robot.get_logger().info("done")
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
