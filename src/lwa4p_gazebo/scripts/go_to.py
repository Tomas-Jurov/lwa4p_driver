#!/usr/bin/env python3
import argparse
import sys

import rclpy

from simple_moveit import SimpleMoveIt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("state", nargs="?", help="named state from the SRDF")
    ap.add_argument("--joints", nargs=6, type=float, metavar="DEG")
    ap.add_argument("--pose", nargs=3, type=float, metavar=("X", "Y", "Z"))
    ap.add_argument("--gap", type=float, help="gripper gap in metres")
    ap.add_argument("--speed", type=float, default=0.5, help="velocity scaling 0..1")
    a = ap.parse_args()
    rclpy.init()
    robot = SimpleMoveIt("go_to", velocity_scale=a.speed)
    try:
        if a.state:
            if a.state not in robot.states:
                sys.exit(f"unknown state '{a.state}', known: {', '.join(robot.states)}")
            ok = robot.named(a.state)
        elif a.joints:
            ok = robot.joints(a.joints)
        elif a.pose:
            ok = robot.pose(*a.pose)
        elif a.gap is not None:
            ok = robot.gripper(a.gap)
        else:
            ap.print_help()
            ok = True
        print("OK" if ok else "FAILED")
    finally:
        robot.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
