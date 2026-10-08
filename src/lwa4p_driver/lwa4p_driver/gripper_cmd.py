import argparse
import sys

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["grip", "move", "open", "home", "stop", "state"])
    ap.add_argument("mm", nargs="?", type=float, help="part size / gap in mm")
    ap.add_argument("--force", type=float, default=35.0, help="grip force N (grip only)")
    a = ap.parse_args()
    if a.action in ("grip", "move") and a.mm is None:
        sys.exit(f"{a.action} needs a size in mm")

    rclpy.init()
    n = Node("gripper_cmd")
    try:
        if a.action == "state":
            box = {}
            n.create_subscription(JointState, "gripper/state", lambda m: box.update(m=m), 10)
            for _ in range(50):
                rclpy.spin_once(n, timeout_sec=0.1)
                if "m" in box:
                    m = box["m"]
                    print(f"gap {m.position[0] * 1000:.1f} mm, force {m.effort[0]:.1f} N")
                    return
            sys.exit("no gripper/state - is the gripper node running?")

        if a.action in ("open", "home", "stop"):
            c = n.create_client(Trigger, f"gripper/{a.action}")
            if not c.wait_for_service(timeout_sec=10.0):
                sys.exit("gripper node not found - is arm_start.sh running?")
            f = c.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(n, f, timeout_sec=60.0)
            r = f.result()
            print(("OK: " if r.success else "FAILED: ") + r.message)
            sys.exit(0 if r.success else 1)

        c = ActionClient(n, GripperCommand, "gripper_controller/gripper_cmd")
        if not c.wait_for_server(timeout_sec=10.0):
            sys.exit("gripper node not found - is arm_start.sh running?")
        g = GripperCommand.Goal()
        g.command.position = a.mm / 1000.0
        g.command.max_effort = a.force if a.action == "grip" else -1.0
        f = c.send_goal_async(g)
        rclpy.spin_until_future_complete(n, f)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(n, rf, timeout_sec=60.0)
        r = rf.result()
        res = r.result
        ok = r.status == 4
        held = " - HOLDING" if res.stalled else ""
        print(f"{'OK' if ok else 'FAILED'}: gap {res.position * 1000:.1f} mm, force {res.effort:.1f} N{held}")
        sys.exit(0 if ok else 1)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
