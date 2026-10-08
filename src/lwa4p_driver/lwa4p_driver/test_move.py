import argparse
import math

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def dur(t):
    return Duration(sec=int(t), nanosec=int((t % 1) * 1e9))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", type=int, default=6, help="1..6")
    ap.add_argument("--deg", type=float, default=10.0)
    ap.add_argument("--time", type=float, default=5.0, help="seconds per direction")
    ap.add_argument("--no-return", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = Node("lwa4p_test_move")
    state = {}
    node.create_subscription(JointState, "joint_states", lambda m: state.update(m=m), 10)
    node.get_logger().info("waiting for /joint_states ...")
    while rclpy.ok() and "m" not in state:
        rclpy.spin_once(node, timeout_sec=0.5)
    js = state["m"]

    names = [f"arm_{i}_joint" for i in range(1, 7)]
    q0 = [js.position[list(js.name).index(n)] for n in names]
    q1 = list(q0)
    q1[args.joint - 1] += math.radians(args.deg)
    print("start:", " ".join(f"{math.degrees(q):7.2f}" for q in q0), "deg")
    print("goal :", " ".join(f"{math.degrees(q):7.2f}" for q in q1), "deg")

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = names
    goal.trajectory.points.append(JointTrajectoryPoint(positions=q1, time_from_start=dur(args.time)))
    if not args.no_return:
        goal.trajectory.points.append(
            JointTrajectoryPoint(positions=q0, time_from_start=dur(2 * args.time + 1.0)))

    client = ActionClient(node, FollowJointTrajectory, "arm_controller/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=5.0):
        print("action server not found - is the driver running?")
        return
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh.accepted:
        print("goal REJECTED - see the driver log (enabled? limits? speed?)")
        return
    print("executing ...")
    res = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res)
    r = res.result().result
    print("result:", "OK" if r.error_code == 0 else f"error {r.error_code}: {r.error_string}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
