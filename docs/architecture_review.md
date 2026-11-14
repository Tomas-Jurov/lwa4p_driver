# Architecture review: the first diagram and the real hardware

This review compares each layer of the first architecture diagram with what the hardware and the existing code allow.
Viktor Budylin wrote it in the archived `RoboCup-FEI/manipulation-Simulation` (`AGENTS.md` §13, commits `27fed6e` and `9ceb0ee`).
It was moved here on 2026-11-14 and updated to the current repository.

The first diagram is in commit `27fed6e` of the archived repository.
The current [manipulation_architecture.txt](manipulation_architecture.txt) already reflects this table.
The proposed implementation is in [manipulation_implementation_proposal.md](manipulation_implementation_proposal.md).

| Layer in the first diagram | Owner | Today | Notes for the plan |
| --- | --- | --- | --- |
| Compute split (GPU, CPU cores 4–5 and 6–7, RT priority) | — | nothing | The GPU part concerns the perception team. Core isolation and RT priority **do not apply** to the real arm: its driver is a 20 Hz Python loop sending Profile Position targets over SDO. |
| Perception and world model (VLM, then a TF2 3D pose) | other team | nothing in this repository | Contract: `PoseStamped` in a TF frame. VLM latency is in seconds, so use it for task-level detection only, never in a closed loop. Grasping thin objects such as tubes needs **orientation and size**, not just a point from a depth lookup. Ask for them, or fit them locally from the point cloud. |
| Task supervisor (BehaviorTree.CPP) | AI team | nothing | This team provides the actions and the failure enumeration. MTC owns the motion stages (see `AGENTS.md` §10). |
| MTC | this team | nothing | MTC tasks need `move_group` with `ExecuteTaskSolutionCapability`. Tomas's `real.launch.py` and `demo.launch.py` do not enable it, so our launch files must. |
| OMPL (RRTConnect) + Pilz LIN/PTP | this team | `move_group` loads OMPL, CHOMP and Pilz from the `moveit_configs_utils` defaults | Prototype result: Pilz LIN failed `Place` on `arm_3_joint` limits even with realistic Cartesian limits, while MTC `CartesianPath` passed. Use `CartesianPath` for straight moves. `pilz_cartesian_limits.yaml` allows 1.0 m/s, far beyond the real arm. |
| Kinematics (pick_ik, TRAC-IK) | this team | KDL (`kinematics.yaml`) | That one file applies to every backend. Prototype result over 132 top-down picks: KDL 56, pick_ik 57 at 2.5× the time, so KDL stays. pick_ik and the TRAC-IK plugin both have Jazzy binaries. |
| Ruckig smoothing | this team | not configured | Deferred. It would only affect OMPL segments, and the real driver resamples every trajectory at 20 Hz. |
| MoveIt Servo, "velocity bypass" | — | — | **Not possible on the real arm.** The driver serves only `FollowJointTrajectory` goals in Profile Position mode and has no velocity or streaming input. At most, sim-only research. |
| `ros2_control` (controller_manager, JTC, forward_command, joint_state_broadcaster) | — | mock, Gazebo, Isaac Sim (planned) | **Simulation only.** On the real robot, `lwa4p_driver` serves `arm_controller/follow_joint_trajectory` and `gripper_node` serves `gripper_controller/gripper_cmd`. There is no controller_manager, no 100–250 Hz loop and no velocity controller. |
| `ros2_canopen` / Lely, 1 Mbit/s, sync PDO | — | — | **Superseded by the driver decision.** The real bus runs at 500 kbit/s, SDO only, Profile Position mode, 20 Hz, through `lwa4p_driver`. |
| Hardware: LWA 4P, E-stop | — | Tomas's driver | The diagram **omits the WSG 50 gripper and the FTM 75 sensor**. Both sit on `can1`; the WSG speaks its own protocol (not CiA 402) and is served by `gripper_node`, the FTM by `ftm_node`. Every grasping skill depends on the gripper. |

The prototype results come from the previous stack (Humble, the old robot model).
Repeat them on Jazzy with Tomas's model before relying on them.
