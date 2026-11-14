# Manipulation Interface Consolidation

- Status: v1 scope, candidate input and combined motion/perception selection cost, worker model and ROS `BUSY` response agreed by Dmitrii; remaining contract details are under discussion.
- Started: 2026-11-14.
- Review: this consolidation has not yet been reviewed jointly or agreed with AI, perception and navigation.
- Target: the first contract of the manipulation skill server (working name `manipulation_skill_server`) on ROS 2 Jazzy.
- Basis: Dmitrii's architecture sketch, lead meeting notes, Viktor's [implementation proposal](manipulation_implementation_proposal.md), and the AI/perception implementations in the RoboCup-FEI organization.

## 1. Purpose and decision authority

This document will record the agreed public interface of the manipulation skill server, including skill semantics, input data, results, cancellation and backend requirements.
The implementation follows the document after the relevant decisions are resolved.

Existing implementations show what their authors currently mean; they do not automatically define the shared interface.
The implementation proposal remains a proposal, and meeting notes define requirements without settling every ROS message field.
Dmitrii makes the key manipulation design decisions, with cross-team agreements recorded explicitly.

The following labels apply throughout:

| Label | Meaning |
| --- | --- |
| Recorded | Present in an identified source; scope and source authority still matter. |
| Proposed | Recommended here for discussion; not an agreed decision. |
| Open | Requires a decision or missing evidence. |
| Agreed | Explicitly resolved in this consolidation, with date and participants. |

The decisions marked `Agreed` below were made by Dmitrii in this discussion on 2026-11-14.
They supersede conflicting earlier proposals and sketches; cross-team review is still pending.
Names and field descriptions below express semantics, not compilable `.action` or `.msg` definitions.

## 2. Sources and their authority

Links to other RoboCup-FEI repositories point to the commits reviewed on 2026-11-14; some of those repositories are visible only to organization members.
Meeting notes and Dmitrii's architecture sketch are not published; ask Dmitrii for the sketch.

| ID | Source | What it contributes |
| --- | --- | --- |
| S1 | Lead meeting, 26.10 (internal notes) | Earlier `Pick`, `Place`, `Handover`, movement and force requirements; ongoing monitoring and explicit failures. |
| S2 | Lead meeting, 02.11 (internal notes) | Interface baseline: `GoToPosition`, `Grip`, force-triggered `Release`, `Wipe`, `PushPullMode`. |
| S3 | Dmitrii's architecture sketch (not published; ask Dmitrii): overview, Command, capability interfaces, busy worker, ROS adapters, MTC discussion, manipulation notes | Dmitrii's architectural baseline: ROS boundary, pure C++ Command/runner, adapters, one worker and MTC integration considerations. |
| S4 | [Viktor's implementation proposal, 01.11](manipulation_implementation_proposal.md) | Composite skills, complete MTC planning, execution split at gripper events, failure propagation, `BUSY`, `plan_only`, simulation gripper adapter. |
| S5 | AI/HMI architecture description (internal) | AI executive between VLM, perception and ROS actions; task-level recovery. |
| S6 | [Middleman prototype](https://github.com/RoboCup-FEI/robocup-ai-hmi-middleman/blob/33fae861e3ab2959ed8adc32e555e505c5f659c8/README.md) and [BT nodes](https://github.com/RoboCup-FEI/robocup-ai-hmi-middleman/blob/33fae861e3ab2959ed8adc32e555e505c5f659c8/src/ai_hmi_bt/src/nodes.cpp) | Local synchronous/asynchronous manipulation stubs; currently no manipulation ROS action client. |
| S7 | [VLM simulation executor](https://github.com/RoboCup-FEI/vlm-simulation/blob/9e5b4c0d6f88349ae5f008c5976240fce6dac418/src/vlm_simulation/execution.py) | `grip` performs a pick; `release` performs placement at a destination; task policies and simulation-specific execution. |
| S8 | [Perception architecture](https://github.com/RoboCup-FEI/perception/blob/3ccebbb0a0614348785c5d4e7c58507ddb6288a4/docs/ARCHITECTURE.md), [ROS acquisition](https://github.com/RoboCup-FEI/perception/blob/3ccebbb0a0614348785c5d4e7c58507ddb6288a4/ROS.md), [export code](https://github.com/RoboCup-FEI/perception/blob/3ccebbb0a0614348785c5d4e7c58507ddb6288a4/results.py) | Ranked target-TCP candidates, pregrasps, openings, observation identity, capture timestamp, frame and TCP conventions. |
| S9 | Navigation analysis, 26.10 (internal) | Safe-to-drive handshake, load/dynamics information, placement and handover completion. |
| S10 | [Current repository rules](../AGENTS.md), [arm driver](../src/lwa4p_driver/lwa4p_driver/driver_node.py), [gripper driver](../src/lwa4p_driver/lwa4p_driver/gripper_node.py), [FTM configuration](../src/lwa4p_driver/config/ftm.yaml) | Current backend behavior, model ownership, driver limits and missing calibrated wrench configuration. |

## 3. Conflicts to resolve

| Topic | Source differences | Consequence |
| --- | --- | --- |
| Skill granularity | S2 exposes movement and gripper primitives; S4 exposes complete `Pick` and `Place`. | Resolved by Dmitrii: v1 has only `GoToPosition`, `Grip`, `Release`, including force behavior; no independent `Pick` or `Place`. |
| Meaning of `Grip` | S2: close/verify the gripper; S7 `grip`: approach, grasp and lift. | An AI skill name needs an explicit mapping to a server action. |
| Meaning of `Release` | S2: open immediately or after a force threshold; S7 `release`: move to a surface and put down. | Placement and opening must have distinct semantics. |
| Movement input | S2 names one target; S4 generates grasps from object data; S8 exports several desired TCP poses. | Resolved by Dmitrii: `GoToPosition` accepts TCP-pose candidates and the server selects a reachable candidate. |
| Candidate selection | S8 has a ranking and a GUI `selected_id`; manipulation validates robot motion. | Resolved by Dmitrii: compare feasible plans using both motion convenience and perception candidate quality within one shared search-time budget; the exact formula, ordering and budget value remain open. |
| Admission | The S3 busy worker rejects concurrent work; its ROS comment proposes goal rejection; S4 accepts a ROS goal then aborts it with `BUSY`. | Resolved by Dmitrii: keep S3's runner policy and use S4's accepted-goal → `ABORTED` result with code `BUSY`. |
| Force features | S2 requires force monitoring and regulation; S4 excludes streaming control. | Distinguish monitoring/thresholds from continuous force regulation. |
| Driver capability | Tomas's current public driver accepts only trajectories; force-control questions are open with Tomas. | Limitations of the current public driver do not establish that the hardware cannot support admittance or streaming control. |
| Simulation | S4 proposes dropping Gazebo; current repository rules retain Tomas's Gazebo and add team-owned Isaac. | Preserve mock, Gazebo, Isaac and real backend coverage in the design. |
| Integration maturity | S5 describes the target architecture; S6 uses local stubs; S7 has its own executor. | A ROS adapter and the executive integration still need an owner and agreed behavior. |

## 4. Recorded project constraints

The current repository rules specify Jazzy only, Tomas's robot model and upstream files, and team additions in separate packages.
New launch/configuration wrappers must enable the capabilities needed by our server without editing Tomas's files in `devel`.

The intended ownership is AI for task orchestration and recovery, perception for observations and grasp proposals, navigation for base motion, and manipulation for skills, motion planning, grasp verification and failure reporting.
BehaviorTree.CPP and the actual BT trees belong to AI.
The MTC skill-server design originates in Dmitrii's architecture work; Viktor's proposal develops implementation details used in this consolidation.

The public arm driver currently accepts `FollowJointTrajectory` and executes Profile Position targets at a configured default of 20 Hz.
Its default checks include a 3-degree drive-limit margin, a 2-degree start tolerance, a speed check at 1.05 times the configured limit and an 8-degree tracking tolerance.
These are current configuration facts, not values to duplicate inside action definitions.
Changing the driver or introducing another control mode requires a separate design decision with Tomas.

The model currently uses `gripper_tcp`, 15 mm above the fingertips.
The centre of the finger pads is 9.5 mm above the fingertips (checked on the robot), and a tool-length discrepancy of about 10 mm is still open.
Until the model changes through the upstream workflow, consumers must use the current TF/model and explicitly agree the TCP reference.

## 5. Agreed v1 scope and architectural baseline

**Agreed by Dmitrii, 2026-11-14:** v1 exposes exactly three operations: `GoToPosition`, `Grip` and `Release`, with force behavior included from the first version.
There are no independent `Pick` or `Place` operations.
AI composes picking and placement from movement and gripper operations.
Other skills can be considered in later extensions; no additional action is part of this v1 decision.

| Operation | Agreed responsibility | Baseline from S2 / details still to resolve |
| --- | --- | --- |
| `GoToPosition` | Accept TCP-pose candidates; compare feasible plans using motion convenience and perception candidate quality within one shared search-time budget, select the best found and move to it with force monitoring. | `max_force`, `PTP` / precise `Approach`; exact profiles, approach geometry, cost formula, budget value and force trigger remain open. |
| `Grip` | Close/grasp at the current arm pose with force control/limiting. | `max_force`; width input, holding evidence and success criteria remain open. |
| `Release` | Open at the current arm pose with force-aware release behavior. | `force_thresh = 0` opens immediately; positive threshold waits for external force in S2; measurement and timeout details remain open. |

`GoToPosition` performs no implicit grasp, release, lift or placement sequence.
Picking and placement are AI-level compositions of the three operations, with sequencing and error handling between calls.
AI S7 `grip` and `release` therefore need adapters/compositions, not direct aliases for the server's `Grip` and `Release`.

**Agreed by Dmitrii, 2026-11-14:** use Dmitrii's architecture sketch (S3) with its busy worker policy, and retain MoveIt/MTC with the relevant implementation ideas from Viktor's proposal.
The earlier sketch's `Pick`/`BinPicking` examples and S4's standalone `Pick`/`Place` list do not define the v1 public interface.

| Layer | Responsibility under this decision |
| --- | --- |
| ROS node | Three action interfaces; convert goals into application commands; bridge feedback, result and cancellation. |
| Command and runner | ROS-independent application tasks and data; one shared `BusyTaskRunner` across all three actions; one worker executes the admitted command. |
| Adapters/backends | MoveIt/MTC/ROS and driver dependencies; implement motion planning, execution, gripper and force access behind application interfaces. |
| ROS executor | Process callbacks without waiting for the whole physical operation; the sketch uses `SingleThreadedExecutor` plus the worker. |

MTC remains an internal planning mechanism for movement, including any linked stages inside one `GoToPosition` profile.
Its adoption does not introduce an additional public pick/place action or force the ROS-independent Command layer to include MTC types.
The precise MTC/backend interfaces and stages remain to be designed for these primitives.
MTC cannot check a complete future picking sequence when grip and subsequent movements arrive as independent requests.
This boundary follows the selected public decomposition.

## 6. Perception-to-GoToPosition input

S8 currently exports JSON files; it is not an implemented ROS manipulation action interface.
Both Katya's classical CV node and AI perception should eventually feed the same semantic input through agreed adapters.

| Data | Proposed contract | Open detail |
| --- | --- | --- |
| Target identity | Optional task/object context kept distinct from the desired TCP pose. | Which context `GoToPosition` actually needs; it also supports camera/general movement. |
| Observation | Observation ID, coordinate frame, capture time and TCP reference. | ROS time encoding and adapter behavior for file inputs without a capture time. |
| Candidates | Agreed: a list of target TCP poses; the server compares feasible plans using motion convenience and perception candidate quality within one shared search-time budget. | Proposed IDs/quaternions, maximum count, search order, score contract, cost formula and budget value. |
| Pregrasp | Candidate pregrasp pose or an agreed approach definition. | Whether to consume the exported pose or derive it from a server profile. |
| Opening | Perception's inner fingertip gap and estimated contact span describe different quantities. | Whether either is used by `Grip` or by movement collision checks; neither is automatically a `GoToPosition` field. |
| Candidate evidence | Agreed: perception candidate quality contributes to the combined plan-selection cost; S8 exports `rank_score` and geometric evidence. | Score representation, direction/scale, normalization across producers and policy for missing scores; which additional evidence stays in logs. |
| Object geometry | Separate collision geometry and its reference pose/transform, including relevant support objects. | How the scene is populated and updated across movement, `Grip` and `Release`; not necessarily part of a movement goal. |
| Execution options | Movement profile, force limit, and proposed speed/`plan_only` options. | Exact fields, defaults, units, bounds and allowed overrides. |

A candidate TCP pose must not be reused as the object-centre pose.
The visible cloud centroid is not necessarily the full object's centre either.
Collision checking while carrying needs a defined relationship between the held object and the TCP; TCP candidates alone do not supply it.

**Agreed by Dmitrii, 2026-11-14:** `GoToPosition` receives the candidate array and compares found feasible movement plans by cost within one shared search-time budget.
Select the best feasible plan found under the chosen cost definition, rather than immediately committing to the first feasible candidate.
**Agreed by Dmitrii, 2026-11-14:** the selection criterion combines motion convenience with the candidate-quality assessment supplied by perception.
The budget is shared across the request's candidate/IK search, not restarted independently for each candidate.
The chosen plan must cover the complete requested movement profile; incomplete or invalid paths are not alternatives to rank by cost.
This policy selects the best plan found within the search budget and does not promise a global optimum or exploration of every candidate/IK solution.

Proposed implementation: preserve candidate identities and validate inputs before planning, then compare feasible plans using the same cost definition across candidates.
Perception ranking may influence search order; it cannot substitute for arm reachability, full-arm collision checks or driver-limit validation.
Return the selected candidate ID and the executed/planned TCP pose for traceability; reporting the selected cost and planning time is also proposed.
The exact motion metric, combination formula, normalization and weights, budget value/configuration, tie-breaking, allocation of search effort and exact deadline/result behavior remain open.
The meaning of GUI `selected_id`—preference or strict restriction—also remains open.

### Combined selection cost

The criterion evaluates a pair: a feasible movement plan and its target candidate.
Different IK solutions/plans for one candidate share that candidate's perception quality but may differ in motion convenience.
The combined ranking applies only after feasibility checks; a favorable quality score cannot compensate for collision, joint-limit or other invalid motion.

The current perception [grasp generator](https://github.com/RoboCup-FEI/perception/blob/3ccebbb0a0614348785c5d4e7c58507ddb6288a4/grasp.py) computes a geometric `rank_score` and sorts it in descending order, so higher currently means preferred.
The score combines several geometric terms; its export does not establish a calibrated success probability or a common scale for other perception producers.
The ROS score contract and conversion into the combined cost need to state direction, scale and missing-data behavior explicitly.

Possible motion terms to discuss include joint-path length, predicted execution time and posture preferences.
No particular motion term, weighted-sum formula or numeric weight has been selected yet.
Define how ordinary camera/general-movement targets and Katya's node provide quality, or how equal/absent quality is represented, before freezing the goal message.
For traceability, reporting the motion term, perception term and combined selection value is proposed.

An empty perception candidate list means no grasp proposal was produced; it is not an execution command.
S8 exports `approved_to_execute: false`; the integration must define validation and dispatch rather than interpreting the export as an instruction to move.
Unknown object orientation must be represented explicitly; a default identity quaternion must not silently invent an orientation for a thin object.

## 7. Proposed coordinates and observation validity

Use metres, newtons and quaternion orientation in the public ROS contract, with a stamped, named reference frame.
RPY may be accepted by an adapter only with its convention and radians made explicit.
S8 currently uses quaternion order `xyzw` and `Rz(yaw) @ Ry(pitch) @ Rx(roll)` for RPY.

Transform wrist-camera observations with the robot/camera transform at image capture time.
Using the latest TF after VLM inference can move the target when the wrist has moved.
The implementation needs capture-time transforms retained with the observation, or enough TF history for the expected processing latency.
Missing transforms, ambiguous time bases or stale observations must produce a defined failure without movement.
Observation identity and capture time serve different purposes and must remain distinct.

The repository currently specifies planning in `world` at the arm base, with the mobile base stationary during execution.
The full mobile TF chain, observation invalidation after base motion, maximum observation age and motion tolerances need agreement with navigation and perception.
The base-motion interlock must be explicit; merely defining `safe_to_drive` does not enforce mutual exclusion.

The gap-to-hardware conversion belongs in one backend adapter.
The real gripper currently converts ROS gap to jaw width using its configured 19.5 mm finger offset.
Perception and AI must not apply that offset again.

## 8. Agreed worker behavior and remaining execution details

The selected busy worker policy of the sketch establishes these runner invariants:

- One shared worker serves all three actions; there is no FIFO queue of waiting requests.
- Admission reserves `busy_` under the mutex before waking the worker; the handoff slot is not a queue.
- A concurrent request gets `Admission::Busy` and cannot replace or preempt the active command.
- Ownership passes through `std::unique_ptr<TaskInterface>` from the caller to the runner and then to the worker.
- The worker executes outside the mutex; `execute()` waits for the physical operation to finish or stop before normal return releases the resource.
- An exception records the failure and retains the unavailable resource state; subsequent admission reports `Faulted`.

**Agreed by Dmitrii, 2026-11-14:** when the runner reports `Busy`, accept the ROS goal and immediately terminate it with action status `ABORTED` and application outcome `BUSY`, as proposed in S4.
This supersedes the sketch's ROS comment proposing goal rejection; the approved runner admission policy is unchanged.
ROS goal acceptance does not grant admission to physical execution.

- The busy request receives a normal action result carrying `BUSY`; its numeric code is still part of the draft failure enumeration.
- It is not submitted for later execution, does not acquire the worker and does not wait for the active operation to finish.
- It does not cancel or replace the active command, alter the planning scene/holding state or mark the active worker idle.
- The AI adapter must read the outcome from the aborted action's result body and expose `BUSY` to its recovery logic.

The ROS handling of other admission failures (`Faulted`, `Stopping`, invalid input), including outcome codes, remains to be agreed.

Other proposed execution behavior remains under review:

- MTC candidate search stays inside `GoToPosition`; task-level retry, new observations and base repositioning belong to AI.
- Feedback identifies planning/execution phase and a stable step name.
- `plan_only` performs no hardware movement and must not change the live held-object state or leave trial attachments in the shared scene.
- A planning result does not reserve an executable plan; a later execution validates current robot/scene state again.

Each terminal result should contain the semantic outcome, failing step, diagnostic text, relevant raw backend error, whether motion occurred and the resulting holding state.
`GoToPosition` should additionally identify the chosen candidate.
Holding needs an explicit unknown/unavailable representation when evidence is missing; a Boolean alone may be insufficient.
An action transport success and an application success must agree.
An interrupted skill must not appear successful to the AI adapter.

Proposed cancellation behavior: request stop from the active planner/arm/gripper operation, preserve holding where possible, and do not start retreat, opening or other recovery motion after cancellation.
Report whether stopping was confirmed and whether an object remains held.
A cancel acknowledgement alone must not be treated as proof that physical motion has stopped.
When the stop cannot be confirmed, keep the server in a fault/uncertain state and prevent the next movement until resolved.
Actual cancellation/stop timing on Jazzy and the real robot requires measurement.

Automatic cleanup after a failed grasp is a separate decision.
S4 proposes opening and retreating after an empty or wrong-width grasp; those compound recovery movements do not follow from the selected `Grip` primitive and must not be copied implicitly.

## 9. Proposed failure and status contract

S4 §7 supplies the starting failure taxonomy: invalid goal/precondition, `BUSY`, unreachable/no path/planning timeout, empty/wrong-width/lost object, execution/gripper/scene failure, hardware readiness/fault, safety check and internal error.
Its numeric values remain draft values until the shared interface is agreed.
Resolve missing distinctions for unavailable/stale TF, stale observation, unsupported capability, force limit and unconfirmed stop before assigning final numbers.
Planner failure alone must not claim that a target is mathematically unreachable without supporting evidence.

Keep one low-level-to-public error mapping and machine-readable codes in `manipulation_interfaces`.
Diagnostic text is for people and logs; AI must not parse it to decide recovery.
Client-side server-unavailable and timeout outcomes remain distinguishable from a result returned by the server.
On an aborted accepted action, the AI adapter must preserve the result body and failure code.
`BUSY` is returned through an accepted, aborted goal, not through ROS goal rejection.
Other ROS rejection/transport failures must remain distinguishable from this server-provided outcome; their mapping is still open.
On a client timeout, the adapter must resolve cancellation/state before issuing another physical attempt.

Proposed continuous status includes the active skill, server/backend health, known held-object identity, holding evidence/validity, current transport readiness and last outcome.
Object loss and backend loss must be observable even when no skill is running.
Timestamp, freshness policy, QoS and restart behavior remain open.
A stale or unknown status cannot grant permission for base motion.

## 10. Force and backend boundaries

Force requirements have different control needs:

| Requirement | Needed behavior | Current evidence / unresolved issue |
| --- | --- | --- |
| `Grip(max_force)` | Apply gripper force and verify holding. | Real gripper supports a force command; simulated force and holding semantics need an adapter and physics validation. |
| `GoToPosition(max_force)` | Monitor external force during motion and stop at the limit. | Three interpretations documented in §10.1; selection explicitly left open by Dmitrii on 2026-11-14. Requires calibrated/fresh wrench and measured stop response. |
| `Release(force_thresh > 0)` | Detect an external pull and open. | Define axis/norm, frame, baseline, duration/hysteresis and wait timeout. |
| `Wipe(req_force)` — later scope | Regulate contact force while following a surface. | Outside v1; needs a suitable continuous control path rather than a threshold monitor. |
| `PushPullMode` — later scope | Coordinate compliant arm behavior with base/object motion. | Outside v1; requirements and controller interface are not yet defined. |

### 10.1 GoToPosition.max_force: three open options

**Open by Dmitrii's request, 2026-11-14:** document all three options without selecting one or defining a default.
These are alternative interpretations of the external-force threshold during arm movement.
They do not define the gripper closing force in `Grip` or the trigger semantics of `Release`.

Let `F` be the calibrated external linear-force vector from the agreed FTM signal, in newtons.
Torques are separate quantities in newton-metres and are not included in the scalar force expressions below.
The formulas illustrate the monitored quantity; exact trigger timing and the choice of strict/non-strict threshold comparison remain open.

| Option | Monitored quantity | Behavior and tradeoff | Contract/configuration needed |
| --- | --- | --- | --- |
| 1. Magnitude across XYZ | `sqrt(Fx^2 + Fy^2 + Fz^2)` | Force in every direction contributes to one limit. The magnitude is independent of a rotation of the coordinate axes. The criterion does not distinguish axial contact from transverse contact. | Scalar threshold in newtons and the agreed external-force signal; no monitoring axis is needed in the goal. |
| 2. Component along TCP Z / approach axis | `abs(dot(F, z_tcp))`, with both vectors expressed in the same frame | Measures force along the chosen TCP Z / approach axis in either direction. Transverse force does not contribute to this criterion. Appropriate axis semantics must also be defined for general `PTP` movement, which need not have one constant approach direction. | Scalar threshold and a fixed convention for the axis. Decide whether it is the selected candidate's target TCP Z, the current TCP Z or another explicitly defined approach axis. |
| 3. Axis/direction supplied in the goal | `abs(dot(F, a))` for a normalized axis `a`, after transformation into the force frame | Allows each request to monitor its intended direction, including a direction different from TCP Z. Adds goal validation and frame/time semantics to the interface. As in option 2, transverse force does not contribute to this single-axis criterion. | Scalar threshold, a nonzero direction vector and its frame reference; normalize/validate the vector. Define whether the axis stays fixed or follows a moving frame. |

The absolute projection in options 2 and 3 represents a two-sided limit.
If a one-sided directional trigger is desired, its sign and comparison must be specified separately; neither behavior is selected here.
For a moving TCP/frame, using the axis at goal/candidate time and recomputing it during execution give different monitoring semantics.
The contract must choose the rule and align force and axis transforms in time before either axis option is implemented.

Shared open details include the external-force signal's bias/gravity/payload treatment, filtering and threshold persistence/hysteresis, sample freshness, missing-data behavior, threshold bounds/zero semantics and result reporting after a stop.
None of the three options establishes the robot's physical stopping time; that remains an execution measurement.
No option or default is approved, and the question remains in D10.

### 10.2 Backend dependencies

The public FTM node supports calibrated wrench output, but the current `ftm.yaml` does not provide the calibration matrix.
Raw magnitude must not silently stand in for a force in newtons.
Whether a streaming or admittance input becomes available is an open question with Tomas; the current public trajectory driver has none.
Calibrated force access and measured stop/release behavior are dependencies of the agreed v1 force features, not reasons to silently postpone them.
Continuous force regulation for wiping is outside the three-operation v1 scope.
Admittance remains a later control-path question rather than a prerequisite automatically imposed on threshold monitoring/release.

The target is the same skill interface on real, mock, Tomas's Gazebo and the team's Isaac backend.
The current gripper controllers differ; S4's `sim_gripper_server` is a proposed way to normalize their behavior.
It must expose measured/unsupported evidence honestly: mock success cannot establish physical grasping or contact-force control.
Unsupported force requests should return a defined failure rather than run with the force requirement ignored.
The exact backend action/topic set and capability advertisement remain open.

Division of work proposed on 2026-11-14: Dmitrii owns interface design, the primitive skill server, MTC and `GoToPosition` candidate selection; Viktor owns the Isaac backend and simulation gripper adapter.
This is a proposed assignment pending joint confirmation.
The backend action boundary must be defined independently of simulator internals.

## 11. Navigation coordination

S9 requires a handshake before driving: the arm is in a validated transport pose and any required payload is held.
An idle arm is not necessarily safe to drive.
Safe-to-drive computation must account for configuration, holding requirements, fresh status and faults.

S9 also requests load/dynamics information and a completion signal for placement or handover.
Decide whether AI forwards these events to navigation or navigation consumes manipulation status directly.
The robot must have an agreed response to object loss during transport, including who requests the base stop.
Cart kinematics and coordinated push/pull remain outside v1 until separately specified.

## 12. Decision register

Record a final resolution, date and participants in the last column when each item is agreed.
Until then the proposals above are discussion material.

| ID | Decision | Current position / required input | Resolution |
| --- | --- | --- | --- |
| D01 | v1 scope and skill granularity | Exactly `GoToPosition`, `Grip`, `Release`, with force from v1; independent `Pick`/`Place` excluded. | Agreed — Dmitrii, 2026-11-14 |
| D02 | Movement goal and candidate selection | `GoToPosition` accepts TCP-pose candidates; compare feasible plans using a combined motion-convenience/perception-quality cost within one shared search-time budget and select the best found. Formula, normalization/weights, missing-quality policy, budget and deadline details remain open. | Candidate array, server selection, bounded search and both cost components agreed — Dmitrii, 2026-11-14; metric/configuration details open |
| D03 | Frame, TCP and capture time | Proposed capture-time TF and explicit TCP; retain current model pending upstream changes. | Open |
| D04 | Errors, admission, cancel and recovery | Busy worker: one worker, immediate reservation, no waiting queue, fault retention. ROS busy goal accepted then immediately `ABORTED` with outcome `BUSY`; no execution or effect on the active command. Other failures, cancel and taxonomy remain open. | Worker and ROS `BUSY` mapping agreed — Dmitrii, 2026-11-14; other protocol details open |
| D05 | Public names and AI mapping | Resolve `grip`/`Grip` and `release`/`Release` with AI. | Open |
| D06 | Perception and scene data | Agree one semantic output for Katya and AI, geometry/support data and observation validity. | Open |
| D07 | Application architecture and AI adapter | Sketch's ROS/Command/runner/adapter separation and MoveIt/MTC retained; AI composes picking/placement from primitives. | Architecture agreed — Dmitrii, 2026-11-14; integration details open |
| D08 | Navigation handshake | Agree interlock, transport poses, payload status and event routing. | Open |
| D09 | Backend contract and ownership | Review simulation gripper adapter, force capabilities and Dmitrii/Viktor work split. | Open |
| D10 | Force behavior | Force belongs in v1 for all three operations. For `GoToPosition.max_force`, §10.1 compares XYZ magnitude, TCP Z / approach-axis component and a goal-specified axis/direction; no option/default is selected. Exact measurement, thresholds and fault behavior remain open. | Inclusion agreed — Dmitrii, 2026-11-14; all three `max_force` interpretations explicitly left open at Dmitrii's request, 2026-11-14 |
| D11 | Additional skills | `Wipe`, `PushPullMode`, separate named-state actions and other additions are outside the three-operation v1. | v1 exclusion agreed — Dmitrii, 2026-11-14 |

## 13. Evidence needed before implementation acceptance

The following are planned acceptance checks, not completed test results:

- A v1 client can build against `manipulation_interfaces` without MoveIt or a simulator.
- Success, invalid goals, `BUSY`, aborted results, cancel and client timeout preserve the agreed outcome and state.
- `GoToPosition` selects a candidate using a complete valid plan for its movement profile; it performs no implicit `Grip`, `Release` or future lift planning.
- When search finds feasible plans of different costs, `GoToPosition` selects the best found under the agreed metric rather than automatically using the first feasible candidate; the search-time budget is shared across candidates.
- The combined ranking accounts for both motion convenience and perception candidate quality; score direction/normalization and missing-quality handling follow the eventual agreed contract, while invalid paths stay ineligible.
- Concurrent calls to any pair of the three actions cannot both acquire the runner; admission is reserved before worker startup and is retained on an uncertain failure.
- A busy ROS goal is accepted and immediately returns `ABORTED` with `BUSY`; its result reaches AI while the active command and its resource reservation remain intact.
- A wrist-camera observation remains geometrically consistent after wrist motion, using the captured transform; invalid/stale observations fail explicitly.
- `plan_only` causes no movement or persistent held-object/attachment changes.
- `Grip` distinguishes empty/uncertain grasps from verified holding before AI requests a lift; object loss remains observable between primitive calls.
- `GoToPosition` force-limit stopping, `Grip` force limiting and `Release` immediate/force-triggered behavior satisfy the agreed measurement and timeout rules.
- All supported backends implement the agreed semantics or report an unsupported capability explicitly.
- A real-robot trial measures stop behavior and validates grasp/placement evidence at the existing conservative limits, with a person at the E-stop.

S4's earlier prototype results used Humble and the previous model/simulator.
They inform the design but do not establish that this contract works on Jazzy, Isaac or the current real robot.
