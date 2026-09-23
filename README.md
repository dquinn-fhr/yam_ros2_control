# yam_ros2_control

ROS2 hardware interface for the i2rt YAM family of 6-DOF arms (standard `yam` and the larger `big_yam`), integrated with ros2_control and MoveIt for motion planning and control.

## Packages

| Package | Purpose |
|---|---|
| `i2rt_can_driver` | SocketCAN transport + DM motor protocol (MIT mode) used by the hardware interface. |
| `i2rt_description` | URDF/xacro for both arm models, meshes, controller config (`yam_controllers.yaml`), the RViz-only visualization launch, the shared per-arm registry (`launch/arm_registry.py`), and the CAN adapter udev rules (`udev/`). |
| `i2rt_hardware_interface` | `ros2_control` `SystemInterface` plugin (`YamSystemInterface`) that talks to the real arm over CAN, with KDL-based gravity feed-forward (live-tunable per joint, see below). |
| `i2rt_moveit_config` | MoveIt 2 config (SRDF, kinematics, OMPL) + the main single-arm bringup launch. |
| `i2rt_msgs` | Custom motor feedback/status messages. |
| `i2rt_teleop` | Leader/follower teleop node + dual-arm bringup launch (supports mixed arm models and concurrent sessions). |
| `i2rt_vla_bridge` | OpenVLA → IK → arm bridge (see its own [README](i2rt_vla_bridge/README.md)). |

## Setup

Requires ROS 2 (Jazzy) with `colcon`, `xacro`, and MoveIt 2 installed.

```bash
# from the workspace root (this repo)
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Re-source `install/setup.bash` in every new shell before running any `ros2 launch`/`ros2 run` command below.

If you're driving a real arm, make sure the SocketCAN interface is up first, e.g.:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

### One-time: persistent CAN interface names

If you have more than one physical arm, plugging USB-CAN adapters in in a different order (or into different ports) can otherwise reassign which one gets called `can0` vs `can1`. `i2rt_description/udev/99-i2rt-can.rules` pins each adapter's interface name to its USB serial number instead, so it's stable regardless of plug order:

```bash
sudo cp i2rt_description/udev/99-i2rt-can.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
# then unplug/replug every adapter
```

See that file's own header comment for how to identify a new/replacement adapter's serial number. This is what backs the `can_yam1`/`can_yam2`/`can_bigyam1`/`can_bigyam2` names used below and in `arm_registry.py`.

## Naming your arms

`i2rt_description/launch/arm_registry.py` is a small shared registry mapping an arbitrary name you assign to a specific physical arm (e.g. `"yam1"`) to its robot model, usual CAN channel, and any of its own validated parameter overrides (currently `gravity_comp_factor`, per joint — see that file's own docstring for the full format). It's used by all three launch files below via an `arm_name` argument (`leader_arm_name`/`follower_arm_name` for teleop):

```bash
ros2 launch i2rt_moveit_config demo.launch.py arm_name:=yam1 use_mock_hardware:=false
```

This resolves `robot` and `can_channel` automatically — no need to separately remember which model a given arm is or which channel it's usually on. A recognized `arm_name` is authoritative for the robot model and for its saved overrides; `can_channel` is different — an *explicitly* passed `can_channel` always wins even over a recognized `arm_name`'s usual channel (useful for temporary rewiring), falling back to the registry's channel only when left unset. Without an `arm_name` (or with one the registry doesn't recognize), everything falls back to the plain `robot`/`can_channel` arguments described in each section below, exactly as if this registry didn't exist.

To add a new arm, add an entry to `ARM_REGISTRY` in `arm_registry.py` — see that file's docstring for the live-tuning workflow used to find a physical unit's own parameter overrides (a ROS parameter on the hardware component's diagnostics node, settable with `ros2 param set` while the arm is running, no relaunch needed).

## Getting started

Everything below defaults to **mock hardware** (`use_mock_hardware:=true`) — no CAN interface or physical arm required — so it's safe to try first. Only add `use_mock_hardware:=false` once you've confirmed the real arm holds position cleanly. Everything below also defaults to the arm having the linear_4310 **gripper installed** (`use_gripper:=true`); pass `use_gripper:=false` throughout if you're working with a bare-wrist arm — the URDF correctly omits the gripper housing's mesh and mass in that case, not just its fingers.

### Visualize the URDF only (no ros2_control)

```bash
ros2 launch i2rt_description view_yam.launch.py
```
Opens RViz + `joint_state_publisher_gui` sliders. No `controller_manager` is started, so this never opens a CAN connection regardless of arguments. Defaults to `robot:=big_yam_linear_4310`; pass `robot:=yam_linear_4310` (or `arm_name:=<a yam arm's name>`) for the standard yam arm instead.

To check what a no-gripper arm looks like before running teleop without one:
```bash
ros2 launch i2rt_description view_yam.launch.py use_gripper:=false
```

### Single arm (ros2_control + MoveIt)

```bash
ros2 launch i2rt_moveit_config demo.launch.py arm_name:=yam1
```
Brings up `robot_state_publisher`, `controller_manager`, `joint_state_broadcaster`, `joint_trajectory_controller`, `gripper_controller`, `move_group`, and RViz with the MotionPlanning panel. Planning groups: `arm` (joint1-6), `gripper`, `arm_gripper`.

Without `arm_name`, `robot` is required (one of `yam_linear_4310`, `big_yam_linear_4310`):
```bash
ros2 launch i2rt_moveit_config demo.launch.py robot:=big_yam_linear_4310
```

If the arm has no gripper installed, add `use_gripper:=false`: `gripper_controller` is not spawned, and only the `arm` planning group is available (no `gripper`/`arm_gripper` groups or gripper end effector).

Real hardware — only after independently confirming activation holds position cleanly with plain ros2_control first, no MoveIt involved:
```bash
ros2 launch i2rt_moveit_config demo.launch.py arm_name:=yam1 use_mock_hardware:=false
```

To watch the real arm's live position in RViz while hand-guiding it (gravity comp holds it up, no active position control), add `compliant_mode:=true`:
```bash
ros2 launch i2rt_moveit_config demo.launch.py arm_name:=yam1 use_mock_hardware:=false compliant_mode:=true
```

Key args: `arm_name` (see "Naming your arms" above), `robot` (`big_yam_linear_4310` | `yam_linear_4310`, required unless `arm_name` resolves), `use_mock_hardware` (default `true`), `can_channel` (falls back to `arm_name`'s registry channel, then `can0`, ignored in mock mode), `use_gripper` (default `true`), `compliant_mode` (default `false`).

### Teleop (dual-arm leader/follower)

The leader-follower node stays inert (logs "Still inert: alignment_confirmed is false" every 10s and never commands the follower) until `alignment_confirmed:=true` is passed — this check is unconditional, not just a real-hardware thing. On real hardware only set it after physically commanding both arms to the same known pose and visually confirming they match (see `leader_follower_node.cpp`'s startup log for the full reasoning); in mock hardware there's no physical arm to misalign, so it's safe to set right away.

```bash
ros2 launch i2rt_teleop leader_follower.launch.py alignment_confirmed:=true
```
Mock hardware on both arms by default, gripper included. The leader is hand-backdrivable (compliant/gravity-comp only, no position controller); the follower mirrors it with a normal stiff position hold.

Real hardware, named arms — only after independently confirming each arm holds position cleanly on its own, **and** manually verifying leader/follower joint-zero alignment:
```bash
ros2 launch i2rt_teleop leader_follower.launch.py \
    leader_arm_name:=yam1 follower_arm_name:=bigyam1 \
    use_mock_hardware:=false use_gripper:=false alignment_confirmed:=true
```

Without named arms, the equivalent is:
```bash
ros2 launch i2rt_teleop leader_follower.launch.py use_mock_hardware:=false \
    leader_robot:=yam_linear_4310 follower_robot:=big_yam_linear_4310 \
    leader_can_channel:=can0 follower_can_channel:=can1 alignment_confirmed:=true
```

`leader_robot`/`follower_robot` are independently selectable (a big_yam leader can drive a standard yam follower, or vice versa, as well as matching pairs). Mirroring is raw 1:1 joint-angle copying with no inverse kinematics — big_yam and yam have different link lengths, so the follower's gripper will *not* track the leader's gripper position/pose in space, only its joint angles. Each arm's own real joint limits are still respected (the follower's, specifically). When the two models differ, a per-joint sign correction (`mirror_sign`) is applied automatically to account for the two arms' independently-authored CAD models not necessarily agreeing on which rotation direction is "positive" for a given joint — see `leader_follower.launch.py`'s `MIRROR_SIGN_FOR_MISMATCHED_PAIR` if you ever see a joint rotate opposite between leader and follower despite this.

**Running more than one pair at once** (e.g. a yam+big_yam pair and a second yam+big_yam pair simultaneously): give each session a unique `session_name` so they don't collide on node/topic/service names, and run each in its own terminal:
```bash
# terminal 1
ros2 launch i2rt_teleop leader_follower.launch.py session_name:=pair1 \
    leader_arm_name:=yam1 follower_arm_name:=bigyam1 \
    use_mock_hardware:=false use_gripper:=false alignment_confirmed:=true

# terminal 2
ros2 launch i2rt_teleop leader_follower.launch.py session_name:=pair2 \
    leader_arm_name:=yam2 follower_arm_name:=bigyam2 \
    use_mock_hardware:=false use_gripper:=false alignment_confirmed:=true
```

Key args: `leader_arm_name`/`follower_arm_name` (see "Naming your arms" above), `leader_robot`/`follower_robot` (`big_yam_linear_4310` | `yam_linear_4310`, each default `big_yam_linear_4310`, ignored if the corresponding `arm_name` resolves), `session_name` (default `""` — required for concurrent sessions, see above), `use_mock_hardware` (default `true`), `leader_can_channel`/`follower_can_channel` (fall back to the registry channel, then `can0`/`can1`), `leader_compliant_mode` (default `true`), `alignment_confirmed` (default `false`), `max_joint_velocity` (rad/s clamp, default `1.0`), `use_gripper` (default `true`).

Quick command I use:
```bash
ros2 launch i2rt_teleop leader_follower.launch.py leader_arm_name:=yam1 follower_arm_name:=bigyam1 \
    use_mock_hardware:=false alignment_confirmed:=true max_joint_velocity:=3.0
```

## Gravity compensation

`YamSystemInterface` computes a KDL-based per-joint gravity feed-forward torque, scaled by each joint's `gravity_comp_factor` (see `yam_ros2_control.xacro`/`big_yam_ros2_control.xacro`'s header comments for the current per-joint values and how they were derived — real hardware doesn't match the upstream config's numbers closely enough to trust as-is). It's live-tunable per joint at runtime without relaunching, via a ROS parameter on the hardware component's internal diagnostics node — see `yam_system_interface.hpp`'s class comment (gravity compensation section) for the exact parameter name and usage. Once a value is validated for a specific physical arm, add it to that arm's entry in `arm_registry.py` (or, if it turns out to be a universal fix rather than a per-unit difference, update the shared xacro default instead) to make it permanent.

## VLA inference bridge

See [i2rt_vla_bridge/README.md](i2rt_vla_bridge/README.md) for the OpenVLA → IK → arm pipeline (requires a single, unnamespaced arm via `i2rt_moveit_config/demo.launch.py`, not the dual-arm teleop launch).
