# yam_ros2_control

ROS2 hardware interface for the i2rt YAM 6-DOF arm, integrated with ros2_control and MoveIt for motion planning and control.

## Packages

| Package | Purpose |
|---|---|
| `i2rt_can_driver` | SocketCAN transport + DM motor protocol (MIT mode) used by the hardware interface. |
| `i2rt_description` | URDF/xacro, meshes, controller config (`yam_controllers.yaml`), and an RViz-only visualization launch. |
| `i2rt_hardware_interface` | `ros2_control` `SystemInterface` plugin (`YamSystemInterface`) that talks to the real arm over CAN, with KDL-based gravity feed-forward. |
| `i2rt_moveit_config` | MoveIt 2 config (SRDF, kinematics, OMPL) + the main single-arm bringup launch. |
| `i2rt_msgs` | Custom motor feedback/status messages. |
| `i2rt_teleop` | Leader/follower teleop node + dual-arm bringup launch. |
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

## Getting started

Everything below defaults to **mock hardware** (`use_mock_hardware:=true`) — no CAN interface or physical arm required — so it's safe to try first. Only add `use_mock_hardware:=false` once you've confirmed the real arm holds position cleanly. Everything below also defaults to the arm having the linear_4310 **gripper installed** (`use_gripper:=true`); pass `use_gripper:=false` throughout if you're working with a bare-wrist arm.

### Visualize the URDF only (no ros2_control)

```bash
ros2 launch i2rt_description view_yam.launch.py
```
Opens RViz + `joint_state_publisher_gui` sliders. No `controller_manager` is started, so this never opens a CAN connection regardless of arguments.

To check what a no-gripper arm looks like before running teleop without one:
```bash
ros2 launch i2rt_description view_yam.launch.py use_gripper:=false
```

### Single arm (ros2_control + MoveIt)

```bash
ros2 launch i2rt_moveit_config demo.launch.py robot:=big_yam_linear_4310
```
Brings up `robot_state_publisher`, `controller_manager`, `joint_state_broadcaster`, `joint_trajectory_controller`, `gripper_controller`, `move_group`, and RViz with the MotionPlanning panel. Planning groups: `arm` (joint1-6), `gripper`, `arm_gripper`.

If the arm has no gripper installed, add `use_gripper:=false`: `gripper_controller` is not spawned, and only the `arm` planning group is available (no `gripper`/`arm_gripper` groups or gripper end effector).
```bash
ros2 launch i2rt_moveit_config demo.launch.py robot:=big_yam_linear_4310 use_gripper:=false
```

Real hardware:
```bash
ros2 launch i2rt_moveit_config demo.launch.py robot:=big_yam_linear_4310 \
    use_mock_hardware:=false can_channel:=can0
```

Key args: `robot` (`big_yam_linear_4310` | `yam_crank_4310`, required), `use_mock_hardware` (default `true`), `can_channel` (default `can0`, ignored in mock mode), `use_gripper` (default `true`; set `false` if the arm has no gripper installed).

### Teleop (dual-arm leader/follower)

The leader-follower node stays inert (logs "Still inert: alignment_confirmed is false" every 10s and never commands the follower) until `alignment_confirmed:=true` is passed — this check is unconditional, not just a real-hardware thing. On real hardware only set it after physically commanding both arms to the same known pose and visually confirming they match (see leader_follower_node.cpp's startup log for the full reasoning); in mock hardware there's no physical arm to misalign, so it's safe to set right away.

```bash
ros2 launch i2rt_teleop leader_follower.launch.py alignment_confirmed:=true
```
Mock hardware on both arms by default, gripper included. The leader is hand-backdrivable (compliant/gravity-comp only, no position controller); the follower mirrors it with a normal stiff position hold.

If neither arm has a gripper installed, still try this in mock hardware first:
```bash
ros2 launch i2rt_teleop leader_follower.launch.py use_gripper:=false alignment_confirmed:=true
```

Real hardware — only after independently confirming each arm holds position cleanly on its own, **and** manually verifying leader/follower joint-zero alignment:
```bash
ros2 launch i2rt_teleop leader_follower.launch.py use_mock_hardware:=false \
    leader_can_channel:=can0 follower_can_channel:=can1 alignment_confirmed:=true
```
Add `use_gripper:=false` to the above as well if neither arm has a gripper installed.

Key args: `use_mock_hardware` (default `true`), `leader_can_channel` (default `can0`), `follower_can_channel` (default `can1`), `leader_compliant_mode` (default `true`), `alignment_confirmed` (default `false`; the node stays inert without it regardless of mock/real hardware - only skip the physical joint-zero check it describes when using mock hardware), `max_joint_velocity` (rad/s clamp, default `1.0`), `use_gripper` (default `true`; set `false` if neither arm has a gripper installed).

Quick command I use:
```bash
ros2 launch i2rt_teleop leader_follower.launch.py use_mock_hardware:=false alignment_confirmed:=true max_joint_velocity:=3.0
```

### VLA inference bridge

See [i2rt_vla_bridge/README.md](i2rt_vla_bridge/README.md) for the OpenVLA → IK → arm pipeline (requires a single, unnamespaced arm via `i2rt_moveit_config/demo.launch.py`, not the dual-arm teleop launch).
