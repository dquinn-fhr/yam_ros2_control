"""Central per-physical-arm registry shared across every launch file that
brings up a real YAM-family arm (i2rt_description/launch/view_yam.launch.py,
i2rt_moveit_config/launch/demo.launch.py,
i2rt_teleop/launch/leader_follower.launch.py) - the single source of truth
for which robot model a given arm_name is, its usual CAN channel, and any
per-unit parameter overrides (gravity_comp_factor today, potentially other
<ros2_control> hardware-component params in the future - see ARM_REGISTRY's
own comment below).

Not an installed Python package - these packages are ament_cmake, and
adding a full ament_python module just for this felt heavier than
warranted. Instead, every launch file that needs this inserts
i2rt_description's installed share/launch directory onto sys.path at launch
time via ament_index_python's get_package_share_directory() - a standard,
stable ROS2 API - then imports this module normally (see any of the three
files above for the real import block). This is a well-established pattern
for sharing a small launch-time helper module across packages without a
full package split, not a fragile hack; i2rt_moveit_config already
exec_depends on i2rt_description for its URDF/config paths, and i2rt_teleop
now does too (see its package.xml), for the same reason.

Usage in a launch file: resolve(arm_name) to get
{"robot", "can_channel", "overrides"} or None (unknown/blank arm_name);
overrides_params_file(arm_name) to get a generated wildcarded ROS params
YAML path (or None) for that entry's "overrides" dict, suitable to append
directly into a ros2_control_node's `parameters=[...]` list - the same
pattern yam_controllers.yaml already uses, and for the same reason: this
needs to reach YamSystemInterface's diagnostics_node_, which isn't the
ros2_control_node process's own default/primary node, so a plain
(non-wildcarded) params dict targeting that Node() action's own resolved
name would silently never match it.

When arm_name resolves to a known entry, it is authoritative: the calling
launch file uses that entry's robot/can_channel instead of whatever a
separately-passed robot/can_channel argument said, and applies its
overrides. A blank or unrecognized arm_name is a silent no-op - the caller
falls back to its own explicit robot/can_channel arguments with no
overrides applied, exactly as if this registry didn't exist.
"""
import tempfile

import yaml

# arm_name -> {robot, can_channel, overrides}. arm_name is an arbitrary
# label YOU assign (not tied to CAN wiring or a leader/follower slot/
# session - see this file's module docstring). "robot" is which xacro
# variant that named arm actually is, so specifying arm_name also selects
# the robot model - no need to separately pass robot:=... and risk it
# disagreeing with which arm_name you meant. "can_channel" is that arm's
# usual udev-pinned CAN interface name (see the repo's CAN-adapter udev
# rules); still overridable per-launch via the ordinary can_channel
# argument if you ever need to (e.g. temporary rewiring) - passing both
# arm_name and a conflicting can_channel is not an error, arm_name simply
# wins. "overrides" is a dict of literal <ros2_control> hardware-component
# parameter names (exactly as YamSystemInterface's on_init()/
# diagnostics_node_ declare them - see yam_system_interface.hpp's class
# comment) to that unit's own validated value - currently only
# gravity_comp_factor.<joint> entries exist, but this is deliberately
# generic; any parameter name the hardware component or diagnostics_node_
# declares works the same way, for whatever other per-unit calibration
# need comes up later (e.g. a per-unit compliant_kd, if that's ever found
# to vary too).
#
# Add/update an entry once a physical unit's own value is validated - e.g.
# via `ros2 param set /<namespace>/<prefix>yam_system_diagnostics
# gravity_comp_factor.<joint> <value>` (see yam_system_interface.hpp's
# class comment for the live-tuning mechanism itself) against that unit's
# own running session - to make it permanent and automatic everywhere,
# regardless of which launch file, leader/follower slot, or session that
# specific arm ends up used in.
ARM_REGISTRY = {
    "yam1": {
        "robot": "yam_linear_4310",
        "can_channel": "can_yam1",
        # No overrides - yam1's re-tuned joint2/3/4 values (0.9/0.9/0.95,
        # 2026-09-22) matched yam2's exactly once link_6's gripper-housing-
        # mass bug was fixed (see yam_macro.xacro's link_6 comment and
        # yam_ros2_control.xacro's header comment for the full story), so
        # they were promoted to yam_ros2_control.xacro's shared defaults
        # instead of staying as per-arm overrides here. If a genuine
        # per-unit difference ever shows up again (confirmed via the same
        # braced live-tuning methodology, not just a one-off symptom), add
        # it back here.
        "overrides": {},
    },
    "yam2": {
        "robot": "yam_linear_4310",
        "can_channel": "can_yam2",
        # No overrides - see yam1's comment above; both units converged on
        # identical values after the link_6 mass fix, now yam_ros2_control.
        # xacro's shared defaults.
        "overrides": {},
    },
    "bigyam1": {
        "robot": "big_yam_linear_4310",
        "can_channel": "can_bigyam1",
        "overrides": {},
    },
    "bigyam2": {
        "robot": "big_yam_linear_4310",
        "can_channel": "can_bigyam2",
        "overrides": {},
    },
}


def resolve(arm_name):
    """Returns ARM_REGISTRY[arm_name] if arm_name is a known, non-blank
    name, else None."""
    return ARM_REGISTRY.get(arm_name) if arm_name else None


def overrides_params_file(arm_name):
    """Returns a path to a generated wildcarded ("/**") ROS params YAML
    with arm_name's "overrides" dict, or None if arm_name is unknown or has
    no overrides - see this file's module docstring for the full why."""
    entry = resolve(arm_name)
    overrides = entry.get("overrides") if entry else None
    if not overrides:
        return None
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix=f"arm_overrides_{arm_name}_"
    ) as f:
        yaml.safe_dump({"/**": {"ros__parameters": dict(overrides)}}, f)
        return f.name
