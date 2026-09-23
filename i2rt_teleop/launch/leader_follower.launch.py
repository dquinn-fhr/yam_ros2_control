#!/usr/bin/env python3
"""Leader-follower bringup for two YAM arms: one ros2_control stack per arm
under its own namespace ('leader'/'follower'; each still uses unprefixed
joint1..joint6 internally, disambiguated only by namespace - see
leader_follower_node.cpp's class comment for why no cross-arm zero-alignment
is assumed or checked automatically) plus the mirroring node itself.

The leader arm is hand-backdrivable by default (compliant_mode - see
big_yam_ros2_control.xacro's macro comment, which quotes and matches
MotorChainRobot's zero_gravity_mode exactly): every joint's kp falls back
to exactly 0.0 (gravity comp alone holds the arm up) and kd falls back to
that joint's own "compliant_kd" param (the ported, already hardware-tuned
grav_comp_kd value, not a guess), and no position controller is spawned at
all, since it's meant to be hand-guided and only read from, never
commanded. The follower keeps its normal stiff position-hold behavior and
full controller set, since it's the one being driven.

Defaults to mock hardware on both sides for safe, no-CAN-needed testing
(mock hardware has no impedance/MIT control model, so compliant_mode has no
real effect there - the follower's own position tracking is still visible,
just not the leader's backdrivability):

    ros2 launch i2rt_teleop leader_follower.launch.py

Real hardware (only after independently confirming both arms hold position
cleanly with plain ros2_control first, no teleop node involved, AND manually
verifying leader/follower joint-zero alignment - see leader_follower_node.cpp):

    ros2 launch i2rt_teleop leader_follower.launch.py use_mock_hardware:=false \\
        leader_can_channel:=can0 follower_can_channel:=can1 alignment_confirmed:=true

Both arms default to having the linear_4310 gripper installed. If neither arm
has a gripper physically attached, pass use_gripper:=false - this drops the
gripper links/joint from both arms' URDFs, skips spawning the follower's
gripper_controller, and disables the leader-follower node's gripper mirroring:

    ros2 launch i2rt_teleop leader_follower.launch.py use_gripper:=false

Defaults to big_yam_linear_4310 for both arms. leader_robot/follower_robot
are independently selectable, so a big_yam arm can drive a standard yam arm
(or vice versa) as well as matching pairs:

    ros2 launch i2rt_teleop leader_follower.launch.py \\
        leader_robot:=big_yam_linear_4310 follower_robot:=yam_linear_4310

IMPORTANT when mixing variants: mirroring is raw 1:1 joint-angle copying
(joint1..joint6 by name, see arm_nodes()'s docstring above) with NO inverse
kinematics or scaling. big_yam and yam have different link lengths (big_yam
is the larger arm), so the same joint angles put their end effectors in
DIFFERENT points in space - the follower's gripper will NOT track the
leader's gripper position/pose, only its joint angles. Each arm's own real
joint limits are still respected independently (joint_lower_limits/
joint_upper_limits sent to leader_follower_node are chosen to match
follower_robot - see launch_setup() below), so this is safe to run, just be
aware it isn't spatially matched between differently-sized arms.

Also: yam's and big_yam's URDFs are independently-authored CAD exports, so
some joints' "positive" rotation direction disagrees between them even
though each arm's own calibration is internally correct (confirmed
2026-09-22, real hardware: joint1/joint4/joint5/joint6 - leader and follower
visibly rotated opposite ways on those joints with a naive 1:1 copy).
leader_follower_node's mirror_sign parameter corrects this per joint; this
launch file sets it automatically for a mismatched pair (see
MIRROR_SIGN_FOR_MISMATCHED_PAIR below) - a matched pair (same variant both
sides) needs no correction. If you still see a joint rotate opposite between
leader and follower after this, override mirror_sign directly.

Multiple sessions at once (e.g. one yam pair and one big_yam pair
simultaneously): pass a unique session_name to each, e.g.:

    ros2 launch i2rt_teleop leader_follower.launch.py session_name:=pair1 \\
        leader_can_channel:=can_yam1 follower_can_channel:=can_bigyam1 ...
    ros2 launch i2rt_teleop leader_follower.launch.py session_name:=pair2 \\
        leader_can_channel:=can_yam2 follower_can_channel:=can_bigyam2 ...

Without session_name (default ""), namespaces are the plain "leader"/
"follower" used throughout this file's examples above - fine for one
session at a time, but running a second concurrent instance that way WILL
collide: both would claim identical node/topic/service names ("/leader/...",
"/follower/...", plus leader_follower_node itself has no namespace at all
without session_name), so one session's arm state could silently leak into
the other's. session_name namespaces everything under
"<session_name>_leader"/"<session_name>_follower" instead, and gives
leader_follower_node its own "<session_name>" namespace, so two (or more)
sessions with different session_name values are fully isolated from each
other on the ROS graph.

Named arms (leader_arm_name/follower_arm_name): give either slot an
arbitrary name YOU assign (e.g. "yam1") and it resolves, via
i2rt_description/launch/arm_registry.py's shared ARM_REGISTRY, to that
specific physical arm's robot model, USUAL CAN channel, AND its own
validated per-unit parameter overrides (gravity_comp_factor today,
potentially other <ros2_control> params later) - all three at once, from
one name, instead of separately passing leader_robot/leader_can_channel and
hoping they agree with which physical unit you actually mean:

    ros2 launch i2rt_teleop leader_follower.launch.py \\
        leader_arm_name:=yam1 follower_arm_name:=bigyam1 ...

A recognized arm_name is authoritative for robot and for the parameter
overrides - always. can_channel is different: a physical arm can't always
be assumed to be on its usual channel (e.g. temporary rewiring), so an
EXPLICIT leader_can_channel/follower_can_channel always wins there, even
over a recognized arm_name's registry default - e.g. yam1 usually means
can_yam1, but leader_arm_name:=yam1 leader_can_channel:=can_yam2 gets
yam1's model and overrides while actually talking to can_yam2:

    ros2 launch i2rt_teleop leader_follower.launch.py \\
        leader_arm_name:=yam1 leader_can_channel:=can_yam2 ...

leader_arm_name/follower_arm_name default to "" (no registry entry - robot/
can_channel fall back to leader_robot/leader_can_channel etc. exactly as if
arm_registry.py didn't exist, and no overrides are applied); an
unrecognized name is the same no-op, not an error. See arm_registry.py's
own module docstring for the full registry format and how to add/update a
physical unit's entry once a genuine per-unit difference is found and
validated via live tuning. No entries are currently populated for the
standard yam arm: an initial round of testing (2026-09-22) found yam1/yam2
apparently needing different gravity_comp_factor values, which looked like
real per-unit variance - it wasn't. link_6 (the wrist-roll link) was wrongly
carrying the linear_4310 gripper's full mesh/mass even in bare-wrist mode
(see yam_macro.xacro's link_6 comment); once that was fixed, both units
converged on identical values, which were promoted to yam_ros2_control.
xacro's shared defaults instead of staying here as per-arm overrides. This
mechanism still exists for if a genuine per-unit difference is ever found.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# See this file's module docstring, "Named arms" - arm_registry.py lives in
# i2rt_description (an already-ament_cmake, not importable-as-a-Python-
# package, package - see arm_registry.py's own docstring for why this
# sys.path approach rather than a proper package import).
sys.path.insert(0, os.path.join(get_package_share_directory("i2rt_description"), "launch"))
import arm_registry  # noqa: E402

# joint1..joint6 [lower, upper] limits (radians) per robot variant, matching
# each one's own urdf/<variant>_macro.xacro <limit> tags - used to pick which
# set leader_follower_node clamps commanded targets to, based on whichever
# arm is actually the follower (see launch_setup()). Defaults hardcoded in
# leader_follower_node.cpp's declare_parameter calls match BIG_YAM_LIMITS
# below; kept in sync here rather than relied upon, so mixing in a yam
# follower doesn't silently clamp it to big_yam's (looser, in places) range.
BIG_YAM_LIMITS = (
    [-2.61799, 0.0, 0.0, -1.5708, -1.5708, -2.0944],
    [3.05433, 3.14159, 3.14159, 1.5708, 1.5708, 2.0944],
)
YAM_LIMITS = (
    [-2.61799, 0.0, 0.0, -1.69297, -1.5708, -2.0944],
    [3.05433, 3.14159, 3.66519, 1.5708, 1.5708, 2.0944],
)
ROBOT_JOINT_LIMITS = {
    "big_yam_linear_4310": BIG_YAM_LIMITS,
    "yam_linear_4310": YAM_LIMITS,
}

# joint1..joint6 sign multiplier applied to the LEADER's reported position
# before mirroring (see leader_follower_node.cpp's mirror_sign parameter
# comment for the full why). yam_macro.xacro and big_yam_macro.xacro are
# independently-authored onshape exports, so nothing guarantees they agree on
# which rotation direction is "positive" for a given joint even though each
# arm's own direction/offset calibration is internally correct. Only matters
# when leader_robot != follower_robot - a matched pair uses IDENTITY_SIGN (no
# correction needed, both sides share one URDF's convention).
#
# joint6 corrected (2026-09-22): an initial pass (yam + big_yam pairing, real
# hardware) found joint1/joint4/joint5/joint6 needing a flip - but that test
# predates per-unit direction being individually confirmed correct on all
# four physical arms (yam1/yam2/bigyam1/bigyam2, each independently verified
# via demo.launch.py's compliant_mode + RViz, matching commanded direction).
# With every arm's OWN calibration now known-good, re-testing teleop directly
# (yam1 leading bigyam1) showed joint6 STILL backwards with the -1 flip
# applied - meaning the original -1 for joint6 was itself wrong, not a
# leftover per-unit issue. Corrected to 1.0 (no flip needed for joint6).
# joint1/joint4/joint5 haven't shown a problem in this same re-test and are
# left as originally found; if one of them turns out wrong too under the
# same rigorous check, fix it here the same way.
IDENTITY_SIGN = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
MIRROR_SIGN_FOR_MISMATCHED_PAIR = [-1.0, 1.0, 1.0, -1.0, -1.0, 1.0]

# Per-physical-unit robot model/can_channel/gravity_comp_factor overrides
# now live in the shared arm_registry module (see this file's module
# docstring, "Named arms") - imported above as arm_registry, used in
# launch_setup() below via arm_registry.resolve()/overrides_params_file().


def arm_nodes(
    namespace, can_channel, compliant_mode, spawn_position_controllers, use_gripper, robot,
    gravity_comp_override_file=None,
):
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("i2rt_description"), "urdf", [robot, ".urdf.xacro"]]
    )
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_file,
                " use_mock_hardware:=",
                LaunchConfiguration("use_mock_hardware"),
                " can_channel:=",
                can_channel,
                " compliant_mode:=",
                compliant_mode,
                " use_gripper:=",
                use_gripper,
            ]
        ),
        value_type=str,
    )
    ros2_controllers_yaml = os.path.join(
        get_package_share_directory("i2rt_description"), "config", "yam_controllers.yaml"
    )
    ros2_control_node_params = [{"robot_description": robot_description}, ros2_controllers_yaml]
    if gravity_comp_override_file:
        ros2_control_node_params.append(gravity_comp_override_file)

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[{"robot_description": robot_description}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace,
            parameters=ros2_control_node_params,
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["joint_state_broadcaster"],
        ),
    ]
    # A compliant/hand-guided arm (the leader) has nothing for these to
    # usefully do - joint_trajectory_controller would hold whatever position
    # it was in at spawn time via the "position" command interface, which is
    # harmless once compliant_mode has already zeroed kp (the hold becomes a
    # no-op), but there's no reason to run a position controller on an arm
    # that's never meant to be commanded, and it removes a footgun if
    # compliant_mode is ever misconfigured.
    if spawn_position_controllers:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=["joint_trajectory_controller"],
            )
        )
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=["gripper_controller"],
                condition=IfCondition(use_gripper),
            )
        )
    return nodes


def generate_launch_description():
    session_name_arg = DeclareLaunchArgument(
        "session_name",
        default_value="",
        description=(
            "Unique name for this leader/follower pair, required to run more than one pair "
            "concurrently without node/topic/service collisions. Empty (default): namespaces are "
            "the plain 'leader'/'follower' - fine for a single session. Non-empty: namespaces "
            "become '<session_name>_leader'/'<session_name>_follower', and leader_follower_node "
            "itself is namespaced under '<session_name>' - see this file's module docstring."
        ),
    )
    leader_robot_arg = DeclareLaunchArgument(
        "leader_robot",
        default_value="big_yam_linear_4310",
        description="Which robot variant's URDF the leader arm uses. One of: yam_linear_4310, big_yam_linear_4310.",
        choices=["yam_linear_4310", "big_yam_linear_4310"],
    )
    follower_robot_arg = DeclareLaunchArgument(
        "follower_robot",
        default_value="big_yam_linear_4310",
        description=(
            "Which robot variant's URDF the follower arm uses. One of: yam_linear_4310, "
            "big_yam_linear_4310. Also selects which arm's real joint limits leader_follower_node "
            "clamps commanded targets to."
        ),
        choices=["yam_linear_4310", "big_yam_linear_4310"],
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description=(
            "true (default): ros2_control's built-in simulator for both arms, no CAN/hardware needed. "
            "false: real i2rt_hardware_interface on both sides - only after independently confirming each arm "
            "holds position cleanly with plain ros2_control, and manually verifying leader/follower joint-zero "
            "alignment (see leader_follower_node.cpp's class comment)."
        ),
        choices=["true", "false"],
    )
    leader_can_channel_arg = DeclareLaunchArgument(
        "leader_can_channel",
        default_value="",
        description=(
            "Leader arm's SocketCAN interface. Default '' - an arm can't always be assumed to be on "
            "its usual channel (e.g. temporary rewiring), so an explicit value here always wins, even "
            "over a recognized leader_arm_name's registry default. Truly unset ('') falls back to "
            "leader_arm_name's registry can_channel if it resolves, else the hardcoded 'can0'."
        ),
    )
    follower_can_channel_arg = DeclareLaunchArgument(
        "follower_can_channel",
        default_value="",
        description=(
            "Follower arm's SocketCAN interface. Default '' - an arm can't always be assumed to be on "
            "its usual channel (e.g. temporary rewiring), so an explicit value here always wins, even "
            "over a recognized follower_arm_name's registry default. Truly unset ('') falls back to "
            "follower_arm_name's registry can_channel if it resolves, else the hardcoded 'can1'."
        ),
    )
    leader_arm_name_arg = DeclareLaunchArgument(
        "leader_arm_name",
        default_value="",
        description=(
            "Arbitrary name identifying the specific physical arm in the leader slot (e.g. 'yam1') - "
            "when recognized in i2rt_description/launch/arm_registry.py's ARM_REGISTRY, overrides "
            "leader_robot/leader_can_channel and applies that arm's own validated parameter overrides. "
            "Default '' (no registry entry - falls back to leader_robot/leader_can_channel with no "
            "overrides). See this file's module docstring, 'Named arms' section."
        ),
    )
    follower_arm_name_arg = DeclareLaunchArgument(
        "follower_arm_name",
        default_value="",
        description=(
            "Arbitrary name identifying the specific physical arm in the follower slot (e.g. 'bigyam1') - "
            "when recognized in i2rt_description/launch/arm_registry.py's ARM_REGISTRY, overrides "
            "follower_robot/follower_can_channel and applies that arm's own validated parameter overrides. "
            "Default '' (no registry entry - falls back to follower_robot/follower_can_channel with no "
            "overrides). See this file's module docstring, 'Named arms' section."
        ),
    )
    leader_compliant_mode_arg = DeclareLaunchArgument(
        "leader_compliant_mode",
        default_value="true",
        description=(
            "true (default): leader arm is hand-backdrivable (kp=0, kd=each joint's ported grav_comp_kd "
            "value, no position controller spawned) - see big_yam_ros2_control.xacro's macro comment. Real "
            "hardware only; mock hardware has no impedance model so this has no effect there."
        ),
        choices=["true", "false"],
    )
    alignment_confirmed_arg = DeclareLaunchArgument(
        "alignment_confirmed",
        default_value="false",
        description=(
            "Must be manually set true after physically verifying the leader and follower agree on what joint "
            "position 0 means (command both to the same known pose independently and visually confirm they "
            "match) - see leader_follower_node.cpp's class comment. The node stays inert until this is true."
        ),
        choices=["true", "false"],
    )
    max_joint_velocity_arg = DeclareLaunchArgument(
        "max_joint_velocity",
        default_value="1.0",
        description="rad/s clamp applied to every commanded step, including the startup ramp's minimum duration.",
    )
    use_gripper_arg = DeclareLaunchArgument(
        "use_gripper",
        default_value="true",
        description=(
            "true (default): both arms are built with the linear_4310 gripper (URDF links/joint and the "
            "ros2_control gripper_joint interface), the follower spawns gripper_controller, and the "
            "leader-follower node mirrors gripper position. false: bare-wrist arms with no gripper links, "
            "joint, or controller, and gripper mirroring disabled - use when neither arm has a gripper "
            "physically installed."
        ),
        choices=["true", "false"],
    )

    return LaunchDescription(
        [
            session_name_arg,
            leader_robot_arg,
            follower_robot_arg,
            use_mock_hardware_arg,
            leader_can_channel_arg,
            follower_can_channel_arg,
            leader_arm_name_arg,
            follower_arm_name_arg,
            leader_compliant_mode_arg,
            alignment_confirmed_arg,
            max_joint_velocity_arg,
            use_gripper_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):
    # Every value here is only known once launch args are resolved
    # (context.perform), unlike everything else in this file which stays as
    # lazily-resolved Substitutions - needed because joint_lower_limits/
    # joint_upper_limits/mirror_sign are typed (vector<double>) node
    # parameters (so the right literal list has to be picked in Python
    # before the node is constructed), and because arm_registry.resolve()
    # needs a concrete arm_name string to look up, not a Substitution.
    session_name = LaunchConfiguration("session_name").perform(context)
    leader_arm_name = LaunchConfiguration("leader_arm_name").perform(context)
    follower_arm_name = LaunchConfiguration("follower_arm_name").perform(context)
    leader_entry = arm_registry.resolve(leader_arm_name)
    follower_entry = arm_registry.resolve(follower_arm_name)

    # A recognized arm_name is authoritative for robot/can_channel - see
    # this file's module docstring, "Named arms". Falls back to the
    # explicit leader_robot/leader_can_channel (etc.) arguments when
    # leader_arm_name/follower_arm_name is blank or unrecognized, exactly
    # as if arm_registry.py didn't exist.
    leader_robot = leader_entry["robot"] if leader_entry else LaunchConfiguration("leader_robot").perform(context)
    follower_robot = (
        follower_entry["robot"] if follower_entry else LaunchConfiguration("follower_robot").perform(context)
    )

    # can_channel precedence: an EXPLICIT leader_can_channel/follower_can_channel
    # (non-blank) always wins, even over a recognized arm_name's registry
    # default - an arm can't always be assumed to be on its usual channel
    # (e.g. temporary rewiring). Only falls back to the registry default (if
    # arm_name resolves) or the hardcoded "can0"/"can1" when truly unset.
    leader_can_channel_explicit = LaunchConfiguration("leader_can_channel").perform(context)
    follower_can_channel_explicit = LaunchConfiguration("follower_can_channel").perform(context)
    leader_can_channel = leader_can_channel_explicit or (
        leader_entry["can_channel"] if leader_entry else "can0"
    )
    follower_can_channel = follower_can_channel_explicit or (
        follower_entry["can_channel"] if follower_entry else "can1"
    )
    joint_lower_limits, joint_upper_limits = ROBOT_JOINT_LIMITS[follower_robot]
    mirror_sign = IDENTITY_SIGN if leader_robot == follower_robot else MIRROR_SIGN_FOR_MISMATCHED_PAIR

    # Skipped entirely under mock hardware, which never loads
    # YamSystemInterface at all (mock_components/GenericSystem has no
    # gravity model to override).
    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context) == "true"
    leader_gravity_comp_override_file = (
        None if use_mock_hardware else arm_registry.overrides_params_file(leader_arm_name)
    )
    follower_gravity_comp_override_file = (
        None if use_mock_hardware else arm_registry.overrides_params_file(follower_arm_name)
    )

    # Empty session_name (default) keeps the plain "leader"/"follower"
    # namespaces every example in this file's docstring uses, unchanged from
    # before this arg existed. A non-empty session_name disambiguates a
    # concurrent second (third, ...) pair - see the module docstring's
    # "Multiple sessions at once" section for why this is required, not
    # optional, for running more than one pair at a time.
    leader_ns = f"{session_name}_leader" if session_name else "leader"
    follower_ns = f"{session_name}_follower" if session_name else "follower"

    leader_follower_node = Node(
        package="i2rt_teleop",
        executable="leader_follower_node",
        namespace=session_name if session_name else None,
        parameters=[
            {
                "alignment_confirmed": LaunchConfiguration("alignment_confirmed"),
                "max_joint_velocity": LaunchConfiguration("max_joint_velocity"),
                "mirror_gripper": ParameterValue(LaunchConfiguration("use_gripper"), value_type=bool),
                "joint_lower_limits": joint_lower_limits,
                "joint_upper_limits": joint_upper_limits,
                "mirror_sign": mirror_sign,
                # Defaults in leader_follower_node.cpp hardcode "/leader"/
                # "/follower" - override explicitly here so this still
                # points at the right topics/action whenever session_name
                # changes leader_ns/follower_ns away from those literals.
                "leader_joint_states_topic": f"/{leader_ns}/joint_states",
                "follower_joint_states_topic": f"/{follower_ns}/joint_states",
                "follower_trajectory_topic": f"/{follower_ns}/joint_trajectory_controller/joint_trajectory",
                "follower_gripper_action_name": f"/{follower_ns}/gripper_controller/gripper_cmd",
            }
        ],
        output="screen",
    )

    return [
        *arm_nodes(
            leader_ns,
            leader_can_channel,
            LaunchConfiguration("leader_compliant_mode"),
            spawn_position_controllers=False,
            use_gripper=LaunchConfiguration("use_gripper"),
            robot=leader_robot,
            gravity_comp_override_file=leader_gravity_comp_override_file,
        ),
        *arm_nodes(
            follower_ns,
            follower_can_channel,
            "false",
            spawn_position_controllers=True,
            use_gripper=LaunchConfiguration("use_gripper"),
            robot=follower_robot,
            gravity_comp_override_file=follower_gravity_comp_override_file,
        ),
        leader_follower_node,
    ]
