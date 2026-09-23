#!/usr/bin/env python3
"""MoveIt 2 for YAM-family arms. Defaults to mock hardware — no CAN, no arm
needed — for planning/IK/URDF checks.

Brings up robot_state_publisher, controller_manager (mock_components/
GenericSystem by default, or the real i2rt_hardware_interface/
YamSystemInterface if use_mock_hardware:=false), joint_state_broadcaster +
joint_trajectory_controller + gripper_controller, move_group, and RViz with
the MotionPlanning panel pre-loaded. MoveIt exposes three planning groups:
"arm" (joint1-6), "gripper" (gripper_joint alone), and "arm_gripper" (all 7
at once) - pick whichever one you want from RViz's Planning tab.

    ros2 launch i2rt_moveit_config demo.launch.py robot:=big_yam_linear_4310

To drive the REAL arm — only after independently confirming activation holds
position cleanly with plain ros2_control first, no MoveIt/trajectory
controller involved — add:

    use_mock_hardware:=false can_channel:=can0

If the arm has no gripper installed, add use_gripper:=false: the URDF is
built with no gripper links/joint, gripper_controller is not spawned, and
the SRDF's "gripper"/"arm_gripper" groups and "gripper" end effector are
left out of the semantic description served to move_group/RViz - only the
"arm" group is available to plan with. See i2rt_moveit_config/config/
i2rt.srdf's header comment for how the two SRDF variants are built.

To watch the real arm's live position in RViz while hand-guiding it, add
compliant_mode:=true: every joint's kp fallback becomes exactly 0.0 (gravity
comp alone holds the arm up) and kd falls back to that joint's
"compliant_kd" param - see big_yam_ros2_control.xacro's macro comment, and
i2rt_teleop/launch/leader_follower.launch.py which uses the same mechanism
for its leader arm. joint_trajectory_controller only ever claims the
"position" command interface (see i2rt_description/config/
yam_controllers.yaml), never kp/kd, so this fallback stays in effect even
while it's spawned and even if you press Execute in RViz - a position target
with kp=0 has no torque authority to actually track it, so planning/
execution is effectively inert while compliant_mode:=true, not just
"harmless." Set it back to false (the default) once you're done hand-guiding
and want the arm to actually execute trajectories again.

    ros2 launch i2rt_moveit_config demo.launch.py robot:=big_yam_linear_4310 \\
        use_mock_hardware:=false can_channel:=can0 compliant_mode:=true

Named arm (arm_name): give it an arbitrary name YOU assign (e.g. "yam1")
instead of separately passing robot/can_channel, and it resolves - via
i2rt_description/launch/arm_registry.py's shared ARM_REGISTRY - to that
specific physical arm's robot model, usual CAN channel, AND its own
validated per-unit parameter overrides (gravity_comp_factor today,
potentially other <ros2_control> params later):

    ros2 launch i2rt_moveit_config demo.launch.py arm_name:=yam1 \\
        use_mock_hardware:=false compliant_mode:=true

A recognized arm_name is authoritative for robot and for the parameter
overrides - always. can_channel is different: a physical arm can't always
be assumed to be on its usual channel (e.g. temporary rewiring), so an
EXPLICIT can_channel always wins there, even over a recognized arm_name's
registry default:

    ros2 launch i2rt_moveit_config demo.launch.py arm_name:=yam1 can_channel:=can_yam2 \\
        use_mock_hardware:=false compliant_mode:=true

arm_name defaults to "" (no registry entry - robot/can_channel fall back to
the robot/can_channel arguments below, exactly as if arm_registry.py didn't
exist, and no overrides are applied) - in that case robot is required (no
default), same as before this mechanism existed. See arm_registry.py's own
module docstring for the full registry format.
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

# See this file's module docstring, "Named arm" - arm_registry.py lives in
# this same package (i2rt_description), installed alongside this file, but
# still needs the share-directory + sys.path treatment since it's imported
# as a plain Python module, not part of an ament_python package - see
# arm_registry.py's own docstring for the full why.
sys.path.insert(0, os.path.join(get_package_share_directory("i2rt_description"), "launch"))
import arm_registry  # noqa: E402


def generate_launch_description():
    arm_name_arg = DeclareLaunchArgument(
        "arm_name",
        default_value="",
        description=(
            "Arbitrary name identifying a specific physical arm (e.g. 'yam1') - when recognized in "
            "i2rt_description/launch/arm_registry.py's ARM_REGISTRY, overrides robot/can_channel and "
            "applies that arm's own validated parameter overrides. Default '' (no registry entry - "
            "falls back to robot/can_channel below with no overrides). See this file's module "
            "docstring, 'Named arm' section."
        ),
    )
    robot_arg = DeclareLaunchArgument(
        "robot",
        default_value="",
        description=(
            "Which robot variant's URDF to load. One of: yam_linear_4310, big_yam_linear_4310. "
            "Required unless arm_name resolves to a registry entry instead (see arm_name above)."
        ),
        choices=["", "yam_linear_4310", "big_yam_linear_4310"],
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description=(
            "true (default): ros2_control's built-in simulator, no CAN/arm needed. "
            "false: the real i2rt_hardware_interface plugin — only after confirming "
            "activation holds position cleanly with plain ros2_control first, no MoveIt "
            "involved."
        ),
        choices=["true", "false"],
    )
    can_channel_arg = DeclareLaunchArgument(
        "can_channel",
        default_value="",
        description=(
            "SocketCAN interface the arm is on. Ignored when use_mock_hardware:=true. Default '' - an "
            "arm can't always be assumed to be on its usual channel (e.g. temporary rewiring), so an "
            "explicit value here always wins, even over a recognized arm_name's registry default. "
            "Truly unset ('') falls back to arm_name's registry can_channel if it resolves, else the "
            "hardcoded 'can0'."
        ),
    )
    use_gripper_arg = DeclareLaunchArgument(
        "use_gripper",
        default_value="true",
        description=(
            "true (default): arm is built with the linear_4310 gripper - URDF links/joint, "
            "ros2_control gripper_joint interface, gripper_controller spawner, and the SRDF's "
            "gripper/arm_gripper groups and gripper end effector. false: bare-wrist arm with "
            "none of the above - only the 'arm' planning group is available."
        ),
        choices=["true", "false"],
    )
    compliant_mode_arg = DeclareLaunchArgument(
        "compliant_mode",
        default_value="false",
        description=(
            "false (default): normal stiff position-hold, MoveIt plans/executes normally. "
            "true: hand-backdrivable (kp=0, kd=each joint's compliant_kd param, gravity comp holds "
            "it up) - for watching the real arm's live position in RViz while hand-guiding it. "
            "Real hardware only; mock hardware has no impedance model so this has no effect there. "
            "Execution is effectively inert while this is true - see this file's module docstring."
        ),
        choices=["true", "false"],
    )

    return LaunchDescription(
        [
            arm_name_arg,
            robot_arg,
            use_mock_hardware_arg,
            can_channel_arg,
            use_gripper_arg,
            compliant_mode_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context, *args, **kwargs):
    arm_name = LaunchConfiguration("arm_name").perform(context)
    entry = arm_registry.resolve(arm_name)

    # robot: a recognized arm_name is authoritative - see this file's module
    # docstring, "Named arm". Falls back to the explicit robot argument when
    # arm_name is blank or unrecognized, exactly as if arm_registry.py
    # didn't exist.
    robot = entry["robot"] if entry else LaunchConfiguration("robot").perform(context)

    # can_channel: an EXPLICIT can_channel (non-blank) always wins, even
    # over a recognized arm_name's registry default - an arm can't always be
    # assumed to be on its usual channel (e.g. temporary rewiring). Only
    # falls back to the registry default (if arm_name resolves) or the
    # hardcoded "can0" when truly unset.
    can_channel_explicit = LaunchConfiguration("can_channel").perform(context)
    can_channel = can_channel_explicit or (entry["can_channel"] if entry else "can0")

    if not robot:
        raise RuntimeError(
            "demo.launch.py: either arm_name must resolve to a known entry in "
            "i2rt_description/launch/arm_registry.py's ARM_REGISTRY, or robot must be set explicitly "
            "(one of: yam_linear_4310, big_yam_linear_4310)."
        )

    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context) == "true"
    gravity_comp_override_file = None if use_mock_hardware else arm_registry.overrides_params_file(arm_name)

    urdf_file = PathJoinSubstitution([FindPackageShare("i2rt_description"), "urdf", [robot, ".urdf.xacro"]])
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_file,
                " use_mock_hardware:=",
                LaunchConfiguration("use_mock_hardware"),
                " can_channel:=",
                can_channel,
                " use_gripper:=",
                LaunchConfiguration("use_gripper"),
                " compliant_mode:=",
                LaunchConfiguration("compliant_mode"),
            ]
        ),
        value_type=str,
    )

    moveit_config_share = get_package_share_directory("i2rt_moveit_config")
    srdf_file = os.path.join(moveit_config_share, "config", "i2rt.srdf")
    robot_description_semantic = ParameterValue(
        Command(["xacro ", srdf_file, " use_gripper:=", LaunchConfiguration("use_gripper")]),
        value_type=str,
    )

    kinematics_yaml = os.path.join(moveit_config_share, "config", "kinematics.yaml")
    joint_limits_yaml = os.path.join(moveit_config_share, "config", "joint_limits.yaml")
    ompl_yaml = os.path.join(moveit_config_share, "config", "ompl_planning.yaml")
    moveit_controllers_yaml = os.path.join(moveit_config_share, "config", "moveit_controllers.yaml")
    rviz_config = os.path.join(moveit_config_share, "config", "moveit.rviz")

    ros2_controllers_yaml = os.path.join(
        get_package_share_directory("i2rt_description"), "config", "yam_controllers.yaml"
    )
    controller_manager_params = [{"robot_description": robot_description}, ros2_controllers_yaml]
    if gravity_comp_override_file:
        controller_manager_params.append(gravity_comp_override_file)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=controller_manager_params,
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        condition=IfCondition(LaunchConfiguration("use_gripper")),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            kinematics_yaml,
            joint_limits_yaml,
            ompl_yaml,
            moveit_controllers_yaml,
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
            # MoveIt's default (0.01 rad) rejects execution if the real arm's
            # current position has drifted this much from the planned
            # trajectory's start point by the time Execute is pressed.
            # i2rt_hardware_interface does compute a KDL-based gravity feed-
            # forward (see YamSystemInterface's class comment), but it's
            # never perfectly exact, so some small hold-position sag is
            # still expected. Raised as a stopgap; tighten this back down if
            # gravity comp is ever retuned closely enough not to need it.
            {"trajectory_execution.allowed_start_tolerance": 0.05},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            kinematics_yaml,
            ompl_yaml,
        ],
    )

    return [
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        gripper_controller_spawner,
        move_group_node,
        rviz_node,
    ]
