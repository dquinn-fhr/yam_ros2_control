#!/usr/bin/env python3
"""
Visualize a YAM-family arm + linear_4310 description in RViz.

Brings up robot_state_publisher (fed by <robot>.urdf.xacro, default
big_yam_linear_4310) plus joint_state_publisher_gui for manual slider
control, and RViz. No controller_manager is started here, so the
<ros2_control> block in the xacro is inert -- the CAN channel is never
opened, regardless of can_channel/use_mock_hardware (see
big_yam_linear_4310.urdf.xacro's comment). Those args only matter once this
description is consumed by a real control launch file.

Pass robot:=yam_linear_4310 to visualize the standard yam arm instead:

    ros2 launch i2rt_description view_yam.launch.py robot:=yam_linear_4310

RViz defaults to i2rt_description/rviz/view_robot.rviz (RobotModel sourced
from the /robot_description topic + TF + Grid, Fixed Frame "world" -- matches
this xacro's root link). Pass rviz_config:="" to fall back to RViz's default
unconfigured view instead, or point it at a different config entirely.

Pass use_gripper:=false to check a bare-wrist URDF with no gripper links/
joint -- see i2rt_teleop/launch/leader_follower.launch.py, which accepts the
same argument for teleop.

Named arm (arm_name): give it an arbitrary name YOU assign (e.g. "yam1")
instead of remembering which robot variant that physical arm is - resolves
via arm_registry.py's shared ARM_REGISTRY (see that module's docstring) to
the right robot model (and can_channel, though irrelevant here since no
controller_manager runs in this file - no CAN connection is ever opened,
same as before). A recognized arm_name overrides robot always; an EXPLICIT
can_channel still wins over the registry default if you pass one (an arm
can't always be assumed to be on its usual channel). Blank or unrecognized
arm_name is a no-op, falls back to robot/can_channel below:

    ros2 launch i2rt_description view_yam.launch.py arm_name:=yam1
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# See this file's module docstring, "Named arm" - see arm_registry.py's own
# docstring for why this sys.path approach rather than a proper package
# import (arm_registry.py lives in this same package, but still isn't
# importable as a plain Python module without this).
sys.path.insert(0, os.path.join(get_package_share_directory("i2rt_description"), "launch"))
import arm_registry  # noqa: E402


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "arm_name",
            default_value="",
            description=(
                "Arbitrary name identifying a specific physical arm (e.g. 'yam1') - when recognized "
                "in i2rt_description/launch/arm_registry.py's ARM_REGISTRY, overrides robot/"
                "can_channel below. Default '' (no registry entry). See this file's module "
                "docstring, 'Named arm' section."
            ),
        ),
        DeclareLaunchArgument(
            "robot",
            default_value="big_yam_linear_4310",
            description="Which robot variant's URDF to load. One of: yam_linear_4310, big_yam_linear_4310.",
            choices=["yam_linear_4310", "big_yam_linear_4310"],
        ),
        DeclareLaunchArgument(
            "can_channel",
            default_value="",
            description=(
                "CAN interface name forwarded into the xacro's ros2_control block (inert here - no "
                "controller_manager runs in this file). Default '' - an explicit value here always "
                "wins over a recognized arm_name's registry default; truly unset ('') falls back to "
                "arm_name's registry can_channel if it resolves, else the hardcoded 'can0'."
            ),
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description=(
                "Forwarded into the xacro's ros2_control block. Irrelevant here since no "
                "controller_manager is started by this launch file, but kept true by default "
                "so this file never implies a real CAN connection."
            ),
        ),
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            description=(
                "Forwarded into the xacro. true (default): includes the linear_4310 gripper "
                "links/joint. false: bare-wrist URDF with no gripper - use to check what a "
                "no-gripper arm looks like before running teleop with use_gripper:=false."
            ),
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_joint_state_publisher_gui",
            default_value="true",
            description="Launch joint_state_publisher_gui for manual joint sliders.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("i2rt_description"), "rviz", "view_robot.rviz"]
            ),
            description=(
                "Path to an RViz config file. Pass an empty string to launch RViz with its "
                "default (unsaved, no displays configured) view instead."
            ),
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


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

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("i2rt_description"), "urdf", [robot, ".urdf.xacro"]]
    )

    robot_description_content = Command(
        [
            "xacro ",
            xacro_file,
            " can_channel:=",
            can_channel,
            " use_mock_hardware:=",
            LaunchConfiguration("use_mock_hardware"),
            " use_gripper:=",
            LaunchConfiguration("use_gripper"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_joint_state_publisher_gui")),
    )

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    rviz_node_no_config = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        condition=IfCondition(
            PythonExpression(["'", use_rviz, "' == 'true' and '", rviz_config, "' == ''"])
        ),
    )

    rviz_node_with_config = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(
            PythonExpression(["'", use_rviz, "' == 'true' and '", rviz_config, "' != ''"])
        ),
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node_no_config,
        rviz_node_with_config,
    ]
