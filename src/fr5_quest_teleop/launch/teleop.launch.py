"""
teleop.launch.py — FR5 Quest2ROS teleop + RViz2 visualisation.

Brings up:
  * robot_state_publisher  (FR5 URDF from the frcobot_ros2 fairino_description pkg)
  * rviz2                  (shows the live robot; /joint_states comes from teleop)
  * fr5_quest_teleop teleop node

Prerequisites (separate terminals — see README):
  * ros2 launch ros_tcp_endpoint endpoint.py   (Quest app connects to this)
  * Quest app running, IP/port set to this machine

Example:
  ros2 launch fr5_quest_teleop teleop.launch.py fr5_ip:=192.168.58.2 \
      active_hand:=right position_scale:=1.0 control_orientation:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, Command,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("fr5_ip", default_value="192.168.58.2"),
        DeclareLaunchArgument("active_hand", default_value="right"),
        DeclareLaunchArgument("position_scale", default_value="1.0"),
        DeclareLaunchArgument("rotation_scale", default_value="1.0"),
        DeclareLaunchArgument("control_orientation", default_value="true"),
        DeclareLaunchArgument("servo_vel", default_value="15.0"),
        DeclareLaunchArgument("gripper_enable", default_value="true"),
        DeclareLaunchArgument("clutch_mode", default_value="hold"),
        DeclareLaunchArgument("clutch_field", default_value="press_middle"),
        DeclareLaunchArgument("gripper_mode", default_value="analog"),
        DeclareLaunchArgument("gripper_field", default_value="press_index"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "urdf",
            default_value=PathJoinSubstitution(
                [FindPackageShare("fairino_description"), "urdf", "fairino5_v6.urdf"]
            ),
            description="FR5 URDF (from the frcobot_ros2 workspace).",
        ),
    ]

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("urdf")]), value_type=str
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    teleop = Node(
        package="fr5_quest_teleop",
        executable="teleop",
        output="screen",
        parameters=[{
            "fr5_ip": LaunchConfiguration("fr5_ip"),
            "active_hand": LaunchConfiguration("active_hand"),
            "position_scale": LaunchConfiguration("position_scale"),
            "rotation_scale": LaunchConfiguration("rotation_scale"),
            "control_orientation": LaunchConfiguration("control_orientation"),
            "servo_vel": LaunchConfiguration("servo_vel"),
            "gripper_enable": LaunchConfiguration("gripper_enable"),
            "clutch_mode": LaunchConfiguration("clutch_mode"),
            "clutch_field": LaunchConfiguration("clutch_field"),
            "gripper_mode": LaunchConfiguration("gripper_mode"),
            "gripper_field": LaunchConfiguration("gripper_field"),
        }],
    )

    return LaunchDescription(args + [rsp, rviz, teleop])
