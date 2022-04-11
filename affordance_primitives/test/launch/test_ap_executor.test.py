import launch
import launch_testing
import os
import sys
import unittest
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import TimerAction

sys.path.append(os.path.dirname(__file__))

def generate_test_description():
    ap_executor_gtest = Node(
        executable=PathJoinSubstitution(
            [LaunchConfiguration("test_binary_dir"), "test_ap_executor"]
        ),
        output="screen",
    )

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            TimerAction(period=2.0, actions=[ap_executor_gtest]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "ap_executor_gtest": ap_executor_gtest,
    }
