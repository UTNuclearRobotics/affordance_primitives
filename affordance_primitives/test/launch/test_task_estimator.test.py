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
    task_estimator_gtest = Node(
        executable=PathJoinSubstitution(
            [LaunchConfiguration("test_binary_dir"), "test_task_estimator"]
        ),
        output="screen",
    )

    tf_pub_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub",
        arguments=["--frame-id", "/moving_frame", "--child-frame-id", "/task_frame", "-x", "2", "-qx", "0.7071", "-qw", "0.7071"]
    )

    return LaunchDescription(
        [
            tf_pub_node,
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            TimerAction(period=2.0, actions=[task_estimator_gtest]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "task_estimator_gtest": task_estimator_gtest,
    }
