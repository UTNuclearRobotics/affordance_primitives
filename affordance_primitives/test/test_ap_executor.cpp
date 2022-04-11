///////////////////////////////////////////////////////////////////////////////
//      Title     : test_ap_executor.cpp
//      Project   : affordance_primitives
//      Created   : 03/28/2022
//      Author    : Adam Pettinger
//      Copyright : Copyright© The University of Texas at Austin, 2014-2022. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative license is explicitly included within the text of
//          each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation, including but not limited to those resulting from
//          defects in software and/or documentation, or loss or inaccuracy of
//          data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>

#include <affordance_primitives/ap_executor/ap_executor.hpp>
#include <affordance_primitives/msg_types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

constexpr double EPSILON = 1e-4;

class APExecutorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Robot params
    params.max_wrench.rot_x = 5;
    params.max_wrench.rot_y = 5;
    params.max_wrench.rot_z = 5;
    params.max_wrench.trans_x = 50;
    params.max_wrench.trans_y = 50;
    params.max_wrench.trans_z = 50;
    params.max_torque = 7.5;
    params.max_force = 75;
    params.max_ee_velocity.rot_x = 0.1;
    params.max_ee_velocity.rot_y = 0.1;
    params.max_ee_velocity.rot_z = 0.1;
    params.max_ee_velocity.trans_x = 0.5;
    params.max_ee_velocity.trans_y = 0.5;
    params.max_ee_velocity.trans_z = 0.5;

    // Exec params
    executor_params.monitor_ft_topic_name = "/ft";
    executor_params.param_manager_plugin_name = "affordance_primitives::EmptyParameterManager";
    executor_params.task_estimator_plugin_name = "affordance_primitives::KinematicTaskEstimator";
  }

  // void TearDown() override {}

  affordance_primitives::APRobotParameter params;
  affordance_primitives::ExecutorParameters executor_params;
};

TEST_F(APExecutorTest, enforce_individual_limits)
{
  // Set feedback that will violate individual limits but not total limits
  affordance_primitives::AffordancePrimitiveFeedback feedback;
  feedback.expected_wrench.wrench.torque.x = -5.5;
  feedback.expected_wrench.wrench.torque.y = 5.5;
  feedback.expected_wrench.wrench.force.z = -51;
  feedback.expected_wrench.wrench.force.y = 55;
  feedback.expected_wrench.wrench.force.x = 1;
  feedback.moving_frame_twist.twist.angular.x = -0.2;
  feedback.moving_frame_twist.twist.linear.y = 0.4;

  // Check that we enforce the limits
  ASSERT_NO_THROW(affordance_primitives::enforceAPLimits(params, feedback));
  EXPECT_NEAR(feedback.expected_wrench.wrench.torque.x, -5, EPSILON);
  EXPECT_NEAR(feedback.expected_wrench.wrench.torque.y, 5, EPSILON);
  EXPECT_NEAR(feedback.expected_wrench.wrench.force.z, -50, EPSILON);
  EXPECT_NEAR(feedback.expected_wrench.wrench.force.y, 50, EPSILON);
  EXPECT_NEAR(feedback.expected_wrench.wrench.force.x, 1, EPSILON);
  EXPECT_NEAR(feedback.moving_frame_twist.twist.angular.x, -0.1, EPSILON);
  EXPECT_NEAR(feedback.moving_frame_twist.twist.linear.y, 0.2, EPSILON);
  EXPECT_NEAR(feedback.moving_frame_twist.twist.linear.z, 0.0, EPSILON);
}

TEST_F(APExecutorTest, enforce_total_limits)
{
  // Set a feedback that violates the total wrench
  affordance_primitives::AffordancePrimitiveFeedback feedback;
  feedback.expected_wrench.wrench.force.x = 49;
  feedback.expected_wrench.wrench.force.y = -49;
  feedback.expected_wrench.wrench.force.z = -49;
  feedback.expected_wrench.wrench.torque.x = -4.9;
  feedback.expected_wrench.wrench.torque.y = -4.9;
  feedback.expected_wrench.wrench.torque.z = 4.9;

  // Check that the expected wrench was scaled down to be within limits
  ASSERT_NO_THROW(affordance_primitives::enforceAPLimits(params, feedback));
  Eigen::Vector3d force, torque;
  tf2::fromMsg(feedback.expected_wrench.wrench.force, force);
  tf2::fromMsg(feedback.expected_wrench.wrench.torque, torque);
  EXPECT_NEAR(force.norm(), 75, EPSILON);
  EXPECT_NEAR(torque.norm(), 7.5, EPSILON);
}

TEST_F(APExecutorTest, enforce_no_limits)
{
  // Change some params to unenforced
  params.max_torque = 0;
  params.max_wrench.trans_y = 0;
  params.max_ee_velocity.trans_x = 0;

  // Set some feedback that would violate prior limits
  affordance_primitives::AffordancePrimitiveFeedback feedback;
  feedback.expected_wrench.wrench.force.y = 70;
  feedback.expected_wrench.wrench.torque.x = -4.9;
  feedback.expected_wrench.wrench.torque.y = -4.9;
  feedback.expected_wrench.wrench.torque.z = 4.9;
  feedback.moving_frame_twist.twist.linear.x = 100;
  feedback.moving_frame_twist.twist.angular.z = 0.05;

  // Check that no values were changed
  ASSERT_NO_THROW(affordance_primitives::enforceAPLimits(params, feedback));
  EXPECT_NEAR(feedback.expected_wrench.wrench.force.y, 70, EPSILON);
  EXPECT_NEAR(feedback.expected_wrench.wrench.torque.x, -4.9, EPSILON);
  EXPECT_NEAR(feedback.expected_wrench.wrench.torque.y, -4.9, EPSILON);
  EXPECT_NEAR(feedback.expected_wrench.wrench.torque.z, 4.9, EPSILON);
  EXPECT_NEAR(feedback.moving_frame_twist.twist.linear.x, 100, EPSILON);
  EXPECT_NEAR(feedback.moving_frame_twist.twist.angular.z, 0.05, EPSILON);
}

TEST_F(APExecutorTest, initialize_and_end_properly)
{
  auto node = std::make_shared<rclcpp::Node>("test_ap_executor");

  // Try initializing with valid plugins, should not throw
  std::shared_ptr<affordance_primitives::APExecutor> executor;
  bool init_result = false;
  ASSERT_NO_THROW(
    executor = std::make_shared<affordance_primitives::APExecutor>(node, "action_name"));
  ASSERT_NO_THROW(init_result = executor->initialize(executor_params));
  EXPECT_TRUE(init_result);

  // Should not throw when we destruct the object
  ASSERT_NO_THROW(executor.reset());

  // Should fail gracefully when we initialize it with invalid plugin names
  executor_params.param_manager_plugin_name = "invalid_plugin";
  init_result = false;
  ASSERT_NO_THROW(
    executor = std::make_shared<affordance_primitives::APExecutor>(node, "action_name"));
  ASSERT_NO_THROW(init_result = executor->initialize(executor_params));
  EXPECT_FALSE(init_result);
  ASSERT_NO_THROW(executor.reset());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  return result;
}
