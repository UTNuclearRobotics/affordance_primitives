///////////////////////////////////////////////////////////////////////////////
//      Title     : test_screw_execution.cpp
//      Project   : affordance_primitives
//      Created   : 12/30/2021
//      Author    : Adam Pettinger
//      Copyright : Copyright© The University of Texas at Austin, 2014-2021. All
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

#include <affordance_primitives/msg_types.hpp>
#include <affordance_primitives/screw_model/affordance_utils.hpp>
#include <affordance_primitives/screw_model/screw_execution.hpp>
#include <rclcpp/rclcpp.hpp>

const std::string TASK_FRAME_NAME = "task_frame";
const std::string MOVING_FRAME_NAME = "moving_frame";
constexpr double EPSILON = 1e-4;

inline void checkVector(const affordance_primitives::Vector3 & vec, double x, double y, double z)
{
  EXPECT_NEAR(vec.x, x, EPSILON);
  EXPECT_NEAR(vec.y, y, EPSILON);
  EXPECT_NEAR(vec.z, z, EPSILON);
}

inline void checkVector(const Eigen::Vector3d & vec, double x, double y, double z)
{
  EXPECT_NEAR(vec.x(), x, EPSILON);
  EXPECT_NEAR(vec.y(), y, EPSILON);
  EXPECT_NEAR(vec.z(), z, EPSILON);
}

inline void checkQuaternion(
  const affordance_primitives::Quaternion & quat, double x, double y, double z, double w)
{
  EXPECT_NEAR(quat.x, x, EPSILON);
  EXPECT_NEAR(quat.y, y, EPSILON);
  EXPECT_NEAR(quat.z, z, EPSILON);
  EXPECT_NEAR(quat.w, w, EPSILON);
}

/*
    TEST CASE:

    Screw axis at the z-axis of "task_frame" (origin = [0,0,0], axis = [0,0,1])
    Transform from "moving_frame" to "task_frame" is:
      Translation [2,0,0]
      Rotate 0.5*pi about X-axis

    Pure Rotation results in:
      [0, 0, -2*delta_theta] translational velocity
      [0, -1*delta_theta, 0] rotational velocity

    Screw Motion (with pitch) results in:
      [0, -1*pitch*delta_theta, -2*delta_theta] translational velocity
      [0, -1*delta_theta, 0] rotational velocity

    Pure Translation results in:
      [0, -1*delta_theta, 0] translational velocity
      [0, 0, 0] rotational velocity
*/

TEST(ScrewExecution, providedTF)
{
  auto node = std::make_shared<rclcpp::Node>("test_screw_executor");
  affordance_primitives::APScrewExecutor exec(node);
  affordance_primitives::AffordancePrimitiveGoal ap_goal;
  affordance_primitives::AffordancePrimitiveFeedback ap_feedback;

  // Check for bogus input case
  ap_goal.moving_frame_source = -1;
  bool bool_result = true;
  ASSERT_NO_THROW(bool_result = exec.setStreamingCommands(ap_goal, ap_feedback));
  EXPECT_FALSE(bool_result);

  // Set up Screw msg
  ap_goal.moving_frame_source = ap_goal.PROVIDED;
  affordance_primitives::ScrewStamped screw_msg;
  screw_msg.header.frame_id = TASK_FRAME_NAME;
  screw_msg.axis.z = 1;

  // Check case where the transform is not (moving_frame -> task_frame)
  affordance_primitives::TransformStamped tf_msg;
  tf_msg.header.frame_id = MOVING_FRAME_NAME;
  tf_msg.child_frame_id = "some_random_frame";
  ap_goal.moving_to_task_frame = tf_msg;
  ap_goal.screw = screw_msg;
  bool_result = true;
  ASSERT_NO_THROW(bool_result = exec.setStreamingCommands(ap_goal, ap_feedback));
  EXPECT_FALSE(bool_result);

  // Set up transform msg
  tf_msg.child_frame_id = TASK_FRAME_NAME;
  tf_msg.transform.translation.x = 2;
  tf_msg.transform.rotation.x = 0.5 * sqrt(2);
  tf_msg.transform.rotation.w = 0.5 * sqrt(2);

  // Check valid rotation case
  ap_goal.moving_to_task_frame = tf_msg;
  ap_goal.theta_dot = 0.75;
  ap_goal.screw.is_pure_translation = false;
  ap_goal.screw.pitch = 0;
  bool_result = false;
  ASSERT_NO_THROW(bool_result = exec.setStreamingCommands(ap_goal, ap_feedback));
  EXPECT_TRUE(bool_result);
  checkVector(ap_feedback.moving_frame_twist.twist.linear, 0, 0, -2 * ap_goal.theta_dot);
  checkVector(ap_feedback.moving_frame_twist.twist.angular, 0, -1 * ap_goal.theta_dot, 0);

  // Check transform matches what we sent
  EXPECT_EQ(ap_feedback.tf_moving_to_task.header.frame_id, MOVING_FRAME_NAME);
  EXPECT_EQ(ap_feedback.tf_moving_to_task.child_frame_id, TASK_FRAME_NAME);
  checkVector(ap_feedback.tf_moving_to_task.transform.translation, 2, 0, 0);
  checkQuaternion(
    ap_feedback.tf_moving_to_task.transform.rotation, 0.5 * sqrt(2), 0, 0, 0.5 * sqrt(2));

  // Check valid screw motion case
  ap_goal.screw.pitch = 1.5;
  bool_result = false;
  ASSERT_NO_THROW(bool_result = exec.setStreamingCommands(ap_goal, ap_feedback));
  EXPECT_TRUE(bool_result);
  checkVector(
    ap_feedback.moving_frame_twist.twist.linear, 0, -1 * ap_goal.screw.pitch * ap_goal.theta_dot,
    -2 * ap_goal.theta_dot);
  checkVector(ap_feedback.moving_frame_twist.twist.angular, 0, -1 * ap_goal.theta_dot, 0);

  // Check valid translation case
  ap_goal.screw.is_pure_translation = true;
  bool_result = false;
  ASSERT_NO_THROW(bool_result = exec.setStreamingCommands(ap_goal, ap_feedback));
  EXPECT_TRUE(bool_result);
  checkVector(ap_feedback.moving_frame_twist.twist.linear, 0, -1 * ap_goal.theta_dot, 0);
  checkVector(ap_feedback.moving_frame_twist.twist.angular, 0, 0, 0);
}

TEST(ScrewExecution, lookupTF)
{
  auto node = std::make_shared<rclcpp::Node>("test_screw_executor");
  affordance_primitives::APScrewExecutor exec(node);
  affordance_primitives::AffordancePrimitiveGoal ap_goal;
  affordance_primitives::AffordancePrimitiveFeedback ap_feedback;

  // Set up Screw msg
  ap_goal.moving_frame_source = ap_goal.LOOKUP;
  affordance_primitives::ScrewStamped screw_msg;
  screw_msg.header.frame_id = TASK_FRAME_NAME;
  screw_msg.axis.z = 1;

  // If we look up a bogus frame, we expect no exception but a false return
  ap_goal.moving_frame_name = "some_bogus_frame";
  bool bool_result = true;
  ASSERT_NO_THROW(bool_result = exec.setStreamingCommands(ap_goal, ap_feedback));
  EXPECT_FALSE(bool_result);

  // If we lookup the correct frame, we should get good results
  // Check valid rotation case
  ap_goal.moving_frame_name = MOVING_FRAME_NAME;
  ap_goal.screw = screw_msg;
  ap_goal.theta_dot = 0.75;
  ap_goal.screw.is_pure_translation = false;
  ap_goal.screw.pitch = 0;
  bool_result = false;
  ASSERT_NO_THROW(bool_result = exec.setStreamingCommands(ap_goal, ap_feedback));
  EXPECT_TRUE(bool_result);
  checkVector(ap_feedback.moving_frame_twist.twist.linear, 0, 0, -2 * ap_goal.theta_dot);
  checkVector(ap_feedback.moving_frame_twist.twist.angular, 0, -1 * ap_goal.theta_dot, 0);

  // Check the lookup worked and returned correctly
  EXPECT_EQ(ap_feedback.tf_moving_to_task.header.frame_id, MOVING_FRAME_NAME);
  EXPECT_EQ(ap_feedback.tf_moving_to_task.child_frame_id, TASK_FRAME_NAME);
  checkVector(ap_feedback.tf_moving_to_task.transform.translation, 2, 0, 0);
  checkQuaternion(
    ap_feedback.tf_moving_to_task.transform.rotation, 0.5 * sqrt(2), 0, 0, 0.5 * sqrt(2));
}

TEST(ScrewExecution, helperFunctions)
{
  // Set up Screw msg
  affordance_primitives::ScrewStamped screw_msg;
  screw_msg.header.frame_id = TASK_FRAME_NAME;
  screw_msg.axis.z = 1;

  // Set up test case TF
  Eigen::Isometry3d tf_moving_to_task;
  tf_moving_to_task.translation() = Eigen::Vector3d(2, 0, 0);
  tf_moving_to_task.linear() =
    Eigen::Quaterniond(sqrt(2) / 2, sqrt(2) / 2, 0, 0).toRotationMatrix();

  // Check for affordance twist
  const double delta_theta = 1.0;
  Eigen::Matrix<double, 6, 1> affordance_twist;
  ASSERT_NO_THROW(
    affordance_twist = affordance_primitives::calculateAffordanceTwist(screw_msg, delta_theta));

  // This is actually really easy.. we expect rotation about 0 with no translation
  checkVector(affordance_twist.head(3), 0, 0, 0);
  checkVector(affordance_twist.tail(3), 0, 0, 1);

  // Now check the affordance wrench
  const double impedance_translation = 100;
  const double impedance_rotation = 5;
  Eigen::Matrix<double, 6, 1> affordance_wrench;
  ASSERT_NO_THROW(
    affordance_wrench = affordance_primitives::calculateAffordanceWrench(
      screw_msg, affordance_twist, impedance_translation, impedance_rotation));

  // Expect no force, and some torque
  checkVector(affordance_wrench.head(3), 0, 0, 0);
  checkVector(affordance_wrench.tail(3), 0, 0, impedance_rotation);

  // Finally, we check calculated the wrench we must apply at the moving frame
  affordance_primitives::APRobotParameter limits;
  Eigen::Matrix<double, 6, 1> applied_wrench;
  ASSERT_NO_THROW(
    applied_wrench = affordance_primitives::calculateAppliedWrench(
      affordance_wrench, tf_moving_to_task, screw_msg, limits));

  // Expect forces and torques to apply this wrench
  checkVector(applied_wrench.head(3), 0, 0, -2 * impedance_rotation);
  checkVector(applied_wrench.tail(3), 0, -1 * impedance_rotation, 0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  return result;
}
