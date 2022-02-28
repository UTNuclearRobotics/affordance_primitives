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
#include <ros/ros.h>

#include <affordance_primitives/screw_model/affordance_utils.hpp>
#include <affordance_primitives/msg_types.hpp>
#include <affordance_primitives/screw_model/screw_execution.hpp>

const std::string TASK_FRAME_NAME = "task_frame";
const std::string MOVING_FRAME_NAME = "moving_frame";
constexpr double EPSILON = 1e-4;

inline void checkVector(const affordance_primitives::Vector3& vec, double x, double y, double z)
{
  EXPECT_NEAR(vec.x, x, EPSILON);
  EXPECT_NEAR(vec.y, y, EPSILON);
  EXPECT_NEAR(vec.z, z, EPSILON);
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
  affordance_primitives::APScrewExecutor exec;
  affordance_primitives::AffordancePrimitive::Request ap_request;
  affordance_primitives::AffordancePrimitive::Response ap_result;

  // Check for bogus input case
  ap_request.moving_frame_source = -1;
  bool bool_result = true;
  ASSERT_NO_THROW(bool_result = exec.getScrewTwist(ap_request, ap_result));
  EXPECT_FALSE(bool_result);

  // Set up Screw msg
  ap_request.moving_frame_source = ap_request.PROVIDED;
  affordance_primitives::ScrewStamped screw_msg;
  screw_msg.header.frame_id = TASK_FRAME_NAME;
  screw_msg.axis.z = 1;

  // Check case where the transform is not (moving_frame -> task_frame)
  affordance_primitives::TransformStamped tf_msg;
  tf_msg.header.frame_id = MOVING_FRAME_NAME;
  tf_msg.child_frame_id = "some_random_frame";
  ap_request.moving_to_task_frame = tf_msg;
  ap_request.screw = screw_msg;
  bool_result = true;
  ASSERT_NO_THROW(bool_result = exec.getScrewTwist(ap_request, ap_result));
  EXPECT_FALSE(bool_result);

  // Set up transform msg
  tf_msg.child_frame_id = TASK_FRAME_NAME;
  tf_msg.transform.translation.x = 2;
  tf_msg.transform.rotation.x = 0.5 * sqrt(2);
  tf_msg.transform.rotation.w = 0.5 * sqrt(2);

  // Check valid rotation case
  ap_request.moving_to_task_frame = tf_msg;
  ap_request.theta_dot = 0.75;
  ap_request.screw.is_pure_translation = false;
  ap_request.screw.pitch = 0;
  bool_result = false;
  ASSERT_NO_THROW(bool_result = exec.getScrewTwist(ap_request, ap_result));
  EXPECT_TRUE(bool_result);
  checkVector(ap_result.moving_frame_twist.twist.linear, 0, 0, -2 * ap_request.theta_dot);
  checkVector(ap_result.moving_frame_twist.twist.angular, 0, -1 * ap_request.theta_dot, 0);

  // Check valid screw motion case
  ap_request.screw.pitch = 1.5;
  bool_result = false;
  ASSERT_NO_THROW(bool_result = exec.getScrewTwist(ap_request, ap_result));
  EXPECT_TRUE(bool_result);
  checkVector(ap_result.moving_frame_twist.twist.linear, 0, -1 * ap_request.screw.pitch * ap_request.theta_dot,
              -2 * ap_request.theta_dot);
  checkVector(ap_result.moving_frame_twist.twist.angular, 0, -1 * ap_request.theta_dot, 0);

  // Check valid translation case
  ap_request.screw.is_pure_translation = true;
  bool_result = false;
  ASSERT_NO_THROW(bool_result = exec.getScrewTwist(ap_request, ap_result));
  EXPECT_TRUE(bool_result);
  checkVector(ap_result.moving_frame_twist.twist.linear, 0, -1 * ap_request.theta_dot, 0);
  checkVector(ap_result.moving_frame_twist.twist.angular, 0, 0, 0);
}

TEST(ScrewExecution, lookupTF)
{
  affordance_primitives::APScrewExecutor exec;
  affordance_primitives::AffordancePrimitive::Request ap_request;
  affordance_primitives::AffordancePrimitive::Response ap_result;

  // Wait for tf to publish
  ros::NodeHandle nh;
  ros::topic::waitForMessage<tf2_msgs::TFMessage>("/tf", nh);

  // Set up Screw msg
  ap_request.moving_frame_source = ap_request.LOOKUP;
  affordance_primitives::ScrewStamped screw_msg;
  screw_msg.header.frame_id = TASK_FRAME_NAME;
  screw_msg.axis.z = 1;

  // If we look up a bogus frame, we expect no exception but a false return
  ap_request.moving_frame_name = "some_bogus_frame";
  bool bool_result = true;
  ASSERT_NO_THROW(bool_result = exec.getScrewTwist(ap_request, ap_result));
  EXPECT_FALSE(bool_result);

  // If we lookup the correct frame, we should get good results
  // Check valid rotation case
  ap_request.moving_frame_name = MOVING_FRAME_NAME;
  ap_request.screw = screw_msg;
  ap_request.theta_dot = 0.75;
  ap_request.screw.is_pure_translation = false;
  ap_request.screw.pitch = 0;
  bool_result = false;
  ASSERT_NO_THROW(bool_result = exec.getScrewTwist(ap_request, ap_result));
  EXPECT_TRUE(bool_result);
  checkVector(ap_result.moving_frame_twist.twist.linear, 0, 0, -2 * ap_request.theta_dot);
  checkVector(ap_result.moving_frame_twist.twist.angular, 0, -1 * ap_request.theta_dot, 0);
}

TEST(ScrewExecution, ROSServiceInterface)
{
  ros::NodeHandle nh;
  affordance_primitives::APScrewExecutor exec(nh, "test_service");
  affordance_primitives::AffordancePrimitive ap_srv;

  // Set up client
  ros::ServiceClient client = nh.serviceClient<affordance_primitives::AffordancePrimitive>("test_service");

  // Set up service request
  ap_srv.request.moving_frame_source = ap_srv.request.PROVIDED;
  ap_srv.request.moving_to_task_frame.header.frame_id = MOVING_FRAME_NAME;
  ap_srv.request.moving_to_task_frame.child_frame_id = TASK_FRAME_NAME;
  ap_srv.request.moving_to_task_frame.transform.translation.x = 2.0;
  ap_srv.request.moving_to_task_frame.transform.rotation.x = 0.5 * sqrt(2);
  ap_srv.request.moving_to_task_frame.transform.rotation.w = 0.5 * sqrt(2);
  ap_srv.request.screw.header.frame_id = TASK_FRAME_NAME;
  ap_srv.request.screw.axis.z = 1;
  ap_srv.request.screw.is_pure_translation = false;
  ap_srv.request.theta_dot = 0.75;

  // Call service and check response
  ASSERT_TRUE(client.call(ap_srv));
  EXPECT_EQ(ap_srv.response.moving_frame_twist.header.frame_id, MOVING_FRAME_NAME);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_screw_execution");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(8);
  spinner.start();

  int result = RUN_ALL_TESTS();
  return result;
}
