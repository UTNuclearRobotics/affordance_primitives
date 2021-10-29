// Copyright 2021 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <affordance_primitives/affordance_utils.hpp>

const std::string ROOT_FRAME_NAME = "AT_frame_name";
const std::string PLANNING_FRAME_NAME = "world";
constexpr double EPSILON = 1e-4;

inline void checkVector(const geometry_msgs::msg::Vector3& vec, double x, double y, double z)
{
  EXPECT_NEAR(vec.x, x, EPSILON);
  EXPECT_NEAR(vec.y, y, EPSILON);
  EXPECT_NEAR(vec.z, z, EPSILON);
}
inline void checkVector(const Eigen::Vector3d& vec, double x, double y, double z)
{
  EXPECT_NEAR(vec.x(), x, EPSILON);
  EXPECT_NEAR(vec.y(), y, EPSILON);
  EXPECT_NEAR(vec.z(), z, EPSILON);
}
inline void checkPoint(const geometry_msgs::msg::Point& point, double x, double y, double z)
{
  EXPECT_NEAR(point.x, x, EPSILON);
  EXPECT_NEAR(point.y, y, EPSILON);
  EXPECT_NEAR(point.z, z, EPSILON);
}
inline void checkQuaternion(const geometry_msgs::msg::Quaternion& quat, double x, double y, double z, double w)
{
  EXPECT_NEAR(quat.x, x, EPSILON);
  EXPECT_NEAR(quat.y, y, EPSILON);
  EXPECT_NEAR(quat.z, z, EPSILON);
  EXPECT_NEAR(quat.w, w, EPSILON);
}
inline void checkQuaternion(const Eigen::Quaterniond& quat, double x, double y, double z, double w)
{
  EXPECT_NEAR(quat.x(), x, EPSILON);
  EXPECT_NEAR(quat.y(), y, EPSILON);
  EXPECT_NEAR(quat.z(), z, EPSILON);
  EXPECT_NEAR(quat.w(), w, EPSILON);
}

TEST(AffordanceUtils, getPoseInATFrame)
{
  geometry_msgs::msg::PoseStamped AT_base_pose, other_pose, result;

  AT_base_pose.header.frame_id = PLANNING_FRAME_NAME;
  AT_base_pose.pose.position.x = 1;
  AT_base_pose.pose.position.y = 1;
  AT_base_pose.pose.position.z = 1;

  // Orientation is a 90-degree roll about the X-axis
  other_pose.pose.orientation.x = 0.5 * sqrt(2);
  other_pose.pose.orientation.y = 0;
  other_pose.pose.orientation.z = 0;
  other_pose.pose.orientation.w = 0.5 * sqrt(2);

  // First, if the other_pose is in the AT frame already, it should return as-is
  other_pose.header.frame_id = ROOT_FRAME_NAME;
  ASSERT_NO_THROW(result = affordance_primitives::getPoseInATFrame(ROOT_FRAME_NAME, AT_base_pose, other_pose));
  checkPoint(result.pose.position, 0, 0, 0);
  checkQuaternion(result.pose.orientation, 0.5 * sqrt(2), 0, 0, 0.5 * sqrt(2));

  // Second, if the other_pose is also in the planning frame, a conversion should happen
  other_pose.header.frame_id = PLANNING_FRAME_NAME;
  ASSERT_NO_THROW(result = affordance_primitives::getPoseInATFrame(ROOT_FRAME_NAME, AT_base_pose, other_pose));

  // The result should be AT frame -> input pose
  checkPoint(result.pose.position, -1, -1, -1);
  checkQuaternion(result.pose.orientation, 0.5 * sqrt(2), 0, 0, 0.5 * sqrt(2));

  // Finally, it should throw if it cannot convert the pose
  other_pose.header.frame_id = "some_other_frame";
  EXPECT_THROW(result = affordance_primitives::getPoseInATFrame(ROOT_FRAME_NAME, AT_base_pose, other_pose),
               std::runtime_error);
}

TEST(AffordanceUtils, getTwistFromPoses)
{
  geometry_msgs::msg::PoseStamped start_pose, end_pose;

  start_pose.header.frame_id = ROOT_FRAME_NAME;
  end_pose.header.frame_id = ROOT_FRAME_NAME;

  end_pose.pose.position.x = 1;
  end_pose.pose.position.y = 1;
  end_pose.pose.position.z = 0;

  // Orientation is a 90-degree yaw about the Z-axis
  end_pose.pose.orientation.x = 0;
  end_pose.pose.orientation.y = 0;
  end_pose.pose.orientation.z = 0.5 * sqrt(2);
  end_pose.pose.orientation.w = 0.5 * sqrt(2);

  // Check the calculated twist
  geometry_msgs::msg::TwistStamped result;
  ASSERT_NO_THROW(result = affordance_primitives::getTwistFromPoses(start_pose, end_pose));
  EXPECT_EQ(result.header.frame_id, ROOT_FRAME_NAME);
  checkVector(result.twist.linear, 1, 1, 0);
  checkVector(result.twist.angular, 0, 0, 0.5 * M_PI);

  // Finally, it should throw if the poses have different headers
  end_pose.header.frame_id = "some_other_frame";
  EXPECT_THROW(result = affordance_primitives::getTwistFromPoses(start_pose, end_pose), std::runtime_error);
}

TEST(AffordanceUtils, convertPoseToNewFrame)
{
  geometry_msgs::msg::PoseStamped grasp_pose, screw_origin_pose;
  Eigen::Isometry3d result;

  // First make sure we throw if the frames do not match
  grasp_pose.header.frame_id = ROOT_FRAME_NAME;
  screw_origin_pose.header.frame_id = PLANNING_FRAME_NAME;
  EXPECT_THROW(result = affordance_primitives::convertPoseToNewFrame(grasp_pose, screw_origin_pose), std::runtime_error);

  // If the new base is identity, we should get back what we put in
  screw_origin_pose.header.frame_id = ROOT_FRAME_NAME;
  screw_origin_pose.pose.position.x = 1;
  screw_origin_pose.pose.position.y = 1;
  screw_origin_pose.pose.position.z = 0;

  // Orientation is a 90-degree yaw about the Z-axis
  screw_origin_pose.pose.orientation.x = 0;
  screw_origin_pose.pose.orientation.y = 0;
  screw_origin_pose.pose.orientation.z = 0.5 * sqrt(2);
  screw_origin_pose.pose.orientation.w = 0.5 * sqrt(2);

  // Check the result
  ASSERT_NO_THROW(result = affordance_primitives::convertPoseToNewFrame(grasp_pose, screw_origin_pose));
  checkVector(result.translation(), 1, 1, 0);
  checkQuaternion(Eigen::Quaterniond(result.linear()), 0, 0, 0.5 * sqrt(2), 0.5 * sqrt(2));

  // Now check harder case
  grasp_pose.pose.position.x = 1;
  grasp_pose.pose.position.y = 0;
  grasp_pose.pose.position.z = 0;

  // Check the result
  ASSERT_NO_THROW(result = affordance_primitives::convertPoseToNewFrame(grasp_pose, screw_origin_pose));
  checkVector(result.translation(), 0, 1, 0);
  checkQuaternion(Eigen::Quaterniond(result.linear()), 0, 0, 0.5 * sqrt(2), 0.5 * sqrt(2));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
