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

#include <affordance_primitives/screw_motion.hpp>

const std::string MOVING_FRAME_NAME = "moving_frame_name";
constexpr double EPSILON = 1e-4;

void checkVector(const Eigen::Vector3d& vec, const double x, const double y, const double z)
{
  EXPECT_NEAR(vec.x(), x, EPSILON);
  EXPECT_NEAR(vec.y(), y, EPSILON);
  EXPECT_NEAR(vec.z(), z, EPSILON);
}
void checkVector(const geometry_msgs::msg::Vector3& vec, const double x, const double y, const double z)
{
  EXPECT_NEAR(vec.x, x, EPSILON);
  EXPECT_NEAR(vec.y, y, EPSILON);
  EXPECT_NEAR(vec.z, z, EPSILON);
}

TEST(ScrewAxis, test_constructor)
{
  affordance_primitives::ScrewAxis screw_axis(MOVING_FRAME_NAME);
  EXPECT_EQ(screw_axis.getFrame(), MOVING_FRAME_NAME);
  EXPECT_TRUE(screw_axis.getQVector().isZero());
  EXPECT_TRUE(screw_axis.getAxis().isZero());
  EXPECT_TRUE(screw_axis.getLinearVector().isZero());
}

TEST(ScrewAxis, calculate_linear_component)
{
  Eigen::Vector3d z_axis(0, 0, 1);
  Eigen::Vector3d q_vector(-1, 0, 0);
  double pitch = 0;

  Eigen::Vector3d linear_velocity = affordance_primitives::calculateLinearVelocity(z_axis, q_vector, pitch);

  // This should result in positive y velocity only
  checkVector(linear_velocity, 0.0, 1.0, 0.0);

  // Changing the q-vector should result in nonzero X-Y velocity
  q_vector.x() = -1 * sqrt(2) / 2;
  q_vector.y() = sqrt(2) / 2;
  linear_velocity = affordance_primitives::calculateLinearVelocity(z_axis, q_vector, pitch);
  EXPECT_NEAR(linear_velocity.y(), linear_velocity.x(), EPSILON);
  double norm = sqrt(pow(linear_velocity.y(), 2) + pow(linear_velocity.x(), 2));
  EXPECT_NEAR(norm, 1.0, EPSILON);

  // Changing pitch should give us z velocity
  pitch = 1;
  linear_velocity = affordance_primitives::calculateLinearVelocity(z_axis, q_vector, pitch);
  EXPECT_NEAR(linear_velocity.z(), 1.0, EPSILON);
}

TEST(ScrewAxis, two_pose_setter_rotation)
{
  affordance_primitives::ScrewAxis screw_axis(MOVING_FRAME_NAME, false);

  geometry_msgs::msg::Pose first_pose, second_pose;

  // Expect a false return if the poses are on top of each other (axis length 0)
  EXPECT_FALSE(screw_axis.setScrewAxis(first_pose, second_pose));

  // Origin at [1,0,0], axis of [0,0,1]
  first_pose.position.x = 1.0;
  second_pose.position.x = 1.0;
  second_pose.position.z = 2.0;

  ASSERT_TRUE(screw_axis.setScrewAxis(first_pose, second_pose));
  checkVector(screw_axis.getQVector(), 1.0, 0.0, 0.0);
  checkVector(screw_axis.getAxis(), 0.0, 0.0, 1.0);
  checkVector(screw_axis.getLinearVector(), 0.0, -1.0, 0.0);
  EXPECT_NEAR(screw_axis.getPitch(), 0.0, EPSILON);

  // Try a pitch
  ASSERT_TRUE(screw_axis.setScrewAxis(first_pose, second_pose, 0.5));
  checkVector(screw_axis.getLinearVector(), 0.0, -1.0, 0.5);
  EXPECT_NEAR(screw_axis.getPitch(), 0.5, EPSILON);

  // Check the function with Eigen::Isometry
  Eigen::Isometry3d eigen_first_pose, eigen_second_pose;
  Eigen::fromMsg(first_pose, eigen_first_pose);
  Eigen::fromMsg(second_pose, eigen_second_pose);

  ASSERT_TRUE(screw_axis.setScrewAxis(first_pose, second_pose));
  checkVector(screw_axis.getQVector(), 1.0, 0.0, 0.0);
  checkVector(screw_axis.getAxis(), 0.0, 0.0, 1.0);
  checkVector(screw_axis.getLinearVector(), 0.0, -1.0, 0.0);
  EXPECT_NEAR(screw_axis.getPitch(), 0.0, EPSILON);
}

TEST(ScrewAxis, two_pose_setter_linear)
{
  affordance_primitives::ScrewAxis screw_axis(MOVING_FRAME_NAME, true);

  // Origin at [1,0,0], axis of [0,0,1]
  geometry_msgs::msg::Pose first_pose, second_pose;
  first_pose.position.x = 1.0;
  second_pose.position.x = 1.0;
  second_pose.position.z = 2.0;

  ASSERT_TRUE(screw_axis.setScrewAxis(first_pose, second_pose));
  checkVector(screw_axis.getQVector(), 1.0, 0.0, 0.0);
  checkVector(screw_axis.getLinearVector(), 0.0, 0.0, 1.0);
  EXPECT_TRUE(screw_axis.getAxis().isZero());
}

TEST(ScrewAxis, pose_axis_setter_rotation)
{
  affordance_primitives::ScrewAxis screw_axis(MOVING_FRAME_NAME, false);

  // Origin at [1,0,0], axis of [0,0,1]
  geometry_msgs::msg::Pose first_pose;
  first_pose.position.x = 1.0;
  Eigen::Vector3d z_axis(0, 0, 2);

  ASSERT_TRUE(screw_axis.setScrewAxis(first_pose, z_axis));
  checkVector(screw_axis.getQVector(), 1.0, 0.0, 0.0);
  checkVector(screw_axis.getAxis(), 0.0, 0.0, 1.0);
  checkVector(screw_axis.getLinearVector(), 0.0, -1.0, 0.0);
  EXPECT_NEAR(screw_axis.getPitch(), 0.0, EPSILON);

  // Try a pitch
  ASSERT_TRUE(screw_axis.setScrewAxis(first_pose, z_axis, 0.5));
  checkVector(screw_axis.getLinearVector(), 0.0, -1.0, 0.5);
  EXPECT_NEAR(screw_axis.getPitch(), 0.5, EPSILON);

  // Check the function with Eigen::Isometry
  Eigen::Isometry3d eigen_first_pose;
  Eigen::fromMsg(first_pose, eigen_first_pose);

  ASSERT_TRUE(screw_axis.setScrewAxis(first_pose, z_axis));
  checkVector(screw_axis.getQVector(), 1.0, 0.0, 0.0);
  checkVector(screw_axis.getAxis(), 0.0, 0.0, 1.0);
  checkVector(screw_axis.getLinearVector(), 0.0, -1.0, 0.0);
  EXPECT_NEAR(screw_axis.getPitch(), 0.0, EPSILON);

  // With a zero-vector passed, we need it to fail
  Eigen::Vector3d zero_axis(0, 0, 0);
  EXPECT_FALSE(screw_axis.setScrewAxis(first_pose, zero_axis));
}

TEST(ScrewAxis, pose_axis_setter_linear)
{
  affordance_primitives::ScrewAxis screw_axis(MOVING_FRAME_NAME, true);

  // Origin at [1,0,0], axis of [0,0,1]
  geometry_msgs::msg::Pose first_pose;
  first_pose.position.x = 1.0;
  Eigen::Vector3d wonky_axis(2, 0, 2);
  first_pose.position.x = 1.0;

  ASSERT_TRUE(screw_axis.setScrewAxis(first_pose, wonky_axis));
  checkVector(screw_axis.getQVector(), 1.0, 0.0, 0.0);
  checkVector(screw_axis.getLinearVector(), 0.5 * sqrt(2), 0.0, 0.5 * sqrt(2));
  EXPECT_TRUE(screw_axis.getAxis().isZero());

  // Setting the pitch doesn't matter internally, but shouldn't cause problems
  ASSERT_TRUE(screw_axis.setScrewAxis(first_pose, wonky_axis, 42));
}

TEST(ScrewAxis, get_twist)
{
  affordance_primitives::ScrewAxis screw_axis(MOVING_FRAME_NAME);

  // Origin at [1,0,0], axis of [1,1,0]
  geometry_msgs::msg::Pose first_pose;
  first_pose.position.x = 1.0;
  Eigen::Vector3d wonky_axis(1, 1, 0);

  // The (instantaneous) velocity here is just Z axis
  ASSERT_TRUE(screw_axis.setScrewAxis(first_pose, wonky_axis));
  auto twist_msg = screw_axis.getTwist(0.5);
  checkVector(twist_msg.twist.linear, 0.0, 0.0, 0.25 * sqrt(2));  // 0.5 vel * sqrt(2)/2 distance

  // The rotation is crazy, but is Roll and Pitch: 0.5 vel * sqrt(2)/2 distance
  checkVector(twist_msg.twist.angular, 0.25 * sqrt(2), 0.25 * sqrt(2), 0.0);

  // Now check linear only motion
  affordance_primitives::ScrewAxis screw_axis_linear(MOVING_FRAME_NAME, true);
  ASSERT_TRUE(screw_axis_linear.setScrewAxis(first_pose, wonky_axis));
  twist_msg = screw_axis_linear.getTwist(0.5);

  // Should be along axis with 0.5 vel * norm([1,1,0]) velocity in X-Y
  checkVector(twist_msg.twist.linear, 0.25 * sqrt(2), 0.25 * sqrt(2), 0.0);

  // No rotation expected
  checkVector(twist_msg.twist.angular, 0.0, 0.0, 0.0);
}

TEST(ScrewAxis, get_transform)
{
  affordance_primitives::ScrewAxis screw_axis(MOVING_FRAME_NAME);

  // Origin at [1,0,0], axis of [0, 0, 1]
  geometry_msgs::msg::Pose first_pose;
  first_pose.position.x = 1.0;
  Eigen::Vector3d z_axis(0, 0, 1);

  // Will test a 90-degree rotation
  ASSERT_TRUE(screw_axis.setScrewAxis(first_pose, z_axis));
  auto transform = screw_axis.getTF(M_PI / 2);
  checkVector(transform.translation(), 1.0, -1.0, 0.0);  // With respect to starting position

  // Directly check the rotation matrix
  checkVector(transform.linear().col(0), 0, 1, 0);   // New x axis faces old y axis
  checkVector(transform.linear().col(1), -1, 0, 0);  // New y axis faces old -x axis
  checkVector(transform.linear().col(2), 0, 0, 1);   // z axis did not change

  // Now check linear only motion
  affordance_primitives::ScrewAxis screw_axis_linear(MOVING_FRAME_NAME, true);
  ASSERT_TRUE(screw_axis_linear.setScrewAxis(first_pose, z_axis));
  transform = screw_axis_linear.getTF(M_PI / 2);

  // No rotation expected
  EXPECT_TRUE((transform.linear() - Eigen::Matrix3d::Identity()).isZero());

  // Linear motion along z-axis by amount moved
  checkVector(transform.translation(), 0.0, 0.0, M_PI / 2);  // With respect to starting position
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
