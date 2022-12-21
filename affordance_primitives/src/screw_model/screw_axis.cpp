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

/*
   Author: Adam Pettinger
   Desc: Defines a library used to calculate screw motion affordance parameters
 */

#include <affordance_primitives/screw_model/screw_axis.hpp>

namespace affordance_primitives
{
Eigen::Vector3d calculateLinearVelocity(
  const Eigen::Vector3d & axis, const Eigen::Vector3d & q_vector, double pitch)
{
  Eigen::Vector3d pitch_component = pitch * axis;
  Eigen::Vector3d rotation_component = axis.cross(q_vector);
  return pitch_component - rotation_component;
}

ScrewAxis::ScrewAxis()
{
  axis_.setZero();
  origin_.setZero();
  translation_component_.setZero();
}

ScrewAxis::ScrewAxis(const std::string moving_frame_name, bool is_pure_translation)
: moving_frame_name_(moving_frame_name), is_pure_translation_(is_pure_translation)
{
  axis_.setZero();
  origin_.setZero();
  translation_component_.setZero();
}

bool ScrewAxis::setScrewAxis(const Pose & origin_pose, const Pose & axis_pose, double pitch)
{
  // Set the pitch and origin
  pitch_ = pitch;
  tf2::fromMsg(origin_pose.position, origin_);

  // Calculate the axis as the line between the input poses
  Eigen::Vector3d axis_pose_position;
  tf2::fromMsg(axis_pose.position, axis_pose_position);

  Eigen::Vector3d calculated_axis = axis_pose_position - origin_;
  if (calculated_axis.isZero()) {
    return false;
  }

  if (is_pure_translation_) {
    translation_component_ = calculated_axis.normalized();
  } else {
    axis_ = calculated_axis.normalized();
    translation_component_ = calculateLinearVelocity(axis_, origin_, pitch_);
  }
  return true;
}

bool ScrewAxis::setScrewAxis(const Pose & origin_pose, const Eigen::Vector3d & axis, double pitch)
{
  // Set all components
  pitch_ = pitch;
  tf2::fromMsg(origin_pose.position, origin_);

  if (axis.isZero()) {
    return false;
  }

  if (is_pure_translation_) {
    translation_component_ = axis.normalized();
  } else {
    axis_ = axis.normalized();
    translation_component_ = calculateLinearVelocity(axis_, origin_, pitch_);
  }
  return true;
}

bool ScrewAxis::setScrewAxis(const ScrewStamped & screw_msg)
{
  // Override moving frame and translation information
  moving_frame_name_ = screw_msg.header.frame_id;
  is_pure_translation_ = screw_msg.is_pure_translation;

  // Convert message types
  Pose origin_pose;
  origin_pose.position = screw_msg.origin;
  Eigen::Vector3d eigen_axis;
  tf2::fromMsg(screw_msg.axis, eigen_axis);

  // Set up screw axis using origin-axis formulation
  return setScrewAxis(origin_pose, eigen_axis, screw_msg.pitch);
}

TwistStamped ScrewAxis::getTwist(double theta_dot) const
{
  TwistStamped output;
  output.header.frame_id = moving_frame_name_;

  // All we need is to multiply the theta_dot by motion vectors
  if (is_pure_translation_) {
    auto linear_velocity = theta_dot * translation_component_;
    tf2::toMsg(linear_velocity, output.twist.linear);
  } else {
    auto linear_velocity = theta_dot * translation_component_;
    auto angular_velocity = theta_dot * axis_;
    tf2::toMsg(linear_velocity, output.twist.linear);
    tf2::toMsg(angular_velocity, output.twist.angular);
  }
  return output;
}

Eigen::Isometry3d ScrewAxis::getTF(double delta_theta) const
{
  Eigen::Isometry3d output;

  // The translation case is easy:
  if (is_pure_translation_) {
    // No rotation and move linearly by displacement
    output.linear() = Eigen::Matrix3d::Identity();
    output.translation() = delta_theta * translation_component_;
    return output;
  }

  // For rotational case, use Chasles-Mozzi Theorem to calculate the transformation
  // This is essentially exp([S]*theta), where
  // 'exp' is the matrix exponential and
  // [S] is the skew-symmetric (4x4) representation of the Screw Axis
  auto skew_axis = getSkewSymmetricMatrix(axis_);
  Eigen::Matrix3d skew_axis_squared = skew_axis * skew_axis;

  output.translation() =
    ((Eigen::Matrix3d::Identity() * delta_theta) + (1 - cos(delta_theta)) * skew_axis +
     (delta_theta - sin(delta_theta)) * skew_axis_squared) *
    translation_component_;

  // Just use Eigen's Axis-Angle to Rotation Matrix calculations for the rotation component
  Eigen::AngleAxisd axis_angle_rotation(delta_theta, axis_);
  output.linear() = Eigen::Matrix3d(axis_angle_rotation);

  return output;
}

std::vector<Eigen::Isometry3d> ScrewAxis::getWaypoints(double theta_step, size_t num_steps)
{
  std::vector<Eigen::Isometry3d> output;

  // Check for invalid inputs
  if (num_steps < 1 || fabs(theta_step) < 1e-8) {
    return output;
  }

  // Iterate over number of steps to generate waypoints
  output.reserve(num_steps + 1);
  for (size_t i = 0; i < num_steps + 1; i++) {
    output.push_back(getTF(i * theta_step));
  }
  return output;
}

Eigen::Matrix4d getScrewSkewSymmetricMatrix(const Eigen::Matrix<double,6,1> & vec)
{
  Eigen::Matrix4d output(4,4);
  output.block(0,0,3,3) = getSkewSymmetricMatrix(vec.head(3));
  output.block(0,3,3,1) = vec.tail(3);
  output.block(3,0,1,4) = Eigen::MatrixXd::Zero(1,4);

  return output;
}

ScrewStamped ScrewAxis::toMsg()
{
  ScrewStamped output;
  output.header.frame_id = moving_frame_name_;
  output.is_pure_translation = is_pure_translation_;
  output.pitch = pitch_;
  output.origin = tf2::toMsg(origin_);
  tf2::toMsg(axis_, output.axis);

  return output;
}
}  // namespace affordance_primitives
