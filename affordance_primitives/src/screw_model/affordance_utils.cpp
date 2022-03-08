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
   Desc: Utility functions for working with affordance templates and primitives
 */

#include <affordance_primitives/screw_model/affordance_utils.hpp>

namespace affordance_primitives
{
PoseStamped getPoseInATFrame(const std::string& root_frame_name, const PoseStamped& root_frame_pose,
                             const PoseStamped& incoming_frame)
{
  // If the frame is already in the AT root, just return it
  if (incoming_frame.header.frame_id == root_frame_name)
  {
    return incoming_frame;
  }

  // Otherwise, try to convert it to the AT root frame
  Eigen::Isometry3d tf_AT_root_to_pose = convertPoseToNewFrame(root_frame_pose, incoming_frame);

  PoseStamped output;
  output.header.frame_id = root_frame_name;
  output.pose = tf2::toMsg(tf_AT_root_to_pose);
  return output;
}

TwistStamped getTwistFromPoses(const PoseStamped& start_pose, const PoseStamped& end_pose)
{
  // Can't do it if they are in different frames
  if (start_pose.header.frame_id != end_pose.header.frame_id)
  {
    throw std::runtime_error("Poses must be in same frame");
  }

  // Get TF start -> end
  Eigen::Isometry3d eigen_start_pose, eigen_end_pose;
  Eigen::fromMsg(start_pose.pose, eigen_start_pose);
  Eigen::fromMsg(end_pose.pose, eigen_end_pose);

  // (start -> header_frame) * (header_frame -> end)
  Eigen::Isometry3d diff_pose = eigen_start_pose.inverse() * eigen_end_pose;

  // Use Axis Angle representation to get the Twist rotational values
  Eigen::AngleAxisd axis_angle(diff_pose.linear());

  // Populate the output
  TwistStamped output;
  output.header.frame_id = start_pose.header.frame_id;
  tf2::toMsg(diff_pose.translation(), output.twist.linear);
  tf2::toMsg(axis_angle.angle() * axis_angle.axis(), output.twist.angular);

  return output;
}

Eigen::Isometry3d convertPoseToNewFrame(const PoseStamped& new_base_frame, const PoseStamped& transformed_pose)
{
  if (new_base_frame.header.frame_id != transformed_pose.header.frame_id)
  {
    throw std::runtime_error("Poses must be in same frame");
  }

  // Convert to Eigen
  Eigen::Isometry3d tf_root_to_new_base_frame, tf_root_to_transformed_pose;
  tf2::fromMsg(new_base_frame.pose, tf_root_to_new_base_frame);
  tf2::fromMsg(transformed_pose.pose, tf_root_to_transformed_pose);

  // Do (new -> root) * (root -> input) = (new -> input)
  return tf_root_to_new_base_frame.inverse() * tf_root_to_transformed_pose;
}

ScrewStamped transformScrew(const ScrewStamped& input_screw, const TransformStamped& transform)
{
  if (input_screw.header.frame_id == transform.child_frame_id)
  {
    // Already in the correct frame
    return input_screw;
  }
  else if (input_screw.header.frame_id != transform.header.frame_id)
  {
    throw std::runtime_error("Cannot transform screw: transform frame does not match incoming screw frame");
  }

  ScrewStamped output;
  output.header.frame_id = transform.child_frame_id;

  // Convert to Eigen types
  auto tf_new_to_old = (tf2::transformToEigen(transform.transform)).inverse();
  Eigen::Vector3d screw_axis, screw_origin;
  tf2::fromMsg(input_screw.axis, screw_axis);
  tf2::fromMsg(input_screw.origin, screw_origin);

  // Perform the transform
  Eigen::Vector3d rotated_axis = tf_new_to_old.linear() * screw_axis;
  Eigen::Vector3d transformed_origin = tf_new_to_old.translation() + tf_new_to_old.linear() * screw_origin;

  // Convert back from Eigen types
  output.origin = tf2::toMsg(transformed_origin);
  tf2::toMsg(rotated_axis, output.axis);
  return output;
}

Eigen::Matrix3d getSkewSymmetricMatrix(const Eigen::Vector3d& vec)
{
  Eigen::Matrix3d output;
  output.setZero();
  output(0, 1) = -vec(2);
  output(1, 0) = vec(2);
  output(0, 2) = vec(1);
  output(2, 0) = -vec(1);
  output(1, 2) = -vec(0);
  output(2, 1) = vec(0);

  return output;
}

Eigen::MatrixXd getAdjointMatrix(const Eigen::Isometry3d& transform)
{
  Eigen::MatrixXd adjoint(6, 6);

  // Block matrix with
  // |R    [p]R|
  // |0      R |
  adjoint.block<3, 3>(0, 0) = transform.linear();
  adjoint.block<3, 3>(3, 0).setZero();
  adjoint.block<3, 3>(0, 3) = getSkewSymmetricMatrix(transform.translation()) * transform.linear();
  adjoint.block<3, 3>(3, 3) = transform.linear();

  return adjoint;
}

Eigen::MatrixXd getAdjointMatrix(const Transform& transform)
{
  return getAdjointMatrix(tf2::transformToEigen(transform));
}

std::string twistToStr(const TwistStamped& twist)
{
  std::stringstream stream;
  stream << "\nHeader: " << twist.header.frame_id << "\nX: " << twist.twist.linear.x << "\nY: " << twist.twist.linear.y
         << "\nZ: " << twist.twist.linear.z << "\nRoll: " << twist.twist.angular.x
         << "\nPitch: " << twist.twist.angular.y << "\nYaw: " << twist.twist.angular.z;
  return stream.str();
}

std::string poseToStr(const PoseStamped& pose)
{
  std::stringstream stream;
  stream << "\nHeader: " << pose.header.frame_id << "\nX: " << pose.pose.position.x << "\nY: " << pose.pose.position.y
         << "\nZ: " << pose.pose.position.z << "\nQX: " << pose.pose.orientation.x
         << "\nQY: " << pose.pose.orientation.y << "\nQZ: " << pose.pose.orientation.z
         << "\nQW: " << pose.pose.orientation.w;
  return stream.str();
}

std::string screwMsgToStr(const ScrewStamped& screw)
{
  std::string pitch;
  if (screw.is_pure_translation)
    pitch = "Infinity";
  else
    pitch = std::to_string(screw.pitch);

  std::stringstream stream;
  stream << "\nHeader: " << screw.header.frame_id << "\nOrigin X: " << screw.origin.x
         << "\nOrigin Y: " << screw.origin.y << "\nOrigin Z: " << screw.origin.z << "\nAxis X: " << screw.axis.x
         << "\nAxis Y: " << screw.axis.y << "\nAxis Z: " << screw.axis.z << "\nPitch: " << pitch;
  return stream.str();
}
}  // namespace affordance_primitives
