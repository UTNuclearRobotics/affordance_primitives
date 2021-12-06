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
   Desc: Useful conversions for working with screw math
 */

#pragma once

#include <affordance_primitive_msgs/ScrewStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_eigen/tf2_eigen.h>

#include <sstream>

namespace affordance_primitives
{
/**
 * @brief Converts an axis vector to 3x3 skew symmetric matrix, for math
 */
inline Eigen::Matrix3d getSkewSymmetricMatrix(const Eigen::Vector3d& vec)
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

/**
 * @brief Converts a transformation into an adjoint matrix for transforming twists and wrenches
 *
 * Usage: twist_in_A = Adjoint(tf_from_A_to_B) * twist_in_B
 *
 * When using this adjoint, it assumes the twist is a 6x1 vector with angular on top [angular ; linear]. The tf2_eigen
 * library does not convert Twist messages to Eigen Vectors this way, so use the functions included here for converting
 * between these data types
 */
inline Eigen::MatrixXd getAdjointMatrix(const Eigen::Isometry3d& transform)
{
  Eigen::MatrixXd adjoint(6, 6);

  // Block matrix with
  // |R     0|
  // |[p]R  R|
  adjoint.block<3, 3>(0, 0) = transform.linear();
  adjoint.block<3, 3>(0, 3).setZero();
  adjoint.block<3, 3>(3, 0) = getSkewSymmetricMatrix(transform.translation()) * transform.linear();
  adjoint.block<3, 3>(3, 3) = transform.linear();

  return adjoint;
}

/**
 * @brief Converts a transformation into an adjoint matrix for transforming twists and wrenches
 *
 * Usage: twist_in_A = Adjoint(tf_from_A_to_B) * twist_in_B
 *
 * When using this adjoint, it assumes the twist is a 6x1 vector with angular on top [angular ; linear]. The tf2_eigen
 * library does not convert Twist messages to Eigen Vectors this way, so use the functions included here for converting
 * between these data types
 */
inline Eigen::MatrixXd getAdjointMatrix(const geometry_msgs::Transform& transform)
{
  return getAdjointMatrix(tf2::transformToEigen(transform));
}

/**
 * @brief Converts a twist message to a 6x1 vector with [angular ; linear]
 */
inline Eigen::VectorXd twistToVector(const geometry_msgs::Twist& twist)
{
  Eigen::Vector3d angular, linear;
  tf2::fromMsg(twist.angular, angular);
  tf2::fromMsg(twist.linear, linear);

  Eigen::VectorXd output(6);
  output.head(3) = angular;
  output.tail(3) = linear;

  return output;
}

/**
 * @brief Converts a 6x1 vector with [angular ; linear] to a twist message
 */
inline geometry_msgs::Twist vectorToTwist(const Eigen::VectorXd& vec)
{
  geometry_msgs::Twist output;

  tf2::toMsg(vec.head(3), output.angular);
  tf2::toMsg(vec.tail(3), output.linear);

  return output;
}

inline std::string twistToStr(const geometry_msgs::TwistStamped& twist)
{
  std::stringstream stream;
  stream << "\nHeader: " << twist.header.frame_id << "\nX: " << twist.twist.linear.x << "\nY: " << twist.twist.linear.y
         << "\nZ: " << twist.twist.linear.z << "\nRoll: " << twist.twist.angular.x
         << "\nPitch: " << twist.twist.angular.y << "\nYaw: " << twist.twist.angular.z;
  return stream.str();
}

inline std::string poseToStr(const geometry_msgs::PoseStamped& pose)
{
  std::stringstream stream;
  stream << "\nHeader: " << pose.header.frame_id << "\nX: " << pose.pose.position.x << "\nY: " << pose.pose.position.y
         << "\nZ: " << pose.pose.position.z << "\nQX: " << pose.pose.orientation.x
         << "\nQY: " << pose.pose.orientation.y << "\nQZ: " << pose.pose.orientation.z
         << "\nQW: " << pose.pose.orientation.w;
  return stream.str();
}

inline std::string screwMsgToStr(const affordance_primitive_msgs::ScrewStamped& screw)
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
