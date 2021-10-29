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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

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

inline std::string twistToStr(const geometry_msgs::msg::TwistStamped& twist)
{
  std::stringstream stream;
  stream << "\nHeader: " << twist.header.frame_id << "\nX: " << twist.twist.linear.x << "\nY: " << twist.twist.linear.y
         << "\nZ: " << twist.twist.linear.z << "\nRoll: " << twist.twist.angular.x
         << "\nPitch: " << twist.twist.angular.y << "\nYaw: " << twist.twist.angular.z;
  return stream.str();
}

inline std::string poseToStr(const geometry_msgs::msg::PoseStamped& pose)
{
  std::stringstream stream;
  stream << "\nHeader: " << pose.header.frame_id << "\nX: " << pose.pose.position.x << "\nY: " << pose.pose.position.y
         << "\nZ: " << pose.pose.position.z << "\nQX: " << pose.pose.orientation.x
         << "\nQY: " << pose.pose.orientation.y << "\nQZ: " << pose.pose.orientation.z
         << "\nQW: " << pose.pose.orientation.w;
  return stream.str();
}

}  // namespace affordance_primitives
