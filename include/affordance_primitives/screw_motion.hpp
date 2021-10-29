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

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <affordance_primitives/screw_conversions.hpp>

namespace affordance_primitives
{
/**
 * @brief A helper function that calculates the linear velocity component of a Screw Axis
 * @param axis The axis of rotation
 * @param q_vector The vector pointing from the ScrewAxis frame to the axis of rotation
 * @param pitch The pitch of the screw axis
 * @return A vector representing the "normalized" (which may not have norm==1) linear velocity of the screw
 */
Eigen::Vector3d calculateLinearVelocity(const Eigen::Vector3d& axis, const Eigen::Vector3d& q_vector,
                                        const double pitch);

class ScrewAxis
{
public:
  ScrewAxis(const std::string moving_frame_name, bool is_pure_translation = false);

  // Setup Screw Axis
  /**
   * @brief Sets up the Screw axis, using an axis defined by a line between 2 poses
   * @param origin_pose The first pose will be used as the origin of the Screw. This must be defined with respect to the
   * Screw's frame
   * @param axis_pose The second pose is just needed to define an axis. This must be defined with respect to the Screw's
   * frame
   * @param pitch Sets the pitch of the screw
   * @return True if the setup was successful, false otherwise
   */
  bool setScrewAxis(const geometry_msgs::msg::Pose& origin_pose, const geometry_msgs::msg::Pose& axis_pose,
                    double pitch = 0);
  bool setScrewAxis(const Eigen::Isometry3d& origin_pose, const Eigen::Isometry3d& axis_pose, double pitch = 0)
  {
    return setScrewAxis(Eigen::toMsg(origin_pose), Eigen::toMsg(axis_pose), pitch);
  };

  /**
   * @brief Sets up the Screw axis, using an axis defined pose and a vector
   * @param origin_pose The first pose will be used as the origin of the Screw. This must be defined with respect to the
   * Screw's frame
   * @param axis Sets the axis of the screw. This must be defined with respect to the Screw's frame
   * @param pitch Sets the pitch of the screw
   * @return True if the setup was successful, false otherwise
   */
  bool setScrewAxis(const geometry_msgs::msg::Pose& origin_pose, const Eigen::Vector3d& axis, double pitch = 0);
  bool setScrewAxis(const Eigen::Isometry3d& origin_pose, const Eigen::Vector3d& axis, double pitch = 0)
  {
    return setScrewAxis(Eigen::toMsg(origin_pose), axis, pitch);
  };

  /**
   * @brief Sets up the Screw axis, a twist message (TODO: implement)
   * @param twist The twist gets normalized based on whether or not the Screw is purely translational
   * @param pitch Sets the pitch of the screw
   * @return True if the setup was successful, false otherwise
   */
  // bool setScrewAxis(const geometry_msgs::msg::Twist& twist, const double pitch=0);

  // Use the Screw to generate motion profiles
  /**
   * @brief Gets the Twist corresponding to moving along this screw axis at the given velocity
   * @param theta_dot The "velocity" (linear or rotational) to move
   * @return The calculated twist, defined with respect to the frame of the Screw Axis
   */
  geometry_msgs::msg::TwistStamped getTwist(double theta_dot) const;

  /**
   * @brief Gets the transform corresponding to moving along this screw axis for a given displacement
   * @param delta_theta The "displacement" (linear or rotational) to move
   * @return The calculated transformation, defined with respect to the frame of the Screw Axis
   */
  Eigen::Isometry3d getTF(double delta_theta) const;

  // Getters
  std::string getFrame() const
  {
    return moving_frame_name_;
  };
  Eigen::Vector3d getQVector() const
  {
    return origin_;
  };
  Eigen::Vector3d getAxis() const
  {
    return axis_;
  };
  Eigen::Vector3d getLinearVector() const
  {
    return translation_component_;
  };
  double getPitch() const
  {
    return pitch_;
  };

private:
  // The name of the moving coordinate frame (e.g end-effector)
  // Every other parameter is defined with respect to this frame
  std::string moving_frame_name_;

  // We do different math if this Screw Axis is only a linear, translational motion
  bool is_pure_translation_;

  // The origin of the screw axis, also known as the 'q vector'
  Eigen::Vector3d origin_;

  // The unit vector describing the axis direction, also known as the 's hat vector'
  Eigen::Vector3d axis_;

  // The unit vector translational motion of the screw axis. Also known as 'v vector'
  Eigen::Vector3d translation_component_;

  // The pitch relates angular rotation around the axis to linear translation along it
  // 0 pitch = no linear motion
  // infinity pitch = no rotation
  double pitch_;
};
}  // namespace affordance_primitives
