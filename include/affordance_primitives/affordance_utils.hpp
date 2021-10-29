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

#pragma once

#include <optional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <affordance_primitives/screw_conversions.hpp>

namespace affordance_primitives
{
/**
 * @brief Converts a pose to be in the AT frame if necessary
 * @param root_frame_name The name of the AT's root frame. The returned Pose will be with respect to this frame
 * @param root_frame_pose The Pose of the AT's root frame (usually defined with respect to the planning frame)
 * @param incoming_frame The Pose to convert. It should either be defined with respect to the AT root frame or the same
 * frame as root_frame_pose
 * @return The converted pose
 * @exception Throws a std::runtime_error if the pose could not be converted
 */
geometry_msgs::msg::PoseStamped getPoseInATFrame(const std::string& root_frame_name,
                                                 const geometry_msgs::msg::PoseStamped& root_frame_pose,
                                                 const geometry_msgs::msg::PoseStamped& incoming_frame);

/**
 * @brief Get the twist that represents moving from a start_pose to an end_pose
 * @param start_pose The name of the AT's root frame. The returned Pose will be with respect to this frame
 * @param end_pose The Pose of the AT's root frame (usually defined with respect to the planning frame)
 * @return The found twist
 * @exception Throws a std::runtime_error if the poses are not in the same frame
 */
geometry_msgs::msg::TwistStamped getTwistFromPoses(const geometry_msgs::msg::PoseStamped& start_pose,
                                                   const geometry_msgs::msg::PoseStamped& end_pose);

/**
 * @brief Calculates the pose of a frame with respect to a new pose, given that all input frames are defined w.r.t. the
 * root frame
 * @param new_base_frame The pose the returned frame will be defined with respect to (e.g. the grasp pose)
 * @param transformed_pose The pose to transform into the new frame (e.g. the screw origin pose)
 * @return The new pose
 * @exception Throws a std::runtime_error if the poses are not in the same frame
 */
Eigen::Isometry3d convertPoseToNewFrame(const geometry_msgs::msg::PoseStamped& new_base_frame,
                                        const geometry_msgs::msg::PoseStamped& transformed_pose);
}  // namespace affordance_primitives
