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

#include <affordance_primitives/msg_types.hpp>
#include <optional>
#include <sstream>
#include <tf2_eigen/tf2_eigen.hpp>

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
PoseStamped getPoseInATFrame(
  const std::string & root_frame_name, const PoseStamped & root_frame_pose,
  const PoseStamped & incoming_frame);

/**
 * @brief Get the twist that represents moving from a start_pose to an end_pose
 * @param start_pose The name of the AT's root frame. The returned Pose will be with respect to this frame
 * @param end_pose The Pose of the AT's root frame (usually defined with respect to the planning frame)
 * @return The found twist
 * @exception Throws a std::runtime_error if the poses are not in the same frame
 */
TwistStamped getTwistFromPoses(const PoseStamped & start_pose, const PoseStamped & end_pose);

/**
 * @brief Calculates the pose of a frame with respect to a new pose, given that all input frames are defined w.r.t. the
 * root frame
 * @param new_base_frame The pose the returned frame will be defined with respect to (e.g. the grasp pose)
 * @param transformed_pose The pose to transform into the new frame (e.g. the screw origin pose)
 * @return The new pose
 * @exception Throws a std::runtime_error if the poses are not in the same frame
 */
Eigen::Isometry3d convertPoseToNewFrame(
  const PoseStamped & new_base_frame, const PoseStamped & transformed_pose);

/**
 * @brief Transforms a ScrewStamped msg by the prescribed Transform
 * @param input_screw Screw message to transform
 * @param transform The transform to perform. The frame_id should match in the input screw
 * @return The transformed ScrewStamped
 * @exception Throws a std::runtime_error if the transform cannot be completed
 */
ScrewStamped transformScrew(const ScrewStamped & input_screw, const TransformStamped & transform);

/**
 * @brief Converts an axis vector to 3x3 skew symmetric matrix, for math
 */
Eigen::Matrix3d getSkewSymmetricMatrix(const Eigen::Vector3d & vec);

/**
 * @brief Converts a transformation into an adjoint matrix for transforming twists and wrenches
 *
 * Usage: twist_in_A = Adjoint(tf_from_A_to_B) * twist_in_B
 *
 * When using this adjoint, it assumes the twist is a 6x1 vector with linear on top [linear ; angular]. This follows
 * from the way tf2_eigen converts between twist messages and Eigen vectors, but is opposite from certain math notation
 */
Eigen::MatrixXd getAdjointMatrix(const Eigen::Isometry3d & transform);

/**
 * @brief Converts a transformation into an adjoint matrix for transforming twists and wrenches
 *
 * Usage: twist_in_A = Adjoint(tf_from_A_to_B) * twist_in_B
 *
 * When using this adjoint, it assumes the twist is a 6x1 vector with linear on top [linear ; angular]. This follows
 * from the way tf2_eigen converts between twist messages and Eigen vectors, but is opposite from certain math notation
 */
Eigen::MatrixXd getAdjointMatrix(const Transform & transform);

/**
 * @brief Gets the pretty string format for a TwistStamped message
 */
std::string twistToStr(const TwistStamped & twist);

/**
 * @brief Transforms CartesianFloat message to Eigen Vector
 * Assumes the linear 3 values are first in the vector
 * @param cart_float Message to transform
 * @return A 6x1 Eigen Vector
 */
Eigen::Matrix<double, 6, 1> CartesianFloatToVector(const CartesianFloat & cart_float);

/**
 * @brief Transforms an Eigen Vector to CartesianFloat message
 * Assumes the linear 3 values are first in the vector
 * @param vector Vector to transform
 * @return CartesianFloat message
 */
CartesianFloat VectorToCartesianFloat(const Eigen::Matrix<double, 6, 1> & vector);

/**
 * @brief Transforms Wrench message to Eigen Vector
 * Assumes the linear 3 values are first in the vector
 * @param wrench Message to transform
 * @return A 6x1 Eigen Vector
 */
Eigen::Matrix<double, 6, 1> WrenchToVector(const Wrench & wrench);

/**
 * @brief Transforms an Eigen Vector to Wrench message
 * Assumes the linear 3 values are first in the vector
 * @param vector Vector to transform
 * @return Wrench message
 */
Wrench VectorToWrench(const Eigen::Matrix<double, 6, 1> & vector);

/**
 * @brief Gets the pretty string format for a PoseStamped message
 */
std::string poseToStr(const PoseStamped & pose);

/**
 * @brief Gets the pretty string format for a ScrewStamped message
 */
std::string screwMsgToStr(const ScrewStamped & screw);

}  // namespace affordance_primitives
