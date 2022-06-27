
///////////////////////////////////////////////////////////////////////////////
//      Title     : msg_types.h
//      Project   : affordance_primitives
//      Created   : 12/20/2021
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

#pragma once

/*
    The intent of this file is to have one place all messages are included and
   aliased. I understand this sort of type aliasing is not optimal. However,
   while developping on ROS 1 and 2 at the same time this sort of hack makes
   transfering code between versions much easier
*/

#include <affordance_primitive_msgs/action/affordance_primitive.hpp>
#include <affordance_primitive_msgs/msg/affordance_trajectory.hpp>
#include <affordance_primitive_msgs/msg/affordance_waypoint.hpp>
#include <affordance_primitive_msgs/msg/ap_robot_parameter.hpp>
#include <affordance_primitive_msgs/msg/cartesian_float.hpp>
#include <affordance_primitive_msgs/msg/screw_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace affordance_primitives
{
using AffordanceTrajectory = affordance_primitive_msgs::msg::AffordanceTrajectory;
using AffordanceWaypoint = affordance_primitive_msgs::msg::AffordanceWaypoint;
using APRobotParameter = affordance_primitive_msgs::msg::APRobotParameter;
using AffordancePrimitive = affordance_primitive_msgs::action::AffordancePrimitive;
using AffordancePrimitiveGoal = affordance_primitive_msgs::action::AffordancePrimitive::Goal;
using AffordancePrimitiveFeedback =
  affordance_primitive_msgs::action::AffordancePrimitive::Feedback;
using AffordancePrimitiveResult = affordance_primitive_msgs::action::AffordancePrimitive::Result;
using CartesianFloat = affordance_primitive_msgs::msg::CartesianFloat;
using ScrewStamped = affordance_primitive_msgs::msg::ScrewStamped;
using PointStamped = geometry_msgs::msg::PointStamped;
using Point = geometry_msgs::msg::Point;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;
using Quaternion = geometry_msgs::msg::Quaternion;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Transform = geometry_msgs::msg::Transform;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using Twist = geometry_msgs::msg::Twist;
using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;
using Vector3 = geometry_msgs::msg::Vector3;
using WrenchStamped = geometry_msgs::msg::WrenchStamped;
using Wrench = geometry_msgs::msg::Wrench;
}  // namespace affordance_primitives
