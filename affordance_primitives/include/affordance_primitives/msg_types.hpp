
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

#include <affordance_primitive_msgs/APRobotParameter.h>
#include <affordance_primitive_msgs/AffordancePrimitiveAction.h>
#include <affordance_primitive_msgs/AffordanceTrajectory.h>
#include <affordance_primitive_msgs/AffordanceWaypoint.h>
#include <affordance_primitive_msgs/CartesianFloat.h>
#include <affordance_primitive_msgs/ScrewStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>

namespace affordance_primitives
{
using APRobotParameter = affordance_primitive_msgs::APRobotParameter;
using AffordancePrimitiveAction = affordance_primitive_msgs::AffordancePrimitiveAction;
using AffordancePrimitiveGoal = affordance_primitive_msgs::AffordancePrimitiveGoal;
using AffordancePrimitiveFeedback = affordance_primitive_msgs::AffordancePrimitiveFeedback;
using AffordancePrimitiveResult = affordance_primitive_msgs::AffordancePrimitiveResult;
using AffordanceTrajectory = affordance_primitive_msgs::AffordanceTrajectory;
using AffordanceWaypoint = affordance_primitive_msgs::AffordanceWaypoint;
using CartesianFloat = affordance_primitive_msgs::CartesianFloat;
using ScrewStamped = affordance_primitive_msgs::ScrewStamped;
using PointStamped = geometry_msgs::PointStamped;
using Point = geometry_msgs::Point;
using PoseStamped = geometry_msgs::PoseStamped;
using Pose = geometry_msgs::Pose;
using QuaternionStamped = geometry_msgs::QuaternionStamped;
using Quaternion = geometry_msgs::Quaternion;
using TransformStamped = geometry_msgs::TransformStamped;
using Transform = geometry_msgs::Transform;
using TwistStamped = geometry_msgs::TwistStamped;
using Twist = geometry_msgs::Twist;
using Vector3Stamped = geometry_msgs::Vector3Stamped;
using Vector3 = geometry_msgs::Vector3;
using WrenchStamped = geometry_msgs::WrenchStamped;
using Wrench = geometry_msgs::Wrench;
}  // namespace affordance_primitives
