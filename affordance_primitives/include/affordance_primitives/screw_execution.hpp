///////////////////////////////////////////////////////////////////////////////
//      Title     : ap_screw_executor.h
//      Project   : affordance_primitives
//      Created   : 10/07/2021
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

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <affordance_primitives/msg_types.hpp>
#include <memory>

namespace affordance_primitives
{
class APScrewExecutor
{
public:
  APScrewExecutor();
  APScrewExecutor(ros::NodeHandle& nh, const std::string& server_name_);

  ~APScrewExecutor(){};

  bool getScrewTwist(AffordancePrimitive::Request& req, AffordancePrimitive::Response& res);

private:
  ros::ServiceServer screw_twist_server_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
};
}  // namespace affordance_primitives
