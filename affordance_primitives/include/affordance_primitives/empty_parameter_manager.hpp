///////////////////////////////////////////////////////////////////////////////
//      Title     : empty_parameter_manager.h
//      Project   : affordance_primitives
//      Created   : 12/29/2021
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
#include <affordance_primitives/parameter_manager.hpp>

#include <utility>

namespace affordance_primitives
{
/**
 * An example of a ParameterManager plugin implementation that is trivial
 */
class EmptyParameterManager : public affordance_primitives::ParameterManager
{
public:
  EmptyParameterManager(){};
  void initialize(const ros::NodeHandle& nh)
  {
    nh_ = ros::NodeHandle(nh);
  }

  /** Tries to set a robot's parameters
   *
   * @param params The parameters to get set
   * @return The first value is true if everything was set correctly, second
   * value is a string that provides logging messages
   */
  std::pair<bool, std::string> setParameters(const affordance_primitives::AffordanceParameter& params)
  {
    return std::make_pair(true, "");
  };

  ~EmptyParameterManager(){};
};
}  // namespace affordance_primitives
