///////////////////////////////////////////////////////////////////////////////
//      Title     : test_parameter_manager.cpp
//      Project   : affordance_primitives
//      Created   : 01/20/2022
//      Author    : Adam Pettinger
//      Copyright : Copyright© The University of Texas at Austin, 2014-2022. All
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

#include <gtest/gtest.h>

#include <affordance_primitives/configs_interface/empty_parameter_manager.hpp>
#include <affordance_primitives/msg_types.hpp>
#include <pluginlib/class_loader.hpp>

TEST(ParameterManager, test_empty_parameter_manager)
{
  auto node = std::make_shared<rclcpp::Node>("test_parameter_manager");

  // Load plugin
  auto plugin_loader =
    std::make_shared<pluginlib::ClassLoader<affordance_primitives::ParameterManager>>(
      "affordance_primitives", "affordance_primitives::ParameterManager");
  std::shared_ptr<affordance_primitives::ParameterManager> param_manager;
  ASSERT_NO_THROW(
    param_manager =
      plugin_loader->createSharedInstance("affordance_primitives::EmptyParameterManager"));

  // Initialize plugin
  ASSERT_NO_THROW(param_manager->initialize(node));

  // Test basic functionality
  affordance_primitives::APRobotParameter parameters;
  std::pair<bool, std::string> out;
  ASSERT_NO_THROW(out = param_manager->setParameters(parameters));
  EXPECT_TRUE(out.first);
  EXPECT_TRUE(out.second.empty());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  return result;
}
