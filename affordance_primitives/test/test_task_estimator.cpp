///////////////////////////////////////////////////////////////////////////////
//      Title     : test_task_estimator.cpp
//      Project   : affordance_primitives
//      Created   : 02/04/2022
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

#include <affordance_primitives/task_estimator/kinematic_task_estimator.hpp>
#include <affordance_primitives/msg_types.hpp>
#include <pluginlib/class_loader.h>

constexpr double EPSILON = 1e-4;

TEST(ParameterManager, test_kinematic_task_estimator)
{
  // Load plugin
  pluginlib::ClassLoader<affordance_primitives::TaskEstimator> plugin_loader("affordance_primitives",
                                                                             "affordance_primitives::TaskEstimator");
  boost::shared_ptr<affordance_primitives::TaskEstimator> task_estimator;
  ASSERT_NO_THROW(task_estimator = plugin_loader.createInstance("affordance_primitives::KinematicTaskEstimator"));

  // Initialize plugin
  ros::NodeHandle nh;
  ASSERT_NO_THROW(task_estimator->initialize(nh));

  // Set up test AP
  affordance_primitives::AffordancePrimitive::Request ap;
  ap.moving_frame_source = ap.PROVIDED;
  ap.screw.header.frame_id = "Screw frame";
  ap.moving_to_task_frame.child_frame_id = "Screw frame";

  // Test resetting with valid input
  ASSERT_NO_THROW(task_estimator->resetTaskEstimation(42.0));
  std::optional<double> output;
  ASSERT_NO_THROW(output = task_estimator->estimateTaskAngle(ap));
  ASSERT_TRUE(output.has_value());
  EXPECT_NEAR(output.value(), 42.0, EPSILON);

  // Now with invalid input
  ap.moving_to_task_frame.child_frame_id = "not Screw frame";
  ASSERT_NO_THROW(task_estimator->resetTaskEstimation(0));
  ASSERT_NO_THROW(task_estimator->estimateTaskAngle(ap));
  output.reset();
  ASSERT_NO_THROW(output = task_estimator->estimateTaskAngle(ap));
  EXPECT_FALSE(output.has_value());

  // Try with a 30 degree rotation about X
  ap.moving_to_task_frame.child_frame_id = "Screw frame";
  ASSERT_NO_THROW(task_estimator->estimateTaskAngle(ap));
  ap.moving_to_task_frame.transform.rotation.x = 0.2588;
  ap.moving_to_task_frame.transform.rotation.w = 0.9659;
  output.reset();
  ASSERT_NO_THROW(output = task_estimator->estimateTaskAngle(ap));
  ASSERT_TRUE(output.has_value());
  EXPECT_NEAR(output.value(), 0.5235, EPSILON);  // Radians

  // Try with a linear screw
  ASSERT_NO_THROW(task_estimator->resetTaskEstimation(0));
  ap.moving_to_task_frame.transform = affordance_primitives::Transform();
  ap.screw.is_pure_translation = true;
  ASSERT_NO_THROW(output = task_estimator->estimateTaskAngle(ap));
  ap.moving_to_task_frame.transform.translation.x = 0.5;
  ap.moving_to_task_frame.transform.translation.y = 0.75;
  output.reset();
  ASSERT_NO_THROW(output = task_estimator->estimateTaskAngle(ap));
  ASSERT_TRUE(output.has_value());
  EXPECT_NEAR(output.value(), sqrt(0.5 * 0.5 + 0.75 * 0.75), EPSILON);  // Meters
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_parameter_manager");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(8);
  spinner.start();

  int result = RUN_ALL_TESTS();
  return result;
}
