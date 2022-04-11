///////////////////////////////////////////////////////////////////////////////
//      Title     : test_task_monitor.cpp
//      Project   : affordance_primitives
//      Created   : 02/28/2022
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

#include <affordance_primitives/msg_types.hpp>
#include <affordance_primitives/task_monitor/task_monitor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using AffordancePrimitiveResult = affordance_primitive_msgs::action::AffordancePrimitive::Result;

TEST(TaskMonitor, start_stop_monitor)
{
  auto node = std::make_shared<rclcpp::Node>("test_task_monitor");

  // Default parameters
  affordance_primitives::APRobotParameter params;

  // Create and start monitor with no timeout
  affordance_primitives::TaskMonitor monitor(node, "ft_topic_name");
  ASSERT_NO_THROW(monitor.startMonitor(params));

  // Wait some time, then stop monitor
  rclcpp::sleep_for(std::chrono::milliseconds(250));
  ASSERT_NO_THROW(monitor.stopMonitor());

  // Result should be STOP_REQUESTED
  std::optional<AffordancePrimitiveResult> result = monitor.getResult();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->result, result->STOP_REQUESTED);

  // Now try interupting a timed run
  ASSERT_NO_THROW(monitor.startMonitor(params, 0.25));
  ASSERT_NO_THROW(monitor.stopMonitor());
  result = monitor.getResult();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->result, result->STOP_REQUESTED);
}

TEST(TaskMonitor, monitor_timing)
{
  auto node = std::make_shared<rclcpp::Node>("test_task_monitor");
  const double test_duration = 0.25;

  // Default parameters
  affordance_primitives::APRobotParameter params;

  // Create and start monitor with timeout
  affordance_primitives::TaskMonitor monitor(node, "ft_topic_name");
  ASSERT_NO_THROW(monitor.startMonitor(params, test_duration));

  // Immediately getting the result shouldn't work
  EXPECT_FALSE(monitor.getResult().has_value());

  // Wait some time and grab result
  std::chrono::duration<double> wait_time(1.05 * test_duration);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(wait_time));
  std::optional<AffordancePrimitiveResult> result = monitor.getResult();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->result, result->TIME_OUT);

  // Now test blocking call to waitForResult()
  ASSERT_NO_THROW(monitor.startMonitor(params, test_duration));
  AffordancePrimitiveResult result2;

  rclcpp::Time start = node->now();
  result2 = monitor.waitForResult();
  EXPECT_LE(
    (node->now() - start).nanoseconds(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(wait_time).count());
  EXPECT_EQ(result2.result, result2.TIME_OUT);
}

TEST(TaskMonitor, monitor_ft_readings)
{
  auto node = std::make_shared<rclcpp::Node>("test_task_monitor");
  auto ft_pub = node->create_publisher<affordance_primitives::WrenchStamped>("ft_topic_name", 1);

  const double test_duration = 0.25;
  const double ft_pub_period = 0.005;  // 200 hz
  const size_t num_publish = test_duration / ft_pub_period;

  // Set some force limits in parameter
  affordance_primitives::APRobotParameter params;
  params.max_wrench.rot_x = 10;
  params.max_wrench.rot_y = 10;
  params.max_wrench.rot_z = 10;
  params.max_wrench.trans_x = 100;
  params.max_wrench.trans_y = 100;
  params.max_wrench.trans_z = 100;
  params.max_force = 120;
  params.max_torque = 12;

  // Start publishing FT readings of 0
  affordance_primitives::WrenchStamped pub_wrench;
  std::thread t([&ft_pub, &pub_wrench, num_publish, ft_pub_period]() {
    std::chrono::duration<double> wait_time_sec(ft_pub_period);
    auto wait_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wait_time_sec);
    for (size_t i = 0; i < num_publish; ++i) {
      ft_pub->publish(pub_wrench);
      rclcpp::sleep_for(wait_time_ns);
    }
  });

  // Start the monitor and verify it times out
  affordance_primitives::TaskMonitor monitor(node, "ft_topic_name");
  ASSERT_NO_THROW(monitor.startMonitor(params, test_duration));
  AffordancePrimitiveResult result = monitor.waitForResult();
  EXPECT_EQ(result.result, result.TIME_OUT);
  t.join();

  // Now publish FT readings to violate max force limit
  t = std::thread([&ft_pub, &pub_wrench, num_publish, ft_pub_period]() {
    std::chrono::duration<double> wait_time_sec(ft_pub_period);
    auto wait_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wait_time_sec);
    for (size_t i = 0; i < num_publish; ++i) {
      if (i == num_publish / 3) {
        pub_wrench.wrench.force.x = 99;
        pub_wrench.wrench.force.y = 99;
        pub_wrench.wrench.force.z = 99;
      }
      ft_pub->publish(pub_wrench);
      rclcpp::sleep_for(wait_time_ns);
    }
  });

  // Start monitor and verify FT limit is violated
  ASSERT_NO_THROW(monitor.startMonitor(params, test_duration));
  result = monitor.waitForResult();
  EXPECT_EQ(result.result, result.FT_VIOLATION);
  t.join();

  // Now publish to violate individual limits
  pub_wrench = affordance_primitives::WrenchStamped();
  t = std::thread([&ft_pub, &pub_wrench, num_publish, ft_pub_period]() {
    std::chrono::duration<double> wait_time_sec(ft_pub_period);
    auto wait_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wait_time_sec);
    for (size_t i = 0; i < num_publish; ++i) {
      if (i == num_publish / 3) {
        pub_wrench.wrench.torque.z = 11;
      } else {
        pub_wrench.wrench.torque.z = 9;
      }
      ft_pub->publish(pub_wrench);
      rclcpp::sleep_for(wait_time_ns);
    }
  });

  // Start monitor and verify FT limit is violated
  ASSERT_NO_THROW(monitor.startMonitor(params, test_duration));
  result = monitor.waitForResult();
  EXPECT_EQ(result.result, result.FT_VIOLATION);
  t.join();

  // Now check to make sure NOT setting a limit results in no limit
  params.max_force = 0;
  t = std::thread([&ft_pub, &pub_wrench, num_publish, ft_pub_period]() {
    std::chrono::duration<double> wait_time_sec(ft_pub_period);
    auto wait_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wait_time_sec);
    for (size_t i = 0; i < num_publish; ++i) {
      if (i == num_publish / 3) {
        pub_wrench.wrench.force.x = 99;
        pub_wrench.wrench.force.y = 99;
        pub_wrench.wrench.force.z = 99;
      }
      ft_pub->publish(pub_wrench);
      rclcpp::sleep_for(wait_time_ns);
    }
  });

  // Start the monitor and verify it times out
  ASSERT_NO_THROW(monitor.startMonitor(params, test_duration));
  result = monitor.waitForResult();
  EXPECT_EQ(result.result, result.TIME_OUT);
  t.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  return result;
}
