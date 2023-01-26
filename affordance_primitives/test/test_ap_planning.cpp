///////////////////////////////////////////////////////////////////////////////
//      Title     : test_ap_planning.cpp
//      Project   : affordance_primitives
//      Created   : 01/24/2023
//      Author    : Adam Pettinger
//      Copyright : Copyright© The University of Texas at Austin, 2014-2023. All
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

#include <affordance_primitives/screw_planning/screw_planning.hpp>

constexpr double EPSILON = 1e-4;

inline void checkVector(const Eigen::Vector3d & vec, double x, double y, double z)
{
  EXPECT_NEAR(vec.x(), x, EPSILON);
  EXPECT_NEAR(vec.y(), y, EPSILON);
  EXPECT_NEAR(vec.z(), z, EPSILON);
}

TEST(ScrewPlanning, chainConversions)
{
  Eigen::VectorXd lower_bounds(3), upper_bounds(3), phi_in(3);
  lower_bounds << -1, 0, -1;
  upper_bounds << 2, 2, 0;
  auto bounds = std::make_pair(lower_bounds, upper_bounds);

  // Test first case without getting the index
  phi_in << 0, 0, -1;
  double lambda_out;
  ASSERT_NO_THROW(lambda_out = affordance_primitives::getLambda(phi_in, bounds));
  EXPECT_NEAR(lambda_out, 1.0, EPSILON);

  // Now try again, but get the index
  size_t s_index;
  ASSERT_NO_THROW(lambda_out = affordance_primitives::getLambda(phi_in, bounds, &s_index));
  EXPECT_NEAR(lambda_out, 1.0, EPSILON);
  EXPECT_EQ(s_index, 0);

  // Now go the other way without index
  Eigen::VectorXd phi_out;
  ASSERT_NO_THROW(phi_out = affordance_primitives::getPhi(lambda_out, bounds));
  checkVector(phi_out, 0, 0, -1);

  // Try the other way with index
  ASSERT_NO_THROW(phi_out = affordance_primitives::getPhi(lambda_out, bounds, &s_index));
  checkVector(phi_out, 0, 0, -1);
  EXPECT_EQ(s_index, 0);

  // Try other cases
  phi_in = lower_bounds;
  ASSERT_NO_THROW(lambda_out = affordance_primitives::getLambda(phi_in, bounds, &s_index));
  EXPECT_NEAR(lambda_out, 0.0, EPSILON);
  EXPECT_EQ(s_index, 0);
  ASSERT_NO_THROW(phi_out = affordance_primitives::getPhi(lambda_out, bounds, &s_index));
  checkVector(phi_out, -1, 0, -1);
  EXPECT_EQ(s_index, 0);

  phi_in(0) = 2;
  phi_in(1) = 1;
  phi_in(2) = -1;
  ASSERT_NO_THROW(lambda_out = affordance_primitives::getLambda(phi_in, bounds, &s_index));
  EXPECT_NEAR(lambda_out, 4.0, EPSILON);
  EXPECT_EQ(s_index, 1);
  ASSERT_NO_THROW(phi_out = affordance_primitives::getPhi(lambda_out, bounds, &s_index));
  checkVector(phi_out, 2, 1, -1);
  EXPECT_EQ(s_index, 1);

  phi_in(0) = 2;
  phi_in(1) = 2;
  phi_in(2) = -0.5;
  ASSERT_NO_THROW(lambda_out = affordance_primitives::getLambda(phi_in, bounds, &s_index));
  EXPECT_NEAR(lambda_out, 5.5, EPSILON);
  EXPECT_EQ(s_index, 2);
  ASSERT_NO_THROW(phi_out = affordance_primitives::getPhi(lambda_out, bounds, &s_index));
  checkVector(phi_out, 2, 2, -0.5);
  EXPECT_EQ(s_index, 2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
