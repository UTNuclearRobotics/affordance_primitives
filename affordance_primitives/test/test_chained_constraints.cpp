///////////////////////////////////////////////////////////////////////////////
//      Title     : test_chained_constraints.cpp
//      Project   : affordance_primitives
//      Created   : 02/05/2023
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

#include <affordance_primitives/screw_planning/chained_screws.hpp>

constexpr double EPSILON = 1e-4;

inline void checkVector(const affordance_primitives::Vector3 & vec, double x, double y, double z)
{
  EXPECT_NEAR(vec.x, x, EPSILON);
  EXPECT_NEAR(vec.y, y, EPSILON);
  EXPECT_NEAR(vec.z, z, EPSILON);
}
inline void checkVector(const std::vector<double> & vec, double x, double y, double z)
{
  ASSERT_EQ(vec.size(), 3);
  EXPECT_NEAR(vec[0], x, EPSILON);
  EXPECT_NEAR(vec[1], y, EPSILON);
  EXPECT_NEAR(vec[2], z, EPSILON);
}
inline void checkVector(
  const affordance_primitives::Vector3 & vec1, const affordance_primitives::Vector3 & vec2)
{
  EXPECT_NEAR(vec1.x, vec2.x, EPSILON);
  EXPECT_NEAR(vec1.y, vec2.y, EPSILON);
  EXPECT_NEAR(vec1.z, vec2.z, EPSILON);
}
inline void checkVector(const Eigen::Vector3d & vec, double x, double y, double z)
{
  EXPECT_NEAR(vec.x(), x, EPSILON);
  EXPECT_NEAR(vec.y(), y, EPSILON);
  EXPECT_NEAR(vec.z(), z, EPSILON);
}
inline void checkPoint(const affordance_primitives::Point & point, double x, double y, double z)
{
  EXPECT_NEAR(point.x, x, EPSILON);
  EXPECT_NEAR(point.y, y, EPSILON);
  EXPECT_NEAR(point.z, z, EPSILON);
}
inline void checkPoint(
  const affordance_primitives::Point & point1, const affordance_primitives::Point & point2)
{
  EXPECT_NEAR(point1.x, point2.x, EPSILON);
  EXPECT_NEAR(point1.y, point2.y, EPSILON);
  EXPECT_NEAR(point1.z, point2.z, EPSILON);
}
inline void checkQuaternion(
  const affordance_primitives::Quaternion & quat1, const affordance_primitives::Quaternion & quat2)
{
  EXPECT_NEAR(quat1.x, quat2.x, EPSILON);
  EXPECT_NEAR(quat1.y, quat2.y, EPSILON);
  EXPECT_NEAR(quat1.z, quat2.z, EPSILON);
  EXPECT_NEAR(quat1.w, quat2.w, EPSILON);
}
inline void checkQuaternion(
  const affordance_primitives::Quaternion & quat, double x, double y, double z, double w)
{
  EXPECT_NEAR(quat.x, x, EPSILON);
  EXPECT_NEAR(quat.y, y, EPSILON);
  EXPECT_NEAR(quat.z, z, EPSILON);
  EXPECT_NEAR(quat.w, w, EPSILON);
}
inline void checkQuaternion(const Eigen::Quaterniond & quat, double x, double y, double z, double w)
{
  EXPECT_NEAR(quat.x(), x, EPSILON);
  EXPECT_NEAR(quat.y(), y, EPSILON);
  EXPECT_NEAR(quat.z(), z, EPSILON);
  EXPECT_NEAR(quat.w(), w, EPSILON);
}

TEST(ScrewConstraints, basicConstructor)
{
  // Run some basic constructor tests
  affordance_primitives::ChainedScrews chain1;
  EXPECT_EQ(chain1.size(), 0);
  EXPECT_EQ(chain1.axes().size(), 0);
  EXPECT_EQ(chain1.startPhi().size(), 0);
  EXPECT_EQ(chain1.goalPhi().size(), 0);
  EXPECT_EQ(chain1.lowerBounds().size(), 0);
  EXPECT_EQ(chain1.upperBounds().size(), 0);
  EXPECT_EQ(chain1.referenceFrame().translation(), Eigen::Vector3d::Zero());
  EXPECT_EQ(chain1.referenceFrame().linear(), Eigen::Matrix3d::Identity());

  // Add two axes
  affordance_primitives::ScrewStamped screw_msg;
  screw_msg.is_pure_translation = false;
  screw_msg.axis.x = 1;
  screw_msg.origin.z = 1;
  ASSERT_NO_THROW(chain1.addScrewAxis(screw_msg, -1, 1));

  screw_msg.is_pure_translation = true;
  screw_msg.origin.x = 0.2;
  affordance_primitives::ScrewAxis screw_axis;
  screw_axis.setScrewAxis(screw_msg);
  ASSERT_NO_THROW(chain1.addScrewAxis(screw_msg, -1, 1));

  // Check everything went smoothly
  EXPECT_EQ(chain1.size(), 2);
  EXPECT_EQ(chain1.axes().size(), 2);
  EXPECT_EQ(chain1.startPhi().size(), 2);
  EXPECT_EQ(chain1.goalPhi().size(), 2);
  EXPECT_EQ(chain1.lowerBounds().size(), 2);
  EXPECT_EQ(chain1.upperBounds().size(), 2);
  EXPECT_NEAR(chain1.axes()[1].getAxis().x(), 1.0, EPSILON);
  EXPECT_NEAR(chain1.lambdaMax(), 4.0, EPSILON);
}

TEST(ScrewConstraints, complexConstructor)
{
  // Add two axes
  std::vector<affordance_primitives::ScrewStamped> axes;
  affordance_primitives::ScrewStamped screw_msg;
  screw_msg.is_pure_translation = false;
  screw_msg.axis.x = 1;
  screw_msg.origin.z = 1;
  axes.push_back(screw_msg);

  screw_msg.is_pure_translation = true;
  screw_msg.origin.x = 0.2;
  axes.push_back(screw_msg);

  std::vector<double> lower, upper;
  lower.push_back(-1);
  upper.push_back(1);
  lower.push_back(0);
  upper.push_back(1.3);

  Eigen::Isometry3d tf_m_to_s;
  tf_m_to_s.translation().x() = 0.4;

  // Run some basic constructor tests
  affordance_primitives::ChainedScrews chain1(axes, lower, upper, tf_m_to_s);
  EXPECT_EQ(chain1.size(), 2);
  EXPECT_EQ(chain1.axes().size(), 2);
  EXPECT_EQ(chain1.startPhi().size(), 2);
  EXPECT_EQ(chain1.goalPhi().size(), 2);
  EXPECT_EQ(chain1.lowerBounds().size(), 2);
  EXPECT_EQ(chain1.upperBounds().size(), 2);
  EXPECT_NEAR(chain1.startPhi().at(0), chain1.lowerBounds().at(0), EPSILON);
  EXPECT_NEAR(chain1.goalPhi().at(0), chain1.upperBounds().at(0), EPSILON);
  EXPECT_NEAR(chain1.referenceFrame().translation().x(), 0.4, EPSILON);
  EXPECT_NEAR(chain1.lambdaMax(), 3.3, EPSILON);
}

TEST(ScrewConstraints, lambdaPhiConversions)
{
  // Add two axes
  std::vector<affordance_primitives::ScrewStamped> axes;
  affordance_primitives::ScrewStamped screw_msg;
  screw_msg.is_pure_translation = false;
  screw_msg.axis.x = 1;
  screw_msg.origin.z = 1;
  axes.push_back(screw_msg);
  axes.push_back(screw_msg);
  axes.push_back(screw_msg);

  std::vector<double> lower{-1, 0, -1};
  std::vector<double> upper{2, 2, 0};

  // Make the object
  affordance_primitives::ChainedScrews chain1(axes, lower, upper, Eigen::Isometry3d::Identity());

  // Test getting lambda with and without the index
  std::vector<double> phi1{0, 0, -1};
  double lambda_out;
  ASSERT_NO_THROW(lambda_out = chain1.getLambda(phi1));
  EXPECT_NEAR(lambda_out, 1.0, EPSILON);

  size_t s_index;
  ASSERT_NO_THROW(lambda_out = chain1.getLambda(phi1, &s_index));
  EXPECT_NEAR(lambda_out, 1.0, EPSILON);
  EXPECT_EQ(s_index, 0);

  // Now go the other way with and without index
  std::vector<double> phi_out;
  ASSERT_NO_THROW(phi_out = chain1.getPhi(lambda_out));
  checkVector(phi_out, 0, 0, -1);

  ASSERT_NO_THROW(phi_out = chain1.getPhi(lambda_out, &s_index));
  checkVector(phi_out, 0, 0, -1);
  EXPECT_EQ(s_index, 0);

  // Try other cases
  auto phi2 = lower;
  ASSERT_NO_THROW(lambda_out = chain1.getLambda(phi2, &s_index));
  EXPECT_NEAR(lambda_out, 0.0, EPSILON);
  EXPECT_EQ(s_index, 0);
  ASSERT_NO_THROW(phi_out = chain1.getPhi(lambda_out, &s_index));
  checkVector(phi_out, -1, 0, -1);
  EXPECT_EQ(s_index, 0);

  std::vector<double> phi3{2, 1, -1};
  ASSERT_NO_THROW(lambda_out = chain1.getLambda(phi3, &s_index));
  EXPECT_NEAR(lambda_out, 4.0, EPSILON);
  EXPECT_EQ(s_index, 1);
  ASSERT_NO_THROW(phi_out = chain1.getPhi(lambda_out, &s_index));
  checkVector(phi_out, 2, 1, -1);
  EXPECT_EQ(s_index, 1);

  std::vector<double> phi4{2, 2, -0.5};
  ASSERT_NO_THROW(lambda_out = chain1.getLambda(phi4, &s_index));
  EXPECT_NEAR(lambda_out, 5.5, EPSILON);
  EXPECT_EQ(s_index, 2);
  ASSERT_NO_THROW(phi_out = chain1.getPhi(lambda_out, &s_index));
  checkVector(phi_out, 2, 2, -0.5);
  EXPECT_EQ(s_index, 2);
}

TEST(ScrewConstraints, getPose)
{
  // Add two axes
  std::vector<affordance_primitives::ScrewStamped> axes;
  affordance_primitives::ScrewStamped screw_msg;
  screw_msg.is_pure_translation = false;
  screw_msg.axis.z = 1;
  screw_msg.origin.x = 1;
  axes.push_back(screw_msg);

  screw_msg.is_pure_translation = true;
  axes.push_back(screw_msg);

  std::vector<double> lower{0, 0};
  std::vector<double> upper{0.5 * M_PI, 0.2};

  // Create object
  affordance_primitives::ChainedScrews chain1(axes, lower, upper, Eigen::Isometry3d::Identity());

  // Check both calls with bogus input
  Eigen::Isometry3d tf_out;
  std::vector<double> phi_bogus{0, 0, 9};
  double lambda_bogus = 100;
  ASSERT_NO_THROW(tf_out = chain1.getPose(phi_bogus));
  EXPECT_EQ(tf_out.translation(), Eigen::Vector3d::Zero());
  EXPECT_EQ(tf_out.linear(), Eigen::Matrix3d::Identity());
  ASSERT_NO_THROW(tf_out = chain1.getPose(lambda_bogus));
  EXPECT_EQ(tf_out.translation(), Eigen::Vector3d::Zero());
  EXPECT_EQ(tf_out.linear(), Eigen::Matrix3d::Identity());

  // Check the start (should be I)
  double lambda_in = 0;
  ASSERT_NO_THROW(tf_out = chain1.getPose(lambda_in));
  EXPECT_EQ(tf_out.translation(), Eigen::Vector3d::Zero());
  EXPECT_EQ(tf_out.linear(), Eigen::Matrix3d::Identity());

  // Check after first screw and before second
  // Should just be a rotation
  lambda_in = upper[0];
  ASSERT_NO_THROW(tf_out = chain1.getPose(lambda_in));
  checkVector(tf_out.translation(), 1, -1, 0);
  checkVector(tf_out.linear().col(0), 0, 1, 0);
  checkVector(tf_out.linear().col(1), -1, 0, 0);
  checkVector(tf_out.linear().col(2), 0, 0, 1);

  // Check at the end
  // Should be the same rotation, translated up a bit
  lambda_in += upper[1];
  ASSERT_NO_THROW(tf_out = chain1.getPose(lambda_in));
  checkVector(tf_out.translation(), 1, -1, 0.2);
  checkVector(tf_out.linear().col(0), 0, 1, 0);
  checkVector(tf_out.linear().col(1), -1, 0, 0);
  checkVector(tf_out.linear().col(2), 0, 0, 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
