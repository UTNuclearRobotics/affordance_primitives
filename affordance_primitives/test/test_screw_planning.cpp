#include <gtest/gtest.h>

#include <affordance_primitives/screw_planning/screw_planning.hpp>

const std::string LINK_NAME = "link_name";
constexpr double EPSILON = 1e-4;

void nearVectors(const Eigen::Vector3d & vec1, const Eigen::Vector3d & vec2)
{
  EXPECT_NEAR(vec1.x(), vec2.x(), EPSILON);
  EXPECT_NEAR(vec1.y(), vec2.y(), EPSILON);
  EXPECT_NEAR(vec1.z(), vec2.z(), EPSILON);
}

TEST(ScrewAxis, test_encode_decode)
{
  // Create a screw axis to encode into message form
  const Eigen::Vector3d axis(1, 1.5, 0.6);
  const Eigen::Vector3d origin(0, 0.5, -2);
  const double pitch = 0.2;
  Eigen::Isometry3d origin_tf;
  origin_tf.translation() = origin;
  affordance_primitives::ScrewAxis input_screw("planning_frame", false);
  input_screw.setScrewAxis(origin_tf, axis, pitch);

  // Create a start pose
  const Eigen::Vector3d start_origin(-5, -3.5, 1);
  Eigen::Isometry3d start_pose;
  start_pose.translation() = start_origin;
  start_pose.linear() = (Eigen::Quaterniond(0.707, 0.707, 0, 0).normalized()).toRotationMatrix();

  // This information will be encoded into a moveit constraints message
  moveit_msgs::msg::Constraints constraints_msg;
  ASSERT_TRUE(affordance_primitives::encodeConstraintsMsg(
    input_screw, start_pose, LINK_NAME, constraints_msg));

  // Now we should be able to decode it and check they match
  affordance_primitives::ScrewAxis output_screw;
  Eigen::Isometry3d output_start_pose;
  std::string output_link_name;
  ASSERT_TRUE(affordance_primitives::decodeConstraintsMsg(
    constraints_msg, output_screw, output_start_pose, output_link_name));

  // Extract some info
  const Eigen::AngleAxisd aa_start(start_pose.linear());
  const Eigen::AngleAxisd aa_out(output_start_pose.linear());

  // The decoding transforms the screw message to be in the starting pose, transform it back
  auto tfed_screw_msg =
    affordance_primitives::transformScrew(output_screw.toMsg(), start_pose.inverse());
  output_screw.setScrewAxis(tfed_screw_msg);

  // Verify equality
  EXPECT_EQ(LINK_NAME, output_link_name);
  nearVectors(start_pose.translation(), output_start_pose.translation());
  EXPECT_NEAR(aa_start.angle(), aa_out.angle(), EPSILON);
  nearVectors(aa_start.axis(), aa_out.axis());
  nearVectors(input_screw.getAxis(), output_screw.getAxis());
  nearVectors(input_screw.getLinearVector(), output_screw.getLinearVector());
  EXPECT_NEAR(input_screw.getPitch(), output_screw.getPitch(), EPSILON);
  nearVectors(input_screw.getQVector(), output_screw.getQVector());
  EXPECT_EQ(input_screw.isPureTranslation(), output_screw.isPureTranslation());

  // Repeat the encode-decode test, but with a pure translation case
  affordance_primitives::ScrewAxis input_screw2("planning_frame", true);
  input_screw2.setScrewAxis(origin_tf, axis);

  // Encode-decode
  moveit_msgs::msg::Constraints constraints_msg2;
  ASSERT_TRUE(affordance_primitives::encodeConstraintsMsg(
    input_screw2, start_pose, LINK_NAME, constraints_msg2));
  ASSERT_TRUE(affordance_primitives::decodeConstraintsMsg(
    constraints_msg2, output_screw, output_start_pose, output_link_name));

  EXPECT_EQ(input_screw2.isPureTranslation(), output_screw.isPureTranslation());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
