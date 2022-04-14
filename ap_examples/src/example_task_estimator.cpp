#include <ap_examples/example_task_estimator.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace ap_examples
{
void ExampleTaskEstimator::initialize(rclcpp::Node::SharedPtr node) { node_ = node; }

// This function is called at the start of every AP move
bool ExampleTaskEstimator::resetTaskEstimation(double reset_val)
{
  start_time_ = node_->now();
  return true;
}

// This function is called every cycle during AP execution
// The return value is used to determine when the move has been completed
std::optional<double> ExampleTaskEstimator::estimateTaskAngle(
  const affordance_primitives::AffordancePrimitiveGoal & ap_goal)
{
  // You can return a std::nullopt to indicate the estimate is invalid
  bool something_is_invalid = false;
  if (something_is_invalid) {
    return std::nullopt;
  }

  // For this demonstration, we wait a bit, then send a high estimate
  if ((node_->now() - start_time_).nanoseconds() > 5e9) {
    return M_PI;
  } else {
    return 0.0;
  }
}
}  // namespace ap_examples

// Here we export the plugin so the executor can look it up and use it
PLUGINLIB_EXPORT_CLASS(ap_examples::ExampleTaskEstimator, affordance_primitives::TaskEstimator);
