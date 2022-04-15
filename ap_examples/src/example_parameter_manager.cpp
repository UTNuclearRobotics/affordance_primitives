#include <ap_examples/example_parameter_manager.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace ap_examples
{
// Do initial setup. Here we just print
void ExampleParameterManager::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  RCLCPP_INFO_STREAM(node_->get_logger(), "ParameterManager initialization");
}

// Every time an AP is started, this will be called with the requested parameters
std::pair<bool, std::string> ExampleParameterManager::setParameters(
  const affordance_primitives::APRobotParameter & params)
{
  // If something failed to set, return false and the AP will not be attempted
  bool something_invalid = false;
  if (something_invalid) {
    // The string here is just an error message
    return std::make_pair(false, "Something bad happened!");
  }

  // Here we can call service clients, publish parameters, etc, to change the controller configuration
  // In the example, we will print the parameters and exit successfully
  RCLCPP_INFO_STREAM(node_->get_logger(), "Setting params");
  return std::make_pair(true, "");
};
}  // namespace ap_examples

// This exports the plugin so the executor can load it
PLUGINLIB_EXPORT_CLASS(
  ap_examples::ExampleParameterManager, affordance_primitives::ParameterManager)
