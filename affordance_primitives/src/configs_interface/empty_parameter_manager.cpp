#include <affordance_primitives/configs_interface/empty_parameter_manager.hpp>
#include <pluginlib/class_list_macros.h>

namespace affordance_primitives
{
void EmptyParameterManager::initialize(const ros::NodeHandle& nh)
{
  nh_ = nh;
}

std::pair<bool, std::string> EmptyParameterManager::setParameters(const affordance_primitives::APRobotParameter& params)
{
  return std::make_pair(true, "");
};

}  // namespace affordance_primitives

PLUGINLIB_EXPORT_CLASS(affordance_primitives::EmptyParameterManager, affordance_primitives::ParameterManager)
