#include <affordance_primitives/empty_parameter_manager.hpp>
#include <pluginlib/class_list_macros.h>

namespace affordance_primitives
{
EmptyParameterManager::EmptyParameterManager()
{
}

void EmptyParameterManager::initialize(const ros::NodeHandle& nh)
{
  nh_ = nh;
}

std::pair<bool, std::string>
EmptyParameterManager::setParameters(const affordance_primitives::AffordanceParameter& params)
{
  return std::make_pair(true, "");
};

}  // namespace affordance_primitives

PLUGINLIB_EXPORT_CLASS(affordance_primitives::EmptyParameterManager, affordance_primitives::ParameterManager)
