#include <affordance_primitives/empty_parameter_manager.hpp>

namespace affordance_primitives
{
EmptyParameterManager::EmptyParameterManager()
{
}

void EmptyParameterManager::initialize(const ros::NodeHandle& nh)
{
  nh_ = ros::NodeHandle(nh);
}

std::pair<bool, std::string>
EmptyParameterManager::setParameters(const affordance_primitives::AffordanceParameter& params)
{
  return std::make_pair(true, "");
};

}  // namespace affordance_primitives

PLUGINLIB_EXPORT_CLASS(affordance_primitives::EmptyParameterManager, affordance_primitives::ParameterManager)
