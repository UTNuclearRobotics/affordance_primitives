#include <pluginlib/class_list_macros.h>

// Parameter manager
#include <affordance_primitives/empty_parameter_manager.hpp>
#include <affordance_primitives/parameter_manager.hpp>
PLUGINLIB_EXPORT_CLASS(affordance_primitives::EmptyParameterManager, affordance_primitives::ParameterManager)

// Task estimator
#include <affordance_primitives/kinematic_task_estimator.hpp>
#include <affordance_primitives/task_estimator.hpp>
PLUGINLIB_EXPORT_CLASS(affordance_primitives::KinematicTaskEstimator, affordance_primitives::TaskEstimator)
