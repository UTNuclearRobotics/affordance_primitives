#include <math.h>

#include <affordance_primitives/ap_executor/ap_executor.hpp>
#include <affordance_primitives/screw_model/affordance_utils.hpp>

namespace affordance_primitives
{
void enforceAPLimits(
  const APRobotParameter & params, AffordancePrimitiveFeedback & feedback, const double epsilon)
{
  // Convert EE velocity twist and limits to Eigen
  Eigen::Matrix<double, 6, 1> twist_limit = CartesianFloatToVector(params.max_ee_velocity);
  Eigen::Matrix<double, 6, 1> twist_requested;
  tf2::fromMsg(feedback.moving_frame_twist.twist, twist_requested);

  // Apply twist limit
  double scale = 1.0;
  for (size_t i = 0; i < 6; i++) {
    if (fabs(twist_limit[i]) > epsilon && fabs(twist_requested[i] / twist_limit[i]) > 1) {
      scale = std::max(scale, fabs(twist_requested[i] / twist_limit[i]));
    }
  }
  twist_requested /= scale;

  // Convert wrench and limits to Eigen
  Eigen::Matrix<double, 6, 1> wrench_limit = CartesianFloatToVector(params.max_wrench);
  Eigen::Matrix<double, 6, 1> wrench_requested = WrenchToVector(feedback.expected_wrench.wrench);

  // Check for limit along each dimension
  for (size_t i = 0; i < 6; i++) {
    if (fabs(wrench_limit[i]) > epsilon) {
      if (wrench_requested[i] < -1 * wrench_limit[i] || wrench_requested[i] > wrench_limit[i]) {
        wrench_requested[i] = copysign(wrench_limit[i], wrench_requested[i]);
      }
    }
  }

  // Check for total magnitude
  if (fabs(params.max_force) > epsilon) {
    const double scale_force = wrench_requested.head(3).norm() / params.max_force;
    if (scale_force > 1.0) {
      wrench_requested.head(3) /= scale_force;
    }
  }
  if (fabs(params.max_torque) > epsilon) {
    const double scale_torque = wrench_requested.tail(3).norm() / params.max_torque;
    if (scale_torque > 1.0) {
      wrench_requested.tail(3) /= scale_torque;
    }
  }

  // Convert back to message
  feedback.moving_frame_twist.twist = tf2::toMsg(twist_requested);
  feedback.expected_wrench.wrench = VectorToWrench(wrench_requested);
}

APExecutor::APExecutor(rclcpp::Node::SharedPtr node, const std::string action_name) : node_(node)
{
  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<AffordancePrimitive>(
    node_, action_name, std::bind(&APExecutor::handle_goal, this, _1, _2),
    std::bind(&APExecutor::handle_cancel, this, _1),
    std::bind(&APExecutor::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse APExecutor::handle_goal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const AffordancePrimitive::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse APExecutor::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<AffordancePrimitive>> goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void APExecutor::handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<AffordancePrimitive>> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&APExecutor::execute, this, _1), goal_handle}.detach();
}

bool APExecutor::initialize(const ExecutorParameters & params)
{
  // Load ParameterManager
  pm_loader_ = std::make_shared<pluginlib::ClassLoader<ParameterManager>>(
    "affordance_primitives", "affordance_primitives::ParameterManager");
  try {
    parameter_manager_ = pm_loader_->createSharedInstance(params.param_manager_plugin_name);
    parameter_manager_->initialize(node_);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(
      node_->get_logger(), "Parameter Manager plugin failed to load, error was: %s", ex.what());
    return false;
  }

  // Load TaskEstimator
  te_loader_ = std::make_shared<pluginlib::ClassLoader<TaskEstimator>>(
    "affordance_primitives", "affordance_primitives::TaskEstimator");
  try {
    task_estimator_ = te_loader_->createSharedInstance(params.task_estimator_plugin_name);
    task_estimator_->initialize(node_);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(
      node_->get_logger(), "Task Estimator plugin failed to load, error was: %s", ex.what());
    return false;
  }

  // Create the execution monitor
  monitor_ = std::make_unique<TaskMonitor>(node_, params.monitor_ft_topic_name);

  // This handles the screw math
  screw_executor_ = std::make_unique<APScrewExecutor>(node_);

  return true;
}

AffordancePrimitiveResult APExecutor::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<AffordancePrimitive>> goal_handle)
{
  // End any currently running task
  stop();

  std::shared_ptr<AffordancePrimitiveResult> ap_result;
  auto goal = goal_handle->get_goal();

  // Update parameters
  if (!updateParams(goal->robot_params)) {
    ap_result->result = ap_result->PARAM_FAILURE;
    goal_handle->succeed(ap_result);
    return *ap_result;
  }

  // Check input for errors
  const double epislon = 1e-8;
  if (fabs(goal->theta_dot) < epislon) {
    RCLCPP_ERROR(node_->get_logger(), "No velocity set, skipping");
    ap_result->result = ap_result->PARAM_FAILURE;
    goal_handle->succeed(ap_result);
    return *ap_result;
  }
  if (fabs(goal->screw_distance) < epislon) {
    ap_result->result = ap_result->SUCCESS;
    goal_handle->succeed(ap_result);
    return *ap_result;
  }

  // Set current mode
  {
    const std::lock_guard<std::mutex> lock(mode_mutex_);
    current_mode_ = EXECUTING;
  }

  // Start the execution monitor
  const double timeout =
    2 * fabs(goal->screw_distance / goal->theta_dot);  // Allow 2x expected time
  monitor_->startMonitor(goal->robot_params, timeout);

  // Execute commands while monitoring
  rclcpp::Rate loop_rate(100);
  ap_result->result = ap_result->INVALID_RESULT;

  // Start estimating task
  if (!task_estimator_->resetTaskEstimation(0)) {
    stop();
    ap_result->result = ap_result->KIN_VIOLATION;
    goal_handle->succeed(ap_result);
    return *ap_result;
  }

  while (rclcpp::ok()) {
    // Check executor status
    {
      const std::lock_guard<std::mutex> lock(mode_mutex_);
      if (current_mode_ != EXECUTING || !goal_handle->is_active()) {
        ap_result->result = ap_result->STOP_REQUESTED;
        break;
      }
    }
    // Check if the monitor has ended
    std::optional<AffordancePrimitiveResult> monitor_result = monitor_->getResult();
    if (monitor_result.has_value()) {
      *ap_result = monitor_result.value();
      break;
    }
    // Otherwise, send commands
    else {
      std::shared_ptr<AffordancePrimitiveFeedback> ap_feedback;
      if (!screw_executor_->getScrewTwist(*goal, *ap_feedback)) {
        ap_result->result = ap_result->KIN_VIOLATION;
        break;
      }

      // Check if we have reached the goal
      std::optional<double> total_delta_theta = task_estimator_->estimateTaskAngle(*goal);
      if (!total_delta_theta.has_value()) {
        ap_result->result = ap_result->KIN_VIOLATION;
        break;
      } else if (total_delta_theta.value() >= fabs(goal->screw_distance)) {
        ap_result->result = ap_result->SUCCESS;
        break;
      }

      // Publish feedback
      ap_feedback->moving_frame_twist.header.stamp = node_->get_clock()->now();
      enforceAPLimits(goal->robot_params, *ap_feedback);
      goal_handle->publish_feedback(ap_feedback);

      loop_rate.sleep();
    }
  }

  // Cleanup
  stop();
  postExecuteReset(goal_handle);

  goal_handle->succeed(ap_result);
  return *ap_result;
}

void APExecutor::stop()
{
  // Stop the monitor
  if (monitor_) {
    monitor_->stopMonitor();
  }

  // Set the mode to inactive
  const std::lock_guard<std::mutex> lock(mode_mutex_);
  current_mode_ = INACTIVE;
}

bool APExecutor::updateParams(const APRobotParameter & parameters)
{
  auto set_params = parameter_manager_->setParameters(parameters);
  if (!set_params.first) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(), "Could not set parameters, error was: " << set_params.second);
    return false;
  }

  return true;
}

bool APExecutor::postExecuteReset(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<AffordancePrimitive>> goal_handle)
{
  // We want to publish a 0-motion twist just in case things are still moving
  std::shared_ptr<AffordancePrimitiveFeedback> ap_feedback;
  goal_handle->publish_feedback(ap_feedback);

  return true;
}
}  // namespace affordance_primitives
