#include <affordance_primitives/task_monitor/task_monitor.hpp>

namespace affordance_primitives
{
TaskMonitor::TaskMonitor(const ros::NodeHandle& nh, const std::string ft_topic_name) : nh_(nh), result_(STOP_REQUESTED)
{
  ft_sub_ = nh_.subscribe(ft_topic_name, 1, &TaskMonitor::ftCB, this);
};

TaskMonitor::~TaskMonitor()
{
  stopMonitor();
}

void TaskMonitor::startMonitor(const AffordanceParameter& parameters, const double timeout)
{
  {
    // Save variables for starting the move
    const std::lock_guard<std::mutex> lock(mutex_);
    parameters_ = parameters;
    continue_monitoring_ = true;

    // Reset the result
    result_ = INVALID_RESULT;

    // Set end time if passed a timeout parameter
    if (timeout > 0)
    {
      has_timeout_ = true;
      end_time_ = ros::Time::now() + ros::Duration(timeout);
    }
    else
    {
      has_timeout_ = false;
    }
  }

  if (!active_monitor_)
  {
    // Start the monitoring
    if (thread_.joinable())
    {
      thread_.join();
    }
    thread_ = std::thread([this] { mainLoop(); });
  }

  // Wait for monitor to go active
  std::unique_lock<std::mutex> lk(mutex_);
  condition_variable_.wait(lk, [this] { return active_monitor_ == true; });
}

ExecutionResult TaskMonitor::waitForResult()
{
  // All we really have to do is wait for the thread to finish
  if (thread_.joinable())
  {
    thread_.join();
  }

  return result_;
}

std::optional<ExecutionResult> TaskMonitor::getResult()
{
  // If the monitor is still running, get outta here
  if (active_monitor_)
  {
    return std::nullopt;
  }

  // Otherwise, just get the ready result
  return result_;
}

void TaskMonitor::stopMonitor()
{
  // Request a stop and wait for it to happen
  continue_monitoring_ = false;
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void TaskMonitor::ftCB(const WrenchStamped& msg)
{
  last_wrench_msgs_.push(msg);
}

std::optional<ExecutionResult> TaskMonitor::checkForViolaton(const AffordanceParameter& params)
{
  while (!last_wrench_msgs_.empty())
  {
    auto wrench = last_wrench_msgs_.front().wrench;
    last_wrench_msgs_.pop();
    const double forces_sum_sqs = pow(wrench.force.x, 2) + pow(wrench.force.y, 2) + pow(wrench.force.z, 2);
    const double torques_sum_sqs = pow(wrench.torque.x, 2) + pow(wrench.torque.y, 2) + pow(wrench.torque.z, 2);
    if (forces_sum_sqs > pow(params.max_force, 2) || torques_sum_sqs > pow(params.max_torque, 2))
    {
      return FT_VIOLATION;
    }

    if (fabs(wrench.force.x) > fabs(params.max_wrench.trans_x) ||
        fabs(wrench.force.y) > fabs(params.max_wrench.trans_y) ||
        fabs(wrench.force.z) > fabs(params.max_wrench.trans_z) ||
        fabs(wrench.torque.x) > fabs(params.max_wrench.rot_x) ||
        fabs(wrench.torque.y) > fabs(params.max_wrench.rot_y) || fabs(wrench.torque.z) > fabs(params.max_wrench.rot_z))
    {
      return FT_VIOLATION;
    }
  }
  return std::nullopt;
}

void TaskMonitor::mainLoop()
{
  ros::Rate loop_rate(100);

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    active_monitor_ = true;
    condition_variable_.notify_all();
  }

  // Clear wrench queue before starting
  last_wrench_msgs_ = std::queue<WrenchStamped>();

  while (ros::ok() && continue_monitoring_)
  {
    ros::spinOnce();
    {
      const std::lock_guard<std::mutex> lock(mutex_);

      // Check if F/T violated limits
      auto ft_result = checkForViolaton(parameters_);
      if (ft_result.has_value())
      {
        result_ = ft_result.value();
        break;
      }

      // Check if we have passed the end time
      if (has_timeout_ && ros::Time::now() > end_time_)
      {
        result_ = TIME_OUT;
        break;
      }
    }
    loop_rate.sleep();
  }

  // Check for a stop requested
  if (!continue_monitoring_)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    result_ = STOP_REQUESTED;
  }

  // Set flag saying result is ready
  active_monitor_ = false;
}
}  // namespace affordance_primitives
