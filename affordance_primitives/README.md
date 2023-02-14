# Affordance Primitives

Affordance Primitives (APs) use screw theory to model contact-task affordances. So far, our work has concentrated on articulated objects (e.g, doors, valves, drawers, bolts), and an AP can represent almost task with these types of motions. An AP also considers the forces or torques that the manipulator must apply to perform the desired manipulation.

See [this video](https://www.youtube.com/watch?v=3TGl3F_4W_8) for an overview.

## Conceptual Overview
An AP is a constraint the environment places on a manipulator that generates instantaneous motion and force profiles the manipulator must follow. We consider these constrains by modeling how an articulated object must move during manipulation using screw theory.

# AP Framework
This package includes a library for performing AP moves with a generic manipulator. See the paper for more details. The major components of the framework are given below. Red nodes are plugins that allow customization
![ap_framework](doc/images/AP_Framework.png "Affordance Primitive Framework")

## Software Overview
The AP exector is implemented as a ROS action server. See the [ap_examples](../ap_examples/) package for basic examples on how to set up and use the action.

The AP executor requires a few parameters to initialize correctly. These are passed directly to the action server, and are **not** read from the ROS parameter server internally.
  - `param_manager_plugin_name`, which determines the plugin to use for the parameter manager. This class reads the AP action goal, and can set robot-specific parameters (e.g. admittance values) for a robot.
  - `task_estimator_plugin_name`, which determines the plugin used for the task estimator. This class tracks how far along a screw axis the robot has moved, and is responsible for determining when a task is completed. The default implemetation only uses robot kinematics to measure the task angle, but your fancy custom plugin can be used for higher fidelity estimates.
  - `monitor_ft_topic_name`. This is the name of the F/T sensor. If you do any processing on the data stream past the raw values from the sensor, you should use the processed topic here. This topic is monitored during the task, so the AP will be abandoned if the contact force with the environment is too high

When a goal is sent to the AP executor, it does not move the robot directly. Instead, it uses the action feedback topic to publish the calculated velocity and wrench the robot should follow. You will need to implement a callback on the feedback topic to direct these data streams to the correct place for your robot, see [ap_examples](../ap_examples/) directory for examples.
