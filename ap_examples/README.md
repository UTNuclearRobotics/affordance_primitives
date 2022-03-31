# Affordance Primitive Examples

This package serves as basic examples for using Affordance Primitives, including correctly using `pluginlib` to create your own custom plugins.

The source files are intended to be the documentation and are well commented. The components of this package include:
  1. `ap_executor`: sets up an AP executor (action server) and shows how to access the output (commanded velocity and wrench)
  2. `ap_client`: sets up an AP (action) client, covers some of the parameters in an AP, then calls the action

## Basic Demonstration
To run the basic AP action examples, you will need 3 terminals. Run the following:

```sh
roscore
```
```sh
rosrun ap_examples ap_executor
```
```sh
rosrun ap_examples ap_client
```

This should create an AP action server and client, then call a basic AP execution
