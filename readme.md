# Constrained planning plugin for MoveIt, using OMPL

This repository is supposed to become an MVP to validate that we can use the constrained planning stuff in OMPL through a simple interface.
I'm not sure if it will work.

## Setting up the workspace

## Running the examples

### Panda robot
Most of the examples use the [panda_moveit_config](http://wiki.ros.org/panda_moveit_config) robot. You can find the specific settings that go with a robot at the top of a source file:
```C++
const std::string FIXED_FRAME = "panda_link0";
const std::string PLANNING_GROUP = "panda_arm";
```

So launching the MoveIt interface for the corresponding robot and running the example should work:
```bash
roslaunch panda_moveit_config demo.launch
```
and run the specific example node, for example this one to test the planning plugin:
```bash
rosrun planning_plugin_examples compl_planner_example
```
or this one to run some Jacobian calculation experiments:
```
rosrun planning_plugin_examples try_jacobian_projection
```


## Notes

I would like to separate the MoveIt and OMPL code as much as possible. So someone who understands MoveIt but not OMPL can understand the MoveIt part and visa versa.

## TODO

- Use `Eigen::Isometry::linear()` instead of `Eigen::Isometry3d::rotation()` whenever possible. This is more efficient (and can be used when the rotation matrix is known to be valid?).
