#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>

int main(int argc, char** argv)
{
  const std::string NODE_NAME = "compl_example";  // also used for logging
  const std::string PLANNING_GROUP = "panda_arm";
  const std::string ROBOT_DESCRIPTION = "robot_description";
  const std::string BASE_CLASS = "planning_interface::PlannerManager";
  const std::string PLANNING_PLUGIN = "compl_interface/COMPLPlanner";

  ros::init(argc, argv, NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
  robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  // I should probably use the planning scene monitor here
  // but I do not uderstand the planning scene monitor engough to use it.
  // planning_scene::PlanningScene planning_scene(robot_model);
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  // std::string planner_plugin_name;

  planner_plugin_loader.reset(
      new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", BASE_CLASS));

  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(PLANNING_PLUGIN));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << PLANNING_PLUGIN << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }

  // setup joint space goal
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  req.group_name = "panda_arm";

  // create start
  robot_state::RobotState start_state(robot_model);
  std::vector<double> start_joint_values{ 0.666988104319289,   -0.9954030434136065, -1.1194235704518019,
                                          -1.9946073045682555, -2.772101772642487,  3.4631937276027194,
                                          -1.2160652080175647 };
  start_state.setJointGroupPositions(joint_model_group, start_joint_values);
  moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);
 
  // create goal
  robot_state::RobotState goal_state(robot_model);
  std::vector<double> goal_joint_values{ 1.7301680303369467, -0.7342165592762893, -0.5358506493073328,
                                         -2.214051132383283, -1.9148221683474542, 1.8324940020482856,
                                         -1.588014538557859 };
  goal_state.setJointGroupPositions(joint_model_group, goal_joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  auto context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

  if (context)
  {
    ROS_INFO_STREAM("Planning context for joint goal created.");
    auto success = context->solve(res);
    if (res.trajectory_)
    {
      ROS_INFO_STREAM("Path found: " << res.trajectory_->getFirstWayPoint());
    }
  }
  else
  {
    ROS_INFO_STREAM("Failed to load planning context.");
  }
}