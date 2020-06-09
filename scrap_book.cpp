/** attempt at cleaning up the example that is not working yet. */
class Example
{
public:
  Example(const std::string& planning_group)
  {
    robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_"
                                                                                                           "descriptio"
                                                                                                           "n"));
    robot_model_ = robot_model_loader_->getModel();
    robot_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model_));
    joint_model_group_ = robot_state_->getJointModelGroup(planning_group);
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    planning_scene_->getCurrentStateNonConst().setToDefaultValues(joint_model_group_, "ready");
  }

  const robot_state::JointModelGroup* getJointModelGroup()
  {
    return joint_model_group_;
  }

  bool loadPlanningPlugin(const std::string& plugin_name, ros::NodeHandle& node_handle)
  {
    const std::string BASE_CLASS = "planning_interface::PlannerManager";
    try
    {
      planner_plugin_loader_.reset(
          new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", BASE_CLASS));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
      planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance(plugin_name));
      if (!planner_instance_->initialize(robot_model_, node_handle.getNamespace()))
      {
        ROS_FATAL_STREAM("Could not initialize planner instance");
      }
      ROS_INFO_STREAM("Using planning interface '" << planner_instance_->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader_->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << plugin_name << "': " << ex.what() << std::endl
                                                           << "Available plugins: " << ss.str());
    }
  }

  planning_interface::MotionPlanRequest createPTPProblem()
  {
    planning_interface::MotionPlanRequest req;

    req.group_name = "panda_arm";

    // create start
    robot_state::RobotState start_state(robot_model_);
    std::vector<double> start_joint_values{ 0.666988104319289,   -0.9954030434136065, -1.1194235704518019,
                                            -1.9946073045682555, -2.772101772642487,  3.4631937276027194,
                                            -1.2160652080175647 };
    start_state.setJointGroupPositions(joint_model_group_, start_joint_values);
    moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

    // create goal
    robot_state::RobotState goal_state(robot_model_);
    std::vector<double> goal_joint_values{ 1.7301680303369467, -0.7342165592762893, -0.5358506493073328,
                                           -2.214051132383283, -1.9148221683474542, 1.8324940020482856,
                                           -1.588014538557859 };
    goal_state.setJointGroupPositions(joint_model_group_, goal_joint_values);
    moveit_msgs::Constraints joint_goal =
        kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group_);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    return req;
  }

  planning_interface::MotionPlanResponse solveProblem(planning_interface::MotionPlanRequest req)
  {
    planning_interface::MotionPlanResponse res;

    auto context = planner_instance_->getPlanningContext(planning_scene_, req, res.error_code_);
    bool success{ false };
    if (context)
    {
      ROS_INFO_STREAM("Planning context for joint goal created.");
      success = context->solve(res);
      if (success)
      {
        ROS_INFO_STREAM("Solved motion planning request.");
      }
      else
      {
        ROS_INFO_STREAM("Motion planning request failed.");
      }
    }
    else
    {
      ROS_INFO_STREAM("Failed to load planning context.");
    }

    return res;
  }

private:
  // planning setup components
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  planning_scene::PlanningScenePtr planning_scene_;

  // planner plugin
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader_;
  planning_interface::PlannerManagerPtr planner_instance_;
};