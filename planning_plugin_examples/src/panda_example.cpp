#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // toMsg(...)

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include <planning_plugin_examples/example_util.h>

/** Change this parameters for different robots or planning plugins. */
const std::string FIXED_FRAME = "panda_link0";
const std::string PLANNING_GROUP = "panda_arm";

// const std::string FIXED_FRAME = "base_link";
// const std::string PLANNING_GROUP = "manipulator";

const std::string ROBOT_DESCRIPTION = "robot_description";
const std::string BASE_CLASS = "planning_interface::PlannerManager";
const std::string PLANNING_PLUGIN = "compl_interface/COMPLPlanner";
// const std::string PLANNING_PLUGIN = "ompl_interface/OMPLPlanner";

/** Create point-topoint planning problem.
 *
 * Move from a joint space start to goal configurations. These configurations
 * are chooses specifically to move the end-effector from position
 * [0.3, -0.3, 0.6] to position [0.3, 0.3, 0.7].
 *
 * This makes it easier to come up with path constraints for which the start and goal are valid.
 *
 * */
planning_interface::MotionPlanRequest createPTPProblem(robot_model::RobotModelPtr& robot_model,
                                                       const robot_state::JointModelGroup* joint_model_group)
{
  planning_interface::MotionPlanRequest req;

  req.group_name = PLANNING_GROUP;

  // create start
  Eigen::Isometry3d start_pose =
      Eigen::Translation3d(0.3, -0.3, 0.6) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
  robot_state::RobotState start_state(robot_model);
  std::vector<double> start_joint_values{ 0.666988104319289,   -0.9954030434136065, -1.1194235704518019,
                                          -1.9946073045682555, -2.772101772642487,  3.4631937276027194,
                                          -1.2160652080175647 };
  // start_state.setJointGroupPositions(joint_model_group, start_joint_values);

  bool success = start_state.setFromIK(joint_model_group, start_pose);
  ROS_INFO_STREAM("Start pose IK: " << (success ? "succeeded." : "failed."));

  moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

  // create goal
  Eigen::Isometry3d goal_pose =
      Eigen::Translation3d(0.3, 0.3, 0.7) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
  robot_state::RobotState goal_state(robot_model);
  std::vector<double> goal_joint_values{ 1.7301680303369467, -0.7342165592762893, -0.5358506493073328,
                                         -2.214051132383283, -1.9148221683474542, 1.8324940020482856,
                                         -1.588014538557859 };
  // goal_state.setJointGroupPositions(joint_model_group, goal_joint_values);
  goal_state.setFromIK(joint_model_group, goal_pose);
  ROS_INFO_STREAM("Goal pose IK: " << (success ? "succeeded." : "failed."));
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // path constraints on end-effector
  shape_msgs::SolidPrimitive box_constraint;
  box_constraint.type = shape_msgs::SolidPrimitive::BOX;
  // box_constraint.dimensions = { 1e-6, 0.6, 0.1 }; /* use -1 to indicate no constraints. */
  box_constraint.dimensions = { 0.01, 0.6, 0.1 }; /* use -1 to indicate no constraints. */

  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0.3;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.65;
  box_pose.orientation.w = 1.0;

  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = FIXED_FRAME;
  position_constraint.link_name = joint_model_group->getLinkModelNames().back(); /* end-effector link */
  position_constraint.constraint_region.primitives.push_back(box_constraint);
  position_constraint.constraint_region.primitive_poses.push_back(box_pose);

  // orientation constraints
  tf2::Quaternion desired_orientation;
  desired_orientation.setRPY(0, M_PI_2, 0); /** Todo, use intrinsic xyz as in MoveIt's constraitns. */

  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = FIXED_FRAME;
  orientation_constraint.link_name = joint_model_group->getLinkModelNames().back(); /* end-effector link */
  orientation_constraint.orientation = tf2::toMsg(desired_orientation);
  orientation_constraint.absolute_x_axis_tolerance = 0.1;
  orientation_constraint.absolute_y_axis_tolerance = 0.1;
  orientation_constraint.absolute_z_axis_tolerance = -1.0;

  req.path_constraints.position_constraints.push_back(position_constraint);
  //   req.path_constraints.orientation_constraints.push_back(orientation_constraint);

  req.allowed_planning_time = 5.0;
  return req;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compl_example");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // BEGIN_TUTORIAL
  // Planning scene setup
  // ^^^^^

  // The usual spiel to setup all MoveIt objects to manage robot state and planning scene
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

  // Load the planning plugin
  // ^^^^^^^^^^^^^^^^^^^^^^^^

  ClassLoaderSPtr planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  loadPlanningPlugin(planner_plugin_loader, planner_instance, robot_model, node_handle, BASE_CLASS, PLANNING_PLUGIN);

  // Create a motion planning request
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  auto req = createPTPProblem(robot_model, joint_model_group);
  // auto req = createKukaProblem(robot_model, joint_model_group);
  planning_interface::MotionPlanResponse res;

  // Visualization
  // ^^^^^^^^^^^^^

  // All ugly book keeping for visuals is hidden inside the `Visuals` class.
  RvizVisuals visuals(FIXED_FRAME, node_handle);

  if (req.path_constraints.position_constraints.size() > 0)
  {
    visuals.showPositionConstraints(req.path_constraints.position_constraints[0]);
  }

  //   geometry_msgs::Pose nominal_pose_constraints;
  //   nominal_pose_constraints.position.x = 0.9;
  //   nominal_pose_constraints.position.y = 0.0;
  //   nominal_pose_constraints.position.z = 0.2;
  //   tf2::Quaternion nominal_quat;
  //   nominal_quat.setRPY(0, M_PI_2, 0);
  //   nominal_pose_constraints.orientation = tf2::toMsg(nominal_quat);

  //   visuals.rvt_->publishAxis(nominal_pose_constraints);
  //   visuals.rvt_->publishCuboid(nominal_pose_constraints, 0.05, 0.4, 0.05, rviz_visual_tools::GREEN);
  visuals.rvt_->publishRobotState(req.start_state.joint_state.position, joint_model_group);
  visuals.rvt_->trigger();

  // Solve the problem
  // ^^^^^^^^^^^^^^^^^
  auto context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  bool success{ false };
  if (context)
  {
    ROS_INFO_STREAM("Planning context for joint goal created.");
    success = context->solve(res);
    if (res.trajectory_)
    {
      ROS_INFO_STREAM("Path found: " << res.trajectory_);
    }
  }
  else
  {
    ROS_INFO_STREAM("Failed to load planning context.");
  }

  if (res.trajectory_)
  {
    // The path is interpolated by OMPL (taking the constraints into account).
    // So we should have quite a number of waypoints.
    ROS_INFO_STREAM(res.trajectory_->getWayPointCount());

    visuals.displaySolution(res, joint_model_group);

    // END_TUTORIAL
  }

  ros::shutdown();
  return 0;
}