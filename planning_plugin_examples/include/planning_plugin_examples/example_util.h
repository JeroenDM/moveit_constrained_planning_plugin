#ifndef CONSTRAINED_PLANNING_EXAMPLE_UTIL
#define CONSTRAINED_PLANNING_EXAMPLE_UTIL

#include <string>
#include <Eigen/Geometry>

#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model/robot_model.h>

/** Make this ugly long type a bit shorter. */
typedef boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> ClassLoaderSPtr;

/** Everyting that has to do with visualization in Rviz
 * is grouped in this class.
 * */
class RvizVisuals
{
public:
  RvizVisuals(const std::string& reference_frame, ros::NodeHandle& node_handle)
  {
    rvt_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(reference_frame, "/visualization_marker_array");
    rvt_->loadRobotStatePub("/display_robot_state");
    rvt_->enableBatchPublishing();
    ros::Duration(0.1).sleep();
    rvt_->deleteAllMarkers();
    rvt_->trigger();
    display_publisher =
        node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  }

  /** publish a green box at the nominal position, with dimensions
   * according to the tolerances.
   *
   * Note: the nominal orientation of position constraints is not a thing yet.
   * */
  void showPositionConstraints(moveit_msgs::PositionConstraint pos_con)
  {
    auto dims = pos_con.constraint_region.primitives.at(0).dimensions;
    // if dim = -1 -> make it large (could be workspace size in the future.)
    const double UNBOUNDED_SIZE{ 3.0 };
    for (auto& dim : dims)
    {
      if (dim == -1.0)
        dim = UNBOUNDED_SIZE;
    }
    rvt_->publishCuboid(pos_con.constraint_region.primitive_poses.at(0), dims.at(0), dims.at(1), dims.at(2),
                        rviz_visual_tools::GREEN);
    rvt_->trigger();
  }

  /** TODO change name. **/
  void plotPose(const Eigen::Isometry3d pose)
  {
    rvt_->publishAxis(pose, rviz_visual_tools::LARGE);
    rvt_->trigger();
  }

  /** Display trajectory using the DisplayTrajectory publisher and
   * show end-effector path using moveit visual tools.
   * */
  void displaySolution(planning_interface::MotionPlanResponse res,
                       const robot_state::JointModelGroup* joint_model_group)
  {
    moveit_msgs::DisplayTrajectory display_trajectory;

    // /* Visualize the trajectory */
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    rvt_->publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    rvt_->trigger();
    display_publisher.publish(display_trajectory);
  }

  moveit_visual_tools::MoveItVisualToolsPtr rvt_;
  ros::Publisher display_publisher;
};

/** Inverse of the Conversion matrix from roll-pitch-yaw velocity to angular velocity.
 *
 * w = B(rpy) * rpy_dot
 * w = angular velocity, B = matrix returned by this function,
 * rpy = roll-pitch-yaw angles
 * rpy_dot = roll-pitch-yaw time derivatives.
 *
 * This function directly calculates B^-1
 * and contains a singularity for ry = +/- pi / 2
 *
 * from:
 * https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
 *
 * */
Eigen::Matrix3d angularVelocityToRPYRates(double rx, double ry)
{
  double TOLERANCE{ 1e-9 }; /* TODO what tolerance to use here? */
  Eigen::Matrix3d E;
  double cosy{ std::cos(ry) };

  // check for singular case
  if (std::abs(cosy) < TOLERANCE)
  {
    ROS_ERROR_STREAM("Singularity in orientation path constraints.");
  }

  double cosx{ std::cos(rx) };
  double sinx{ std::sin(rx) };
  double siny{ std::sin(ry) };
  E << 1, sinx * siny / cosy, -cosx * siny / cosy, 0, cosx, sinx, 0, -sinx / cosy, cosx / cosy;
  return E;
}

Eigen::Vector3d rotationToRPY(const Eigen::Matrix3d& r)
{
  Eigen::Vector3d xyz = r.eulerAngles(0, 1, 2);
  // in MoveIt some wrapping is done sometimes, but I'm not sure why or when to apply it yet
  // probably to convert the eigen convention: [0:pi]x[-pi:pi]x[-pi:pi] to a MoveIt convention?
  // but it looks the same to me
  // it does not seem to work in some cases.
  // xyz(0) = std::min(fabs(xyz(0)), M_PI - fabs(xyz(0)));
  // xyz(1) = std::min(fabs(xyz(1)), M_PI - fabs(xyz(1)));
  // xyz(2) = std::min(fabs(xyz(2)), M_PI - fabs(xyz(2)));
  return xyz;
}

Eigen::Vector3d poseToRPY(const Eigen::Isometry3d& p)
{
  return rotationToRPY(p.rotation());
}

Eigen::Vector3d poseToAA(const Eigen::Isometry3d& p)
{
  // TODO check the angle? see trajopt
  // https://github.com/ros-industrial-consortium/trajopt_ros/blob/a393abb99b221c97603ae40fcbd0f172f94a7fcc/trajopt/include/trajopt/utils.hpp#L161
  Eigen::AngleAxisd aa(p.rotation());
  return aa.axis() * aa.angle();
}

/** Get the sign of a number.
 * https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
 * */
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

/** Pendantic loading of the planning plugin
 *
 * Really usefull for debugging configuration issues.
 */
void loadPlanningPlugin(ClassLoaderSPtr& planner_plugin_loader, planning_interface::PlannerManagerPtr& planner_instance,
                        robot_model::RobotModelPtr& robot_model, ros::NodeHandle& node_handle,
                        const std::string& base_class, const std::string& plugin_name)
{
  try
  {
    planner_plugin_loader.reset(
        new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", base_class));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
    {
      ss << classes[i] << " ";
    }
    ROS_ERROR_STREAM("Exception while loading planner '" << plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }
}

#endif  // CONSTRAINED_PLANNING_EXAMPLE_UTIL