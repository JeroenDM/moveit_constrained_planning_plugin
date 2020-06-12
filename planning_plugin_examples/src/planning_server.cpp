#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/GetMotionPlan.h>

/** Make this ugly long type a bit shorter. */
typedef boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> ClassLoaderSPtr;

const std::string ROBOT_DESCRIPTION = "robot_description";
const std::string BASE_CLASS = "planning_interface::PlannerManager";
const std::string PLANNING_PLUGIN = "compl_interface/COMPLPlanner";

void loadPlanningPlugin(ClassLoaderSPtr& planner_plugin_loader, planning_interface::PlannerManagerPtr& planner_instance,
                        robot_model::RobotModelPtr& robot_model, ros::NodeHandle& node_handle)
{
  try
  {
    planner_plugin_loader.reset(
        new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", BASE_CLASS));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
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
}

class Planner
{
public:
  Planner(const std::string& robot_description = "robot_description")
  {
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(robot_description);
    robot_model_ = robot_model_loader_->getModel();
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    loadPlanningPlugin(planner_plugin_loader_, planner_instance_, robot_model_, nh_);

    service_ = nh_.advertiseService("ompl_constrained_planning", &Planner::callback, this);
  }

  bool callback(moveit_msgs::GetMotionPlan::Request& req, moveit_msgs::GetMotionPlan::Response& res)
  {
    ROS_INFO_STREAM("Request: " << req);
    planning_interface::MotionPlanRequest request(req.motion_plan_request);
    planning_interface::MotionPlanResponse response;
    auto context = planner_instance_->getPlanningContext(planning_scene_, request, response.error_code_);
    bool success{ false };
    if (context)
    {
      ROS_INFO_STREAM("Planning context for joint goal created.");
      success = context->solve(response);
      if (response.trajectory_)
      {
        ROS_INFO_STREAM("Path found: " << response.trajectory_);
      }
    }
    else
    {
      ROS_INFO_STREAM("Failed to load planning context.");
    }
    response.getMessage(res.motion_plan_response);
    return true;
  }

private:
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  ClassLoaderSPtr planner_plugin_loader_;
  planning_interface::PlannerManagerPtr planner_instance_;
  ros::ServiceServer service_;
  ros::NodeHandle nh_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "constrained_planning_server");
  ros::AsyncSpinner async_spinner(4);
  async_spinner.start();
  //   ros::AsyncSpinner spinner(2);
  //   spinner.start();
  // ros::NodeHandle node_handle("~");

  Planner planner(ROBOT_DESCRIPTION);

  ROS_INFO("Ready to receive planning requests for constrained planning with OMPL.");

  ros::waitForShutdown();

  return 0;
}