#ifndef COMPL_INTERFACE_H
#define COMPL_INTERFACE_H

#include <memory>

#include <moveit/macros/class_forward.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <constrained_planning_plugin/constraint.h>

namespace compl_interface
{
namespace ob = ompl::base;
namespace og = ompl::geometric;

/** Validator for collion checking
 * this should be passed to the interface from the parent
 * because it is ROS specific.
 * */
bool isValid(const ob::State* state);

MOVEIT_CLASS_FORWARD(COMPLInterface);

class COMPLInterface
{
public:
  COMPLInterface();

  void preSolve(robot_model::RobotModelConstPtr robot_model, const std::string& group);

  bool solve();

  void postSolve();

private:
  std::shared_ptr<ob::RealVectorStateSpace> state_space_;
  std::shared_ptr<COMPLConstraint> constraints_;

  std::shared_ptr<ob::ProjectedStateSpace> constrained_state_space_;
  std::shared_ptr<ob::ConstrainedSpaceInformation> constrained_state_space_info_;
  std::shared_ptr<og::SimpleSetup> simple_setup_;
  std::shared_ptr<og::RRTConnect> planner_;
};
}  // namespace compl_interface

#endif  // COMPL_INTERFACE_H