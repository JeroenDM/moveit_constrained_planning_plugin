#ifndef COMPL_INTERFACE_H
#define COMPL_INTERFACE_H

#include <memory>

#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_request.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

/** OMPL interface class
 *
 * Maybe change name, as this is more an implementation than an interface.
 * I would like to introduce the symbol q for joint_positions. this would make
 * a lot of code cleaner.
 * */
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

  void preSolve(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                planning_interface::MotionPlanRequest request);

  /** directly pass joint positions for start and goal in this minimal example
   *
   * Should this be part of the preSolve setup?
   * */
  bool solve(const Eigen::Ref<const Eigen::VectorXd>& start_joint_positions,
             const Eigen::Ref<const Eigen::VectorXd>& goal_joint_positions, double allowed_planning_time);

  void postSolve();

  /** TODO too much Eigen copy operations. */
  std::vector<Eigen::VectorXd> getSolutionPath()
  {
    return solution_path_;
  }

private:
  std::shared_ptr<ob::RealVectorStateSpace> state_space_;
  std::shared_ptr<PositionConstraint> constraints_;

  std::shared_ptr<ob::ProjectedStateSpace> constrained_state_space_;
  std::shared_ptr<ob::ConstrainedSpaceInformation> constrained_state_space_info_;
  std::shared_ptr<og::SimpleSetup> simple_setup_;
  std::shared_ptr<og::RRTConnect> planner_;

  std::vector<Eigen::VectorXd> solution_path_;
  std::size_t num_dofs_; /* initialized in preSolve method based on robot model. */
};
}  // namespace compl_interface

#endif  // COMPL_INTERFACE_H