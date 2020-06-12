#include "constrained_planning_plugin/compl_interface.h"

#include <iostream>

namespace compl_interface
{
bool isValid(const ob::State* state)
{
  return true;
}

COMPLInterface::COMPLInterface()
{
  ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
}

void COMPLInterface::preSolve(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                              planning_interface::MotionPlanRequest request)
{
  state_space_ = std::make_shared<ob::RealVectorStateSpace>(7);
  ob::RealVectorBounds bounds(7);
  bounds.setLow(-2 * M_PI);
  bounds.setHigh(2 * M_PI);
  state_space_->setBounds(bounds);

  // constraints should be passed from the planning constext based on what is in the planning request
  // and then through some kind of constrained factory class we can create the appropriate ones.
  constraints_ = std::make_shared<COMPLConstraint>(robot_model, group, request.path_constraints);

  constrained_state_space_ = std::make_shared<ob::ProjectedStateSpace>(state_space_, constraints_);
  constrained_state_space_info_ = std::make_shared<ob::ConstrainedSpaceInformation>(constrained_state_space_);
  simple_setup_ = std::make_shared<og::SimpleSetup>(constrained_state_space_info_);
  planner_ = std::make_shared<og::RRTConnect>(constrained_state_space_info_);
  simple_setup_->setPlanner(planner_);

  simple_setup_->setStateValidityChecker(isValid);
}

bool COMPLInterface::solve(const Eigen::Ref<const Eigen::VectorXd>& start_joint_positions,
                           const Eigen::Ref<const Eigen::VectorXd>& goal_joint_positions, double allowed_planning_time)
{
  // check start and goal state
  auto rpy_start =
      (constraints_->forwardKinematics(start_joint_positions).rotation().transpose() * constraints_->desired_ee_quat_)
          .eulerAngles(0, 1, 2);
  auto rpy_goal =
      (constraints_->forwardKinematics(goal_joint_positions).rotation().transpose() * constraints_->desired_ee_quat_)
          .eulerAngles(0, 1, 2);

  ROS_INFO_STREAM("Start and goal rpy angles: ");
  ROS_INFO_STREAM(rpy_start.transpose());
  ROS_INFO_STREAM(rpy_goal.transpose());

  ob::ScopedState<> start(constrained_state_space_);
  ob::ScopedState<> goal(constrained_state_space_);
  // start->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
  // goal->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);
  start->as<ob::ConstrainedStateSpace::StateType>()->copy(start_joint_positions);
  goal->as<ob::ConstrainedStateSpace::StateType>()->copy(goal_joint_positions);
  simple_setup_->setStartAndGoalStates(start, goal);

  // solving it
  simple_setup_->setup();
  ob::PlannerStatus stat = simple_setup_->solve(allowed_planning_time);
  if (stat)
  {
    return true;
  }
  else
  {
    OMPL_WARN("No solution found!");
    return false;
  }
}

void COMPLInterface::postSolve()
{
  simple_setup_->simplifySolution(5.);
  auto path = simple_setup_->getSolutionPath();
  path.interpolate();

  // path.printAsMatrix(std::cout);

  std::cout << "Writing path from OMPL to generic format." << std::endl;

  // write path to generic format indepenent from OMPL to pass it to ROS?
  solution_path_.clear();
  for (auto& state : path.getStates())
  {
    const Eigen::Map<Eigen::VectorXd>& x = *state->as<ob::ConstrainedStateSpace::StateType>();

    Eigen::VectorXd joint_position(x);
    solution_path_.push_back(joint_position);
  }
}
}  // namespace compl_interface