#include "constrained_planning_plugin/compl_interface.h"

#include <iostream>

namespace compl_interface
{
bool isValid(const ob::State* state)
{
  return true;
}

void ZUpConstraints::function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const
{
  out[0] = 0.0;
}

COMPLInterface::COMPLInterface()
{
}

void COMPLInterface::preSolve()
{
  state_space_ = std::make_shared<ob::RealVectorStateSpace>(7);
  ob::RealVectorBounds bounds(7);
  bounds.setLow(-2);
  bounds.setHigh(2);
  state_space_->setBounds(bounds);

  constraints_ = std::make_shared<ZUpConstraints>();

  constrained_state_space_ = std::make_shared<ob::ProjectedStateSpace>(state_space_, constraints_);
  constrained_state_space_info_ = std::make_shared<ob::ConstrainedSpaceInformation>(constrained_state_space_);
  simple_setup_ = std::make_shared<og::SimpleSetup>(constrained_state_space_info_);
  planner_ = std::make_shared<og::RRTConnect>(constrained_state_space_info_);
  simple_setup_->setPlanner(planner_);

  simple_setup_->setStateValidityChecker(isValid);
}

bool COMPLInterface::solve()
{
  // problem specific admin
  Eigen::VectorXd sv(7), gv(7);
  sv << 0, 0, 1, 0, 0, 0, 1;
  gv << 0, 0, -1, 0, 0, 0, 1;
  ob::ScopedState<> start(constrained_state_space_);
  ob::ScopedState<> goal(constrained_state_space_);
  start->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
  goal->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);
  simple_setup_->setStartAndGoalStates(start, goal);

  // solving it
  simple_setup_->setup();
  ob::PlannerStatus stat = simple_setup_->solve(5.);
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
  // path.interpolate();

  path.printAsMatrix(std::cout);
}
}  // namespace compl_interface