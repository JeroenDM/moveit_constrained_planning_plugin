#include "constrained_planning_plugin/compl_interface.h"

#include <iostream>

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace compl_interface
{
namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isValid(const ob::State* state)
{
  return true;
}

class ZUpConstraints : public ob::Constraint
{
public:
  ZUpConstraints() : ob::Constraint(7, 1)
  {
  }

  void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override
  {
    out[0] = 0.0;
  }
};

void planWithConstraints()
{
  // the space is based on ob::RealVectorStateSpace>(7)
  auto space(std::make_shared<ob::RealVectorStateSpace>(7));

  // the bounds on the quaternion part are not used (at least that I know of)
  // the quaterion is normalized when bounds are enforced
  ob::RealVectorBounds bounds(7);
  bounds.setLow(-2);
  bounds.setHigh(2);
  space->setBounds(bounds);

  auto constraint = std::make_shared<ZUpConstraints>();

  // problem setup stuff
  auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
  auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
  auto ss = std::make_shared<og::SimpleSetup>(csi);
  auto pp = std::make_shared<og::RRTConnect>(csi);
  ss->setPlanner(pp);

  // Add the obstacles, the implemetation also uses casting of a state
  // to a real vector state space in this case
  // I could cast to an eigen position and quaternion -> eigen transform
  // for collision checking in the future for more interesting examples
  ss->setStateValidityChecker(isValid);

  // problem specific admin
  Eigen::VectorXd sv(7), gv(7);
  sv << 0, 0, 1, 0, 0, 0, 1;
  gv << 0, 0, -1, 0, 0, 0, 1;
  ob::ScopedState<> start(css);
  ob::ScopedState<> goal(css);
  start->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
  goal->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);
  ss->setStartAndGoalStates(start, goal);

  // solving it
  ss->setup();
  ob::PlannerStatus stat = ss->solve(5.);
  if (stat)
  {
    // Path simplification also works when using a constrained state space!
    ss->simplifySolution(5.);
    auto path = ss->getSolutionPath();
    path.interpolate();

    path.printAsMatrix(std::cout);
  }
  else
  {
    OMPL_WARN("No solution found!");
  }
}

COMPLInterface::COMPLInterface()
{
}

bool COMPLInterface::solve()
{
  planWithConstraints();
  return true;
}
}  // namespace compl_interface