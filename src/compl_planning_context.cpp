#include "constrained_planning_plugin/compl_planning_context.h"

#include <moveit/planning_interface/planning_interface.h>

namespace compl_interface
{
void COMPLPlanningContext::clear()
{
}

bool COMPLPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  return false;
}

bool COMPLPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  return false;
}

bool COMPLPlanningContext::terminate()
{
  return true;
}
}  // namespace compl_interface