#ifndef COMPL_PLANNING_CONTEXT_H
#define COMPL_PLANNING_CONTEXT_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_interface/planning_interface.h>

namespace compl_interface
{
// Macro that forward declares a class and defines the respective smartpointers through MOVEIT_DECLARE_PTR.
MOVEIT_CLASS_FORWARD(COMPLPlanningContext);

class COMPLPlanningContext : public planning_interface::PlanningContext
{
public:
  COMPLPlanningContext(const std::string& name, const std::string& group, const moveit::core::RobotModelConstPtr& model)
    : PlanningContext(name, group), robot_model_(model)
  {
  }

  ~COMPLPlanningContext() = default;

  void clear() override;

  bool solve(planning_interface::MotionPlanResponse& res) override;

  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;

private:
  moveit::core::RobotModelConstPtr robot_model_;
};
}  // namespace compl_interface

#endif  // COMPL_PLANNING_CONTEXT_H