#ifndef COMPL_PLANNING_CONTEXT_H
#define COMPL_PLANNING_CONTEXT_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include "constrained_planning_plugin/compl_interface.h"

namespace compl_interface
{
// Macro that forward declares a class and defines the respective smartpointers through MOVEIT_DECLARE_PTR.
MOVEIT_CLASS_FORWARD(COMPLPlanningContext);

class COMPLPlanningContext : public planning_interface::PlanningContext
{
public:
  COMPLPlanningContext(const std::string& name, const std::string& group, moveit::core::RobotModelConstPtr robot_model)
    : PlanningContext(name, group), robot_model_(robot_model), joint_model_group_(robot_model->getJointModelGroup(group))
  {
    robot_state_.reset(new moveit::core::RobotState(robot_model));
    robot_state_->setToDefaultValues();
    compl_interface_ = COMPLInterfacePtr(new COMPLInterface());
  }

  ~COMPLPlanningContext() = default;

  void clear() override;

  bool solve(planning_interface::MotionPlanResponse& res) override;

  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_;  // Why is this a raw pointer everywhere in MoveIt?

  // save a single state to do forward kinematics (not ideal)
  moveit::core::RobotStatePtr robot_state_;

  // it would be nice to have a kinematics solver instance for the constraints
  // but I don't know how to initialize this member properly
  // const kinematics::KinematicsBaseConstPtr kinematics_solver_;

  // the actual planner goes here
  COMPLInterfacePtr compl_interface_;

};
}  // namespace compl_interface

#endif  // COMPL_PLANNING_CONTEXT_H