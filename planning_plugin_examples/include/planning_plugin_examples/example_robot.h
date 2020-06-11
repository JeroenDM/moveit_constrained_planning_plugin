#ifndef CONSTRAINED_PLANNING_EXAMPLE_ROBOT
#define CONSTRAINED_PLANNING_EXAMPLE_ROBOT

#include <string>
#include <vector>
#include <Eigen/Geometry>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <planning_plugin_examples/example_util.h>

/** Hide all MoveIt stuff in a class
 *
 * q = vector with joint positions
 */
class Robot
{
public:
  Robot(const std::string& planning_group, const std::string& robot_description = "robot_description")
  {
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(robot_description);
    robot_model_ = robot_model_loader_->getModel();
    robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
    joint_model_group_ = robot_state_->getJointModelGroup(planning_group);
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    planning_scene_->getCurrentStateNonConst().setToDefaultValues();
  }

  const Eigen::Isometry3d fk(const std::vector<double>& q, const std::string& frame = "tool_tip") const
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    return robot_state_->getGlobalLinkTransform(frame);
  }

  const Eigen::Isometry3d fk(const Eigen::VectorXd q, const std::string& frame = "tool_tip") const
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    return robot_state_->getGlobalLinkTransform(frame);
  }

  Eigen::MatrixXd jacobian(const Eigen::VectorXd q)
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    return robot_state_->getJacobian(joint_model_group_);
  }

  Eigen::MatrixXd numericalJacobianOrientation(const Eigen::VectorXd q)
  {
    const double h{ 1e-6 }; /* step size for numerical derivation */
    // const std::size_t ndof {q.size()};
    const std::size_t ndof = q.size();

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, ndof);

    // helper matrix for differentiation.
    Eigen::MatrixXd Ih = h * Eigen::MatrixXd::Identity(ndof, ndof);

    for (std::size_t dim{ 0 }; dim < 6; ++dim)
    {
      auto rpy = poseToRPY(fk(q));
      auto rpy_plus_h = poseToRPY(fk(q + Ih.col(dim)));
      Eigen::Vector3d col = (rpy_plus_h - rpy) / h;
      J.col(dim) = col;
    }
    return J;
  }

  Eigen::MatrixXd jacobianOrientation(const Eigen::VectorXd q)
  {
    // const std::size_t ndof {q.size()};
    const std::size_t ndof = q.size();
    auto rpy = poseToRPY(fk(q));
    return angularVelocityToRPYRates(rpy[0], rpy[1]) * jacobian(q).bottomRows(3);
  }

  Eigen::MatrixXd analyticalJacobian(const Eigen::VectorXd q)
  {
    // const std::size_t ndof {q.size()};
    const std::size_t ndof = q.size();
    Eigen::MatrixXd J = jacobian(q);
    auto rpy = poseToRPY(fk(q));
    J.bottomRows(3) = angularVelocityToRPYRates(rpy[0], rpy[1]) * J.bottomRows(3);
    return J;
  }

  void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, const std::vector<double>& q)
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    mvt->publishRobotState(robot_state_, rviz_visual_tools::DEFAULT);
    mvt->trigger();
  }

  void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, const Eigen::VectorXd q)
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    mvt->publishRobotState(robot_state_, rviz_visual_tools::DEFAULT);
    mvt->trigger();
  }

  Eigen::VectorXd getRandomJointPositions()
  {
    Eigen::VectorXd joint_values;
    robot_state_->setToRandomPositions(joint_model_group_);
    robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);
    return joint_values;
  }

private:
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  planning_scene::PlanningScenePtr planning_scene_; /* I should probably use the planning scene monitor */
};

#endif  // CONSTRAINED_PLANNING_EXAMPLE_ROBOT