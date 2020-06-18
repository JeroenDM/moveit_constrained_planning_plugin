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

    // save the number of degrees of freedom of the robot
    num_dof_ = joint_model_group_->getVariableCount();

    // get the end-effector link from the planning group
    end_effector_link_ = joint_model_group_->getLinkModelNames().back();

    ROS_INFO_STREAM("Created robot wrapper for planning group: " << planning_group);
    ROS_INFO_STREAM("with end-effector link: " << end_effector_link_);
    ROS_INFO_STREAM("The robot has " << num_dof_ << " degrees of freedom.");
  }

  const Eigen::Isometry3d fk(const std::vector<double>& q) const
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    return robot_state_->getGlobalLinkTransform(end_effector_link_);
  }

  const Eigen::Isometry3d fk(const Eigen::VectorXd q) const
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    return robot_state_->getGlobalLinkTransform(end_effector_link_);
  }

  Eigen::MatrixXd jacobian(const Eigen::VectorXd q)
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    return robot_state_->getJacobian(joint_model_group_);
  }

  /** Jacobian with rotational part calculated based on
   * roll, pitch, and yaw angles, using finite differences.
   * */
  Eigen::MatrixXd numericalJacobianOrientation(const Eigen::VectorXd q)
  {
    const double h{ 1e-6 }; /* step size for numerical derivation */
    // const std::size_t ndof {q.size()};
    const std::size_t ndof = q.size();

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, ndof);

    // helper matrix for differentiation.
    Eigen::MatrixXd Ih = h * Eigen::MatrixXd::Identity(ndof, ndof);

    for (std::size_t dim{ 0 }; dim < ndof; ++dim)
    {
      auto rpy = poseToRPY(fk(q));
      auto rpy_plus_h = poseToRPY(fk(q + Ih.col(dim)));
      Eigen::Vector3d col = (rpy_plus_h - rpy) / h;
      J.col(dim) = col;
    }
    return J;
  }

  /** Jacobian with rotational part calculated based on
   * Angle-axis representation, using finite differences.
   * */
  Eigen::MatrixXd numericalJacobianAngleAxis(const Eigen::VectorXd q)
  {
    const double h{ 1e-6 }; /* interval width for numerical derivation */
    // const std::size_t ndof {q.size()};
    const std::size_t ndof = q.size();

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, ndof);

    // helper matrix for differentiation.
    Eigen::MatrixXd Ih = h * Eigen::MatrixXd::Identity(ndof, ndof);

    for (std::size_t dim{ 0 }; dim < ndof; ++dim)
    {
      auto aa = poseToAA(fk(q));
      auto aa_plus_h = poseToAA(fk(q + Ih.col(dim)));
      Eigen::Vector3d col = (aa_plus_h - aa) / h;
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

  /** TODO look in Modern Robotics, page 161.
   * 
   * There is still an issue in the formula,
   * I think it could be that the formula is for transforming
   * the jacobian expressed in the end-effector frame, not the one in the world frame.
   * 
   * **/
  Eigen::MatrixXd jacobianAngleAxis(const Eigen::VectorXd q)
  {
    // const std::size_t ndof {q.size()};
    const std::size_t ndof = q.size();
    Eigen::Matrix3d R_ee = fk(q).rotation();
    Eigen::AngleAxisd aa(R_ee);

    // create short variable for readability of complex expression
    double t{ std::abs(aa.angle()) };
    Eigen::Vector3d r{ aa.axis() * sign(aa.angle()) };
    Eigen::Matrix3d r_skew;
    r_skew << 0, -r[2], r[1], r[2], 0, -r[0], -r[1], r[0], 0;

    double A, B;
    A = (1 - std::cos(t)) / (t * t);
    B = (t - std::sin(t)) / (t * t * t);

    auto m_convert = Eigen::Matrix3d::Identity() - A * r_skew + B * r_skew * r_skew;

    return m_convert.inverse() * jacobian(q).bottomRows(3);
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

  std::size_t getDOF()
  {
    return num_dof_;
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
  std::string end_effector_link_;
  std::size_t num_dof_; /* number of degrees of freedom */
};

#endif  // CONSTRAINED_PLANNING_EXAMPLE_ROBOT