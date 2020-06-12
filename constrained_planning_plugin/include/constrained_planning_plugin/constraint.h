#ifndef COMPL_INTERFACE_CONSTRAINT_H
#define COMPL_INTERFACE_CONSTRAINT_H

#include <iostream>
#include <limits>

#include <Eigen/Geometry>

#include <ompl/base/Constraint.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>
// #include <moveit_msgs/PositionConstraint.h>

namespace compl_interface
{
namespace ob = ompl::base;

/** Use Infinity for variables that have no constraints.
 *
 * This makes it easier to write generic code.
 * Otherwise we would have to leave out the bounds for specific
 * position or orientation values and writing the constraint
 * evaluation would be much more complex, within my range
 * of solution I can imagine (jeroendm).
 * */
const double INF = std::numeric_limits<double>::infinity();

/** Represents bounds on a scalar value (double).
 *
 * Equality constraints can be represented by setting
 * the upper bound and lower bound almost equal.
 * I assume that it is better to not have them exactly equal
 * for numerical reasons. Not sure.
 * **/
struct Bound
{
  double lower, upper;

  /** Distance of a given value outside the bounds,
   * zero inside the bounds.
   *
   * Creates a penalty function that looks like this:
   *
   *  \         /
   *   \       /
   *    \_____/
   * (how does ascii art work??)
   * */
  double distance(double value) const
  {
    if (value < lower)
      return lower - value;
    else if (value > upper)
      return value - upper;
    else
      return 0.0;
  }
};

/** Position constraints serves as a base class for generic constraints for now.
 * 
 * Other constraints have to override:
 * - fillBoundsFromConstraintsMsg
 * - calcCurrentValues
 * - calcCurrentJacobian
 * */
class PositionConstraint : public ob::Constraint
{
public:
  PositionConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                     moveit_msgs::Constraints constraints, const unsigned int num_dofs);

  void init(moveit_msgs::Constraints constraints);

  // some kind of factory method, but I can't get it to work yet
  // static std::shared_ptr<PositionConstraint> create(robot_model::RobotModelConstPtr robot_model,
  //                                                   const std::string& group, moveit_msgs::Constraints constraints,
  //                                                   const unsigned int num_dofs);

  void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override;

  void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override;

  virtual void fillBoundsFromConstraintsMsg(moveit_msgs::Constraints constraints);
  virtual Eigen::Vector3d calcCurrentValues(const Eigen::Ref<const Eigen::VectorXd>& x) const;
  virtual Eigen::MatrixXd calcCurrentJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const;

  // Are these actually const, as the robot state is modified? How come it works?
  Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;
  Eigen::MatrixXd geometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

private:
  // MoveIt's robot representation for kinematic calculations
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  std::string link_name_; /** Robot link the constraints are applied to. */

protected:
  std::vector<Bound> bounds_; /** Upper and lower bounds on constrained variables. */
  Eigen::Vector3d target_;    /** target for equality constraints, nominal value for inequality constraints. */
};

/** Specialization of Postion constraints to handle orientation error. */
class RPYConstraints : public PositionConstraint
{
public:
  RPYConstraints(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                 moveit_msgs::Constraints constraints, const unsigned int num_dofs)
    : PositionConstraint(robot_model, group, constraints, num_dofs)
  {
  }
  virtual void fillBoundsFromConstraintsMsg(moveit_msgs::Constraints constraints) override;
  virtual Eigen::Vector3d calcCurrentValues(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
  virtual Eigen::MatrixXd calcCurrentJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;

private:
  Eigen::Quaterniond target_as_quat_;
};

/** Inverse of the Conversion matrix from roll-pitch-yaw velocity to angular velocity.
 *
 * w = B(rpy) * rpy_dot
 * w = angular velocity, B = matrix returned by this function,
 * rpy = roll-pitch-yaw angles
 * rpy_dot = roll-pitch-yaw time derivatives.
 *
 * This function directly calculates B^-1
 * and contains a singularity for ry = +/- pi / 2
 *
 * from:
 * https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
 *
 * */
Eigen::Matrix3d angularVelocityToRPYRates(double rx, double ry);

/** Extract position constraints from the MoveIt message.
 *
 * Assumes there is a single primitive of type box.
 * Only the dimensions and position of this box are used.
 * TODO: also use the link name in future?
 * Now we assume the constraints are for the end-effector link.
 * */
std::vector<Bound> positionConstraintMsgToBoundVector(moveit_msgs::PositionConstraint pos_con);

std::vector<Bound> orientationConstraintMsgToBoundVector(moveit_msgs::OrientationConstraint ori_con);

/** pretty printing of Bounds **/
std::ostream& operator<<(std::ostream& os, const Bound& bound);

}  // namespace compl_interface

#endif  // COMPL_INTERFACE_CONSTRAINT_H
