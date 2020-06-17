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
 * - parseConstraintMsg
 * - calcCurrentValues
 * - calcCurrentJacobian
 * */
class PositionConstraint : public ob::Constraint
{
public:
  PositionConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                     moveit_msgs::Constraints constraints, const unsigned int num_dofs,
                     const unsigned int num_cons_ = 3);

  void init(moveit_msgs::Constraints constraints);

  // some kind of factory method, but I can't get it to work yet
  // static std::shared_ptr<PositionConstraint> create(robot_model::RobotModelConstPtr robot_model,
  //                                                   const std::string& group, moveit_msgs::Constraints constraints,
  //                                                   const unsigned int num_dofs);

  /** OMPL's main constraint evaluation function.
   *
   *  OMPL requires you to override at least "function" which represents the constraint F(q) = 0
   * */
  void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override;

  /** Optionally you can also provide dF(q)/dq, the Jacobian of  the constriants.
   *
   * */
  void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override;

  /** Robot forward kinematics, used to calculate F(q) = 0
   *
   * Are these actually const, as the robot state is modified? How come it works?
   * */
  Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** Robot jacobian as calculated by MoveIt, used to calculate dF(q) / dq
   * */
  Eigen::MatrixXd geometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** Parse bounds on position and orientation parameters from MoveIt's constraint message.
   *
   * This is non-trivial given the often complex structure of these messages.
   * */
  virtual void parseConstraintMsg(moveit_msgs::Constraints constraints);

  /** Calculate the value of the parameter that is being constraint by the bounds.
   *
   * In this Position constraints case, it calculates the x, y and z position
   * of the end-effector.
   * */
  virtual Eigen::Vector3d calcCurrentValues(const Eigen::Ref<const Eigen::VectorXd>& x) const;

  /** Calculate the Jacobian for the current parameters that are being cosntraint.
   *
   * This is the Jacobian without the correction due to the penalty function
   * (adding a minus sign or setting it to zero.)
   * */
  virtual Eigen::MatrixXd calcCurrentJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const;

protected:
  // MoveIt's robot representation for kinematic calculations
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  std::string link_name_;     /** Robot link the constraints are applied to. */
  std::vector<Bound> bounds_; /** Upper and lower bounds on constrained variables. */
  Eigen::Vector3d target_;    /** target for equality constraints, nominal value for inequality constraints. */
};

/** Specialization of Postion constraints to handle orientation error.
 *
 * Constraints on roll, pitch, and yaw angle of the end-effector:
 * R = Rx(roll) * Ry(pitch) * Rz(yaw)
 *
 */
class RPYConstraints : public PositionConstraint
{
public:
  RPYConstraints(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                 moveit_msgs::Constraints constraints, const unsigned int num_dofs)
    : PositionConstraint(robot_model, group, constraints, num_dofs)
  {
  }
  virtual void parseConstraintMsg(moveit_msgs::Constraints constraints) override;
  virtual Eigen::Vector3d calcCurrentValues(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
  virtual Eigen::MatrixXd calcCurrentJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;

private:
  Eigen::Quaterniond target_as_quat_;
};

/** Specialization of Postion constraints to handle orientation error.
 *
 * This constraint assumes a fixed desired orientation specified as a quaternion.
 *
 */
class QuaternionConstraint : public PositionConstraint
{
public:
  QuaternionConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                       moveit_msgs::Constraints constraints, const unsigned int num_dofs)
    : PositionConstraint(robot_model, group, constraints, num_dofs, 3)
  {
  }
  virtual void parseConstraintMsg(moveit_msgs::Constraints constraints) override;

  void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override;
  void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override;

  // virtual Eigen::Vector3d calcCurrentValues(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
  // virtual Eigen::MatrixXd calcCurrentJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;

private:
  Eigen::Quaterniond target_as_quat_;
};

std::shared_ptr<PositionConstraint> createConstraint(robot_model::RobotModelConstPtr robot_model,
                                                     const std::string& group, moveit_msgs::Constraints constraints);

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
