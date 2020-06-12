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

/** Implementation of OMPL's Constraint class required for planning.
 *
 * This class should be replaced with a factory that can create all types of different constraints
 * dependening on what is in the planning request.
 *
 * For the link name member, I would like it const, but I don't know how to properly initialize it then.
 * */
class COMPLConstraint : public ob::Constraint
{
public:
  COMPLConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                  moveit_msgs::Constraints constraints);

  void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override;

  void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override;

  Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_positions) const;
  Eigen::Quaterniond desired_ee_quat_; /** desired or nominal orientation for the end-effector orientation constraitns.
                                        */
private:
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  std::string link_name_;
  std::vector<Bound> position_bounds_;
  std::size_t dimension_; /** number of position or rotation values that have constraints. */

  std::vector<Bound> orientation_bounds_;
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
