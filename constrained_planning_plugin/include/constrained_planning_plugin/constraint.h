#ifndef COMPL_INTERFACE_CONSTRAINT_H
#define COMPL_INTERFACE_CONSTRAINT_H

#include <limits>

#include <ompl/base/Constraint.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>

namespace compl_interface
{
namespace ob = ompl::base;

const double INF = std::numeric_limits<double>::infinity();

/** Represents bounds on a scalar value (double). **/
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
    if (value < lower) return lower - value;
    else if (value > upper) return value - upper;
    else return 0.0;
  }
};

/** Temp dummy constraint class
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

  void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;

  Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_positions) const;

private:
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  std::string link_name_;
  std::vector<Bound> position_bounds_;
  std::size_t dimension_;
};

}  // namespace compl_interface

#endif  // COMPL_INTERFACE_CONSTRAINT_H
