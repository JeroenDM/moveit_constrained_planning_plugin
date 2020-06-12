#!/usr/bin/env python
from __future__ import print_function

import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


# from tf.transformations import quaternion_from_euler

GROUP_NAME = "panda_arm"


def create_path_constraints(ee_link_name):
    """
    TODO add constraints to the planning message,
    now they are read from the ros parameters server
    for the descartes planner.
    """
    rospy.loginfo("Adding constraints for link {}".format(ee_link_name))
    con = moveit_msgs.msg.Constraints()
    con.name = "z_axis_free_constraint"
    ori_con = moveit_msgs.msg.OrientationConstraint()
    ori_con.absolute_x_axis_tolerance = 0
    return con


def joint_state_to_joint_constraints(joint_state):
    constraints = []
    for name, position in zip(joint_state.name, joint_state.position):
        print(name, position)
        jc = moveit_msgs.msg.JointConstraint()
        jc.joint_name = name
        jc.position = position
        jc.tolerance_above = 2e-16
        jc.tolerance_below = 2e-16
        constraints.append(jc)
    return constraints


def create_joint_goal(joint_values, emtpy_valid_robot_state):
    emtpy_valid_robot_state.joint_state.position = joint_values
    joint_constraints = joint_state_to_joint_constraints(
        emtpy_valid_robot_state.joint_state)
    joint_goal = moveit_msgs.msg.JointConstraint()
    goal_con = moveit_msgs.msg.Constraints()
    goal_con.name = "joint_goal"
    goal_con.joint_constraints.extend(joint_constraints)
    return goal_con


def create_position_constraints(reference, tolerance):
    """ Create position tolerance around a reference position. """
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0, 0, 0])
    box.dimensions[SolidPrimitive.BOX_X] = tolerance[0]
    box.dimensions[SolidPrimitive.BOX_Y] = tolerance[1]
    box.dimensions[SolidPrimitive.BOX_Z] = tolerance[2]

    box_pose = Pose()
    box_pose.position.x = reference[0]
    box_pose.position.y = reference[1]
    box_pose.position.z = reference[2]
    box_pose.orientation.w = 1.0

    pos_con = moveit_msgs.msg.PositionConstraint()
    pos_con.constraint_region.primitives.append(box)
    pos_con.constraint_region.primitive_poses.append(box_pose)
    return pos_con


if __name__ == '__main__':
    rospy.init_node('execute_planning_example', anonymous=True)

    robot = moveit_commander.RobotCommander()

    display_publisher = rospy.Publisher(
        "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, latch=True, queue_size=1)

    rospy.wait_for_service("ompl_constrained_planning", timeout=5.0)

    planning_service = rospy.ServiceProxy(
        "ompl_constrained_planning", moveit_msgs.srv.GetMotionPlan)

    request = moveit_msgs.msg.MotionPlanRequest()
    request.group_name = GROUP_NAME

    # start and goal joint values
    j_start = [0.666988104319289, -0.9954030434136065, -1.1194235704518019, -
               1.9946073045682555, -2.772101772642487, 3.4631937276027194, -1.2160652080175647, 0, 0]
    j_goal = [1.7301680303369467, -0.7342165592762893, -0.5358506493073328, -
              2.214051132383283, -1.9148221683474542, 1.8324940020482856, -1.588014538557859]

    start_state = copy.deepcopy(robot.get_current_state())
    start_state.joint_state.position = j_start
    request.start_state = start_state

    joint_goal = create_joint_goal(
        j_goal, copy.deepcopy(robot.get_current_state()))
    request.goal_constraints.append(joint_goal)

    position_constraints = create_position_constraints(
        [0.3, 0.0, 0.65], [-1, -1, 0.1])
    request.path_constraints.position_constraints.append(position_constraints)

    request.allowed_planning_time = 5.0

    # print(request)

    response = planning_service(request)
    print(response)

    disp_traj = moveit_msgs.msg.DisplayTrajectory()
    disp_traj.trajectory_start = response.motion_plan_response.trajectory_start
    disp_traj.trajectory.append(response.motion_plan_response.trajectory)
    display_publisher.publish(disp_traj)
