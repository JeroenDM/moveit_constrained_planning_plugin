#!/usr/bin/env python
from __future__ import print_function

import sys
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from shape_msgs.msg import SolidPrimitive


from tf.transformations import quaternion_from_euler

import rviz_tools_py

GROUP_NAME = "manipulator"


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


class Plotter:
    """
    Wrapper around rviz_tools_py to visualise things.
    No idea how te remove a specific marker instead of
    all markers. (to remove opject when it is picked by the robot,
    but keep pick frame axis visible)
    """

    def __init__(self, topic_name="/visualization_marker", ref_frame="/world", wait_time=None):
        self.rvt = rviz_tools_py.rviz_tools.RvizMarkers(
            ref_frame, topic_name, wait_time=wait_time)
        # give publisher some time to get ready (0.2 to short)
        rospy.sleep(0.5)
        # todo there not realy a quick and easy way to remove the arbitrary 0.5 number
        self.rvt.deleteAllMarkers()

        self.display_publisher = rospy.Publisher(
            "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, latch=True, queue_size=1)

        self.robot_state_publisher = rospy.Publisher(
            "/display_robot_state", moveit_msgs.msg.DisplayRobotState, queue_size=1)

        def cleanup_node():
            print("Shutting down node")
            # self.rvt.deleteAllMarkers()

        rospy.on_shutdown(cleanup_node)

    def plot_axis(self, pose):
        self.rvt.publishAxis(pose, 0.1, 0.01)

    def plot_named_axis(self, pose, text):
        self.rvt.publishAxis(pose, 0.1, 0.01)
        self.rvt.publishText(pose, text, 'black',
                             geometry_msgs.msg.Vector3(0.1, 0.1, 0.1))

    def plot_line(self, start, goal):
        self.rvt.publishLine(start, goal, 'green', 0.05)

    def plot_cube(self, pose, dimensions):
        self.rvt.publishCube(pose, 'green', dimensions)

    def plot_robot(self, robot_state):
        self.robot_state_publisher.publish(
            moveit_msgs.msg.DisplayRobotState(state=start_state))

    def animate_planning_response(self, res):
        disp_traj = moveit_msgs.msg.DisplayTrajectory()
        disp_traj.trajectory_start = res.motion_plan_response.trajectory_start
        disp_traj.trajectory.append(res.motion_plan_response.trajectory)
        self.display_publisher.publish(disp_traj)

    def delete_all_markers(self):
        self.rvt.deleteAllMarkers()


def create_problem_parameters(group):
    """ Find start and goal joint values
    to put the end-effector at the start and end
    of the straight line we will follow with the constraints planner.
    This planner only supports joint goals.

    The above is achieved through planning with the MoveGroup interface
    as the inverse kinematics interface is non existent in Python.
    (Except through ROS service calls with non ideal request format.)
    """
    quat = quaternion_from_euler(0, math.pi / 2, 0, 'rxyz')
    desired_orientation = Quaternion(
        x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    p_start = Pose()
    p_start.position = Point(x=0.9, y=-0.2, z=0.2)
    p_start.orientation = desired_orientation

    p_goal = Pose()
    p_goal.position = Point(x=0.9, y=0.2, z=0.2)
    p_goal.orientation = desired_orientation

    group.set_named_target("home")
    group.go()

    group.set_pose_target(p_start)
    group.go()

    q_start = group.get_current_joint_values()
    print(q_start)

    group.set_pose_target(p_goal)
    group.go()

    q_goal = group.get_current_joint_values()
    print(q_goal)

    return {"start": {"pose": p_start, "joint_values": q_start},
            "goal": {"pose": p_goal, "joint_values": q_goal}}


def load_hardcoded_problem_parameters():
    quat = quaternion_from_euler(0, math.pi / 2, 0, 'rxyz')
    desired_orientation = Quaternion(
        x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    p_start = Pose()
    p_start.position = Point(x=0.9, y=-0.2, z=0.2)
    p_start.orientation = desired_orientation

    p_goal = Pose()
    p_goal.position = Point(x=0.9, y=0.2, z=0.2)
    p_goal.orientation = desired_orientation
    q_start = [0.24956224988146858, -0.7377165531335406, 2.2006779041601154,
               0.2519340025781682, -1.4653447987841997, -0.02738102743149895]
    q_goal = [-0.2494304937133913, -0.7373711994493184, 2.2005769482037416,
              -0.2500433274782332, -1.4670293064865587, 0.027100177473313725]

    return {"start": {"pose": p_start, "joint_values": q_start},
            "goal": {"pose": p_goal, "joint_values": q_goal}}


def show_problem_in_rviz(problem, rviz_handle):
    rviz_handle.plot_axis(problem["start"]["pose"])
    rviz_handle.plot_axis(problem["goal"]["pose"])
    # rviz_handle.plot_line(problem["start"]["pose"], problem["goal"]["pose"])


if __name__ == '__main__':
    ###################################################################
    # Ros / Rviz / MoveIt setup
    ###################################################################
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('execute_planning_example', anonymous=True)

    rviz = Plotter(ref_frame="/base_link")

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(GROUP_NAME)

    ###################################################################
    # Connect to planning server
    ###################################################################
    rospy.wait_for_service("ompl_constrained_planning", timeout=5.0)
    planning_service = rospy.ServiceProxy(
        "ompl_constrained_planning", moveit_msgs.srv.GetMotionPlan)

    ###################################################################
    # Create problem description
    ###################################################################
    # problem = create_problem_parameters(group)
    problem = load_hardcoded_problem_parameters()

    # create planning request message
    request = moveit_msgs.msg.MotionPlanRequest()
    request.group_name = GROUP_NAME

    start_state = copy.deepcopy(robot.get_current_state())
    start_state.joint_state.position = problem["start"]["joint_values"]
    request.start_state = start_state

    joint_goal = create_joint_goal(
        problem["goal"]["joint_values"], copy.deepcopy(robot.get_current_state()))
    request.goal_constraints.append(joint_goal)

    position_constraints = create_position_constraints(
        [0.9, 0.0, 0.2], [0.05, 0.4, 0.05])
    request.path_constraints.position_constraints.append(position_constraints)

    request.allowed_planning_time = 10.0

    ###################################################################
    # Visualize problem
    ###################################################################
    rviz.plot_robot(start_state)
    show_problem_in_rviz(problem, rviz)

    dims = position_constraints.constraint_region.primitives[0].dimensions
    rviz.plot_cube(
        position_constraints.constraint_region.primitive_poses[0],
        Vector3(dims[0], dims[1], dims[2])
    )

    ###################################################################
    # Solve problem and animate solution
    ###################################################################
    response = planning_service(request)
    print(response)
    rviz.animate_planning_response(response)
