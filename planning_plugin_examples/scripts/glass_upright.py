#!/usr/bin/env python
"""
Planning case where the robot moves it's end-effector in between some collision objects.
Only the position of the Cartesian path is specified, the orientation is free to choose.
"""
from __future__ import print_function

import sys
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg

import rviz_tools_py.rviz_tools

from tf.transformations import quaternion_from_euler


GROUP_NAME = "panda_arm"

def create_pose_msg(xyzijk):
    quat = quaternion_from_euler(math.radians(
        xyzijk[3]), math.radians(xyzijk[4]), math.radians(xyzijk[5]))
    p = geometry_msgs.msg.Pose()
    p.position.x = xyzijk[0]
    p.position.y = xyzijk[1]
    p.position.z = xyzijk[2]
    p.orientation.x = quat[0]
    p.orientation.y = quat[1]
    p.orientation.z = quat[2]
    p.orientation.w = quat[3]
    return p


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('execute_cart_planning_example', anonymous=True)

    rvt = rviz_tools_py.rviz_tools.RvizMarkers(
        "/world", "/visualization_marker")
    rospy.sleep(0.5)  # magic wait time to make things work
    rvt.deleteAllMarkers()

    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)

    # print(robot.get_current_state())

    waypoints = []

    # j_start = [-2.1340945712525534, 1.4301424226898651, 2.393103596698277, -2.17416332730931, 1.7076724013639382, 3.752377333765853, 0]
    j_start = [0.666988104319289, -0.9954030434136065, -1.1194235704518019, -1.9946073045682555, -2.772101772642487, 3.4631937276027194, -1.2160652080175647]

    # j_goal = [2.3316948191590554, 0.930834763740868, -2.2281516646451425, -1.6782316332804856, -2.896899925633128, 3.637266755956212, 0]
    j_goal = [1.7301680303369467, -0.7342165592762893, -0.5358506493073328, -2.214051132383283, -1.9148221683474542, 1.8324940020482856, -1.588014538557859]




    start = create_pose_msg([0.3, -0.3, 0.6, 0, 90, 0])
    stop = create_pose_msg([0.3, 0.3, 0.7, 0, 90, 0])

    waypoints.append(start)
    waypoints.append(stop)

    # waypoints.append(move_group.get_current_pose().pose)

    for i, pose in enumerate(waypoints):
        rvt.publishAxis(pose, 0.1, 0.01)
        rvt.publishText(pose, "Pose {}".format(i), "black", 0.05)

    # move_group.set_joint_value_target(start_joint_positions)
    # move_group.go()

    move_group.set_pose_target(start)
    # move_group.set_joint_value_target(j_start)
    move_group.go()
    # print(robot.get_current_state())

    # move_group.set_pose_target(stop)
    # move_group.set_joint_value_target(j_goal)
    # move_group.go()
    # print(robot.get_current_state())

    # (plan, fraction) = move_group.compute_cartesian_path(
    #     waypoints,
    #     0.1,
    #     0.0)