#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("commander", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

move_group = moveit_commander.MoveGroupCommander("manipulator")

waypoints = []

home = move_group.get_current_pose().pose

wp1 = geometry_msgs.msg.Pose()
wp1.position.x = 0.1143
wp1.position.y = 0.0
wp1.position.z = 0.84455
wp1.orientation.w = 1.0

wp2 = geometry_msgs.msg.Pose()
wp2.position.x = 0.1143
wp2.position.y = 0.0
wp2.position.z = 0.9652
wp2.orientation.w = 1.0

waypoints = [home, wp1, wp2]

plan, fraction = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

print("============ Press `Enter` to run path ===============")

raw_input()

move_group.execute(plan, wait=True)

moveit_commander.roscpp_shutdown()


