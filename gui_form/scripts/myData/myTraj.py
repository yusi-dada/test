# -*- coding: utf-8 -*-
##################################
#
#	 library
#
##################################
import os
import sys
import signal
from myData import *
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui

###################################################################
#
# DisplayTrajectory
#  * string model_id
#  * moveit_msgs/RobotTrajectory[] trajectory
#     * trajectory_msgs/JointTrajectory joint_trajectory
#        * std_msgs/Header header
#        * string[] joint_names
#        * trajectory_msgs/JointTrajectoryPoint[] points
#           * float64[] positions
#           * float64[] velocities
#           * float64[] accelerations
#           * float64[] effort
#           * duration time_from_start
#     * trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory
#  * moveit_msgs/RobotState trajectory_start
#
###################################################################
def traj2myData(traj=DisplayTrajectory()):
  Dtrajectory = myData("Displaytrajectory")
  Dtrajectory.set("model_id", traj.model_id)
  
  for i1,v1 in enumerate(traj.trajectory):
    trajectory = myData("trajectory")
    jtrajectory = myData("joint_trajectory")
    jtrajectory.set("header"     , [])
    jtrajectory.set("joint_names", v1.joint_trajectory.joint_names)

    points = myData("points")
    for i2,v2 in enumerate(v1.joint_trajectory.points):
      point = myData("point")
      point.set("positions"    , v2.positions)
      point.set("velocities"   , v2.velocities)
      point.set("accelerations", v2.accelerations)
      point.set("effort"       , v2.effort)
      point.set("duration"     , 0)
      points.set("points"+str(i2), point)
    jtrajectory.set("points", points)

    trajectory.set("joint_trajectory",jtrajectory)
    Dtrajectory.set("trajectory"+str(i1),trajectory)
  
  return Dtrajectory
  
####################################################
def test_traj2myData():
  dtraj = DisplayTrajectory()
  rtraj = RobotTrajectory()
  jtraj = JointTrajectory()

  points = []
  for i in range(50):
    point = JointTrajectoryPoint()
    point.positions = [i,1.00,2.00,3.00,4.00,5.00,6.00,7.00]
    points.append(point)

  jtraj.points = points
  rtraj.joint_trajectory=jtraj
  dtraj.trajectory.append(rtraj)
  
  return traj2myData(dtraj)
####################################################

"""
    └ <traj>
       ├ model_id : 
       └ <trajectory0>
          └ <joint_trajectory>
             ├ header : []
             ├ joint_names : []
             └ <points>
                ├ <points0>
                |  ├ positions : [0, 1, 2, 3, 4]
                |  ├ velocities : []
                |  ├ accelerations : []
                |  ├ effort : []
                |  └ duration : 0
                ├ <points1>
                |  ├ positions : [1, 1, 2, 3, 4]
                |  ├ velocities : []
                |  ├ accelerations : []
                |  ├ effort : []
                |  └ duration : 0
                ├ <points2>
                |  ├ positions : [2, 1, 2, 3, 4]
                |  ├ velocities : []
                |  ├ accelerations : []
                |  ├ effort : []
                |  └ duration : 0
                ├ <points3>
                |  ├ positions : [3, 1, 2, 3, 4]
                |  ├ velocities : []
                |  ├ accelerations : []
                |  ├ effort : []
                |  └ duration : 0
                └ <points4>
                   ├ positions : [4, 1, 2, 3, 4]
                   ├ velocities : []
                   ├ accelerations : []
                   ├ effort : []
                   └ duration : 0
"""

def myData2traj(dat=myData()):
  Dtrajectory = DisplayTrajectory()
  Dtrajectory.model_id = dat.data["model_id"]
  #Dtrajectory.

  points = []
  point = JointTrajectoryPoint()
  

  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
