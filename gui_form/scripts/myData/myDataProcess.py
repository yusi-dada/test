#! /usr/bin/python
# -*- coding: utf-8 -*-
# ********************************
#
#	 library
#
# ********************************
import os
import sys
import copy
import signal
#********************************
#  data construct
#********************************
from myData import * 
from myTraj import * 
#********************************
#  ROS
#********************************
import rospy
from moveit_msgs.msg import DisplayTrajectory
#********************************
#  Qt
#********************************
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui


# ********************************
#
# create dummy data
#
# ********************************
def dummyData():
  dat = myData("dat")
  dat.set("item"  , item())
  dat.set("device", device())
  dat.set("points", points())  
  return dat

# ********************************
#  item
# ********************************
def item():
  item = myData("item")

  item.set("size"  , [200, 200, 200])
  item.set("offset", [0, 0])

  return item
  
# ********************************
#  device
# ********************************
def device():
  device = myData("device") 
  plc    = myData("PLC")
  robot1 = myData("robot1")
  robot2 = myData("robot2")
  camera = myData("camera")
  #
  #
  # define device setting here
  #
  #
  device.set("PLC",plc)
  device.set("robot1",robot1)
  device.set("robot2",robot2)
  device.set("camera",camera)
  return device

# ********************************
#  points
# ********************************
def points():
  points     = myData("points")
  pnt        = myData("point")
  position   = myData("position")
  conditions = myData("conditions")

  for i in range(20):
    #
    # define point setting below
    #
    position.set("Mode"    ,i % 2)          # 0~1
    position.set("Surface" ,i % 5)          # 0~4
    position.set("xGrid"   ,[i%10, 10])     # [division id , division number]
    position.set("yGrid"   ,[i%20, 20])     # [division id , division number]
    position.set("xy"      ,[10*i, 10*i])   # [x,y] units=[mm]

    conditions.set("robot1",[100+i,i,0])    # [D1, P1, T1] units=[mm, deg, deg]
    conditions.set("robot2",[200+i,45+i,0]) # [D2, P2, T2] units=[mm, deg, deg]
    conditions.set("camera",[i,0,0])
    conditions.set("light" ,[0,i,0])
      
    pnt.set("point"     , position.copy())
    pnt.set("conditions", conditions.copy())
    
    #
    # Reserved for storing calculation results
    #
    pnt.set("robot1"    , [i,1,2])
    pnt.set("robot2"    , [i,3,4])
    pnt.set("table"     , i)
    points.set("P"+str(i), pnt.copy())

  return points
  
# ********************************
#  traj
#    p1/p2 - robot1 : list
#          - robot2 : list
#          - table  : float
# ********************************
def oneTraj(p1, p2):
  traj = myData("trajectory")
  s    = myData("start")
  g    = myData("goal")
  s.set("robot1", p1.data["robot1"])
  s.set("robot2", p1.data["robot2"])
  s.set("table" , p1.data["table"])
  g.set("robot1", p2.data["robot1"])
  g.set("robot2", p2.data["robot2"])
  g.set("table" , p2.data["table"])
  traj.set("start",s.copy())
  traj.set("goal",g.copy())
  traj.set("traj",[]) # for displaytrajectory
  return traj

# ********************************
#  traj
# ********************************
def traj(points):
  trajs = myData("trajectory")
  list_ = points.idx
  for i,l in enumerate(list_[0:-1]):
    start = points.data[list_[i]]
    goal  = points.data[list_[i+1]]    
    traj  = oneTraj(start, goal)
    lbl   = list_[i] + "-->" + list_[i+1]
    trajs.set(lbl, traj.copy())
  return trajs
  
# ********************************
#  merge data
#    dat: start/goal
# ********************************
def mergeJoint(dat):
  robot1 = dat.get("robot1")
  robot2 = dat.get("robot2")
  table  = dat.get("table")

  ret = []
  if robot1 is not None and robot2 is not None:
    ret.extend(robot1)
    ret.extend(robot2)
    ret.append(table)
  return ret
  
