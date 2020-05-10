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
from math import pi
from myData import *
import rospy
import tf
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose, Vector3, Quaternion, Point, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker

# ********************************
#  size & offset
# ********************************
def getWorkInfo(dat):
  try:
    size   = dat.get("size")
    offset = dat.get("offset")
    if (size is None) or (offset is None) :
      return None,None
  except:
    return None,None
  return size, offset
# ********************************
#  geometry_msgs/posestamped
# ********************************
def getWorkPose(dat):
  size, offset = getWorkInfo(dat)
  if size is None:
    return None, None

  p = Point()
  p.x = size[0]/2.0/1000.0
  p.y = size[1]/2.0/1000.0
  p.z = size[2]/2.0/1000.0

  ret = PoseStamped()
  ret.header.stamp = rospy.Time.now()
  ret.header.frame_id  = "workpiece"
  ret.pose.position    = p
  ret.pose.orientation = Quaternion(0.0, 0.0, 1.0, 0.0)

  return ret, Vector3(2.0*p.x, 2.0*p.y, 2.0*p.z)

# ********************************
#  Marker
# ********************************
def workMarker(dat, pnt=None):
  size, offset = getWorkInfo(dat)
  if size is None:
    return None

  array  = MarkerArray()
  #
  # cube
  #
  m = Marker()
  m.header.frame_id = "workpiece"
  m.header.stamp = rospy.Time.now()
  m.ns     = "work"
  m.id     = 0
  m.type   = Marker.CUBE
  m.action = Marker.ADD
  m.pose.position.x    = size[0]/2.0/1000.0
  m.pose.position.y    = size[1]/2.0/1000.0
  m.pose.position.z    = size[2]/2.0/1000.0
  m.pose.orientation   = Quaternion(0.0, 0.0, 1.0, 0.0)
  m.scale  = Vector3(size[0]/1000.0,size[1]/1000.0,size[2]/1000.0)
  m.color  = ColorRGBA(1,1,1,0.8)
  m.lifetime = rospy.Duration(1)    
  array.markers.append(m)

  #
  # point & text
  #
  if pnt is not None:
    frameList = ["+Z","+X","-X","+Y","-Y"]
    p  = pnt.get("point")
    s  = int(p.get("Surface"))
    xy = p.get("xy")
    ps = Point(xy[0]/1000.0, xy[1]/1000.0, -0.05)
    pe = Point(xy[0]/1000.0, xy[1]/1000.0, 0)

    # arrow
    m = Marker()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = frameList[s]
    m.ns     = "point"
    m.id     = 1
    m.type   = Marker.ARROW
    m.action = Marker.ADD
    m.scale  = Vector3(0.002, # shaft diameter
                       0.007,  # head diameter
                       0.015)  # head length
    m.points.append(ps)
    m.points.append(pe)
    m.color  = ColorRGBA(1,1,1,1)
    m.lifetime = rospy.Duration(1)    
    array.markers.append(m)
      
    # text
    m = Marker()
    m.header.stamp    = rospy.Time.now()
    m.header.frame_id = frameList[s]
    m.ns     = "text"
    m.id     = 1
    m.type   = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.scale  = Vector3(0.03, 0.03, 0.03)
    ps.z = -0.07
    m.pose.position    = ps
    m.pose.orientation = Quaternion(0.0, 0.0, 1.0, 0.0)
    m.text   = pnt.name
    m.color  = ColorRGBA(1,1,1,1)
    m.lifetime = rospy.Duration(1)    
    array.markers.append(m)

    # tf
    c  = pnt.get("conditions")
    r1 = c.get("robot1")
    r2 = c.get("robot2")

    br = tf.TransformBroadcaster()
    if s==0:
      br.sendTransform((pe.x, pe.y, 0), qFromEuler(r1[1]*pi/180.0, 0, pi + r1[2]*pi/180.0),
                       rospy.Time.now(), "point1", frameList[s])
      br.sendTransform((pe.x, pe.y, 0), qFromEuler(r2[1]*pi/180.0, 0, r2[2]*pi/180.0),
                       rospy.Time.now(), "point2", frameList[s])
    else:
      br.sendTransform((pe.x, pe.y, 0), qFromEuler(r1[1]*pi/180.0, r1[2]*pi/180.0, 0),
                       rospy.Time.now(), "point1", frameList[s])
      br.sendTransform((pe.x, pe.y, 0), qFromEuler(r2[1]*pi/180.0, r2[2]*pi/180.0, 0),
                       rospy.Time.now(), "point2", frameList[s])
    br.sendTransform((0, 0, -r1[0]/1000.0), qFromEuler(0, 0, 0),
                     rospy.Time.now(), "robot1", "point1")
    br.sendTransform((0, 0, -r2[0]/1000.0), qFromEuler(0, 0, 0),
                     rospy.Time.now(), "robot2", "point2")
  return array
    
# ********************************
#  Workpiece axes
# ********************************
def workPieceAxes(dat):
  size, offset = getWorkInfo(dat)
  if size is None:
    return None
  # TF
  br = tf.TransformBroadcaster()
  br.sendTransform((offset[0]/1000.0, offset[1]/1000.0, 0), qFromEuler(0, 0, 0),
                   rospy.Time.now(), "workpiece", "table")
  br.sendTransform((size[0]/1000.0, 0, size[2]/1000.0), qFromEuler(0, pi, 0),
                   rospy.Time.now(), "+Z", "workpiece")
  br.sendTransform((size[0]/1000.0, 0, size[2]/1000.0), qFromEuler(-pi/2, 0, pi/2),
                   rospy.Time.now(), "+X", "workpiece")
  br.sendTransform((0, size[1]/1000.0, size[2]/1000.0), qFromEuler(-pi/2, 0, -pi/2),
                   rospy.Time.now(), "-X", "workpiece")
  br.sendTransform((size[0]/1000.0, size[1]/1000.0, size[2]/1000.0), qFromEuler(-pi/2, 0, pi),
                   rospy.Time.now(), "+Y", "workpiece")
  br.sendTransform((0, 0, size[2]/1000.0), qFromEuler(-pi/2, 0, 0), rospy.Time.now(), "-Y", "workpiece")

# ********************************
#  Table axes
# ********************************
def tableAxes(theta=0, x=0, y=0, z=0):
  br = tf.TransformBroadcaster()
  br.sendTransform((x, y, z),qFromEuler(0,0,theta*pi/180.0),rospy.Time.now(), "table", "world")
  
# ********************************
#  qFromEuler
# ********************************
def qFromEuler(r=0,p=0,y=0):
  return tf.transformations.quaternion_from_euler(r, p, y)
  
# ********************************
#  get XY
# ********************************
def getXY(dat, surf, xGrid, yGrid):
  size, offset = getWorkInfo(dat)
  if size is None:
    return None  

  IdxY, Ny = yGrid[0], yGrid[1]
  IdxX, Nx = xGrid[0], xGrid[1]
  if   surf == 0:
    Lx = size[0]
    Ly = size[1]
  elif surf <= 2:    
    Lx = size[1]
    Ly = size[2]
  elif surf <= 4:
    Lx = size[0]
    Ly = size[2]
  else:
    return None
    
  dx = Lx/Nx
  dy = Ly/Ny
  x0 = -dx/2.0
  y0 = -dy/2.0
  X = x0 + (IdxX+1)*dx
  Y = y0 + (IdxY+1)*dy

  return [X,Y]

def getXY_(dat, pnt):
  p     = pnt.get("point")
  mode  = p.get("Mode")
  surf  = p.get("Surface")
  xy    = p.get("xy")
  xGrid = p.get("xGrid")
  yGrid = p.get("yGrid")
 
  if mode == 1: # direct
    return xy 
  else:
    return getXY(dat, surf, xGrid, yGrid)

