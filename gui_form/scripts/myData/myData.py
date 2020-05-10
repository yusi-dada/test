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
#  Qt
#********************************
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
#********************************
#  ROS
#********************************
from moveit_msgs.msg import DisplayTrajectory

# ********************************
#
# Sub functions
#
# ********************************
def uni2float(s):
  if s is None:
    return s
  elif type(s) is list:
    try:
      return [float(v) for v in s]
    except ValueError:
      return [str(v) for v in s]
  else:
    try:
      return float(s)
    except ValueError:
      return s

# ********************************
#
# Class
#
# ********************************
class myData():
  def __init__(self, name=""):
    self.name  = name  # root name
    self.data  = {}    # 辞書データ
    self.idx   = []    # ページ番号リスト
  #===============================
  # Deep copy
  #===============================
  def copy(self):
    ret = myData()
    ret.name  = self.name
    ret.data  = copy.deepcopy(self.data)
    ret.idx   = copy.deepcopy(self.idx)
    return ret
  #===============================
  # get data name
  #===============================
  def getName(self, idx):
    if idx < 0:
      return None
    try:
      return(self.idx[idx])
    except:
      return None
  #===============================
  # get data index
  #===============================
  def getIndex(self, lbl):
    try:
      return(self.idx.index(lbl))
    except:
      return None
  #===============================
  # get data info
  #===============================
  def getInfo(self, val):
    try:
      if type(val) is str:
        idx = self.getIndex(val)
        lbl = self.getName(idx)
      else:
        lbl = self.getName(val)
        idx = self.getIndex(lbl)
      return idx, lbl
    except:
      return None, None
  #===============================
  # set data
  #===============================
  def set(self, name, dat, idx=None):
    idx_, lbl = self.getInfo(name)
    if idx_ is None: # new data
      self.data[name] = dat
      if idx is None:
        self.idx.append(name)
      else:
        self.idx.insert(idx, name)
    else: # data is exist
      self.data[lbl] = dat
  #===============================
  # get data
  #===============================
  def get(self, idx):
    idx,lbl = self.getInfo(idx)
    if idx is not None:
      return self.data[lbl]
    else:
      return None
  #===============================
  # delete data
  #===============================
  def delete(self, idx):
    idx,lbl = self.getInfo(idx)
    if idx is not None:
      del self.data[lbl]
      del self.idx[idx]
      return True
    else:
      print "[del] error."
      return False # failed
  #===============================
  # swap data
  #===============================
  def swap(self, idx0, idx1):
    idx0,lbl = self.getInfo(idx0)
    idx1,lbl = self.getInfo(idx1)
    if idx0 is not None and idx1 is not None:
      self.idx[idx0], self.idx[idx1] = self.idx[idx1], self.idx[idx0]
      return True
    else:
      print "[swap] error."
      return False # failed
  #===============================
  # rename data
  #===============================
  def rename(self, old, new):
    idx,lbl = self.getInfo(old)
    if idx is not None:
      self.data[new] = self.data.pop(lbl)
      self.idx[idx] = new
      return True
    else:
      print "[rename] error"
      return False # failed
  #===============================
  # get data level
  #===============================
  def getLevel(self):
    def getlevel(dat, count=1):
      l = dat.idx # list
      maxlevel = 0
      for i,s in enumerate(l):
        if isinstance(dat.data[s], myData):
          ret = getlevel(dat.data[s], count)
          maxlevel = max(maxlevel,ret) 
      return count + maxlevel
    return getlevel(self)-1
  #===============================
  # display data
  #===============================
  def disp(self):
    def dispData(dat,space=" "):
      l = dat.idx # list
      for i,s in enumerate(l):
        if isinstance(dat.data[s], myData):
          if i==len(l)-1:
            print space + '└ <' + s + ">"
            dispData(dat.data[s], space+"   ")
          else:
            print space + '├ <' + s + ">"
            dispData(dat.data[s], space+"|  ")
        else:
          if i==len(l)-1:
            v = space + '└ '
          else:
            v = space + '├ '
          print v + s + " : " + str(dat.data[s])
   
    print "<"+self.name+">"
    dispData(self)
  #===============================
  # create tree
  #===============================
  def tree(self, tw, level=99):

    def itree(tw, dat, level=99):
      l = dat.idx # list
      for i,s in enumerate(l):
        b = QTreeWidgetItem(tw)
        b.setText(0, s)
        if isinstance(dat.data[s], myData):
          if level-1 >= 0:
            itree(b, dat.data[s], level-1)
          else:
            b.setText(1, "myData")   
        elif isinstance(dat.data[s], DisplayTrajectory): # custom
          b.setText(1, "Trajectory")                     # custom
        else:
          b.setText(1, str(dat.data[s]))

    tw.clear()
    tw.setColumnCount(2)
    tw.setColumnWidth(0, 110)
    tw.setHeaderLabels([self.name, "value"])
    tw.setAlternatingRowColors(True)
    tw.setColumnHidden(1, True)
    itree(tw, self, level)
  #===============================
  # hide/show tree column
  #===============================
  def hide(self, tw):
    if type(tw) is not list:
      tw = [tw]
    for v in tw:
      flg = v.isColumnHidden(1)
      v.setColumnHidden(1,not flg)
  #===============================
  # Save data
  #===============================
  def save(self,fname="/tmp"):

    def saveItem(f, dat):
      list_ = dat.idx
      f.setValue("__INDEX__", list_)
      for l in list_:
        d = dat.data[l]
        if isinstance(d, myData):
          f.beginGroup(l)
          saveItem(f,d)
          f.endGroup()
        else:
          f.setValue(l, d)

    path = os.path.dirname(os.path.abspath(sys.argv[0]))
    setting = QtCore.QSettings(path+fname+".ini", QtCore.QSettings.IniFormat)
    saveItem(setting, self)
    return path+fname+".ini"
  #===============================
  # Load data
  #===============================
  def load(self,fname="/tmp"):

    def loadItem(f,name="root"):
      dat = myData()
      dat.name = name
      list_ = f.value("__INDEX__")
      dat.idx = [str(v) for v in list_]
      for l in dat.idx:
        f.beginGroup(l)
        list_ = f.value("__INDEX__")
        if list_ is not None:
          dat.set(l,loadItem(f,l))
          f.endGroup()
        else:
          f.endGroup()
          dat.set(l,uni2float(f.value(l)))
      return dat.copy()
    
    path = os.path.dirname(os.path.abspath(sys.argv[0]))
    setting = QtCore.QSettings(path+fname+".ini", QtCore.QSettings.IniFormat)
    dat = loadItem(setting)
    self.data = copy.deepcopy(dat.data)
    self.idx  = copy.deepcopy(dat.idx)
    self.name = dat.name
    self.disp()
    return path+fname+".ini"

