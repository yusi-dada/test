#! /usr/bin/python
# -*- coding: utf-8 -*-

import os
import datetime
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui

################################
#
# get Index
#
################################
def getIndex(tw):
  item = tw.currentItem()
  if item is None:
    return [None, None, None]
  index = tw.indexOfTopLevelItem(item)
  if index == -1:
    top  = item.parent()
    idx0 = tw.indexOfTopLevelItem(top)
    idx1 = top.indexOfChild(item)
    idx2 = tw.currentColumn()
  else:
    idx0 = index
    idx1 = -1
    idx2 = tw.currentColumn()
  return [idx0, idx1, idx2]
################################
#
# is unique
#
################################
def isUnique(tw, s):
  findList = tw.findItems(s,QtCore.Qt.MatchExactly)
  if len(findList)!=0:
    return False
  else:
    return True  # unique
################################
#
# Expand alone
#
################################
def expandAlone(tw):
  idx0, idx1, idx2 = getIndex(tw)
  if idx0 == None:
    return

  item = tw.topLevelItem(idx0)
  tw.collapseAll()
  tw.expandItem(item)
  #tw.setCurrentItem(item)

################################
#
# Up parent branch
#
#  * True : succeeded
#  * False: failed
#
################################
def upBranch(tw):
  item = tw.currentItem()
  if item is None:
    return False
  index = tw.indexOfTopLevelItem(item)
  if index < 0:
    parent = item.parent()
    index = tw.indexOfTopLevelItem(parent)
  if index > 0:
    above  = tw.topLevelItem(index-1)
    above_ = above.clone()
    expand = above.isExpanded()
    tw.takeTopLevelItem(index-1) # remove current branch
    tw.insertTopLevelItem(index, above_)
    if(expand):
      tw.expandItem(above_)
    return True
  return False

################################
#
# Down parent branch
#
#  * True : succeeded
#  * False: failed
#
################################
def downBranch(tw):
  item = tw.currentItem()
  if item is None:
    return False

  index = tw.indexOfTopLevelItem(item) # get current branch index of tree widget
  nidx = tw.topLevelItemCount()
  if index < 0:
    parent = item.parent()
    index = tw.indexOfTopLevelItem(parent)
  if index is not -1 and index < nidx-1:
    below  = tw.topLevelItem(index+1)
    below_ = below.clone()
    expand = below.isExpanded()
    tw.takeTopLevelItem(index+1) # remove current branch
    tw.insertTopLevelItem(index,below_)
    if(expand):
      tw.expandItem(below_)
    return True
  return False
