# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
#####################################
#
# popup dialog format
#
#   * parent must have [setupUi] method
#   * parent must have [stopUi] method
#
#####################################
class popUpDialog(QDialog):
  #**********************************
  #
  # constructor
  #
  #**********************************
  def __init__(self, parent=None):
    QDialog.__init__(self,parent)

    self.parent = None
    if parent is not None:
      self.parent = parent
      self.parent.setupUi(self)

    # always on top + frameless
    self.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint, True)
    self.setWindowFlag(QtCore.Qt.FramelessWindowHint, True)

    # keepWindow flag
    self.setObjectName("ObjName1");
    self.setKeepWindow(False)
    self.setMouseTracking(True)

    # initial cursor position
    self.qPos_pre = QtCore.QPoint(5,5)
    self.size = self.size() 
    
    # show window at current cursor position
    self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
    self.show()
    self.move(QtGui.QCursor.pos() - self.qPos_pre)

  #**********************************
  #
  # destructor
  #
  #**********************************
  def __del__(self):
    print "[popUpDialog] delete popUp Dialog"

  #**********************************
  #
  # cursor leave event
  #
  #**********************************
  def leaveEvent(self, e):
    print "[popUpDialog] leaveEvent"
    
    ePos = self.mapFromGlobal(QtGui.QCursor.pos())
    if ((0 < ePos.x()) and (ePos.x() < self.size.width()) and
        (0 < ePos.y()) and (ePos.y() < self.size.height())):
      return
    if self.keepWindow == False:
      self.stopUi()
      print "[popUpDialog] leaveEvent done"

  def stopUi(self):
    self.parent.stopUi()
    self.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint, False)
    self.close()

  #**********************************
  #
  # keep window setting
  #
  #**********************************
  def setKeepWindow(self, on):
    self.keepWindow = on
    if self.keepWindow:
      self.setStyleSheet("#ObjName1"
      "{border-style:solid; border-color:blue; border-width:3px;}");
    else:
      self.setStyleSheet("#ObjName1 {}");
    
        
  #**********************************
  #
  # mouse click event
  #
  #  * left : save cursor position
  #  * right: keep window or not
  #
  #**********************************
  def mousePressEvent(self,e):
    print "[popUpDialog] mousePress Event"
    
    ePos = self.mapFromGlobal(QtGui.QCursor.pos())
    if e.buttons() == QtCore.Qt.LeftButton:
      self.qPos_pre = e.pos()
    elif e.buttons() == QtCore.Qt.RightButton:
      self.setKeepWindow(not self.keepWindow)
          
  #**********************************
  #
  # mouse move event
  #
  #**********************************
  def mouseMoveEvent(self, e):
    print "[popUpDialog] mouseMoveEvent"
    if e.buttons() == QtCore.Qt.LeftButton:
      self.move(e.globalPos() - self.qPos_pre)   
    elif e.buttons() == QtCore.Qt.RightButton:
      pass

  #**********************************
  #
  # key press event
  #
  #**********************************
  def keyPressEvent(self, e):
    pass
