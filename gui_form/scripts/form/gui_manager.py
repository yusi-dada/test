# -*- coding: utf-8 -*-
##################################
#
#	 library
#
##################################
import os
import sys
import signal
import threading
#********************************
#  data construct
#********************************
import myData.myData as md
import myData.myDataProcess as mdp
import myData.myItem as mi
#********************************
#  ROS
#********************************
import rospy
import tf
from visualization_msgs.msg import MarkerArray
from moveit_msgs.msg import DisplayTrajectory
from gui_form.srv import *
from std_srvs.srv import *

#********************************
#  Qt
#********************************
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
#********************************
#  Qt form
#********************************
from form import Ui_MainWindow # Qt gui form
from qt.Qtw.module import *
from qt.Qpopup.popUpDialog import *
import subform.RobotState.form_manager as fm
import subform.WorkPiece.form_manager as wp
import subform.Custom.form_manager as ct

############################################
#
# Functions
#
############################################
def setIcon(twi, num=None):
  style = QApplication.instance().style()
  icon = [style.standardIcon(QStyle.SP_DialogCloseButton), # NG
          style.standardIcon(QStyle.SP_DialogOkButton),    # OK
          style.standardIcon(QStyle.SP_ToolBarVerticalExtensionButton),        # pause
          style.standardIcon(QStyle.SP_ArrowRight)]        # →

  if num is not None:
    twi.setIcon(0,icon[int(num)])       # set Icon
  else:
    name = twi.icon(0).name()
    for i,n in enumerate(icon):
      if QtGui.QIcon(n).name() == name: # get Icon
        return i
  return None

############################################
#
# Threading
#
############################################
std_lock = threading.Lock()
class pub_thread(threading.Thread):
  # ******************************
  #
  #  constructor
  #
  # ******************************
  def __init__(self, std_lock, master, task=0):
    super(pub_thread, self).__init__()
    self.daemon = True
    self.std_lock = std_lock
    self.master = master
    self.task   = task
    
  # ******************************
  #
  #  thread execution
  #
  # ******************************
  def run(self):
    if self.master.connectedToServer == True:
      with self.std_lock:
        if self.task==0:    # One pose
          tw = self.master.ui.treeWidget
          item = tw.currentItem()
          self.task0(tw, item)   
        elif self.task==1:  # All pose
          tw = self.master.ui.treeWidget
          self.task1(tw)
        elif self.task==2:  # One traj from current pos
          tw = self.master.ui.treeWidget
          self.task2(tw)
        elif self.task==3:  # One traj
          tw = self.master.ui.treeWidget_2
          item = tw.currentItem()
          self.task3(tw, item)
        elif self.task==4:  # All traj
          self.task4(self.master.ui.treeWidget_2)

    self.master.processing = False
      
  # ******************************
  #
  #  task 0
  #
  # ******************************
  def task0(self, tw, item):
    setIcon(item, 3)   # right arrow

    pnt = self.master.dat.get("points")
    p   = pnt.get(str(item.text(0)))
    point = p.get("point")
    conditions = p.get("conditions")

    #
    #  request setting
    #

    req = EmptyRequest()
    res = self.master.sendToServer(req, 0)



    if res is None: # service failed
      pass
    else:
      setIcon(item, 1) # OK

    tw.scroll(0,1)
    tw.scroll(0,-1)
    
  # ******************************
  #
  #  task 1
  #
  # ******************************
  def task1(self, tw):
    for i in range(tw.topLevelItemCount()):
      item = tw.topLevelItem(i)
      self.task0(tw, item)
    
  # ******************************
  #
  #  task 2
  #
  # ******************************
  def task2(self, tw):
    pass

  # ******************************
  #
  #  task 3
  #
  # ******************************
  def task3(self, tw, item):
    setIcon(item, 3)   # right arrow

    traj = self.master.traj.get(str(item.text(0)))
    s = traj.get("start")
    g = traj.get("goal")

    print mdp.mergeJoint(s)
    print mdp.mergeJoint(g)

    # service
    req = planningRequest()
    req.mechno = 0
    req.start = [1,2,3]
    req.goal  = [1,2,3]

    pose, size = mi.getWorkPose(self.master.dat.get("item"))
    if pose is not None:
      req.pose.append(pose)
      req.vec3.append(size)

    res = self.master.sendToServer(req, 1)

    if res is None: # service failed
      pass
    else:
      setIcon(item, 1) # OK

    tw.scroll(0,1)
    tw.scroll(0,-1)

  # ******************************
  #
  #  task 4
  #
  # ******************************
  def task4(self, tw):
    for i in range(tw.topLevelItemCount()):
      item = tw.topLevelItem(i)
      self.task3(tw, item)
  
##################################
#
#	 QT GUI-Form manager
#
##################################
class gui_manager(QMainWindow):
  # ******************************
  #
  #  constructor
  #
  # ******************************
  def __init__(self,parent=None):
    super(gui_manager, self).__init__(parent)
    self.ui = Ui_MainWindow()
    self.ui.setupUi(self)

    # ============================
    # flags & settings
    # ============================
    self.createSub  = False
    self.processing = False
    self.connectedToServer = False
    #self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint) # always on top
    self.setWindowTitle('PyQt5 Window')
    self.setFixedSize(self.size())
    # ============================
    # database
    # ============================
    
    # if you create sample data 
    dat = mdp.dummyData()
    dat.save()
    
    self.dat  = md.myData()
    self.traj = md.myData()
    
    self.dispPoint = md.myData()
    # ============================
    # Publisher
    # ============================
    self.trajPub   = rospy.Publisher('DisplayTraj', DisplayTrajectory, queue_size=1)
    self.mArrayPub = rospy.Publisher("marker", MarkerArray, queue_size = 1)
    self.listener  = tf.TransformListener()
    # ============================
    # menu bar setting
    # ============================
    #self.ui.actionOpen.triggered.connect(self.signalCatch)
    #self.ui.actionSave.triggered.connect(self.signalCatch)
    #self.ui.actionExit.triggered.connect(self.signalCatch)

    # ============================
    # treewidget setting
    # ============================
    style = QApplication.instance().style()
    tw = self.ui.treeWidget
    tw.setContextMenuPolicy( QtCore.Qt.CustomContextMenu )
    tw.customContextMenuRequested.connect( self.createDialog )
    tw.currentItemChanged.connect( self.updateDialog )
    tw.itemDoubleClicked.connect(lambda: self.renameItem(tw))

    tw.setAnimated(True)
    tw.setMouseTracking(True)
    tw.setExpandsOnDoubleClick(False)

    self.tw_shortcut = []
    self.tw_shortcut.append(QShortcut(QtGui.QKeySequence("Space"), tw))
    self.tw_shortcut[0].activated.connect(lambda: self.dat.hide(tw))

    self.pButton = [self.ui.pushButton,     # load
                    self.ui.pushButton_2,   # save
                    self.ui.pushButton_3,   # new
                    self.ui.pushButton_4,   # remove
                    self.ui.pushButton_5,   # up
                    self.ui.pushButton_6,   # down
                    self.ui.pushButton_7,   # plan all
                    self.ui.pushButton_8]   # plan one 
    txt          = ['','','','','▲','▼','All','One']
    toolTip      = ['<b>Load</b>',
                    '<b>Save</b>',
                    '<b>Copy/New</b>',
                    '<b>Remove</b>',
                    '<b>Up</b>',
                    '<b>Down</b>',
                    '',
                    '']
    icon         = [style.standardIcon(QStyle.SP_DirClosedIcon),
                    style.standardIcon(QStyle.SP_DriveFDIcon),
                    style.standardIcon(QStyle.SP_FileIcon),
                    style.standardIcon(QStyle.SP_TrashIcon),
                    None,
                    None,
                    None,
                    None]
    for i,p in enumerate(self.pButton):
      p.clicked.connect(self.signalCatch_TW)
      p.setToolTip(toolTip[i])
      p.setText(txt[i])
      if icon[i] is not None:
        p.setIcon(icon[i])

    self.ui.toolButton.clicked.connect(self.signalCatch_TW)

    # ============================
    # treewidget setting 2
    # ============================
    tw2 = self.ui.treeWidget_2
    tw2.currentItemChanged.connect( self.dispTraj )
    
    tw2.setAnimated(True)
    tw2.setMouseTracking(True)
    tw2.setExpandsOnDoubleClick(False)

    self.pButton2 = [self.ui.pushButton_10,   # load
                     self.ui.pushButton_11,   # save
                     self.ui.pushButton_12,   # reload
                     self.ui.pushButton_13,   # cancel
                     self.ui.pushButton_14,   # play
                     self.ui.pushButton_15,   # pause
                     self.ui.pushButton_16,   # pause
                     self.ui.pushButton_17]   # stop
    txt           = ['','','','','','','','']
    toolTip       = ['<b>Load</b>',
                     '<b>Save</b>',
                     '<b>Reload</b>',
                     '<b>Cancel</b>',
                     '<b>Execute</b>',
                     '<b>Step</b>',
                     '<b>Pause</b>',
                     '<b>Stop</b>']
    icon          = [style.standardIcon(QStyle.SP_DirClosedIcon),
                     style.standardIcon(QStyle.SP_DriveFDIcon),
                     style.standardIcon(QStyle.SP_BrowserReload),
                     style.standardIcon(QStyle.SP_BrowserStop),
                     style.standardIcon(QStyle.SP_MediaPlay),
                     style.standardIcon(QStyle.SP_MediaSeekForward),
                     style.standardIcon(QStyle.SP_MediaPause),
                     style.standardIcon(QStyle.SP_MediaStop)]
    for i,p in enumerate(self.pButton2):
      p.clicked.connect(self.signalCatch_TW2)
      p.setToolTip(toolTip[i])
      p.setText(txt[i])
      if icon[i] is not None:
        p.setIcon(icon[i])

  # ******************************
  #
  #  destructor
  #
  # ******************************
  def __del__(self):
    print "delete gui_manager"
    
  # ******************************
  #
  # showMsg
  #
  # ******************************
  def showMsg(self, string, time):
    self.statusBar().showMessage(string)
    self.tim = time
  
  # ******************************
  #
  # timer
  #
  # ******************************
  def setTimer(self, ms):
    if ms > 0:
      self.tim2s = 0
      self.tim = 0
      self.interval = ms
      self.timer = QtCore.QTimer(self)
      self.timer.timeout.connect(self.update)	# Interval function
      self.timer.start(self.interval)					# msec
    else:
      print "Interval timer setting error."
  
  def update(self):
    if self.tim > 0:
      self.tim = self.tim - self.interval
      if self.tim <= 0:
        self.statusBar().showMessage("")
    
    self.tim2s = self.tim2s - self.interval
    if self.tim2s <= 0:
      self.tim2s = 2000
      if self.connectedToServer == False:
        try:
          self.showMsg("サーバ接続中...", 1000)
          rospy.wait_for_service('dummyServer', timeout=0.01)
          rospy.wait_for_service('PlanningServer', timeout=0.01)
        except:
          print "timeout"
          return
        self.showMsg("サーバ接続中...完了", 2000)
        self.connectedToServer = True
  # ******************************
    
    tw  = self.ui.treeWidget
    idx0, idx1, idx2 = getIndex(tw)
    if idx0 >= 0:
      points = self.dat.data["points"]
      p = points.get(idx0)

      # publish axes
      mi.tableAxes(p.get("table"), x=0, y=0, z=0.1)
      mi.workPieceAxes(self.dat.get("item"))

      # publish marker
      array = mi.workMarker(self.dat.get("item"), p)
      if array is not None:
        self.mArrayPub.publish(array)

      self.getPose()
      
  # ******************************
  # (SIGNAL) button clicked
  #  signal catch 
  #
  # ******************************
  def signalCatch_TW(self):
    if self.connectedToServer == False:
      print "Not connected to server"
      return
    obj = self.sender()
    tw  = self.ui.treeWidget
    tw2 = self.ui.treeWidget_2
    idx0, idx1, idx2 = getIndex(tw)
    Qyes = QMessageBox.Yes
    Qno  = QMessageBox.No
    #-----------------------------
    # load
    #-----------------------------
    if obj is self.ui.pushButton:
      if tw.topLevelItemCount() != 0:
        ret = QMessageBox.question(self, "Warning","Load new file?", Qyes|Qno, Qno)
        if ret == Qno:
          return
      path = self.dat.load()
      self.dat.data["points"].tree(tw)
      self.dat.hide(tw)

      for i in range(tw.topLevelItemCount()):
        setIcon(tw.topLevelItem(i), 2) # pause

      self.th1 = pub_thread(std_lock, self, 1)
      self.th1.start()
      self.proceccing = True

      self.showMsg("Load: " + path, 2000)
      return
    #-----------------------------
    # save
    #-----------------------------
    if obj is self.ui.pushButton_2:
      path = self.dat.save()
      self.showMsg("Save: " + path, 2000)
      return
    #-----------------------------
    # new
    #-----------------------------
    if obj is self.ui.pushButton_3:
      print "New"
      return
    #-----------------------------
    # tool
    #-----------------------------
    if obj is self.ui.toolButton:
      self.workConfig()
      return
    #-----------------------------
    # all
    #-----------------------------
    if obj is self.ui.pushButton_7:
      if tw.topLevelItemCount() < 2:
        self.showMsg("データ数不足（2点以上）", 2000)
      else:
        if not self.checkIcon(tw, 1): # not all ok
          QMessageBox.warning(self, 'Warn', 'Not all data is valid.', QMessageBox.Ok)
          return
        if self.clrTraj(tw2):
          self.ui.tabWidget.setCurrentIndex(1)
          self.traj = mdp.traj(self.dat.data["points"])
          self.traj.tree(tw2)
          self.traj.hide(tw2)
          self.traj.save("/traj")
          for i in range(tw2.topLevelItemCount()):
            setIcon(tw2.topLevelItem(i), 2) # pause

          self.th1 = pub_thread(std_lock, self, 4)
          self.th1.start()
          self.proceccing = True
      return      

    #-----------------------------
    if idx0 is None:
      return
    #-----------------------------

    #-----------------------------
    # delete
    #-----------------------------
    elif obj is self.ui.pushButton_4: # delete
      if self.clrTraj(tw2):
        self.dat.data["points"].delete(idx0)
        self.dat.data["points"].tree(tw)
    #-----------------------------
    # up
    #-----------------------------
    elif obj is self.ui.pushButton_5: # up
      if self.clrTraj(tw2):
        self.dat.data["points"].swap(idx0,idx0-1)
        upBranch(tw)
    #-----------------------------
    # down
    #-----------------------------
    elif obj is self.ui.pushButton_6: # down
      if self.clrTraj(tw2):
        self.dat.data["points"].swap(idx0,idx0+1)
        downBranch(tw)
    #-----------------------------
    # one
    #-----------------------------
    elif obj is self.ui.pushButton_8: # One
      item = tw.currentItem()
      if setIcon(item)==1: # OK icon
        if self.clrTraj(tw2):
          # P_cur = myData()
          #self.traj = mdp.oneTraj(P_cur, self.dat.data["points"].get(idx0))
          print self.dat.data["points"].get(idx0).disp()
          pass
          
      print "One"
      
  # ******************************
  # (SIGNAL) tw double clicked
  # change text of twi
  #
  # ******************************
  def renameItem(self, tw):
    item = tw.currentItem()
    idx0, idx1, idx2 = getIndex(tw)

    if idx1==-1:
      s, ret = QInputDialog.getText(self,'Rename','New name?',0,item.text(0))
      s = s.replace(' ', '')
      if ret and (s!="") and (s!=item.text(0)):
        if isUnique(tw,s):
          self.dat.data["points"].rename(idx0, str(s))
          ### change trajectory name here ###
          item.setText(0,s)
        else:
          QMessageBox.warning(self, 'Warn', 'Already exist!', QMessageBox.Ok)
          self.renameItem(tw)

  # ******************************
  # (SIGNAL) tw clicked
  # popUpDialog
  #
  # ******************************
  def updateDialog(self):
    #idx0, idx1, idx2 = getIndex(self.ui.treeWidget)
    #if idx1==-1:
    #  print "Service!!!"

    if self.createSub == True:
      keep = self.pop.keepWindow
      qPos = self.pop.pos()
      self.pop.stopUi()
      self.createDialog()
      self.pop.setKeepWindow(keep)
      self.pop.move(qPos)

  # ******************************
  def createDialog(self):
    idx0, idx1, idx2 = getIndex(self.ui.treeWidget)
    if idx0 is None or idx0 == -1:
      print "no process"
      return

    if self.createSub == False:
      points = self.dat.data["points"]
      p = points.get(idx0)

      item = self.ui.treeWidget.currentItem()
      lbl  = item.text(0)
      if lbl=="point" or lbl=="conditions" or idx1==-1:
        self.pop = popUpDialog(ct.form_manager(self))
        self.pop.parent.setDATA(p, max(0, idx1))
        self.createSub = True

      elif lbl=="robot1" or lbl=="robot2":
        print "robot"

  # ******************************
  # (SIGNAL) button clicked 2
  #  signal catch 
  #
  # ******************************
  def signalCatch_TW2(self):
    if self.connectedToServer == False:
      print "Not connected to server"
      return
    obj = self.sender()
    tw2 = self.ui.treeWidget_2
    idx0, idx1, idx2 = getIndex(tw2)
    #-----------------------------
    # load
    #-----------------------------
    if obj is self.ui.pushButton_10:
      print "load"
      return
    #-----------------------------
    # save
    #-----------------------------
    if obj is self.ui.pushButton_11:
      print "save"
      return
    #-----------------------------
    # reload
    #-----------------------------
    if obj is self.ui.pushButton_12:
      print "reload"
      return
    #-----------------------------
    # cancel
    #-----------------------------
    if obj is self.ui.pushButton_13:
      print "cancel"
      return
    #-----------------------------
    # execute
    #-----------------------------
    if obj is self.ui.pushButton_14:
      print "execute"
      if not self.checkIcon(tw2, 1): # not all ok
        QMessageBox.warning(self, 'Warn', 'Not all data is valid.', QMessageBox.Ok)
        return
      return
    #-----------------------------
    # step
    #-----------------------------
    if obj is self.ui.pushButton_15:
      print "step"
      return
    #-----------------------------
    # pause
    #-----------------------------
    if obj is self.ui.pushButton_16:
      print "pause"
      return
    #-----------------------------
    # stop
    #-----------------------------
    if obj is self.ui.pushButton_17:
      print "stop"
      return
  # ******************************
  # (SIGNAL) tw clicked
  # clear trajectory
  #
  # ******************************
  def dispTraj(self):
    idx0, idx1, idx2 = getIndex(self.ui.treeWidget_2)
    if idx1==-1:
      traj = self.traj.get(idx0).get("traj")
      if isinstance(traj, DisplayTrajectory):
        self.trajPub.publish(traj)
        print "Publish!!! "

  # ******************************
  # 
  # clear trajectory
  #
  # ******************************
  def clrTraj(self, tw):
    if tw.topLevelItemCount() > 0:
      ret = QMessageBox.question(self, "Warning",
            "Delete the generated\n trajectory as well?", 
             QMessageBox.Yes|QMessageBox.No, QMessageBox.No)
      if ret == QMessageBox.No:
        return False
      else:
        tw.clear()
        self.traj = md.myData()
        return True
    else:
      return True
  
  # ******************************
  #
  # get work size
  #
  # ******************************
  def getWorkOffset(self):
    d = self.dat.get("item")
    if d is not None:
      return d.get("offset")
    return None
    
  # ******************************
  #
  # get work size
  #
  # ******************************
  def getWorkSize(self):
    d = self.dat.get("item")
    if d is not None:
      return d.get("size")
    return None

  # ******************************
  #
  # set work config
  #
  # ******************************
  def workConfig(self):
    if self.dat.get("item") is None:
      QMessageBox.warning(self, 'Warn', 'Target data is not set!', QMessageBox.Ok)
    else:
      self.work = wp.form_manager(self)
      self.work.setValue(self.dat.data["item"])
      #self.work.move(QtGui.QCursor.pos())
      self.work.move(self.pos() + QtCore.QPoint(20,20))
      if self.work.exec_():
        print "recalcurate!"
  # ******************************
  #
  # Check icon
  #
  # ******************************
  def checkIcon(self, tw, idx):
    N = tw.topLevelItemCount()
    for i in range(N):
      item = tw.topLevelItem(i)
      if setIcon(item) != idx:
        return False
    return True # All icon is idx

  # ******************************
  #
  # Event
  #
  # ******************************
  def closeEvent(self, e):
    print "close gui_manager"
    ret = QMessageBox.question(self, "Closing","Quit without saving?",
     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
    if ret == QMessageBox.Yes:
      print("Yes clicked.")
      self.saveSetting()
    else:
      print("No clicked.")
      
  # ******************************
  #
  # Servise Client
  #
  # ******************************
  def sendToServer(self, req, task=0):
    print "Service: task=" + str(task)   
    try:
      if task==0:
        func = rospy.ServiceProxy('dummyServer', Empty)
      elif task==1:
        func = rospy.ServiceProxy('PlanningServer', planning)
        
      res = func(req)
      return res
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e 
      self.connectedToServer = False
      return None
  # ******************************
  #
  # getPose
  #
  # ******************************
  def getPose(self):
    try:
      (trans,rot) = self.listener.lookupTransform('/world', '/robot1', rospy.Time(0))
      #print trans
      #print rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      pass
    
