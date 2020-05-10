# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
import subform.Custom.form as f1
import myData.myItem as mi

import copy
#import rospy
from gui_form.srv import *
from std_srvs.srv import *

#####################################
#
#	 Sub functions
#
#####################################
def setRange(ui, Vmin, Vmax, Vstep, ratio=1.0):
  ui.setRange(Vmin*ratio, Vmax*ratio)
  ui.setSingleStep(Vstep*ratio)

#####################################
#
# form_manager
#
#  * parent must have [createSub] member
#
#####################################
class form_manager(QDialog):
  # ******************************
  #
  #  constructor
  #
  # ******************************
  def __init__(self, parent=None):
    super(form_manager, self).__init__()
    self.parent = parent  # Parent dialog
    self.ui = f1.Ui_Dialog()
  # ******************************
  #
  #  destructor
  #
  # ******************************
  def __del__(self):
    print "[form_manager] delete form_manager"

  # ******************************
  #
  #  stop UI
  #
  # ******************************
  def stopUi(self):
    self.parent.createSub = False
    self.reloadDATA()
    print "stop"
  # ******************************
  #
  #  preset UI
  #
  # ******************************
  def setupUi(self,Form):
    #	Cancel the top display setting of the parent dialog
    self.parent.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint, False)
    self.parent.show()
    self.ui.setupUi(Form)

    # ==============================
    #  Qdialog setting
    # ==============================
    style = QApplication.instance().style()
    self.ratio = 0.01
    self.Form  = Form             # f1 form    
    self.size  = Form.size()

    self.txt          = []
    self.label        = []
    self.spinBox      = []
    self.comboBoxList = []
    
    # ==============================
    # tab1
    # ==============================
    self.txt.append( ['m',
                      's',
                      'X idx', 
                      'X div',
                      'Y idx', 
                      'Y div',
                      'X [mm](0-300)', 
                      'Y [mm](0-300)'])
    # label
    self.label.append( [ self.ui.label_8,
                         self.ui.label_9,
                         self.ui.label_10,
                         self.ui.label_11,
                         self.ui.label_12,
                         self.ui.label_13,
                         self.ui.label_14,
                         self.ui.label_15] )
    # comboBox
    self.comboBox = [ self.ui.comboBox,
                      self.ui.comboBox_2 ]
    self.comboBoxList.append( [ "GRID", "DIRECT"] )
    self.comboBoxList.append( [ "+ Z", "+ X", "- X", "+ Y", "- Y"] )
    
    # spinbox
    self.spinBox.append( [ self.ui.spinBox,
                           self.ui.spinBox_2,
                           self.ui.spinBox_3,
                           self.ui.spinBox_4,
                           self.ui.doubleSpinBox_8,
                           self.ui.doubleSpinBox_9] )
                             
    # ==============================
    # tab2
    # ==============================
    self.txt.append( ['D1 [mm]',
                      'P1 [deg]',
                      'T1 [deg]', 
                      'D2 [mm]',
                      'P2 [deg]', 
                      'T2 [deg]',
                      'θ  [deg]'])
    # label
    self.label.append( [ self.ui.label,
                         self.ui.label_2,
                         self.ui.label_3,
                         self.ui.label_4,
                         self.ui.label_5,
                         self.ui.label_6,
                         self.ui.label_7] )
    # spinBox
    self.spinBox.append( [ self.ui.doubleSpinBox,
                           self.ui.doubleSpinBox_2,
                           self.ui.doubleSpinBox_3,
                           self.ui.doubleSpinBox_4,
                           self.ui.doubleSpinBox_5,
                           self.ui.doubleSpinBox_6,
                           self.ui.doubleSpinBox_7] )
    # slider
    self.slider =  [ self.ui.horizontalSlider,
                     self.ui.horizontalSlider_2,
                     self.ui.horizontalSlider_3,
                     self.ui.horizontalSlider_4,
                     self.ui.horizontalSlider_5,
                     self.ui.horizontalSlider_6,
                     self.ui.horizontalSlider_7 ]

    # ==============================
    # preset
    # ==============================
    self.preset()

  #**********************************
  #
  # preset
  #
  #**********************************
  def preset(self):
    self.ui.pushButton.clicked.connect(self.valueChanged)
    self.ui.pushButton.setText('Reset')
    self.ui.pushButton_2.clicked.connect(self.valueChanged)
    self.ui.pushButton_2.setText('Save')
    self.ui.toolButton.setToolTip('config')
    self.ui.toolButton.clicked.connect(self.valueChanged)

    self.preset_tab1()
    self.preset_tab2()

  #**********************************
  #
  # preset tab1
  #
  #**********************************
  def preset_tab1(self):
    # label & groupBox
    self.setLabel(0)
    self.ui.groupBox_4.setTitle("")
    self.ui.groupBox_5.setTitle("")
    self.ui.groupBox_6.setTitle("")

    # comboBox
    for i,v in enumerate(self.comboBox):
      v.activated.connect(self.valueChanged)
      for v2 in self.comboBoxList[i]:
        v.addItem(v2)

    # spinBox
    self.setRange1(False)
    for s in self.spinBox[0]:
      s.valueChanged.connect(self.valueChanged)

  #**********************************
  #
  # preset tab2
  #
  #**********************************
  def preset_tab2(self):
    # label & groupBox
    self.setLabel(1)
    self.ui.groupBox.setTitle("")
    self.ui.groupBox_2.setTitle("")
    self.ui.groupBox_3.setTitle("")

    # spinBox
    for s in self.spinBox[1]:
      s.valueChanged.connect(self.valueChanged)
    vmin = [  0, -90, -180,   0, -90, -180, -180]
    vmax = [400,  90,  180, 400,  90,  180,  180]
    self.setRange2(vmin, vmax, Vstep=[0.01]*7)

    # slider
    for s in self.slider:
      s.valueChanged.connect(self.valueChanged)

  #**********************************
  # [tab1][tab2]
  # set Label
  #
  #**********************************
  def setLabel(self, idx):
    for i,s in enumerate(self.label[idx]):
      s.setText(self.txt[idx][i])
  #**********************************
  # [tab1]
  # Set range and step size
  #
  #**********************************
  def setRange1(self, on=True):
    setRange(self.ui.doubleSpinBox_8,   0, 300, 0.1)
    setRange(self.ui.doubleSpinBox_9,   0, 300, 0.1)
    setRange(self.ui.spinBox_2, 1, 300, 1) # X div
    setRange(self.ui.spinBox_4, 1, 300, 1) # Y div
    if on:
      setRange(self.ui.spinBox,   0, self.ui.spinBox_2.value()-1, 1)
      setRange(self.ui.spinBox_3, 0, self.ui.spinBox_4.value()-1, 1)
    else:
      setRange(self.ui.spinBox,   0, 299, 1)
      setRange(self.ui.spinBox_3, 0, 299, 1)
  #**********************************
  # [tab2]
  # Set range and step size
  #
  #**********************************
  def setRange2(self, Vmin=[0.0]*7, Vmax=[1.0]*7, Vstep=[0.01]*7):
    if (len(Vmin) == 7) and (len(Vmax) == 7) and (len(Vstep) == 7):
      for i,s in enumerate(self.spinBox[1]):
        setRange(s, Vmin[i], Vmax[i], Vstep[i])
      R = 1.0/self.ratio
      for i,s in enumerate(self.slider):
        setRange(s, Vmin[i], Vmax[i], Vstep[i], R)

  #**********************************
  # (SIGNAL)
  # value changed
  #
  #**********************************
  def valueChanged(self):
    print "value changed"
    obj = self.sender()
    #--------------------
    for s in self.comboBox:
      if obj is s:
        self.changeMode(self.ui.comboBox.currentIndex())
        self.calc()
        self.update()
        return        
    #--------------------
    for s in self.spinBox[0]:
      if obj is s:
        self.setRange1()
        self.calc()
        self.update()
        return
    #--------------------
    for s in self.spinBox[1]:
      if obj is s:
        self.spinBoxToSlider()
        self.update()
        return
    #--------------------
    for s in self.slider:
      if obj is s:
        self.sliderToSpinBox()
        self.update()
        return
    #--------------------
    if obj is self.ui.toolButton:
      self.parent.workConfig()
      self.calc()
      self.update()
      return
    #--------------------
    if obj is self.ui.pushButton:   # reset
      self.setDATA()
      return
    #--------------------
    if obj is self.ui.pushButton_2: # save
      self.backup = self.dat.copy()
      return

  #**********************************
  # [tab1]
  # mode setting
  #
  #**********************************
  def changeMode(self, mode):
    if mode==0:
      self.ui.groupBox_5.setEnabled(True)
      self.ui.groupBox_6.setEnabled(False)
    elif mode==1:
      self.ui.groupBox_5.setEnabled(False)
      self.ui.groupBox_6.setEnabled(True)

  #**********************************
  # [tab1][tab2]
  # Block signal
  #
  #**********************************
  def blockSignal(self, on):
    for s in self.spinBox[0]:
      s.blockSignals(on)
    for s in self.spinBox[1]:
      s.blockSignals(on)
    for s in self.slider:
      s.blockSignals(on)

  #**********************************
  # [tab1]
  # grid -> direct
  #
  #**********************************
  def calc(self):
    mode    = self.ui.comboBox.currentIndex()
    if mode == 1: # direct
      return

    surf  = self.ui.comboBox_2.currentIndex()
    xGrid = [self.ui.spinBox.value(),
             self.ui.spinBox_2.value()]
    yGrid = [self.ui.spinBox_3.value(),
             self.ui.spinBox_4.value()]
    xy = mi.getXY(self.parent.dat.get("item"), surf, xGrid, yGrid)
    if xy is not None:
      self.blockSignal(True)
      self.ui.doubleSpinBox_8.setValue(xy[0])
      self.ui.doubleSpinBox_9.setValue(xy[1])
      self.blockSignal(False)
    
  #**********************************
  # [tab2]
  # SpinBox -> Slider
  #
  #**********************************
  def spinBoxToSlider(self):
    self.blockSignal(True)
    for i,s in enumerate(self.slider):
      s.setValue(self.spinBox[1][i].value()/self.ratio)
    self.blockSignal(False)

  #**********************************
  # [tab2]
  # Slider -> SpinBox
  #
  #**********************************
  def sliderToSpinBox(self):
    self.blockSignal(True)
    for i,s in enumerate(self.spinBox[1]):
      s.setValue(self.slider[i].value()*self.ratio)
    self.blockSignal(False)

    
  #**********************************
  #
  # set DATA
  #
  #**********************************
  def reloadDATA(self):
    self.dat.set("point",      self.backup.get("point").copy())
    self.dat.set("conditions", self.backup.get("conditions").copy())
    self.dat.set("table",      self.backup.get("table"))
    self.dat.set("robot1",     copy.deepcopy(self.backup.get("robot1")))
    self.dat.set("robot2",     copy.deepcopy(self.backup.get("robot2")))
    
  def setDATA(self, dat=None, tab=None):
    if dat is not None:
      self.dat = dat
      self.backup = dat.copy()
    else:
      self.reloadDATA()
  
    point      = self.dat.get("point")
    conditions = self.dat.get("conditions")
    table      = self.dat.get("table")

    val1 = []
    val1.append(point.data["Mode"])
    val1.append(point.data["Surface"])
    val1.extend(point.data["xGrid"])
    val1.extend(point.data["yGrid"])
    xy = mi.getXY_(self.parent.dat.get("item"), self.dat)
    if xy is not None:
      val1.extend(xy)
    else:
      val1.extend(point.data["xy"])
      
    val2 = []
    val2.extend(conditions.data["robot1"])
    val2.extend(conditions.data["robot2"])
    val2.append(table)
    
    # set value
    self.setRange1(False)
    self.setValue(0, val1)
    self.setRange1(True)
    self.setValue(1, val2)
    
    if tab is not None:
      self.ui.tabWidget.setCurrentIndex(tab)
    self.changeMode(self.ui.comboBox.currentIndex())
    self.update()
  #**********************************
  # [tab1][tab2]
  # set current value
  #
  #**********************************
  def setValue(self, tab, val):
    if tab==0 and len(val)==8:
      self.blockSignal(True)
      for i,s in enumerate(self.comboBox):
        s.setCurrentIndex(val[i])
      for i,s in enumerate(self.spinBox[0]):
        s.setValue(val[i+2])
      self.calc()
      self.blockSignal(False)
      
    elif tab==1 and len(val)==7:
      self.blockSignal(True)
      for i,s in enumerate(self.spinBox[1]):
        s.setValue(val[i])
      self.spinBoxToSlider()
      self.blockSignal(False)

  #**********************************
  #
  # update data
  #
  #**********************************
  def update(self):
    val1 = self.getValue(0)
    val2 = self.getValue(1)
    
    point = self.dat.get("point")
    point.set("Mode", val1[0])
    point.set("Surface", val1[1])
    point.set("xGrid", val1[2:4])
    point.set("yGrid", val1[4:6])
    point.set("xy", val1[6:8])
    
    conditions = self.dat.get("conditions")
    conditions.set("robot1", val2[0:3])
    conditions.set("robot2", val2[3:6])
    self.dat.set("table", val2[6])
    
  #**********************************
  # [tab1][tab2]
  # get current value
  #
  #**********************************
  def getValue(self, tab):
    if tab==0:
      ret = [s.currentIndex() for s in self.comboBox]
      for s in self.spinBox[tab]:
        ret.append(s.value())
      return ret
    elif tab==1:
      return [s.value() for s in self.spinBox[tab]]
   
  #**********************************
  #
  # send value to parent
  #
  #**********************************
  def sendToParent(self):
    req = EmptyRequest()
    res = self.parent.sendToServer(req, 0)
    return None

