# -*- coding: utf-8 -*-
##################################
#
#	 library
#
##################################
import os
import sys
import signal
#********************************
#  Qt
#********************************
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
#********************************
#  Qt form
#********************************
import subform.WorkPiece.form as f1
#####################################
#
# form_manager
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

    self.ui.setupUi(self)
    self.setWindowFlags(QtCore.Qt.Popup)
    
    self.ui.groupBox.setTitle("Item size")
    self.ui.groupBox_2.setTitle("Item offset")

    # spinBox
    self.spinBox = [ self.ui.doubleSpinBox,
                     self.ui.doubleSpinBox_2,
                     self.ui.doubleSpinBox_3,
                     self.ui.doubleSpinBox_4,
                     self.ui.doubleSpinBox_5,
                     self.ui.doubleSpinBox_6]
    self.ui.doubleSpinBox_6.setEnabled(False)
    
    for s in self.spinBox:
      s.valueChanged.connect(self.valueChanged)
    
    # label                 
    self.label   = [ self.ui.label,
                     self.ui.label_2,
                     self.ui.label_3,
                     self.ui.label_4,
                     self.ui.label_5,
                     self.ui.label_6]
    self.lbl     = [ "X [mm]",
                     "Y [mm]",
                     "Z [mm]",
                     "X [mm]",
                     "Y [mm]",
                     "θ [deg]"]
                     
    for i,s in enumerate(self.label):
      s.setText(self.lbl[i])
    
    style = QApplication.instance().style()
    self.button  = [ self.ui.pushButton,
                     self.ui.pushButton_2]
    icon         = [style.standardIcon(QStyle.SP_DialogCancelButton),
                    style.standardIcon(QStyle.SP_DialogOkButton)]
    txt          = ["Cancel","Save"]

    for i,s in enumerate(self.button):
      s.setIcon(icon[i])
      s.setText(txt[i])
      s.clicked.connect(self.clicked)

  # ******************************
  #
  #  value changed
  #
  # ******************************
  def valueChanged(self):
    val = [v.value() for v in self.spinBox]
    self.dat.set("size"  , val[0:3])
    self.dat.set("offset", val[3:5])
  # ******************************
  #
  #  set value
  #
  # ******************************
  def setValue(self, dat):
    self.dat = dat
    self.offset = dat.data["offset"]
    self.size   = dat.data["size"]
    val = []
    val.extend(self.size)
    val.extend(self.offset)
    val.append(0)
    for i,s in enumerate(self.spinBox):
      s.setValue(val[i])
    
  # ******************************
  #
  #  click button
  #
  # ******************************
  def clicked(self):
    obj = self.sender()
    if obj is self.button[1]:
      self.done(1)
    else:
      self.dat.set("size"  , self.size)
      self.dat.set("offset", self.offset)
      self.done(0)  
  # ******************************
  #
  #  close event
  #   (without button control)
  # ******************************
  def closeEvent(self, e):
    self.dat.set("size"  , self.size)
    self.dat.set("offset", self.offset)
    self.done(0)

