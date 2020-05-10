#! /usr/bin/python
# -*- coding: utf-8 -*-
# ********************************
#
#	 library
#
# ********************************
import rospy
import os
import sys
import signal

from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui

from form.gui_manager import *
# ********************************
#
#	 signal
#
# ********************************
def sigint_handler(*args):
  QApplication.quit()
# ********************************
#
#	 main
#
# ********************************
if __name__ == '__main__':
  rospy.init_node('gui_form', anonymous=True) 
  signal.signal(signal.SIGINT, sigint_handler)

  print(sys.version)

  app = QApplication(sys.argv)
  
  window = gui_manager()
  window.show()
  window.setTimer(100)    # timer setting
  
  sys.exit(app.exec_())
