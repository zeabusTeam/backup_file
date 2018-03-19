#!/usr/bin/env python
import sys
import numpy as np
import rospy
from PyQt4 import QtGui, QtCore, Qt
from ui2016 import Ui_MainWindow
from rosgraph_msgs.msg import Log

#import from ROS camera
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#thread
from subprocess import call
from threading import Thread
import subprocess

from functools import partial
import roslib
#roslib.load_manifest("zeabus_vision_stereo")
import os
#import flight_display
__file__="~/catkin_ws/src/zeabus_vision/zeabus_vision_stereo/src"
#sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
sys.path.append(os.path.expanduser(__file__))
#print sys.path
#import beer_test
#print sys.path #commented 11.00 08.06.15
#parentPath = os.path.abspath("../../zeabus_vision/zeabus_vision_stereo/src")
#print parentPath
#if parentPath not in sys.path:
#        sys.path.insert(0, parentPath)

#from zeabus_vision.zeabus_vision_stereo import beer_test
#from zeabus_vision.zeabus_vision_stereo.src import beer_test

#for service
#remove for some with no service
from modbus_ascii_ros.srv import SendCommand
from modbus_ascii_ros.srv import ReceiveStatus
from modbus_ascii_ros.srv import GPIOStatus
from modbus_ascii_ros.srv import BatteryInfo
from modbus_ascii_ros.srv import BarometerMode
#from std_msgs.msg import


path_cam=[]
#config variable
path_cam.append("/front_camera/image")
path_cam.append("/front_camera/image")

service_to_be_killed=['flight','gazebo','rviz','rqt_service_caller']
#commented master,joy,pid,drive
#terminal command
command={}
command['minimal'] = 'roslaunch zeabus_bringup minimal.launch'
command['roscore'] = 'roscore'
# command['joy_f710'] = 'roslaunch zeabus_teleop joy_F710.launch'
# command['joy_extream'] = 'roslaunch zeabus_teleop joy_extream_3d.launch'
command['localization'] = 'roslaunch zeabus_bringup zeabus_dvl_localize.launch'
#command['pid'] = 'roslaunch zeabus_control_bringup PID.launch'
command['pid'] = 'roslaunch controller control.launch'
command['cam_open'] = 'roslaunch zeabus_vision_stereo camera_open.launch'
command['hydrophone'] = 'rosrun zeabus_hydrophone nemo_server'
#command['flight_display'] = 'roslaunch zeabus_ui_bringup ui.launch'
command['flight_display'] = 'rosrun zeabus_ui flight_display.py'
command['start_thruster_all'] = 'rosrun controller thrust_mapper_2016.py'
# command['sim'] = 'roslaunch zeabus_demo transdec_gazebo.launch'
# command['rviz'] = 'rviz'
# command['color_picker'] = 'rosrun zeabus_vision_stereo beer_test.py'
# command['service_caller'] = 'rosrun rqt_service_caller rqt_service_caller'
# command['imu_visualization'] = 'rosrun zeabus_bringup imu_3D_visualization.py'
#cam = 'rosrun rqt_image_view rqt_image_view'

#for thread
t = None

#GUI
#cv_image_right currently doesn't work
cv_image_cam_left = None
cv_image_cam_right = None
notWait = True

severity_labels={1:'DEBUG',2:'INFO',4:'WARN',8:'ERROR',16:'FATAL'}

time=[]

log = {'message':[],'severity':[],'node':[],'stamp':[],'topics':[],'location':[]}
cv_image = []
for time in xrange(6): cv_image.append(None)
class Video():
    def __init__(self,number):
        #global cv_image
        #self.capture = cv_image[number]
        self.currentFrame=np.array([])
        self.number = number
    def captureNextFrame(self):
        """
        capture frame and reverse RBG BGR and return opencv image
        """
        global cv_image
        #ret, readFrame=self.capture.read()
        #readFrame = cv_image_cam_left
        #readFrame = self.capture
        readFrame = cv_image[self.number]
        ret = notWait
        #if(ret==True):
        try:
            self.currentFrame=cv2.cvtColor(readFrame,cv2.COLOR_BGR2RGB)
        except: self.currentFrame = None

    def convertFrame(self):
        """     converts frame to format suitable for QtGui            """
        try:
            height,width=self.currentFrame.shape[:2]
            img=QtGui.QImage(self.currentFrame,
                              width,
                              height,
                              QtGui.QImage.Format_RGB888)
            img=QtGui.QPixmap.fromImage(img)
            self.previousFrame = self.currentFrame
            return img
        except:
            return None

class Gui(QtGui.QMainWindow):
    def __init__(self,parent=None):
        rospy.init_node("gui", anonymous=True)

        #rospy.Subscriber("/rosout", Log, self.callback_table #commented 12.22 10.07.15)
        #print log
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.pushButton.clicked.connect(self.start_cam)
        self.ui.pushButton_3.clicked.connect(lambda : self.start_gui_thread(command['flight_display']))
        #self.ui.pushButton_3.clicked.connect(flight_display.start_display)
        self.ui.pushButton_4.clicked.connect( lambda : self.start_terminal_thread(command['sim']))
        self.ui.pushButton_5.clicked.connect( lambda : self.start_terminal_thread(command['minimal']))
        self.ui.pushButton_6.clicked.connect( lambda : self.start_terminal_thread(command['joy_f710']))
        self.ui.pushButton_7.clicked.connect( lambda : self.start_terminal_thread(command['pid']))
        self.ui.pushButton_8.clicked.connect( lambda : self.start_terminal_thread(command['localization']))
        self.ui.pushButton_9.clicked.connect( lambda : self.start_terminal_thread(command['start_thruster_all']))
        self.ui.pushButton_10.clicked.connect(lambda : self.start_terminal_thread(command['cam_open']))
        self.ui.pushButton_11.clicked.connect(lambda : self.start_gui_thread(command['rviz']))
        self.ui.pushButton_12.clicked.connect(lambda : self.start_terminal_thread(command['joy_f710']))
        self.ui.pushButton_2.clicked.connect( lambda : self.start_terminal_thread(command['color_picker']))
        self.ui.pushButton_17.clicked.connect(lambda : self.start_gui_thread(command['service_caller']))
        self.ui.pushButton_18.clicked.connect(lambda : self.start_gui_thread(command['service_caller']))
        self.ui.pushButton_21.clicked.connect(lambda : self.start_gui_thread(command['rviz']))
        self.ui.pushButton_22.clicked.connect(lambda : self.start_gui_thread(command['rviz']))
        self.ui.pushButton_23.clicked.connect(self.start_stop_all_power_board)
        self.ui.pushButton_24.clicked.connect(self.start_stop_all_sensor_board)
        self.ui.pushButton_25.clicked.connect(lambda : self.start_gui_thread(command['imu_visualization']))
        self.ui.pushButton_26.clicked.connect(lambda : self.start_gui_thread(command['imu_visualization']))
        self.ui.pushButton_32.clicked.connect(self.start_sink_all)
        self.ui.pushButton_33.clicked.connect(self.start_stop_sink_all)
        self.ui.checkBox.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=1))
        self.ui.checkBox_2.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=2))
        self.ui.checkBox_3.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=3))
        self.ui.checkBox_4.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=4))
        self.ui.checkBox_5.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=5))
        self.ui.checkBox_6.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=6))
        self.ui.checkBox_7.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=7))
        self.ui.checkBox_8.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=8))
        #self.ui.checkBox_2.toggled.connect(lambda state, attrName='isChecked': self.update_thruster_2(state, attrName='isChecked', attrType='powerBoard'))
        self.ui.checkBox_9.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=13))
        self.ui.checkBox_10.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=12))
        self.ui.checkBox_11.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=14))
        self.ui.checkBox_12.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=15))
        self.ui.checkBox_13.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='powerBoard',positionOfBit=9))
        self.ui.checkBox_14.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='sensorBoard',positionOfBit=3))
        self.ui.checkBox_15.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='sensorBoard',positionOfBit=4))
        self.ui.checkBox_16.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='sensorBoard',positionOfBit=5))
        self.ui.checkBox_17.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='sensorBoard',positionOfBit=6))
        self.ui.checkBox_18.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='sensorBoard',positionOfBit=7))
        self.ui.checkBox_19.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='sensorBoard',positionOfBit=8))
        self.ui.checkBox_20.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='sensorBoard',positionOfBit=10))
        self.ui.checkBox_21.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='GPIOBoard',positionOfBit=1))
        self.ui.checkBox_22.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='GPIOBoard',positionOfBit=2))
        self.ui.checkBox_23.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='GPIOBoard',positionOfBit=3))
        self.ui.checkBox_24.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='GPIOBoard',positionOfBit=4))
        self.ui.checkBox_25.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='GPIOBoard',positionOfBit=5))
        self.ui.checkBox_26.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='GPIOBoard',positionOfBit=6))
        self.ui.checkBox_27.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='GPIOBoard',positionOfBit=7))
        self.ui.radioButton.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='BaroBoard',positionOfBit=1))
        self.ui.radioButton_2.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='BaroBoard',positionOfBit=2))
        self.ui.radioButton_3.toggled.connect(lambda state, attrName='isChecked': self.update_attribute(state, listName='BaroBoard',positionOfBit=2))
        self.ui.pushButton_14.clicked.connect(self.start_emergency_cut_off_thruster)
        self.ui.pushButton_15.clicked.connect(self.start_stop_thruster)
        self.ui.pushButton_19.clicked.connect(self.start_full_option_mode)
        self.ui.pushButton_13.clicked.connect(self.start_commit_thruster)
        self.ui.pushButton_16.clicked.connect(self.start_commit_sensor)
        self.ui.pushButton_27.clicked.connect(self.start_commit_gpio)
        self.ui.pushButton_34.clicked.connect(self.start_commit_barometer_mode)
        self.ui.pushButton_20.clicked.connect(self.start_thruster_all)
        self.ui.pushButton_31.clicked.connect(self.start_refresh_power_board_bit)
        self.ui.pushButton_30.clicked.connect(self.start_refresh_sensor_board_bit)
        self.ui.pushButton_29.clicked.connect(self.start_refresh_gpio_board_bit)
        self.ui.pushButton_28.clicked.connect(self.start_refresh_battery)
        self.ui.pushButton_15.setStyleSheet('background-color: yellow; color: black')
        self.ui.pushButton_14.setStyleSheet('background-color: red; color:white')
        self.listBitThruster=[0 for number in xrange(16)] # 16 bit in listBit for thruster
        self.listBitSensor=[0 for number in xrange(16)] # 16 bit in listBit for thruster
        self.listBitGPIO=[0 for number in xrange(16)] # 16 bit in listBit for thruster
        #self.ui.pushButton_2.clicked.connect(self.start_cam_right)

        #from setupUi
        self.ui.tableWidget.setColumnCount(len(log.keys()))
        self.ui.tableWidget.setRowCount(10000)
        #item = QtGui.QTableWidgetItem()
        #self.ui.tableWidget.setVerticalHeaderItem(0, item)
        #for rowNumber in xrange(len(log['message'])):
        #    item = QtGui.QTableWidgetItem()
        #    self.ui.tableWidget.setVerticalHeaderItem(rowNumber, item)

        #item = QtGui.QTableWidgetItem()
        #self.ui.tableWidget.setHorizontalHeaderItem(0, item)
        #item = QtGui.QTableWidgetItem()
        #self.ui.tableWidget.setHorizontalHeaderItem(1, item)

        for row_number in xrange(len(log.keys())):
            item = QtGui.QTableWidgetItem()
            self.ui.tableWidget.setHorizontalHeaderItem(row_number, item)

        #item = QtGui.QTableWidgetItem()
        #self.ui.tableWidget.setItem(0, 0, item)
        #item = QtGui.QTableWidgetItem()
        #self.ui.tableWidget.setItem(0, 1, item)

        for column_number in xrange(len(log.keys())):
            item = QtGui.QTableWidgetItem()
            self.ui.tableWidget.setItem(0, column_number, item)

        #from retranslateUi

        #item = self.ui.tableWidget.verticalHeaderItem(0)
        #item.setText(_translate("MainWindow", "1", None))

        #for rowNumber in xrange(len(log['message'])):
        #    item = self.ui.tableWidget.verticalHeaderItem(rowNumber)
        #    item.setText(_translate("MainWindow", "1", None))

        #item = self.ui.tableWidget.verticalHeaderItem(1)
        #item.setText(_translate("MainWindow", "1", None))

        #for rowNumber in xrange(len(log['message'])):
        #    item = self.ui.tableWidget.verticalHeaderItem(rowNumber)
        #    item.setText(_translate("MainWindow", "1", None))

        """
        item = self.ui.tableWidget.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "New Column", None))
        item = self.ui.tableWidget.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "message", None))
        """

        for row_number,row_name in enumerate(log.keys()):
            item = self.ui.tableWidget.horizontalHeaderItem(row_number)
            item.setText(_translate("MainWindow", row_name, None))


        #__sortingEnabled = self.ui.tableWidget.isSortingEnabled()
        #self.ui.tableWidget.setSortingEnabled(False)

        """
        item = self.ui.tableWidget.item(0, 0)
        item.setText(_translate("MainWindow", "1", None))
        item = self.ui.tableWidget.item(0, 1)
        item.setText(_translate("MainWindow", "2", None))
        """
        #checkbox for each bitlist
        self.list_checkbox_thruster=[self.ui.checkBox,
                                self.ui.checkBox_2,
                                self.ui.checkBox_3,
                                self.ui.checkBox_4,
                                self.ui.checkBox_5,
                                self.ui.checkBox_6,
                                self.ui.checkBox_7,
                                self.ui.checkBox_8,
                                self.ui.checkBox_13,
                                None,
               None,
               self.ui.checkBox_10,
               self.ui.checkBox_9,
               self.ui.checkBox_11,
               self.ui.checkBox_12,
               None]
        self.list_checkbox_sensor=[None,
               None,
               self.ui.checkBox_14,
               self.ui.checkBox_15,
               self.ui.checkBox_16,
               self.ui.checkBox_17,
               self.ui.checkBox_18,
               self.ui.checkBox_19,
               None,
               self.ui.checkBox_20,
               None,
               None,
               None,
               None,
               None,
               None]
        self.list_checkbox_gpio=[self.ui.checkBox_21,
               self.ui.checkBox_22,
               self.ui.checkBox_23,
               self.ui.checkBox_24,
               self.ui.checkBox_25,
               self.ui.checkBox_26,
               self.ui.checkBox_27,
               self.ui.checkBox_28,
               self.ui.checkBox_29,
               self.ui.checkBox_30,
               self.ui.checkBox_31,
               self.ui.checkBox_32,
               None,
               None,
               None,
               None]

        self.video = {}
        self.update()

    def start_cam(self,number):

        for num in xrange(len(path_cam)):
            print num
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber(path_cam[num],Image,self.callback_cam,num)

            self.video[num] = Video(num)
            self._timer = QtCore.QTimer(self)
            self._timer.timeout.connect(partial(self.play_cam,number=num))
            self._timer.start(27)

    def play_cam(self,number):
            try:
                self.video[number].captureNextFrame()


                videoFrame = {

                    0 : self.ui.videoFrame,
                    1 : self.ui.videoFrame_2,
                    2 : self.ui.videoFrame_3,
                    3 : self.ui.videoFrame_4,
                    4 : self.ui.videoFrame_5,
                    5 : self.ui.videoFrame_6,

                }


                videoFrame[number].setPixmap(
                    self.video[number].convertFrame())
                videoFrame[number].setScaledContents(True)

            except TypeError:
                print "No frame from cam left"

    def callback_cam(self,data,num):
        global cv_image
        try:
          cv_image[num] = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
          print e

    def update_attribute(self,state,listName,positionOfBit):
        print state
        #self.listBitThruster

        #for clipboard copying


        if listName=="powerBoard":
            self.listBitThruster[positionOfBit-1]=int(state)
            self.ui.lineEdit.setText(str(self.listBitThruster))
            self.ui.lineEdit.setFocus()
            #for clipboard copying
            myClipBoard = QtGui.QApplication.clipboard()
            myClipBoard.setText(str(self.listBitThruster))
        elif listName=="sensorBoard":
            self.listBitSensor[positionOfBit-1]=int(state)
            self.ui.lineEdit_2.setText(str(self.listBitSensor))
            self.ui.lineEdit_2.setFocus()
            #for clipboard copying
            myClipBoard = QtGui.QApplication.clipboard()
            myClipBoard.setText(str(self.listBitSensor))
        elif listName=="GPIOBoard":
            self.listBitGPIO[positionOfBit-1]=int(state)
            self.ui.lineEdit_4.setText(str(self.listBitGPIO))
            self.ui.lineEdit_4.setFocus()
            #for clipboard copying
            myClipBoard = QtGui.QApplication.clipboard()
            myClipBoard.setText(str(self.listBitGPIO))
        elif listName=="BaroBoard":
            if self.ui.radioButton_3.isChecked()==True:
                self.ui.lineEdit_3.setEnabled(True)
            else: self.ui.lineEdit_3.setEnabled(False)
    #start or stop checkbox
    def start_emergency_cut_off_thruster(self):
        self.ui.checkBox_9.setCheckState(QtCore.Qt.Unchecked)
        self.listBitThruster[13-1]=int(False)

        for number in xrange(8):
            self.list_checkbox_thruster[number].setCheckState(QtCore.Qt.Unchecked)
            self.listBitThruster[number]=int(False)

        #for commit
        service_thruster = rospy.ServiceProxy('/zeabus_modbus_master/set_regulator_aux',SendCommand)
        status_thruster = False
        try: status_thruster= service_thruster(self.listBitThruster).result
        except:
            self.ui.label_5.setStyleSheet('color: red')
            self.ui.label_5.setText('Status: '+"unable to connect with thruster service, please try again")
        if status_thruster==True:
            QtGui.QMessageBox.critical(self,"Emergency cut off thruster","thrusters stopped with emergency cut off")

        else:
            self.ui.label_5.setStyleSheet('color: red')
            self.ui.label_5.setText('Status: '+"unable to connect with thruster service, please try again")
    def start_stop_thruster(self):
        for number in xrange(8):
            self.list_checkbox_thruster[number].setCheckState(QtCore.Qt.Unchecked)
            self.listBitThruster[number]=int(False)
        #for clipboard copying
        myClipBoard = QtGui.QApplication.clipboard()
        myClipBoard.setText(str(self.listBitThruster))

        #for commit
        service_thruster = rospy.ServiceProxy('/zeabus_modbus_master/set_regulator_aux',SendCommand)
        status_thruster = False
        try: status_thruster= service_thruster(self.listBitThruster).result
        except:
            self.ui.label_5.setStyleSheet('color: red')
            self.ui.label_5.setText('Status: '+"unable to connect with thruster service, please try again")

        if status_thruster==True:
            QtGui.QMessageBox.critical(self,"stop thruster","thrusters stopped")
        else:
            self.ui.label_5.setStyleSheet('color: red')
            self.ui.label_5.setText('Status: '+"unable to connect with thruster service, please try again")
    def start_thruster_all(self):
        for number in xrange(8):
            self.list_checkbox_thruster[number].setCheckState(QtCore.Qt.Checked)
            self.listBitThruster[number]=int(True)
        self.list_checkbox_thruster[13-1].setCheckState(QtCore.Qt.Checked)
        self.listBitThruster[13-1]=int(True)
    def start_stop_all_power_board(self):
        for checkbox in self.list_checkbox_thruster:
            if checkbox!=None:
                checkbox.setCheckState(QtCore.Qt.Unchecked)
        self.listBitThruster=[0 for number in xrange(16)] # 16 bit in listBit for thruster
        #for clipboard copying
        myClipBoard = QtGui.QApplication.clipboard()
        myClipBoard.setText(str(self.listBitThruster))
    def start_stop_all_sensor_board(self):
        for checkbox in self.list_checkbox_sensor:
            if checkbox!=None:
                checkbox.setCheckState(QtCore.Qt.Unchecked)
        self.listBitSensor=[0 for number in xrange(16)] # 16 bit in listBit for thruster
    def start_sink_all(self):
        for number in xrange(7):
            self.listBitGPIO[number]=int(True)
            self.list_checkbox_gpio[number].setCheckState(QtCore.Qt.Checked)
    def start_stop_sink_all(self):
        for number in xrange(7):
            self.listBitGPIO[number]=int(False)
            self.list_checkbox_gpio[number].setCheckState(QtCore.Qt.Unchecked)

    def start_full_option_mode(self):
        #thruster
        for number,checkbox in enumerate(self.list_checkbox_thruster):
            if checkbox==None: continue
            checkbox.setCheckState(QtCore.Qt.Checked)
            self.listBitThruster[number]=int(True)

        #sensor
        for number,checkbox in enumerate(self.list_checkbox_sensor):
            if checkbox==None: continue
            checkbox.setCheckState(QtCore.Qt.Checked)
            self.listBitSensor[number]=int(True)

        #gpio
        for number,checkbox in enumerate(self.list_checkbox_gpio):
            if number>=7: break
            if checkbox==None: continue
            checkbox.setCheckState(QtCore.Qt.Checked)
            self.listBitGPIO[number]=int(True)
    def start_commit_thruster(self):
        #for service
        #remove for some with no service
        service_thruster = rospy.ServiceProxy('/zeabus_modbus_master/set_regulator_aux',SendCommand)
        status_thruster = False
        try: status_thruster= service_thruster(self.listBitThruster).result
        except:
            self.ui.label_5.setStyleSheet('color: red')
            self.ui.label_5.setText('Status: '+"unable to connect with thruster service, please try again")
        #QtGui.QMessageBox.information(self,"Information",str(status_thruster)+"/"+str(type(status_thruster)))

        if status_thruster == True:
            self.ui.label_5.setStyleSheet('color: green')
            self.ui.label_5.setText('Status: '+"thruster have been committed with value: "+str(self.listBitThruster))
        else:
            self.ui.label_5.setStyleSheet('color: red')
            self.ui.label_5.setText('Status: '+"thruster haven't updated, please try again")
    def start_commit_sensor(self):
        #remove for some with no service
        service_sensor = rospy.ServiceProxy('/zeabus_modbus_master/set_regulator_main',SendCommand)
        status_sensor = False
        try: status_sensor = service_sensor(self.listBitSensor).result
        except:
            self.ui.label_5.setStyleSheet('color: red')
            self.ui.label_5.setText('Status: '+"unable to connect with sensor service, please try again")
        #QtGui.QMessageBox.information(self,"Information","sensor have been committed with value: "+str(self.listBitSensor))
        if status_sensor == True:
            self.ui.label_6.setStyleSheet('color: green')
            self.ui.label_6.setText('Status: '+"sensor have been committed with value: "+str(self.listBitSensor))
        else:
            self.ui.label_6.setStyleSheet('color: red')
            self.ui.label_6.setText('Status: '+"sensor haven't updated, please try again")
    def start_commit_gpio(self):
        #remove for some with no service
        service_gpio = rospy.ServiceProxy('/zeabus_modbus_master/set_gpio',SendCommand)
        status_gpio = False
        try: status_gpio = service_gpio(self.listBitGPIO).result
        except:
            self.ui.label_5.setStyleSheet('color: red')
            self.ui.label_5.setText('Status: '+"unable to connect with gpio service, please try again: "+str(sys.exc_info()))
        #QtGui.QMessageBox.information(self,"Information","sensor have been committed with value: "+str(self.listBitSensor))
        if status_gpio == True:
            self.ui.label_16.setStyleSheet('color: green')
            self.ui.label_16.setText('Status: '+"gpio have been committed with value: "+str(self.listBitGPIO))
        else:
            self.ui.label_16.setStyleSheet('color: red')
            self.ui.label_16.setText('Status: '+"gpio haven't updated, please try again")
    def start_commit_barometer_mode(self):
        service_barometer = rospy.ServiceProxy('/zeabus_modbus_master/set_barometer_mode',BarometerMode)
        status_barometer = False
        #adcMode = 2
        #if adcMode==2: windowSize = 8
        #else: windowSize = 0
        #status_barometer = service_barometer(adcMode,windowSize).result
        adcMode = 0 # set default value for barometer mode here 20.04 09.07.15
        windowSize = 128 # set default value for barometer mode here 20.04 09.07.15
        if self.ui.radioButton.isChecked()==True: adcMode = 0
        elif self.ui.radioButton_2.isChecked()==True: adcMode = 1
        elif self.ui.radioButton_3.isChecked()==True: adcMode = 2
        if adcMode == 2:
            try: windowSize = int(self.ui.lineEdit_3.text())
            except:
                self.ui.lineEdit_3.setText(str(128))
        #status_barometer = service_barometer(adcMode,windowSize).result #for debug, commented 20.14 10.07.15
        try: status_barometer = service_barometer([adcMode,windowSize]).result
        except:
            self.ui.label_5.setStyleSheet('color: red')
            self.ui.label_5.setText('Status: '+"unable to connect with barometer service, please try again with error: "+str(sys.exc_info()))

        if status_barometer == True:
            self.ui.label_16.setStyleSheet('color: green')
            self.ui.label_16.setText('Status: '+"barometer have been committed with value: "+str(adcMode)+','+str(windowSize))
        else:
            self.ui.label_16.setStyleSheet('color: red')
            self.ui.label_16.setText('Status: '+"barometer haven't been committed, please try again with error: "+str(sys.exc_info()))
            QtGui.QMessageBox.critical(self,"barometer status","unable to update gpio status, please try again"+str(sys.exc_info()))
    #update bit from service
    def start_refresh_power_board_bit(self):
        service_thruster = rospy.ServiceProxy('/zeabus_modbus_master/get_regulator_aux',ReceiveStatus)
        receive = [0 for number in xrange(16)]
        try: receive = service_thruster(0).result
        except:
            self.ui.label_5.setStyleSheet('color: red')
            self.ui.label_5.setText('Status: '+"unable to refresh bit from thruster service, please try again: "+str(sys.exc_info()))
            return 0
        self.listBitThruster=[int(value) for value in receive]
        self.ui.lineEdit.setText(str(self.listBitThruster))
        for number,checkbox in enumerate(self.list_checkbox_thruster):
            if checkbox==None: continue
            if self.listBitThruster[number]==True:
                checkbox.setCheckState(QtCore.Qt.Checked)
            else: checkbox.setCheckState(QtCore.Qt.Unchecked)
        #self.ui.checkBox_20.setCheckState(QtCore.Qt.Checked)
        self.ui.label_5.setStyleSheet('color: green')
        self.ui.label_5.setText('Status: '+"able to refresh bit from thruster service with value "+str(self.listBitThruster))
    def start_refresh_sensor_board_bit(self):
        service_sensor = rospy.ServiceProxy('/zeabus_modbus_master/get_regulator_main',ReceiveStatus)

        receive=[0 for number in xrange(16)]
        try: receive = service_sensor(0).result
        except:
            self.ui.label_6.setStyleSheet('color: red')
            self.ui.label_6.setText('Status: '+"unable to refresh bit from sensor service, please try again: "+str(sys.exc_info()))
        self.listBitSensor=[int(value) for value in receive]
        self.ui.lineEdit_2.setText(str(self.listBitSensor))
        for number,checkbox in enumerate(self.list_checkbox_sensor):
            if checkbox==None: continue
            if self.listBitSensor[number]==True:
                checkbox.setCheckState(QtCore.Qt.Checked)
            else: checkbox.setCheckState(QtCore.Qt.Unchecked)
        self.ui.label_6.setStyleSheet('color: green')
        self.ui.label_6.setText('Status: '+"able to refresh bit from sensor service, with value "+str(self.listBitSensor))
    def start_refresh_gpio_board_bit(self):
        service_gpio = rospy.ServiceProxy('/zeabus_modbus_master/get_gpio',GPIOStatus)
        print 'service_gpio = '
        print service_gpio
        service_gpio_tmp = service_gpio(0)

        #for number in xrange(7): receive[number]=int(False)
        receive=[0 for number in xrange(16)]
        receive = service_gpio_tmp.controlBits
        try:
            receive = service_gpio_tmp.controlBits
        except:
            self.ui.label_16.setStyleSheet('color: red')
            self.ui.label_16.setText('Status: '+"unable to refresh bit from gpio service, please try again"+str(sys.exc_info()))
            #QtGui.QMessageBox.critical(self,"gpio status","unable to get gpio status, please try again"+str(sys.exc_info()))
            return False
        self.listBitGPIO=[int(value) for value in receive]
        self.ui.lineEdit_4.setText(str(self.listBitGPIO))
        for number,checkbox in enumerate(self.list_checkbox_gpio):
            if checkbox==None: continue
            if self.listBitGPIO[number]==True:
                checkbox.setCheckState(QtCore.Qt.Checked)
            else: checkbox.setCheckState(QtCore.Qt.Unchecked)
        #self.ui.checkBox_20.setCheckState(QtCore.Qt.Checked)

        #barometer_mode=3
        #barometer_window_size=8
        barometer_mode=service_gpio_tmp.adcMode
        barometer_window_size=service_gpio_tmp.windowSize
        checkbox_baro=[self.ui.radioButton,self.ui.radioButton_2,self.ui.radioButton_3]
        print barometer_mode
        checkbox_baro[barometer_mode].setChecked(QtCore.Qt.Checked)
        if barometer_mode==2:
            self.ui.lineEdit_3.setEnabled(True)
            self.ui.lineEdit_3.setText(str(barometer_window_size))
        self.ui.label_16.setStyleSheet('color: green')
        self.ui.label_16.setText('Status: '+"able to refresh bit from gpio service completely with value"+str(self.listBitGPIO)+','+str(barometer_mode)+','+str(barometer_window_size))
    def start_refresh_battery(self):
        service_battery = rospy.ServiceProxy('/zeabus_modbus_master/get_battery_info',BatteryInfo)
        receive = None
        try: receive = service_battery(0)
        except:
            #self.ui.label_6.setText('Status: '+"unable to refresh bit from sensor service, please try again: "+'+'.join(sys.exc_info()))
            QtGui.QMessageBox.critical(self,"battery status","unable to get battery status, please try again"+str(sys.exc_info()))
            return False
        #self.listBitGPIO=receive
        #self.listBitSensor=receive
        voltage = receive.voltage
        current = receive.current
        temperature = receive.temperature
        charge = receive.charge
        self.ui.label_12.setText('Voltage: '+str(voltage)+" V")
        self.ui.label_13.setText('Current: '+str(current)+" A")
        self.ui.label_14.setText('Temperature: '+str(temperature)+" Degree Celsius")
        self.ui.progressBar.setProperty("value", charge)


    def start_terminal_thread(self,cmd):
        cmd = 'xterm -e '+'"'+cmd+' ; sleep 5" &'
        call(cmd,shell=True)

    def start_gui_thread(self,cmd):

        cmd = cmd+' &'
        call(cmd,shell=True)



    def closeEvent(self, event):
        # do stuff
        print 'bye'
        for service_name in service_to_be_killed:
            subprocess.Popen("kill -9 $(ps aux | grep "+service_name+" | cut -d ' ' -f 2)", shell=True, stdout=subprocess.PIPE)
        event.accept() # let the window close



    def callback_table(self,data):
        global log
        print 'enter callback_table'
        log['message'].append(str(data.msg))
        log['stamp'].append(str(float(data.header.stamp.secs)/10**9)+' seconds')
        log['severity'].append(severity_labels[int(data.level)])
        log['node'].append(str(data.name))
        log['topics'].append(','.join(data.topics))
        log['location'].append(str(data.file)+':'+str(data.function)+':'+str(data.line))


        #from retranslateUi
        for row_number in xrange(len(log['message'])):
            for column_number,column_name in enumerate(log.keys()):
                item = QtGui.QTableWidgetItem()
                item.setText(log[column_name][row_number])
                #for your information
                #severity_labels={1:'DEBUG',2:'INFO',4:'WARN',8:'ERROR',16:'FATAL'}
                if log['severity'][row_number]=='WARN':
                    item.setBackgroundColor(QtGui.QColor("yellow"))
                elif log['severity'][row_number]=='DEBUG':
                    item.setBackgroundColor(QtGui.QColor("black"))
                    item.setTextColor(QtGui.QColor("white"))
                elif log['severity'][row_number]=='ERROR':
                    item.setBackgroundColor(QtGui.QColor("blue"))
                    item.setTextColor(QtGui.QColor("white"))
                elif log['severity'][row_number]=='FATAL':
                    item.setBackgroundColor(QtGui.QColor("red"))
                    item.setTextColor(QtGui.QColor("white"))
                self.ui.tableWidget.setItem(row_number, column_number, item)

        #print log
        #self.ui.tableWidget.resizeColumnsToContent()
        #self.ui.tableWidget.resizeRowsToContent()

        #make each column spread their width
        header = self.ui.tableWidget.horizontalHeader()
        header.setStretchLastSection(True)

        #self.ui.tableWidget.setSortingEnabled(__sortingEnabled)


try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

def main():
    app = QtGui.QApplication(sys.argv)
    ex = Gui()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
