#!/usr/bin/env python
import cv2
import serial
import rospy
import dynamic_reconfigure.client
ser = None
client = None
node = None


def get_param(param):
    global node
    return rospy.get_param(str(node) + str(param), False)


def init_serial(port, baud_rate):
    global ser
    ser = serial.Serial(port, baud_rate)
    if not ser:
        ser.open()
    print('init serial')


def set_frame_rate(frameRate):
    global ser
    print('wait for set frame rate to arduino')
    for i in xrange(10):
        ser.write('set ' + str(frameRate) + '\n'.encode('utf-8'))
        rospy.sleep(0.25)
        print('.')
    print('set_frame_rate: ') + str((frameRate))
    # rospy.sleep(1)


def nothing(variable):
    pass


def main():
    global node, client, ser
    windowName = 'frame_rate_controller'
    trackbarName = 'framerate: '
    frameRate = 15
    port = '/dev/ttyUSB0'
    baud_rate = 115200
    init_serial(port, baud_rate)

    while not rospy.is_shutdown():
        line = ser.readline()
        if not type(line) == type("string"):
            line = str(line)
        if line.find("frame") > -1:
            frameRate = line.split(":")[1]
            try:
                frameRate = int(frameRate)
                break
            except:
                print ('try to get frame rate from arduino')

    print('=========== Get frame rate from arduino ===========')
    cv2.namedWindow(windowName, flags=cv2.WINDOW_NORMAL)
    cv2.moveWindow(windowName, 20, 20)
    cv2.resizeWindow(windowName, 600, 30)
    cv2.createTrackbar(trackbarName, windowName, 1, 30, nothing)
    cv2.setTrackbarPos(trackbarName, windowName, frameRate)

    while not rospy.is_shutdown():
        value = int(cv2.getTrackbarPos(trackbarName, windowName))
        if value == 0:
            frameRate = 1
            cv2.setTrackbarPos(trackbarName, windowName, frameRate)
            set_frame_rate(frameRate)
        elif value != frameRate:
            cv2.setTrackbarPos(trackbarName, windowName, value)
            frameRate = value
            set_frame_rate(frameRate)

        print('Current frame rate: ') + str(frameRate)

        key = cv2.waitKey(1) & 0xff
        if key == ord('q'):
            break
        rospy.sleep(0.1)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('frame_rate_controller')
    main()
