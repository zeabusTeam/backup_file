#!/usr/bin/env python
import rospy
from auto_exposure import AutoExposure


def main():
    EVmin = 0.4
    EVdefault = 0.7

    subTopicC = rospy.get_param(
        "/auto_exposure_top/imageTopicC", None)
    clientC = rospy.get_param(
        "/auto_exposure_top/imageClientC", None)

    if not subTopicC is None:
        AEC = AutoExposure(subTopicC, clientC, EVdefault, EVmin)
        AEC.adjust_exposure_time()

if __name__ == '__main__':
    rospy.init_node('Auto_Exposure_Top')
    main()
