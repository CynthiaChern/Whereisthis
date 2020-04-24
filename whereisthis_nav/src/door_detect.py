#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# author: Tianyi Li
# time: 2020/3/25
# 使用激光的前方平均长度作为开门的判断标准
# 使用正前方角度10度（或者5）的数据作为平均值。
# 注意，由于距离过短时激光数据也会失效，因此检测门与机器人的距离必须大于激光的最短检测。

import rospy
import roslib
import numpy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from control.msg import whereisthis

class doorcheck():
    def __init__(self):
        # 状态值
        self.open_door = False
        self.times = 0
        self.max_times = 50
        self.realizing = False
        self.msg = whereisthis()

        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scanCallback, queue_size=1)
        # self.open_pub = rospy.Publisher("/image/oepndoor", Bool, queue_size=1)
        # 控制流
        self.control_pub = rospy.Subscriber("/whereisthis_control",whereisthis,self.controlCallback,queue_size=1)
        self.control_pub = rospy.Publisher("/whereisthis_control",whereisthis,queue_size=1)

        
    def controlCallback(self, msg=whereisthis()):
        self.msg = msg
        if self.msg.NowTask == self.msg.Door_recognition:
            self.realizing = True
            rospy.loginfo("Start door recognition!")
        else:
            self.realizing = False
    
    def scanCallback(self, data):
        if self.msg.NowTask == self.msg.Door_recognition and self.msg.FinishState == False:
            if self.realizing == True:
                # 距离
                distance = 0.0
                number = 0
                # 0度的索引
                zero_ind = abs(data.angle_min)/data.angle_increment
                # 获得-5到5的平均值
                range_5 = int(5/(57.5*data.angle_increment))
                min_ind = int(max(0,zero_ind - range_5))
                max_ind = int(min(len(data.ranges), zero_ind + range_5))
                while min_ind<max_ind:
                    min_ind+=1
                    if numpy.isnan(data.ranges[min_ind]):
                        continue
                    distance += data.ranges[min_ind]
                    number+=1
                    # 当过远时，激光数据会输出nan，因此如果全部nan，则可以认为前方没有障碍物
                if distance == 0.0:
                    distance = 10.0
                else:
                    distance = distance/number
                if distance >2.0:
                    self.times +=1
                else:
                    self.times = 0
                if self.times > self.max_times:
                    self.open_door = True
                    rospy.loginfo("Door is open!")
                # 对接控制流
                self.msg.FinishState = self.open_door
                self.control_pub.publish(self.msg)
                print(distance)
                print(self.open_door)
                print("===============")


if __name__ == '__main__':
    rospy.init_node('whereisthis_door_detect', anonymous=True)
    doorcheck = doorcheck()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
