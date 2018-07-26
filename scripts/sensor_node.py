#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Alisher Khassanov, alisher@aira.life


from sys import version_info
if version_info[0] < 3:
    raise Exception('python3 required')
try:
    import rospy
except ImportError as e:
    link = 'https://github.com/OTL/cozmo_driver#super-hack-to-run-rospy-from-python3'
    raise(ImportError('How to run ROS on python3: ' + link)) from e
from iiot_data_secure.msg import Operation, OperationData
from std_msgs.msg      import String
from std_srvs.srv      import Empty, EmptyRequest, EmptyResponse
from itertools import cycle
import re
import os


class ExampleDevice: # example of specific data source
    def __init__(self, path, specs):
        self.continous_data_array = cycle([ ['foo', 'bar', 'foobar'],
                                            ['fizz', 'buzz', 'fizzbuzz'] ])

    def read(self):
        return next(self.continous_data_array)


class Sensor: # measurer interface
    def __init__(self, device):
        self.device = device
        self.read = self.device.read


class SensorWrapper: # ROS API wrapper
    def __init__(self, sensor: Sensor):
        rospy.init_node('sensor')
        self.sensor = sensor
        
        self.signing = rospy.Publisher('~signing', Operation, queue_size=1000)
        rospy.Service('~measure', Empty, self.measure)
        rospy.loginfo('Sensor ' + rospy.get_name() + ' node started.')
            
    def measure(self, request: EmptyRequest) -> EmptyResponse: # service callback
        rospy.loginfo('Measure request.')

        sensor_data = self.sensor.read()
        rospy.loginfo(sensor_data)

        data = OperationData()
        # fill up your data here

        msg = Operation()
        msg.data = data
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = rospy.get_name()

        self.signing.publish(msg)
        return EmptyResponse()

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    sensor = Sensor(ExampleDevice()) # specify your data input here
    SensorWrapper(sensor).spin()
