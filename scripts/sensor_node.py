#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Alisher A. Khassanov, alisher@aira.life


from sys import version_info
if version_info[0] < 3:
    raise Exception('python3 required')
try:
    import rospy
except ImportError as e:
    link = 'https://github.com/OTL/cozmo_driver#super-hack-to-run-rospy-from-python3'
    raise(ImportError('How to run ROS on python3: ' + link)) from e
from iiot_data_secure.msg import Operation, OperationData
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from openpyxl import load_workbook
from warnings import filterwarnings
if __name__ == '__main__':
    filterwarnings("ignore")
from flask import Flask, request
from flask.json import jsonify
from urllib.parse import urlparse
from itertools import cycle
import re
import os


letter_to_number = lambda c: ord(c) % 32 # calc number of an alphabet

TABLE_FILE = os.path.dirname(os.path.realpath(__file__)) + '/input_file.xlsm'
TABLE_SPECS = { 'worksheet': 0,
                'header': { 'begin': { 'col': letter_to_number('A'),
                                       'row': 11
                                     },
                            'end':   { 'col': letter_to_number('S'),
                                       'row': 11
                                     }
                          },
                'data': { 'begin': { 'col': letter_to_number('A'),
                                     'row': 12
                                   },
                          'end':   { 'col': letter_to_number('S'),
                                     'row': 160
                                   }
                        }
              }


class Table: # specific data source
    def __init__(self, path, specs):
        self.path = path
        self.specs = specs
        self.data_header, self.data_array = self.xlstolist(path, specs)
        self.continous_data_array = cycle(self.data_array)

    def read(self):
        return next(self.continous_data_array)

    @staticmethod
    def xlstolist(xls_path, xls_specs):
        wb = load_workbook(filename=xls_path, read_only=True, data_only=True)
        print(wb.get_sheet_names())

        ws = wb[wb.get_sheet_names()[ xls_specs['worksheet'] ]]
        rows = list(ws.rows)

        header_row = xls_specs['header']['begin']['row'] - 1
        header_col_begin = xls_specs['header']['begin']['col'] - 1
        header_col_end = xls_specs['header']['end']['col']
        header = [ cell.value for cell in rows[header_row] ] [header_col_begin:header_col_end]

        data_rows = rows[xls_specs['data']['begin']['row']-1 :
                         xls_specs['data']['end']['row']]

        def clear_data(header, rows):
            def drop_unsued(header, rows):
                # drop columns without header (with None value header)
                for col, val in enumerate(header):
                    if val is None:
                        del header[col]  # drop from header
                        for row in rows:
                            del row[col] # drop from each row
                return header, rows
            header, rows = drop_unsued(header, list(rows))
            header = [re.sub('\n', '', s) for s in header] # drop string breaks
            return header, rows

        data_rows = list(rows[xls_specs['data']['begin']['row']-1 :
                              xls_specs['data']['end']['row']])
        data = list(map(lambda row: [cell.value for cell in row], data_rows))
        data_header, data_array = clear_data(header, data)
        return data_header, data_array


class Sensor: # measurer interface
    def __init__(self, device):
        self.device = device
        self.read = self.device.read


class SensorWrapper: # ROS API wrapper
    def __init__(self, sensor):
        rospy.init_node('sensor')
        self.sensor = sensor
        
        self.signing = rospy.Publisher('~signing', Operation, queue_size=10)
        rospy.Service('~measure', Empty, self.measure)
        rospy.loginfo('Sensor ' + rospy.get_name() + ' node started.')
            
    def measure(self, request):
        rospy.loginfo('Measure request.')

        sensor_data = self.sensor.read()
        data = OperationData()
        data.deposit = str(sensor_data[0])
        data.bush = str(sensor_data[1])
        data.well = str(sensor_data[2])
        data.date = str(sensor_data[3])
        data.number = str(sensor_data[4])
        data.type = str(sensor_data[5])
        data.depth_start = int(sensor_data[6])
        data.depth_finish = int(sensor_data[7])
        data.weight_start = int(sensor_data[8])
        data.weight_finish = int(sensor_data[9])
        data.t_start = int(sensor_data[10])
        data.t_finish = int(sensor_data[11])
        data.t_operation = int(sensor_data[12])
        data.t_acc = int(sensor_data[13])
        data.t_acc_coil = int(sensor_data[14])
        data.t_m = int(sensor_data[15])
        data.comment = str(sensor_data[16])

        msg = Operation()
        msg.data = data
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = rospy.get_name()
        self.signing.publish(msg)

        return EmptyResponse()

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    sensor = Sensor(Table(TABLE_FILE, TABLE_SPECS))
    SensorWrapper(sensor).spin()
