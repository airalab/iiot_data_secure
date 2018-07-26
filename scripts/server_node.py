#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Alisher A. Khassanov, alisher@aira.life


from sys import version_info
if version_info[0] < 3:
    raise Exception('python3 required')
try:
    import rospy
    import rosbag
except ImportError as e:
    link = 'https://github.com/OTL/cozmo_driver#super-hack-to-run-rospy-from-python3'
    raise(ImportError('How to run rospy from python3: ' + link)) from e
from iiot_data_secure.srv import GetSensorsList, GetSensorsListRequest
from iiot_data_secure.srv import GetSensorData, GetSensorDataRequest
from iiot_data_secure.srv import SetSensorData, SetSensorDataRequest
from iiot_data_secure.msg import Operation, OperationData
from std_msgs.msg      import String, Empty
from std_srvs.srv      import Empty as EmptySrv, EmptyRequest, EmptyResponse 
from flask             import Flask, request, Response
from flask_restful     import reqparse, Resource, Api
from flask.json        import jsonify
from flask_cors        import CORS
from urllib.parse      import urlparse
import os.path
import ipfsapi
import json


class SimulationAPI:
    def __init__(self):
        self.measurements_per_sensor = int(rospy.get_param('~operations_per_sensor', 10))
        self.sensors_ns = rospy.get_param('~sensors_namespace') # </your/namespace/to/>sensor_x

        sensors_etc/ssls = set()
        sensors_topics = rospy.get_published_topics(namespace=self.sensors_ns)
        for topic, _ in sensors_topics:
            ns_end = topic.find('sensor')
            tail = topic[ns_end:]
            name_end = tail.find('/')
            name = tail[:name_end]
            name_with_ns = topic[:topic.find(name) + len(name)]
            sensors_etc/ssls.add(name_with_ns)
        self.sensors_etc/ssls = list(sensors_etc/ssls)

        self.sensorsProxies = list()
        rospy.loginfo('Connecting to sensors proxies:')
        for topic in self.sensors_etc/ssls:
            rospy.loginfo(topic)
            rospy.wait_for_service(topic + '/measure')
            self.sensorsProxies.append(rospy.ServiceProxy(topic + '/measure', EmptySrv))

        rospy.loginfo('Connecting to agent...')
        rospy.wait_for_service('/agent/allow_finish')
        self.finishBlockchainization = rospy.ServiceProxy('/agent/allow_finish', EmptySrv)
        rospy.loginfo('Connecting to observer...')
        rospy.wait_for_service('/observer/reset/')
        self.resetObserver = rospy.ServiceProxy('/observer/reset', EmptySrv)
        rospy.loginfo('Simulation API started.')

    def start(self): # generate data for simulation
        rospy.loginfo('Starting simulation...')
        for _ in range(self.measurements_per_sensor):
            for measure in self.sensorsProxies:
                measure(EmptyRequest())
                rospy.sleep(0.1)
        rospy.loginfo('Measurements complete, %d per sensor.', self.measurements_per_sensor)

    def reset(self):
        rospy.loginfo('Reseting simulation...')
        self.resetObserver()

    def finish_liability(self):
        rospy.loginfo('Allowing to finish liability...')
        self.finishBlockchainization()
        rospy.loginfo('Blockchainization finish allowed.')


rospy.init_node('server_node')
simulation_api = SimulationAPI()

app = Flask(__name__)
CORS(app)
api = Api(app)
parser = reqparse.RequestParser()

rospy.loginfo('Waiting for observer services...')
rospy.wait_for_service('/observer/observing_sensors')
get_devices_list = rospy.ServiceProxy('/observer/observing_sensors', GetSensorsList)
rospy.wait_for_service('/observer/sensor_data')
get_sensor_data = rospy.ServiceProxy('/observer/sensor_data', GetSensorData)
#rospy.wait_for_service('/observer/edit_data')
#get_sensor_data = rospy.ServiceProxy('/observer/edit_data', SetSensorData)


operation_data_fields = rospy.get_param('~operation_data_fields').split(',')

for arg in operation_data_fields:
    parser.add_argument(arg)

class StartSimulation(Resource):
    def get(self):
        simulation_api.start()
        return 

class ResetSimulation(Resource):
    def get(self):
        simulation_api.reset()
        return

class FinishLiability(Resource):
    def get(self):
        simulation_api.finish_liability()
        return

class DevicesList(Resource):
    def get(self):
        devices = get_devices_list(GetSensorsListRequest()).sensor_names
        return Response(json.dumps(devices),  mimetype='application/json')

class DataProvider(Resource):
    def get(self, sensor_name):
        data = get_sensor_data().data # array contains Operation type data from each journal
        response = dict()
        for idx, operation in enumerate(data):
            body = dict()
            for field in operation_data_fields:
                body[field] = getattr(operation.data, field)
                if field == 'type':
                    body[field] = eval(getattr(operation.data, field)).decode() # decode to utf-8
            response[str(idx)] = body
        return jsonify(response)

class DataEditor(Resource):
    def post(self):
        args = parser.parse_args()
        operation = Operation()
        new_data = OperationData()
        operation.data = new_data
        for field in operation_data_fields:
            setattr(new_data, field, args[field])
        return


rospy.loginfo('Starting UI server...')

# Simulation purpose
api.add_resource(StartSimulation, '/start')
api.add_resource(ResetSimulation, '/reset')
api.add_resource(FinishLiability, '/finish_liability')

# Application purpose
api.add_resource(DevicesList, '/load/devices')
api.add_resource(DataProvider, '/load/data', '/load/data/<string:sensor_name>')
api.add_resource(DataEditor,  '/add')


server_address = urlparse(rospy.get_param('~server_address')).netloc.split(':')
rospy.loginfo('Serving on %s...', str(server_address))
app.run(host=server_address[0],
        port=int(server_address[1]),
        ssl_context=(rospy.get_param('~certfile'), rospy.get_param('~keyfile')))
rospy.spin()
