#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Alisher A. Khassanov, alisher@aira.life

from sys import version_info
if version_info[0] < 3:
    raise Exception('python3 required')
try:
    import rosbag
    import rospy
except ImportError as e:
    link = 'https://github.com/OTL/cozmo_driver#super-hack-to-run-rospy-from-python3'
    raise(ImportError('How to run ROS on python3: ' + link)) from e
from std_msgs.msg import Duration
import ipfsapi
from flask import Flask, request
from flask_restful import Resource, Api
from flask.json import jsonify
from flask_cors import CORS
from urllib.parse import urlparse
import tempfile
import shelve
import os


rospy.init_node('objective_composer')

ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

app = Flask(__name__)
CORS(app)
api = Api(app)

per_dir = rospy.get_param('~per_dir', os.path.dirname(os.path.realpath(__file__)))
with shelve.open(per_dir + '/storage.shelve') as db:
    if not 'composed' in db:
        db['composed'] = 0
    if not 'name' in db:
        db['name'] = 'iiot_data_secure'

tmp_dir = rospy.get_param('~tmp_dir', '/tmp/agent')
if not os.path.exists(tmp_dir):
    os.makedirs(tmp_dir)


class Stats(Resource): # composer usage statistics
    def get(self):
        with shelve.open(per_dir + '/storage.shelve') as db:
            rospy.loginfo('Stats request. Composed: %d' % db['composed'])
            return jsonify({'serviceName': db['name'], 'objectivesComposed': db['composed']})

class ComposeObjective(Resource): # ui input to agent liability objective
    def post(self):
        """request example:
           $ curl -H "Content-type: application/json" \
                  -X POST https://0.0.0.0:8888/ \
                  -d '{"duration": 86400}'
        """
        rospy.loginfo('Composer request for: ' + str(request.get_json()))
        bag_name = tmp_dir + '/objective-' + next(tempfile._get_candidate_names()) + '.bag'
        with rosbag.Bag(bag_name, 'w') as bag:
            duration = Duration()
            duration.data.secs = int(request.get_json()['duration'])
            bag.write('/agent/objective/duration', duration)
        with shelve.open(per_dir + '/storage.shelve') as db:
            db['composed'] += 1
        rospy.loginfo('Bag: ' + bag_name)
        objective_hash = ipfs.add(bag_name)[0]['Hash']
        rospy.loginfo('Objective: ' + objective_hash)
        return jsonify({'objective': objective_hash})

api.add_resource(Stats, '/stats')
api.add_resource(ComposeObjective, '/get_objective')


server_address = urlparse(rospy.get_param('~server_address')).netloc.split(':')
app.run(host=server_address[0],
        port=int(server_address[1]),
        ssl_context=(rospy.get_param('~certfile'), rospy.get_param('~keyfile')))
rospy.spin()
