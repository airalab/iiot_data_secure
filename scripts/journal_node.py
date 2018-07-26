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
from iiot_data_secure.msg import Operation
from iiot_data_secure.srv import GetSensorsList, GetSensorsListResponse
from iiot_data_secure.srv import GetSensorData, GetSensorDataResponse
from iiot_data_secure.srv import GetJournals, GetJournalsResponse
from std_msgs.msg      import String
from std_srvs.srv      import Empty as EmptySrv, EmptyResponse
from tempfile          import _get_candidate_names as get_unique_name
from urllib.parse      import urlparse
import os.path
import ipfsapi


class JournalState: # journal states
    OPEN   = 0
    CLOSE  = 1
    SHARED = 2


class Journal:
    state = None
    def __init__(self, device_name: str, path: str, ipfs,
                 parent = {'name': None, 'sha256': None}):
        self.path = path
        self.name = 'journal_' + device_name + '_' + next(get_unique_name()) + '.bag'
        self.parent = parent
        self.ipfs = ipfs
        self.state = JournalState.OPEN
        self._bag = rosbag.Bag(path + '/' +  self.name, 'w')
        rospy.logdebug('New journal started: %s', self.name)
    
    def write(self, topic_name, msg):
        rospy.logdebug('Incoming: %s, %s', topic_name, str(msg))
        if self.state == JournalState.OPEN:
            self._bag.write(topic_name, msg)
        else:
            rospy.logwarn('Writing attemt to journal with state: %d.', self.state)

    def read(self, topics=None):
        if self.state == JournalState.SHARED:
            bag = rosbag.Bag(self.path + '/' +  self.name)
            content = list()
            for topic, msg, time in bag.read_messages(topics=topics):
                content.append([topic, msg, time])
            bag.close()
            return content

    def close(self):
        self.write('/parent/name', String(data=self.parent['name']))
        self.write('/parent/sha256', String(data=self.parent['sha256']))
        self._bag.close()
        self.state = JournalState.CLOSE

    def share(self): # ipfs
        if self.state == JournalState.OPEN:
            self.close()
        self.ipfs_hash = self.ipfs.add(self.path + '/' + self.name)[0]['Hash']
        rospy.logdebug('Journal %s shared.', self.ipfs_hash)
        self.state = JournalState.SHARED
        return self.ipfs_hash


def name_from_topic(topic, keyword, namespace): # keyword is a node name head
    ns_end = topic.find(keyword)
    tail = topic[ns_end:]
    name_end = tail.find('/')
    name = tail[:name_end]
    name_with_ns = topic[:topic.find(name) + len(name)]
    return name, name_with_ns


class ObserverNode:
    def __init__(self):
        rospy.init_node('observer')

        self.journals_dir = rospy.get_param('~journals_dir')
        rospy.loginfo('Journals path: %s', self.journals_dir)
        self.sensors_ns = rospy.get_param('~sensors_namespace') # /your/namespace/to/sensor_x
        rospy.loginfo('Observing sensors at namespace: %s', self.sensors_ns)
        self.observing_topic = rospy.get_param('~observing_topic') # ns/sensor_x/<obs_topic>
        rospy.loginfo('Observing topic: %s', self.observing_topic)

        rospy.wait_for_service('/input/sensor_1/measure') # for simulation purpose
        rospy.wait_for_service('/input/sensor_2/measure')
        rospy.wait_for_service('/input/sensor_3/measure')

        self.sensors_topics = [self.sensors_ns + name + '/' + self.observing_topic
                                                    for name in self._sensors_names]
        rospy.Service('~observing_sensors', GetSensorsList, self.get_sensors_names)
        rospy.loginfo('Observing sensor topics: %s.', str(self.sensors_topics))

        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        self.journals = list()
        for topic in self.sensors_topics:
            rospy.Subscriber(topic, Operation, callback=self.on_msg, callback_args=topic)
        rospy.Service('~journals', GetJournals, self.get_journals)
        rospy.Service('~sensor_data', GetSensorData, self.get_sensor_data)
        rospy.Service('~reset', EmptySrv, self.reset)

        rospy.loginfo('Node ' + rospy.get_name() + ' started.')
    
    def on_msg(self, msg: Operation, topic_name: str):
        name, _ = name_from_topic(topic_name, 'sensor', self.sensors_ns)
        journal = Journal(name, self.journals_dir, self.ipfs)
        journal.write(topic_name, msg)
        journal.share()
        self.journals.append(journal)

    def get_journals(self, request):
        return GetJournalsResponse([j.name + ':' + j.ipfs_hash for j in self.journals])

    @property
    def _sensors_names(self):
        names = set()
        topics = rospy.get_published_topics(namespace=self.sensors_ns)
        for topic in topics:
            tail = topic[0][topic[0].find('sensor'):]
            name = tail[:tail.find('/')]
            names.add(name)
        return list(names)

    def get_sensors_names(self, request):
        return GetSensorsListResponse(self._sensors_names)

    def get_sensor_data(self, request):
        sensor_name = request.sensor_name
        operations = list()
        for journal in self.journals:
            if sensor_name in journal.name:
                topic, msg, time = journal.read()[0]
                operations.append(msg)
        return GetSensorDataResponse(operations)

    def reset(self, request):
        rospy.loginfo('Reset...')
        self.journals = []
        return EmptyResponse()

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    ObserverNode().spin()
