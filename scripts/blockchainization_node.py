#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Alisher A. Khassanov, alisher@aira.life

from sys import version_info
if version_info[0] < 3:
    raise Exception('python3 required')
from iiot_data_secure.srv import GetJournals
from robonomics_lighthouse.msg import Ask, Bid
from ethereum_common.msg import Address, UInt256
from ethereum_common.srv import Approve, ApproveRequest, Accounts, AccountsRequest
from std_srvs.srv import Empty as EmptySrv, EmptyResponse
from std_msgs.msg import Duration, String
try:
    import rospy
except ImportError as e:
    link = 'https://github.com/OTL/cozmo_driver#super-hack-to-run-rospy-from-python3'
    raise(ImportError('Check ' + link)) from e
from web3 import Web3, HTTPProvider


class BlockchainizationAgent:
    busy = None
    def __init__(self):
        rospy.init_node('blockchainization_agent')

        self.model = rospy.get_param('~model')
        self.token = rospy.get_param('~token')
        self.bid_lifetime = rospy.get_param('~bid_lifetime')
        self._web3 = Web3(HTTPProvider(rospy.get_param('~web3_http_provider')))

        rospy.loginfo('Connecting to ERC20 node...')
        rospy.wait_for_service('accounts')
        self.accounts = rospy.ServiceProxy('accounts', Accounts)(AccountsRequest())
        rospy.loginfo(str(self.accounts)) # agent's ethereum addresses

        if rospy.get_param('~approve', 'no') == 'yes':
            self.liability_factory = rospy.get_param('~liability_factory')
            rospy.loginfo('Making approvement to liabilities factory %s.', self.liability_factory)
            rospy.wait_for_service('approve')
            msg = ApproveRequest(spender=Address(address=self.liability_factory),
                                 value=UInt256(uint256=rospy.get_param('~approve_value', "10000")))
            tx = rospy.ServiceProxy('approve', Approve)(msg)
            rospy.loginfo('Approved on tx: %s.', tx)
        else:
            rospy.loginfo('Launching without approve on liabilities factory.')
        
        self._signing_bid = rospy.Publisher('liability/infochan/signing/bid', Bid, queue_size=128)
        
        def on_incoming_ask(incoming_ask):
            rospy.loginfo('Incoming ask: ' + str(incoming_ask))
            if incoming_ask.model == self.model and incoming_ask.token == self.token:
                rospy.loginfo('For my model and token.')
                self.make_bid(incoming_ask)
            else:
                rospy.loginfo('Not fits, skip.')
        rospy.Subscriber('liability/infochan/incoming/ask', Ask, on_incoming_ask)

        rospy.Subscriber('/agent/objective/duration', Duration, self.start_process)

        rospy.loginfo('Connecting to observer node...')
        rospy.wait_for_service('/observer/journals')
        self.get_journals = rospy.ServiceProxy('/observer/journals', GetJournals)

        self.result_topics = dict()
        self.result_topics['journals'] = rospy.Publisher('~result/journals', String, queue_size=1000)
        self.result_topics['sha256'] = rospy.Publisher('~result/sha256', String, queue_size=1000)
        
        rospy.loginfo('Connecting to liability node...')
        self.finish_allowed = False
        rospy.Service('~allow_finish', EmptySrv, self.allow_finish)
        rospy.wait_for_service('liability/finish')
        self._finish_liability = rospy.ServiceProxy('liability/finish', EmptySrv)

        rospy.loginfo('Node ' + rospy.get_name() + ' started.')

    def make_bid(self, incoming_ask):
        rospy.loginfo('Making bid...')

        bid = Bid()
        bid.model = self.model
        bid.objective = incoming_ask.objective
        bid.token = self.token
        bid.cost = incoming_ask.cost
        bid.lighthouseFee = 0
        bid.deadline = self._web3.eth.getBlock('latest').number + self.bid_lifetime

        self._signing_bid.publish(bid)
        rospy.loginfo(bid)

    def start_process(self, duration):
        self.busy = True
        rospy.loginfo('Starting process...' )
        while not self.finish_allowed:
            rospy.sleep(1)
        journals = list(self.get_journals().journals) # name:ipfs_sha256
        for journal in journals:
            name, sha256 = journal.split(':')
            self.result_topics['journals'].publish(String(data=name))
            self.result_topics['sha256'].publish(String(data=sha256))
        self.finish()
        rospy.loginfo('Process complete.')

    def allow_finish(self, request):
        if self.busy:
            self.finish_allowed = True
        return EmptyResponse()

    def finish(self):
        self._finish_liability()
        self.finish_allowed = self.busy = False

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    BlockchainizationAgent().spin()
