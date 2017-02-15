#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger as TriggerService
from std_srvs.srv import TriggerResponse, EmptyResponse
from std_srvs.srv import Empty as EmptyService


class KMRSimulator(object):
    def init(self, msg):
        rospy.loginfo('KMRSimulator: Init called')
        return TriggerResponse(success=True, message='Did not do anything')

    def reset(self, msg):
        rospy.loginfo('KMRSimulator: Reset called')
        return EmptyResponse()

    def clean_up(self, msg):
        rospy.loginfo('KMRSimulator: clean_up called')
        return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('KMRSimulatorInterface')
    kmr_simulator = KMRSimulator()
    init_service = rospy.Service('/robot_simulator/init_robot_simulator', TriggerService,
                                 kmr_simulator.init)
    reset_service = rospy.Service('/robot_simulator/reset_robot_simulator', EmptyService,
                                  kmr_simulator.reset)
    cleanup_service = rospy.Service('/robot_simulator/cleanup_robot_simulator', EmptyService,
                                    kmr_simulator.clean_up)
    rospy.loginfo('Running kmr simulator wrapper')
    rospy.spin()