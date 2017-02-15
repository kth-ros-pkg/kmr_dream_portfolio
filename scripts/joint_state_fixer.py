#! /usr/bin/python
import rospy
from sensor_msgs.msg import JointState
from threading import RLock


class Converter(object):
    def __init__(self, input_kmr_topic, input_robotiq_topic, output_topic):
        rospy.Subscriber(input_kmr_topic, JointState, self.receive_kmr_state)
        rospy.Subscriber(input_robotiq_topic, JointState, self.receive_robotiq_state)
        self._pub = rospy.Publisher(output_topic, JointState, queue_size=1)
        self._last_kmr_state = None
        self._last_robotiq_state = None
        self._lock = RLock()

    def receive_kmr_state(self, msg):
        with self._lock:
            self._last_kmr_state = msg

    def receive_robotiq_state(self, msg):
        with self._lock:
            self._last_robotiq_state = msg

    def publish_state(self, arg):
        if self._last_robotiq_state is None:
            rospy.logwarn('Missing Robotiq state.')
            return
        if self._last_kmr_state is None:
            rospy.logwarn('Missing KMR state')
            return
        with self._lock:
            output_state = JointState()
            for name in self._last_kmr_state.name:
                output_state.name.append(name)
            for value in self._last_kmr_state.position:
                output_state.position.append(value)
            index = self._last_robotiq_state.name.index("scissor_joint")
            output_state.name.append('scissor_joint')
            output_state.position.append(self._last_robotiq_state.position[index])
            index = self._last_robotiq_state.name.index("finger_2_joint_1")
            output_state.name.append('finger_2_joint_1')
            output_state.position.append(self._last_robotiq_state.position[index])
            self._pub.publish(output_state)


if __name__ == "__main__":
    rospy.init_node('joint_state_fixer')
    converter = Converter('/kmr/joint_states', '/robotiq_hand/joint_states', '/kmr/hand_arm_joint_states')
    rospy.Timer(rospy.Duration.from_sec(0.05), converter.publish_state, oneshot=False)
    rospy.spin()
