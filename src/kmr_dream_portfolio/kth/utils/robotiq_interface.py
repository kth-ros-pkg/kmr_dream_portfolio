import rospy
from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput
from sensor_msgs.msg import JointState


class RobotiqHandInterface(object):

    MAX_ROTATIONAL_SPEED = 1.1458
    MIN_SCISSOR_VALUE = -0.192
    MAX_SCISSOR_VALUE = 0.1784
    MIN_FINGER_JOINT_VALUE = 0.0495
    MAX_FINGER_JOINT_VALUE = 1.2218
    SCISSOR_JOINT_NAME = 'scissor_joint'
    FINGER_1_JOINT_NAME = 'finger_1_joint_1'
    FINGER_2_JOINT_NAME = 'finger_2_joint_1'
    FINGER_3_JOINT_NAME = 'finger_middle_joint_1'

    def __init__(self, command_topic, joint_state_topic):
        """
        Creates a new RobotiqHandInterface. An instance of this class provides
        a convenient interface to control a Robotiq S-model hand.
        :param command_topic: ROS topic name on which the Robotiq hand receives commands
        :param joint_state_topic: ROS topic name on which the Robotiq hand publishes its joint state
        """
        self._command_publisher = rospy.Publisher(command_topic, SModelRobotOutput, queue_size=1)
        self._subscriber = rospy.Subscriber(joint_state_topic, JointState, self._report_robotiq_state)
        self._last_msg = None
        self.activate()

    def _report_robotiq_state(self, msg):
        self._last_msg = msg

    def activate(self):
        msg = SModelRobotOutput()
        msg.rACT = 1
        self._command_publisher.publish(msg)

    def send_move_to(self, scissor_pos, finger_1_pos, finger_2_pos, finger_3_pos,
                     scissor_vel, finger_1_vel, finger_2_vel, finger_3_vel,
                     scissor_force, finger_1_force, finger_2_force, finger_3_force):
        """
        Sends the specified commands to the Robotiq hand.
        :*_pos: Position of the respective joint (in Radians)
        :*_vel: Velocity of the respective joint (in Radians / second)
        :*_vel: Force of the respective joint (percentage in [0, 1] of the maximal force)
        """
        output_msg = SModelRobotOutput()
        output_msg.rACT = 1
        output_msg.rGTO = 1
        output_msg.rICF = 1
        output_msg.rICS = 1

        def to_integer_range_value(val, lower_limit, upper_limit):
            return min(max(0, int((val - lower_limit) / (upper_limit - lower_limit) * 255)), 255)

        output_msg.rPRA = to_integer_range_value(finger_1_pos, self.MIN_FINGER_JOINT_VALUE,
                                                 self.MAX_FINGER_JOINT_VALUE)
        output_msg.rPRB = to_integer_range_value(finger_2_pos, self.MIN_FINGER_JOINT_VALUE,
                                                 self.MAX_FINGER_JOINT_VALUE)
        output_msg.rPRC = to_integer_range_value(finger_3_pos, self.MIN_FINGER_JOINT_VALUE,
                                                 self.MAX_FINGER_JOINT_VALUE)
        output_msg.rPRS = to_integer_range_value(scissor_pos, self.MIN_SCISSOR_VALUE,
                                                 self.MAX_SCISSOR_VALUE)
        output_msg.rSPA = to_integer_range_value(finger_1_vel, 0.0,
                                                 self.MAX_ROTATIONAL_SPEED)
        output_msg.rSPB = to_integer_range_value(finger_2_vel, 0.0,
                                                 self.MAX_ROTATIONAL_SPEED)
        output_msg.rSPC = to_integer_range_value(finger_3_vel, 0.0,
                                                 self.MAX_ROTATIONAL_SPEED)
        output_msg.rSPS = to_integer_range_value(scissor_vel, 0.0,
                                                 self.MAX_ROTATIONAL_SPEED)
        output_msg.rFRA = to_integer_range_value(finger_1_force, 0.0, 1.0)
        output_msg.rFRB = to_integer_range_value(finger_2_force, 0.0, 1.0)
        output_msg.rFRC = to_integer_range_value(finger_3_force, 0.0, 1.0)
        output_msg.rFRS = to_integer_range_value(scissor_force, 0.0, 1.0)
        self._command_publisher.publish(output_msg)

    def get_current_hand_state(self):
        """
        Returns the current state (joint positions) of the hand.
        :return: None, if state is unknown, else a dictionary with elements (name, value)
        """
        if self._last_msg is None:
            return None
        indices = [self._last_msg.name.index(self.SCISSOR_JOINT_NAME),
                   self._last_msg.name.index(self.FINGER_1_JOINT_NAME),
                   self._last_msg.name.index(self.FINGER_2_JOINT_NAME),
                   self._last_msg.name.index(self.FINGER_3_JOINT_NAME)]
        names = [self._last_msg.name[i] for i in indices]
        values = [self._last_msg.position[i] for i in indices]
        return dict(zip(names, values))

    def set_goal_configuration(self, **kwargs):
        delta_t = kwargs['delta_t']
        current_configuration = self.get_current_hand_state()
        if 'reduced_dofs' in kwargs:
            reduced_dofs = kwargs['reduced_dofs']
        else:
            reduced_dofs = False
        if not reduced_dofs:
            finger_1_key = self.FINGER_1_JOINT_NAME
            finger_2_key = self.FINGER_2_JOINT_NAME
            finger_3_key = self.FINGER_3_JOINT_NAME
        else:
            finger_1_key = self.FINGER_2_JOINT_NAME
            finger_2_key = self.FINGER_2_JOINT_NAME
            finger_3_key = self.FINGER_2_JOINT_NAME
        if delta_t <= 0.0 or current_configuration is None:
            self.send_move_to(scissor_pos=kwargs[self.SCISSOR_JOINT_NAME], finger_1_pos=kwargs[finger_1_key],
                              finger_2_pos=kwargs[finger_2_key], finger_3_pos=kwargs[finger_3_key],
                              scissor_vel=self.MAX_ROTATIONAL_SPEED, finger_1_vel=self.MAX_ROTATIONAL_SPEED,
                              finger_2_vel=self.MAX_ROTATIONAL_SPEED, finger_3_vel=self.MAX_ROTATIONAL_SPEED,
                              scissor_force=0.5, finger_1_force=0.5, finger_2_force=0.5, finger_3_force=0.5)
        else:
            # we need to compute speed for each finger
            # In case we have reduced DOFs (i.e. 2 instead 4, we need to choose the correct target positions)
            target_positions = {self.SCISSOR_JOINT_NAME: kwargs[self.SCISSOR_JOINT_NAME],
                                self.FINGER_1_JOINT_NAME: kwargs[finger_1_key],
                                self.FINGER_2_JOINT_NAME: kwargs[finger_2_key],
                                self.FINGER_3_JOINT_NAME: kwargs[finger_3_key]}
            velocities = {}
            for (joint_name, value) in current_configuration.iteritems():
                position_error = target_positions[joint_name] - value
                velocities[joint_name] = min(self.MAX_ROTATIONAL_SPEED, abs(position_error / delta_t))
            self.send_move_to(scissor_pos=target_positions[self.SCISSOR_JOINT_NAME], finger_1_pos=target_positions[finger_1_key],
                              finger_2_pos=target_positions[finger_2_key], finger_3_pos=target_positions[finger_3_key],
                              scissor_vel=velocities[self.SCISSOR_JOINT_NAME], finger_1_vel=velocities[self.FINGER_1_JOINT_NAME],
                              finger_2_vel=velocities[self.FINGER_2_JOINT_NAME], finger_3_vel=velocities[self.FINGER_3_JOINT_NAME],
                              finger_1_force=0.5, finger_2_force=0.5, finger_3_force=0.5, scissor_force=0.5)

