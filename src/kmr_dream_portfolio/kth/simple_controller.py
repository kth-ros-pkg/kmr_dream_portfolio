from utils.robotiq_interface import RobotiqHandInterface
from manipulation_dreambed.MethodTypes import ArmController, GraspController
import manipulation_dreambed.ROSUtils
import rospy
from trajectory_msgs.msg import JointTrajectory


class SimpleArmController(ArmController):
    #TODO
    def __init__(self, command_topic):
        self._command_topic = command_topic
        self._traj_pubj = None

    def getName(self):
        return "SimpleArmController"

    def getParameters(self, role, paramPrefix):
        if role != 'ArmController':
            raise ValueError('[SimpleArmController::getParameters] Requested parameters for non implemented role: ' + str(role))
        return {}

    def getConditionals(self, role, paramPrefix):
        if role != 'ArmController':
            raise ValueError('[SimpleArmController::getConditionals] Requested parameters for non implemented role: ' + str(role))
        return {}

    def getForbiddenConfigurations(self, role, paramPrefix):
        return []

    def initialize(self):
        self._traj_pub = rospy.Publisher(self._command_topic, JointTrajectory, queue_size=1)

    def allocateResources(self, roles=None):
        pass

    def releaseResources(self, roles=None):
        pass

    def destroy(self):
        pass

    def hasResourceConflict(self, activeRoles):
        return False

    def supportsBatchProcessing(self, roleSequence):
        return False

    def executeBatch(self, startContext, batchInput, parameters, getWorldStateFn):
        raise NotImplementedError('[SimpleArmController::executeBatch] Not supported.')

    def executeArmTrajectory(self, trajectory, context, paramPrefix, parameters):
        ros_traj = manipulation_dreambed.ROSUtils.toROSTrajectory(trajectory)
        ros_traj.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.01)
        self._traj_pub.publish(ros_traj)
        rospy.sleep(ros_traj.points[-1].time_from_start)
        return True


class SimpleGraspController(GraspController):
    def __init__(self, robotiq_hand_command_topic, robotiq_hand_joints_topic,
                 robotiq_hand_state_topic):
        self._robotiq_hand_command_topic = robotiq_hand_command_topic
        self._robotiq_hand_joints_topic = robotiq_hand_joints_topic
        self._robotiq_hand_state_topic = robotiq_hand_state_topic
        self._robotiq_hand_interface = None

    def getName(self):
        return "SimpleGraspController"

    def getParameters(self, role, paramPrefix):
        if role != 'GraspController':
            raise ValueError('[SimpleGraspController::getParameters] Requested parameters for not implemented role: ' + str(role))
        return {paramPrefix + '_closing_offset_scissor_joint': ('real', [0.0, 0.05], 0.0),
                paramPrefix + '_closing_offset_finger_2_joint_1': ('real', [0.0, 0.1], 0.0)}

    def getConditionals(self, role, paramPrefix):
        if role != 'GraspController':
            raise ValueError('[SimpleGraspController::getConditionals] Requested parameters for not implemented role: ' + str(role))
        return {}

    def getForbiddenConfigurations(self, role, paramPrefix):
        return []

    def initialize(self):
        self._robotiq_hand_interface = RobotiqHandInterface(self._robotiq_hand_command_topic,
                                                            self._robotiq_hand_state_topic,
                                                            self._robotiq_hand_joints_topic)

    def allocateResources(self, roles=None):
        self._robotiq_hand_interface.activate()

    def releaseResources(self, roles=None):
        pass

    def destroy(self):
        pass

    def hasResourceConflict(self, activeRoles):
        # return GraspController in activeRoles
        return False

    def supportsBatchProcessing(self, roleSequence):
        return False

    def executeBatch(self, startContext, batchInput, parameters, getWorldStateFn):
        raise NotImplementedError('[SimpleGraspController::executeBatch] Not supported.')

    def startGraspExecution(self, grasp, context, paramPrefix, parameters):
        input_to_hand = {}
        for (key, value) in grasp.hand_configuration.iteritems():
            closing_offset = parameters[paramPrefix + '_closing_offset_' + key]
            input_to_hand[key] = value + closing_offset

        input_to_hand['delta_t'] = 0.0
        input_to_hand['reduced_dofs'] = True
        input_to_hand['block'] = True
        self._robotiq_hand_interface.set_goal_configuration(input_to_hand)
        # TODO return value based on what simulator claims?
        return True

    def stopGraspExecution(self):
        return True

