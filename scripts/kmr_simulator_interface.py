#!/usr/bin/env python
import rospy
import numpy
from manipulation_dreambed import ROSUtils
from std_srvs.srv import Trigger as TriggerService
from std_srvs.srv import TriggerResponse, EmptyResponse
from std_srvs.srv import Empty as EmptyService
from manipulation_dreambed.srv import GetRobotConfiguration, GetRobotConfigurationResponse
from manipulation_dreambed.srv import SetRobotConfiguration, SetRobotConfigurationResponse
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kmr_dream_portfolio.kth.utils.robotiq_interface import RobotiqHandInterface
from gazebo_msgs.srv import SetModelConfiguration
from std_msgs.msg import Empty
from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, ListControllers, ListControllersRequest
from controller_manager_msgs.srv import UnloadController, UnloadControllerRequest, LoadController, LoadControllerRequest


LBR_JOINT_TRAJECTORY_CONTROLLER = 'lbr_joint_trajectory_controller'


# class KMRSimulator(object):
#     def __init__(self, kmr_controller_service, kmr_list_controller_service,
#                  kmr_unload_service, kmr_load_service, robotiq_topic,
#                  reset_hardware_topic):
#         rospy.wait_for_service(kmr_controller_service)
#         rospy.wait_for_service(kmr_list_controller_service)
#         self._switch_controller_service = rospy.ServiceProxy(kmr_controller_service, SwitchController)
#         self._list_controller_service = rospy.ServiceProxy(kmr_list_controller_service, ListControllers)
#         self._unload_service = rospy.ServiceProxy(kmr_unload_service, UnloadController)
#         self._load_service = rospy.ServiceProxy(kmr_load_service, LoadController)
#         self._robotiq_publisher = rospy.Publisher(robotiq_topic, SModelRobotOutput, queue_size=1)
#         self._reset_hardware_pub = rospy.Publisher(reset_hardware_topic, Empty)
#
#     def _is_controller_loaded(self, controller_name):
#         response = self._list_controller_service(ListControllersRequest())
#         for controller in response.controller:
#             if controller.name == controller_name:
#                 return True
#         return False
#
#     def _is_controller_running(self, controller_name):
#         response = self._list_controller_service(ListControllersRequest())
#         for controller in response.controller:
#             if controller.name == controller_name:
#                 return controller.state == 'running'
#         return False
#
#     def _activate_controller(self, controller_name):
#         if not self._is_controller_running(controller_name):
#             request = SwitchControllerRequest()
#             request.start_controllers = [controller_name]
#             request.strictness = request.STRICT
#             response = self._switch_controller_service(request)
#             if response.ok:
#                 rospy.loginfo('Activated controller %s' % controller_name)
#             else:
#                 rospy.loginfo('Activating controller %s failed' % controller_name)
#             return response.ok
#         return True
#
#     def _load_controller(self, controller_name):
#         if not self._is_controller_loaded(LBR_JOINT_TRAJECTORY_CONTROLLER):
#             request = LoadControllerRequest()
#             request.name = controller_name
#             response = self._load_service(request)
#             if response.ok:
#                 rospy.loginfo('Loaded controller %s' % controller_name)
#             else:
#                 rospy.loginfo('Loading controller %s failed' % controller_name)
#             return response.ok
#         return True
#
#     def _deactivate_controller(self, controller_name):
#         if self._is_controller_running(controller_name):
#             request = SwitchControllerRequest()
#             request.stop_controllers = [controller_name]
#             request.strictness = request.STRICT
#             response = self._switch_controller_service(request)
#             if response.ok:
#                 rospy.loginfo('Deactivated controller %s' % controller_name)
#             else:
#                 rospy.loginfo('Deactivating controller %s failed' % controller_name)
#             return response.ok
#         return True
#
#     def _unload_controller(self, controller_name):
#         if self._is_controller_loaded(controller_name):
#             b_deactivated = self._deactivate_controller(controller_name)
#             if not b_deactivated:
#                 return False
#             request = UnloadControllerRequest()
#             request.name = controller_name
#             response = self._unload_service(request)
#             if response.ok:
#                 rospy.loginfo('Unloaded controller %s' % controller_name)
#             else:
#                 rospy.loginfo('Unloading controller %s failed' % controller_name)
#             return response.ok
#         return True
#
#     def _turn_on_robotiq(self):
#         msg = SModelRobotOutput()
#         msg.rACT = 1
#         self._robotiq_publisher.publish(msg)
#
#     def _turn_off_robotiq(self):
#         msg = SModelRobotOutput()
#         msg.rACT = 0
#         self._robotiq_publisher.publish(msg)
#
#     def init(self, msg):
#         rospy.loginfo('KMRSimulator: Init called')
#         b_kmr_ok = self._load_controller(LBR_JOINT_TRAJECTORY_CONTROLLER)
#         if b_kmr_ok:
#             b_kmr_ok = self._activate_controller(LBR_JOINT_TRAJECTORY_CONTROLLER)
#         self._turn_on_robotiq()
#         return TriggerResponse(success=b_kmr_ok, message='Did not do anything')
#
#     def reset(self, msg):
#         rospy.loginfo('KMRSimulator: Reset called')
#         self._unload_controller(LBR_JOINT_TRAJECTORY_CONTROLLER)
#         self._turn_off_robotiq()
#         self._reset_hardware_pub.publish(Empty())
#         b_kmr_ok = self._load_controller(LBR_JOINT_TRAJECTORY_CONTROLLER)
#         if b_kmr_ok:
#             b_kmr_ok = self._activate_controller(LBR_JOINT_TRAJECTORY_CONTROLLER)
#         self._turn_on_robotiq()
#         if not b_kmr_ok:
#             rospy.logerr('Failed to reset robot')
#         return EmptyResponse()
#
#     def clean_up(self, msg):
#         rospy.loginfo('KMRSimulator: clean_up called')
#         self.reset(msg)
#         return EmptyResponse()

class KMRSimulator(object):

    def __init__(self, joint_states_topic, traj_command_topic,
                 robotiq_command, robotiq_state, robotiq_joints):
        self._subscriber = rospy.Subscriber(joint_states_topic, JointState,
                                            callback=self._receive_joint_state)
        self._traj_pub = rospy.Publisher(traj_command_topic, JointTrajectory, queue_size=1)
        self._last_joint_state = None
        self._traj_time = 1.0
        self._convergence_threshold = 0.001
        self._timeout = 5.0
        self._arm_names = ['lbrAxis1', 'lbrAxis2', 'lbrAxis3',
                           'lbrAxis4', 'lbrAxis5', 'lbrAxis6', 'lbrAxis7']
        self._hand_names = ['scissor_joint', 'finger_2_joint_1']
        self._pauseService = rospy.ServiceProxy('/gazebo/pause_physics', EmptyService)
        self._unpauseService = rospy.ServiceProxy('/gazebo/unpause_physics', EmptyService)
        self._hand_interface = RobotiqHandInterface(robotiq_command, robotiq_state, robotiq_joints)
        self._gazebo_config_service = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self._switch_controller_service = rospy.ServiceProxy('/kmr/controller_manager/switch_controller',
                                                             SwitchController)
        self._list_controller_service = rospy.ServiceProxy('/kmr/controller_manager/list_controllers',
                                                           ListControllers)
        self._unload_service = rospy.ServiceProxy('/kmr/controller_manager/unload_controller', UnloadController)
        self._load_service = rospy.ServiceProxy('/kmr/controller_manager/load_controller',
                                                LoadController)
        self._reset_hardware_pub = rospy.Publisher('/kmr/reset_hardware', Empty)
        self._robotiq_publisher = rospy.Publisher('/robotiq_hand/command', SModelRobotOutput, queue_size=1)

    def _receive_joint_state(self, msg):
        self._last_joint_state = msg

    def _turn_off_robotiq(self):
        msg = SModelRobotOutput()
        msg.rACT = 0
        self._robotiq_publisher.publish(msg)

    def _turn_on_robotiq(self):
        msg = SModelRobotOutput()
        msg.rACT = 1
        self._robotiq_publisher.publish(msg)

    # def _sleep_until_reached(self):
    #     rospy.loginfo('Sleeping until convergence')
    #     start_time = rospy.Time.now().to_sec()
    #     prev_config = numpy.array(self._last_joint_state.position)
    #     rospy.sleep(rospy.Duration.from_sec(0.1))
    #     current_config = numpy.array(self._last_joint_state.position)
    #     while numpy.linalg.norm(prev_config - current_config, ord=1) > self._convergence_threshold:
    #         rospy.loginfo('Dist is: ' + str(numpy.linalg.norm(prev_config - current_config)))
    #         prev_config = current_config
    #         rospy.sleep(rospy.Duration.from_sec(0.1))
    #         current_config = numpy.array(self._last_joint_state.position)
    #         if rospy.Time.now().to_sec() - start_time > self._timeout:
    #             rospy.loginfo('Done, setting failed')
    #             return False
    #     rospy.loginfo('Done, setting worked')
    #     return True

    def init(self, request):
        rospy.loginfo('Init requested')
        self._hand_interface.activate()
        return TriggerResponse(success=True, message='Did not do anything')

    # TRASH????

        # if self._last_joint_state is None:
        #     response.success = False
        #     return response
        # # To Set the configuration we create a dummy trajectory
        # dummy_traj = JointTrajectory()
        # dummy_traj.joint_names = self._arm_names
        # first waypoint is current config
        # wp0 = JointTrajectoryPoint()
        # current_arm_state = ROSUtils.extractPartialJointState(self._last_joint_state,
        # self._arm_names)
        # wp0.positions = current_arm_state.position
        # dummy_traj.points.append(wp0)
        # # second waypoint is goal config
        # wp1 = JointTrajectoryPoint()
        # target_arm_state = ROSUtils.extractPartialJointState(request.configuration,
        #                                                      self._arm_names)
        # wp1.positions = target_arm_state.position
        # wp1.time_from_start = rospy.Duration.from_sec(1.0)
        # dummy_traj.points.append(wp1)
        # dummy_traj.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.01)
        # send the arm there
        # self._traj_pub.publish(dummy_traj)
        # # TODO we also need to move the hand
        # scissor_index = request.configuration.name.index('scissor_joint')
        # finger_2_index = request.configuration.name.index('finger_2_joint_1')
        # hand_input = {'block': False, 'delta_t': 0.0, 'reduced_dofs': True,
        #               'scissor_joint': request.configuration.position[scissor_index],
        #               'finger_2_joint_1': request.configuration.position[finger_2_index]}
        # self._hand_interface.set_goal_configuration(hand_input)
        # response.success = self._sleep_until_reached()
        # self._pauseService()

    def handle_set_configuration(self, request):
        rospy.loginfo('Set configuration request')
        response = SetRobotConfigurationResponse()
        self._unpauseService()
        self._unload_controller(LBR_JOINT_TRAJECTORY_CONTROLLER)
        self._turn_off_robotiq()
        self._reset_hardware_pub.publish(Empty())
        response.success = self._set_gazebo_config(request.configuration)
        b_kmr_ok = self._load_controller(LBR_JOINT_TRAJECTORY_CONTROLLER)
        if b_kmr_ok:
            b_kmr_ok = self._activate_controller(LBR_JOINT_TRAJECTORY_CONTROLLER)
        if not b_kmr_ok:
            rospy.logerr('Failed to reset robot')
        self._turn_on_robotiq()
        self._pauseService()
        return response

    def _set_gazebo_config(self, config):
        names = [name for name in config.name]
        values = [value for value in config.position]
        scissor_value = values[names.index('scissor_joint')]
        finger_value = values[names.index('finger_2_joint_1')]
        names.append('scissor_joint_2')
        values.append(-scissor_value)
        names.append('finger_1_joint_1')
        values.append(finger_value)
        names.append('finger_middle_joint_1')
        values.append(finger_value)
        # TODO add joint values for implicit hand joints
        result = self._gazebo_config_service(model_name='kmr_iiwa_robotiq',
                                             joint_names=names,
                                             joint_positions=values)
        return result.success

    def handle_get_configuration(self, request):
        response = GetRobotConfigurationResponse()
        if self._last_joint_state is not None:
            response.configuration = self._last_joint_state
        else:
            response.configuration = JointState()
        return response

    def clean_up(self, request):
        rospy.loginfo('Cleanup')
        return EmptyResponse()

    def _is_controller_loaded(self, controller_name):
        response = self._list_controller_service(ListControllersRequest())
        for controller in response.controller:
            if controller.name == controller_name:
                return True
        return False

    def _is_controller_running(self, controller_name):
        response = self._list_controller_service(ListControllersRequest())
        for controller in response.controller:
            if controller.name == controller_name:
                return controller.state == 'running'
        return False

    def _activate_controller(self, controller_name):
        if not self._is_controller_running(controller_name):
            request = SwitchControllerRequest()
            request.start_controllers = [controller_name]
            request.strictness = request.STRICT
            response = self._switch_controller_service(request)
            if response.ok:
                rospy.loginfo('Activated controller %s' % controller_name)
            else:
                rospy.loginfo('Activating controller %s failed' % controller_name)
            return response.ok
        return True

    def _load_controller(self, controller_name):
        if not self._is_controller_loaded(LBR_JOINT_TRAJECTORY_CONTROLLER):
            request = LoadControllerRequest()
            request.name = controller_name
            response = self._load_service(request)
            if response.ok:
                rospy.loginfo('Loaded controller %s' % controller_name)
            else:
                rospy.loginfo('Loading controller %s failed' % controller_name)
            return response.ok
        return True

    def _deactivate_controller(self, controller_name):
        if self._is_controller_running(controller_name):
            request = SwitchControllerRequest()
            request.stop_controllers = [controller_name]
            request.strictness = request.STRICT
            response = self._switch_controller_service(request)
            if response.ok:
                rospy.loginfo('Deactivated controller %s' % controller_name)
            else:
                rospy.loginfo('Deactivating controller %s failed' % controller_name)
            return response.ok
        return True

    def _unload_controller(self, controller_name):
        if self._is_controller_loaded(controller_name):
            b_deactivated = self._deactivate_controller(controller_name)
            if not b_deactivated:
                return False
            request = UnloadControllerRequest()
            request.name = controller_name
            response = self._unload_service(request)
            if response.ok:
                rospy.loginfo('Unloaded controller %s' % controller_name)
            else:
                rospy.loginfo('Unloading controller %s failed' % controller_name)
            return response.ok
        return True

if __name__ == '__main__':
    rospy.init_node('KMRSimulatorInterface')
    # kmr_simulator = KMRSimulator('/kmr/controller_manager/switch_controller',
    #                              '/kmr/controller_manager/list_controllers',
    #                              '/kmr/controller_manager/unload_controller',
    #                              '/kmr/controller_manager/load_controller',
    #                              '/robotiq/command',
    #                              '/kmr/reset_hardware')
    kmr_simulator = KMRSimulator('/kmr/hand_arm_joint_states',
                                 '/kmr/lbr_joint_trajectory_controller/command',
                                 '/robotiq_hand/command',
                                 '/robotiq_hand/state',
                                 '/robotiq_hand/joint_states')
    init_service = rospy.Service('/robot_simulator/init_robot_simulator', TriggerService,
                                 kmr_simulator.init)
    get_config_service = rospy.Service('/robot_simulator/get_configuration',
                                       GetRobotConfiguration,
                                       kmr_simulator.handle_get_configuration)
    set_config_service = rospy.Service('/robot_simulator/set_configuration',
                                       SetRobotConfiguration,
                                       kmr_simulator.handle_set_configuration)
    cleanup_service = rospy.Service('/robot_simulator/cleanup_robot_simulator', EmptyService,
                                    kmr_simulator.clean_up)
    rospy.loginfo('Running kmr simulator wrapper')
    rospy.spin()