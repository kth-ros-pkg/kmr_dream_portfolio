from manipulation_dreambed.MethodTypes import ArmPlanner, ArmController, GraspPlanner, GraspController
from manipulation_dreambed.MethodTypes import GraspResult as ContextGraspResult
from manipulation_dreambed.Context import Pose as ContextPose
from manipulation_dreambed.ActionPrimitives import GraspReference
import manipulation_dreambed.DummySimulator as DummySimulator
from manipulation_dreambed import ROSUtils
import actionlib
import dynamic_reconfigure.client
import rospy
import numpy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
from control_msgs.msg import JointTolerance
from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kmr_dream_portfolio.kth.utils.robotiq_interface import RobotiqHandInterface
from hfts_grasp_planner.srv import PlanArmMotion, PlanArmMotionRequest, PlanGraspMotion, PlanGraspMotionRequest, PlanGrasp, PlanGraspRequest
from hfts_grasp_planner.srv import AddObject, AddObjectRequest, RemoveObject, RemoveObjectRequest


def send_parameters(parameter_description, param_prefix, parameters, client):
    params_to_send = {}
    for (name, description) in parameter_description.iteritems():
        short_name = name.replace(param_prefix + '_', '')
        params_to_send[short_name] = parameters[name]

    ROSUtils.guarded_service_call(client.update_configuration,
                                  params_to_send,
                                  client.name + '/set_parameters')


class SceneBuilderInterface(object):
    def __init__(self, add_object_service_name, remove_object_service_name):
        self._add_object_service_name = add_object_service_name
        self._remove_object_service_name = remove_object_service_name
        self._add_object_service = rospy.ServiceProxy(add_object_service_name, AddObject)
        self._remove_object_service = rospy.ServiceProxy(remove_object_service_name, RemoveObject)

    def set_object_pose(self, object_info):
        add_object_request = AddObjectRequest()
        add_object_request.object_identifier = object_info.name
        add_object_request.class_identifier = object_info.object_class
        add_object_request.pose = ROSUtils.toROSPose(object_info.pose, bStamped=True)
        response = ROSUtils.guarded_service_call(self._add_object_service,
                                                 add_object_request,
                                                 self._add_object_service_name)
        if response is None:
            return False
        return response.success

    def set_robot_pose(self, robot_info):
        add_object_request = AddObjectRequest()
        add_object_request.object_identifier = robot_info.name
        add_object_request.class_identifier = robot_info.name
        add_object_request.pose = ROSUtils.toROSPose(robot_info.pose, bStamped=True)
        response = ROSUtils.guarded_service_call(self._add_object_service,
                                                 add_object_request,
                                                 self._add_object_service_name)
        if response is None:
            return False
        # TODO we should also set the configuration here
        return response.success

    def cleanup_scene(self):
        remove_object_request = RemoveObjectRequest()
        remove_object_request.object_identifier = 'all'
        response = ROSUtils.guarded_service_call(self._remove_object_service,
                                                 remove_object_request,
                                                 self._remove_object_service_name)
        if response is None or not response.success:
            raise RuntimeError('Could not clean-up planning scene!')

    def synchronize_scene(self, context):
        """ Synchronizes the scene information stored in the context and the world state
            in our OpenRAVE world."""
        scene_info = context.getSceneInformation()
        self.cleanup_scene()
        robot_info = scene_info.getRobotInfo()
        b_success = self.set_robot_pose(robot_info)
        for object_name in scene_info.getObjectNames():
            tb_success = self.set_object_pose(scene_info.getObjectInfo(object_name))
            b_success = tb_success and b_success
        if not b_success:
            raise RuntimeError('Could not synchronize OpenRAVE planning scene with context')


class BiRRTMotionPlanner(ArmPlanner):
    def __init__(self, service_name, add_object_service_name, remove_object_service_name,
                 node_name):
        self._arm_service_name = service_name
        self._add_object_service_name = add_object_service_name
        self._remove_object_service_name = remove_object_service_name
        self._plan_arm_motion_service = None
        self._scene_interface = None
        self._node_name = node_name
        self._parameter_client = None

    def getName(self):
        return "BiRRTMotionPlanner"

    def getParameters(self, role, paramPrefix):
        if role is not 'ArmPlanner':
            raise ValueError('[BiRRTMotionPlanner::getParameters] Parameters for non implemented role requested: ' + str(role))
        # return {paramPrefix + '_velocity_factor': ('real', [0.001, 1.0], 0.2)}
        return {}

    def getConditionals(self, role, paramPrefix):
        if role is not 'ArmPlanner':
            raise ValueError('[BiRRTMotionPlanner::getConditionals] Conditionals for non implemented role requested: ' + str(role))
        # TODO do we have parameters for this method?
        return {}

    def getForbiddenConfigurations(self, role, paramPrefix):
        if role is not 'ArmPlanner':
            raise ValueError('[BiRRTMotionPlanner::getForbiddenConfigurations] Conditionals for non implemented role requested: ' + str(role))
        # TODO do we have parameters for this method?
        return []

    def initialize(self):
        rospy.loginfo('Waiting for service %s to come up' % self._arm_service_name)
        rospy.wait_for_service(self._arm_service_name)
        self._plan_arm_motion_service = rospy.ServiceProxy(self._arm_service_name, PlanArmMotion)
        self._scene_interface = SceneBuilderInterface(self._add_object_service_name,
                                                      self._remove_object_service_name)
        self._parameter_client = dynamic_reconfigure.client.Client(self._node_name)

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
        raise NotImplementedError('[BiRRTMotionPlanner::executeBatch] Batch processing not supported.')

    def planArmTrajectory(self, goal, context, paramPrefix, parameters):
        # TODO are there any parameters we can set here?
        send_parameters(self.getParameters('ArmPlanner', paramPrefix),
                        paramPrefix, parameters, self._parameter_client)
        self._scene_interface.synchronize_scene(context)
        plan_arm_request = PlanArmMotionRequest()
        start_configuration = context.getRobotInformation().configuration
        if type(goal) is not ContextPose:
            raise NotImplementedError('[BiRRTMotionPlanner::planArmTrajectory] Goals of type %s not supported.' % str(type(goal)))
        plan_arm_request.target_pose = ROSUtils.toROSPose(goal, bStamped=True)
        plan_arm_request.start_configuration = ROSUtils.toJointState(start_configuration)
        plan_arm_request.grasped_object = context.getRobotInformation().grasped_object
        response = ROSUtils.guarded_service_call(self._plan_arm_motion_service,
                                                 plan_arm_request,
                                                 self._arm_service_name)
        if response is not None and response.planning_success:
            return ROSUtils.toContextTrajectory(response.trajectory)
        return None


class HFTSGraspPlanner(GraspPlanner):
    #TODO wrap service call to openrave node that runs HFTS only
    def __init__(self, service_name, node_name):
        self._grasp_planner_service = None
        self._service_name = service_name
        self._node_name = node_name
        self._parameter_client = None

    def getName(self):
        return "HFTSGraspPlanner"

    def getParameters(self, role, paramPrefix):
        if role is not 'GraspPlanner':
            raise ValueError('[HFTSGraspPlanner::getParameters] Parameters for non implemented role requested: ' + str(role))
        return {paramPrefix + '_num_planning_attempts': ('integer', [1, 200], 10),
                paramPrefix + '_num_hfts_iterations': ('integer', [1, 200], 40)}

    def getConditionals(self, role, paramPrefix):
        if role is not 'GraspPlanner':
            raise ValueError('[HFTSGraspPlanner::getConditionals] Conditionals for non implemented role requested: ' + str(role))
        # TODO do we have parameters for this method?
        return {}

    def getForbiddenConfigurations(self, role, paramPrefix):
        if role is not 'GraspPlanner':
            raise ValueError('[HFTSGraspPlanner::getForbiddenConfigurations] Conditionals for non implemented role requested: ' + str(role))
        # TODO do we have parameters for this method?
        return []

    def initialize(self):
        rospy.loginfo('Waiting for service %s to come up' % self._service_name)
        rospy.wait_for_service(self._service_name)
        self._grasp_planner_service = rospy.ServiceProxy(self._service_name, PlanGrasp)
        self._parameter_client = dynamic_reconfigure.client.Client(self._node_name)

    def allocateResources(self, roles=None):
        pass

    def releaseResources(self, roles=None):
        pass

    def destroy(self):
        self._grasp_planner_service = None

    def hasResourceConflict(self, activeRoles):
        return False

    def supportsBatchProcessing(self, roleSequence):
        return False

    def executeBatch(self, startContext, batchInput, parameters, getWorldStateFn):
        raise NotImplementedError('[HFTSGraspPlanner::executeBatch] Batch processing not supported.')

    def planGrasp(self, objectName, context, paramPrefix, parameters):
        send_parameters(self.getParameters('GraspPlanner', paramPrefix),
                        paramPrefix, parameters, self._parameter_client)
        object_info = context.getSceneInformation().getObjectInfo(objectName)
        plan_grasp_request = PlanGraspRequest()
        plan_grasp_request.object_identifier = object_info.object_class
        response = ROSUtils.guarded_service_call(self._grasp_planner_service,
                                                 plan_grasp_request,
                                                 self._service_name)
        if response is not None and response.planning_success:
            pose = ROSUtils.toContextPose(response.grasp_pose)
            configuration = ROSUtils.toContextConfiguration(response.hand_configuration)
            # TODO approach direction?
            grasp_result = ContextGraspResult(grasp_pose=pose, approach_vector=None, hand_configuration=configuration)
            return grasp_result
        return None


class IntegratedHFTSPlanner(GraspPlanner, GraspController, ArmPlanner, ArmController):
    # TODO wrap service calls to openrave node to plan both ArmMotion and Grasps
    def __init__(self, arm_trajectory_action_name, robotiq_hand_command_topic,
                 robotiq_hand_state_topic, robotiq_hand_joints_topic,
                 arm_joint_names, hand_joint_names,
                 integrated_hfts_service_name, arm_service_name,
                 add_object_service_name, remove_object_service_name,
                 node_name, use_dummy_control=False):
        self._arm_trajectory_action_name = arm_trajectory_action_name
        self._robotiq_hand_command_topic_name = robotiq_hand_command_topic
        self._robotiq_hand_state_topic_name = robotiq_hand_state_topic
        self._robotiq_hand_joints_topic_name = robotiq_hand_joints_topic
        self._arm_joint_names = arm_joint_names
        self._hand_joint_names = hand_joint_names
        self._integrated_hfts_service_name = integrated_hfts_service_name
        self._arm_service_name = arm_service_name
        self._add_object_service_name = add_object_service_name
        self._remove_object_service_name = remove_object_service_name
        self._node_name = node_name
        self._parameter_client = None
        self._synched_hand_arm_controller = None
        self._integrated_hfts_service = None
        self._plan_arm_motion_service = None
        self._scene_interface = None
        self._last_integrated_solution = None
        if use_dummy_control:
            self._dummy_controller = DummySimulator.getControllerInstance()
        else:
            self._dummy_controller = None

    def getName(self):
        return "IntegratedHFTSPlanner"

    def getParameters(self, role, paramPrefix):
        if role not in ['ArmPlanner', 'GraspPlanner', 'GraspController', 'ArmController']:
            raise ValueError('[IntegratedHFTSPlanner::getParameters] Parameters for non implemented role requested: ' + str(role))
        if role == 'GraspPlanner':
            return {paramPrefix + '_min_iterations': ('integer', [1, 200], 10),
                    paramPrefix + '_max_iterations': ('integer', [1, 200], 10),
                    paramPrefix + '_free_space_weight': ('real', [0.0, 10.0], 0.5),
                    paramPrefix + '_connected_space_weight': ('real', [0.0, 10.0], 4.0),
                    paramPrefix + '_max_num_hierarchy_descends': ('integer', [0, 20], 0),
                    paramPrefix + '_time_limit': ('real', [0.0, 120.0], 60.0),
                    paramPrefix + '_use_approximates': ('integer', [0, 1], 1)}
        if role == 'ArmPlanner':
            # return {paramPrefix + '_velocity_factor': ('real', [0.001, 1.0], 0.2)}
            return {}
        # closing_offset_scissor_joint, closing_offset_finger_2_joint_1
        return {}

    def getConditionals(self, role, paramPrefix):
        if role not in ['ArmPlanner', 'GraspPlanner', 'GraspController', 'ArmController']:
            raise ValueError('[IntegratedHFTSPlanner::getConditionals] Conditionals for non implemented role requested: ' + str(role))
        return {}

    def getForbiddenConfigurations(self, role, paramPrefix):
        if role not in ['ArmPlanner', 'GraspPlanner', 'GraspController', 'ArmController']:
            raise ValueError('[IntegratedHFTSPlanner::getForbiddenConfigurations] Conditionals for non implemented role requested: ' + str(role))
        if role == 'GraspPlanner':
            return ['{%s_max_iterations < %s_min_iterations}' % (paramPrefix, paramPrefix)]
        return []

    def initialize(self):
        rospy.loginfo('Waiting for service %s to come up' % self._integrated_hfts_service_name)
        rospy.wait_for_service(self._integrated_hfts_service_name)
        rospy.loginfo('Waiting for service %s to come up' % self._arm_service_name)
        rospy.wait_for_service(self._arm_service_name)
        rospy.loginfo('Waiting for service %s to come up' % self._add_object_service_name)
        rospy.wait_for_service(self._add_object_service_name)
        rospy.loginfo('Waiting for service %s to come up' % self._remove_object_service_name)
        rospy.wait_for_service(self._remove_object_service_name)
        if self._dummy_controller is None:
            self._synched_hand_arm_controller = SynchedHandArmController(self._arm_trajectory_action_name,
                                                                         self._robotiq_hand_command_topic_name,
                                                                         self._robotiq_hand_state_topic_name,
                                                                         self._robotiq_hand_joints_topic_name,
                                                                         self._arm_joint_names,
                                                                         self._hand_joint_names)
        self._scene_interface = SceneBuilderInterface(self._add_object_service_name, self._remove_object_service_name)
        self._integrated_hfts_service = rospy.ServiceProxy(self._integrated_hfts_service_name, PlanGraspMotion)
        self._plan_arm_motion_service = rospy.ServiceProxy(self._arm_service_name, PlanArmMotion)
        self._parameter_client = dynamic_reconfigure.client.Client(self._node_name)

    def allocateResources(self, roles=None):
        pass

    def releaseResources(self, roles=None):
        pass

    def destroy(self):
        self._synched_hand_arm_controller = None
        self._integrated_hfts_service = None
        self._plan_arm_motion_service = None

    def hasResourceConflict(self, activeRoles):
        return False

    def supportsBatchProcessing(self, roleSequence):
        return len(roleSequence) == 3 and roleSequence[0] == 'GraspPlanner' \
            and roleSequence[1] == 'ArmPlanner' and roleSequence[2] == 'ArmController'

    def executeBatch(self, context, batchInput, parameters, getWorldStateFn):
        # We can be sure that the batch consists of [GraspPlanner, ArmPlanner, ArmController, GraspController]
        assert len(batchInput) == 3 and batchInput[0][0] == 'GraspPlanner' and batchInput[1][0] == 'ArmPlanner' \
            and batchInput[2][0] == 'ArmController'
        grasp_planner_input = batchInput[0][1]
        grasp_planner_param_prefix = grasp_planner_input['paramPrefix']
        arm_planner_input = batchInput[1][1]
        arm_planner_param_prefix = arm_planner_input['paramPrefix']
        # Send parameters for grasp and arm motion planning (TODO this would be a bit problematic if they intersected)
        send_parameters(self.getParameters('GraspPlanner', grasp_planner_param_prefix),
                        grasp_planner_param_prefix, parameters, self._parameter_client)
        send_parameters(self.getParameters('ArmPlanner', arm_planner_param_prefix),
                        arm_planner_param_prefix, parameters, self._parameter_client)
        self._scene_interface.synchronize_scene(context)
        trajectory, pose = self._call_hfts_planner(grasp_planner_input['objectName'],
                                                   context)
        if trajectory is None or pose is None:
            return 3 * [(False, None)]
        results = []
        grasp_result = self._make_grasp_result(trajectory, pose)
        results.append((True, grasp_result))
        context_traj = ROSUtils.toContextTrajectory(trajectory)
        results.append((True, context_traj))
        # TODO if we had parameters for the controllers, we would need to set these here
        if self._dummy_controller is None:
            control_success = self._synched_hand_arm_controller.execute_trajectory(trajectory)
        else:
            control_success = self._dummy_controller.executeArmTrajectory(context_traj, context,
                                                                          None, parameters)
            # control_success = self._dummy_controller.startGraspExecution(grasp_result, context,
            #                                                              None, parameters)
        results.append((control_success, control_success))
        # results.append(control_success, control_success)
        return results

    def planGrasp(self, objectName, context, paramPrefix, parameters):
        # If this function is called, it means we are only asked to plan a grasp
        send_parameters(self.getParameters('GraspPlanner', paramPrefix),
                        paramPrefix, parameters, self._parameter_client)
        self._scene_interface.synchronize_scene(context)
        trajectory, pose = self._call_hfts_planner(objectName, context)
        return self._make_grasp_result(trajectory, pose)

    def planArmTrajectory(self, goal, context, paramPrefix, parameters):
        # If this function is called, it means we are only asked to plan an arm motion
        send_parameters(self.getParameters('ArmPlanner', paramPrefix),
                        paramPrefix, parameters, self._parameter_client)
        self._scene_interface.synchronize_scene(context)
        plan_arm_request = PlanArmMotionRequest()
        start_configuration = context.getRobotInformation().configuration
        if type(goal) is not ContextPose:
            raise NotImplementedError('[IntegratedHFTSPlanner::planArmTrajectory] Goals of type %s not supported.' % str(type(goal)))
        plan_arm_request.target_pose = ROSUtils.toROSPose(goal, bStamped=True)
        plan_arm_request.start_configuration = ROSUtils.toJointState(start_configuration)
        plan_arm_request.grasped_object = context.getRobotInformation().grasped_object
        response = ROSUtils.guarded_service_call(self._plan_arm_motion_service,
                                                 plan_arm_request,
                                                 self._arm_service_name)
        if response is not None and response.planning_success:
            return ROSUtils.toContextTrajectory(response.trajectory)
        return None

    def executeArmTrajectory(self, trajectory, context, paramPrefix, parameters):
        # If this function is called, it means we are only asked to execute an arm motion
        send_parameters(self.getParameters('ArmController', paramPrefix),
                        paramPrefix, parameters, self._parameter_client)
        if self._dummy_controller is None:
            ros_traj = ROSUtils.toROSTrajectory(trajectory)
            ros_traj.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.01)
            self._synched_hand_arm_controller.execute_arm_trajectory(ros_traj, block=True)
            return True
        else:
            return self._dummy_controller.executeArmTrajectory(trajectory, context, paramPrefix, parameters)

    def startGraspExecution(self, grasp, context, paramPrefix, parameters):
        """ Closes the hand to grasp an object. """
        send_parameters(self.getParameters('GraspController', paramPrefix),
                        paramPrefix, parameters, self._parameter_client)
        if self._dummy_controller is not None:
            return self._dummy_controller.startGraspExecution(grasp, context, paramPrefix, parameters)
        input_to_hand = {}
        for (key, value) in grasp.hand_configuration.iteritems():
            closing_offset = parameters[paramPrefix + '_closing_offset_' + key]
            input_to_hand[key] = value + closing_offset

        input_to_hand['delta_t'] = 0.0
        input_to_hand['reduced_dofs'] = True
        input_to_hand['block'] = True
        self._synched_hand_arm_controller.hand_interface.set_goal_configuration(input_to_hand)
        # TODO return value based on what simulator claims?
        return True

    def stopGraspExecution(self):
        if self._dummy_controller is not None:
            return self._dummy_controller.stopGraspExecution()
        return True

    def _call_hfts_planner(self, object_name, context):
        # TODO set parameters
        object_info = context.getSceneInformation().getObjectInfo(object_name)
        hfts_planner_request = PlanGraspMotionRequest()
        hfts_planner_request.object_identifier = object_name
        hfts_planner_request.model_identifier = object_info.object_class
        hfts_planner_request.object_pose = ROSUtils.toROSPose(object_info.pose, bStamped=True)
        start_config = context.getRobotInformation().configuration
        hfts_planner_request.start_configuration = ROSUtils.toJointState(start_config)
        response = ROSUtils.guarded_service_call(self._integrated_hfts_service,
                                                 hfts_planner_request,
                                                 self._integrated_hfts_service_name)
        if response is None or not response.planning_success:
            return None, None
        return response.trajectory, response.grasp_pose

    def _make_grasp_result(self, trajectory, pose):
        if pose is None or trajectory is None:
            return None
        # Let us extract the grasp configuration from the trajectory
        indices = [i for i in range(len(trajectory.joint_names)) if trajectory.joint_names[i] in self._hand_joint_names]
        grasp_joint_values = [trajectory.points[-1].positions[i] for i in indices]
        grasp_joint_names = [trajectory.joint_names[i] for i in indices]
        # Create output
        configuration = dict(zip(grasp_joint_names, grasp_joint_values))
        pose = ROSUtils.toContextPose(pose)
        # TODO what about approach dir?
        approach_dir = None
        grasp_result = ContextGraspResult(grasp_pose=pose, approach_vector=approach_dir,
                                          hand_configuration=configuration)
        return grasp_result


class SynchedHandArmController(object):
    def __init__(self, arm_trajectory_action_name,
                 robotiq_hand_command_topic, robotiq_hand_state_topic,
                 robotiq_hand_joints_topic,
                 arm_joint_names, hand_joint_names):
        """
            Creates a new SynchedHandArmController.
        """
        self._action_client = actionlib.SimpleActionClient(arm_trajectory_action_name, FollowJointTrajectoryAction)
        self._robotiq_pub = rospy.Publisher(robotiq_hand_command_topic, SModelRobotOutput, queue_size=1)
        self._arm_joint_names = arm_joint_names
        self._hand_joint_names = hand_joint_names
        tolerance = JointTolerance()
        self._path_tolerance = len(arm_joint_names) * [tolerance]
        self._goal_tolerance = len(arm_joint_names) * [tolerance]
        self.hand_interface = RobotiqHandInterface(robotiq_hand_command_topic,
                                                   robotiq_hand_state_topic,
                                                   robotiq_hand_joints_topic)
        self._execution_running = False
        self._current_arm_traj = None
        self._current_hand_traj = None
        self._current_traj = None
        self._start_time = None
        self._fake_feedback_timer = None

    def execute_trajectory(self, traj):
        if self._execution_running:
            rospy.logwarn('Request to execute a trajectory while an old one is already running')
            self._action_client.cancel_goal()
            self._execution_running = False
        traj.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.1)
        self._current_arm_traj = ROSUtils.extract_part_trajectory(self._arm_joint_names, traj)
        self._current_hand_traj = ROSUtils.extract_part_trajectory(self._hand_joint_names, traj)
        self._current_traj = traj
        arm_traj_goal = FollowJointTrajectoryActionGoal()
        arm_traj_goal.goal.trajectory = self._current_arm_traj
        arm_traj_goal.goal.goal_tolerance = self._goal_tolerance
        arm_traj_goal.goal.path_tolerance = self._path_tolerance

        # TODO for some reason we do not receive feedback, so we add a fake feedback caller
        # TODO here
        self._action_client.send_goal(arm_traj_goal.goal, active_cb=self._receive_arm_traj_start_signal,
                                      feedback_cb=self._receive_arm_traj_feedback,
                                      done_cb=self._receive_arm_traj_finish_signal)
        self._fake_feedback_timer = rospy.Timer(rospy.Duration.from_sec(0.05),
                                                self._simulate_feedback, oneshot=False)
        # TODO return whether we grasped an object
        return True

    def execute_arm_trajectory(self, traj, block=False):
        if self._execution_running:
            rospy.logwarn('Request to execute a trajectory while an old one is already running')
            self._action_client.cancel_goal()
            self._execution_running = False
        arm_traj_goal = FollowJointTrajectoryActionGoal()
        self._current_arm_traj = traj
        self._current_hand_traj = None
        arm_traj_goal.goal.trajectory = self._current_arm_traj
        arm_traj_goal.goal.goal_tolerance = self._goal_tolerance
        arm_traj_goal.goal.path_tolerance = self._path_tolerance
        if block:
            self._execution_running = True
            self._action_client.send_goal_and_wait(arm_traj_goal.goal)
            self._execution_running = False
            self._current_arm_traj = None
        else:
            self._action_client.send_goal(arm_traj_goal.goal, active_cb=self._receive_arm_traj_start_signal,
                                          done_cb=self._receive_arm_traj_finish_signal)

    @staticmethod
    def _make_args(configuration, delta_t, reduced_dofs):
        adict = {}
        for (key, value) in configuration.iteritems():
            adict[key] = value
        adict['delta_t'] = delta_t
        adict['reduced_dofs'] = reduced_dofs
        return adict

    def _receive_arm_traj_start_signal(self):
        self._execution_running = True
        self._start_time = rospy.Time.now()
        if self._current_hand_traj is not None:
            target_configuration, time_left = self.get_next_hand_target(0.0)
            # TODO our simulator does not support setting velocities, so instead we interpolate
            # TODO the desired hand configuration at this point in time
            kwargs = self._make_args(target_configuration, delta_t=0.0, reduced_dofs=True)
            self.hand_interface.set_goal_configuration(**kwargs)

    def _receive_arm_traj_feedback(self, feedback):
        rospy.loginfo('[SynchedHandArmController] Receiving feedback from arm controller')
        traj_progress_time = feedback.actual.time_from_start.to_sec()
        if self._current_hand_traj is not None:
            target_hand_configuration, time_left = self.get_next_hand_target(traj_progress_time)
            kwargs = self._make_args(target_hand_configuration, delta_t=0.0, reduced_dofs=True)
            self.hand_interface.set_goal_configuration(**kwargs)

    def _simulate_feedback(self, arg):
        # TODO for some reason we do not receive feedback, so let us fake it here
        feedback = FollowJointTrajectoryActionFeedback()
        feedback.feedback.actual.time_from_start = rospy.Time.now() - self._start_time
        self._receive_arm_traj_feedback(feedback.feedback)

    def _receive_arm_traj_finish_signal(self, terminal_state, result):
        # if terminal_state != 0:
        #     rospy.logwarn('[SynchedHandArmController] Arm trajectory execution failed: ' + result.error_string)
        # else:
        if self._current_hand_traj is not None:
            target_hand_configuration, time_left = self.get_next_hand_target(self._current_hand_traj.points[-1].time_from_start.to_sec())
            kwargs = self._make_args(target_hand_configuration, delta_t=0.0, reduced_dofs=True)
            self.hand_interface.set_goal_configuration(**kwargs)
        self._execution_running = False
        self._current_arm_traj = None
        self._current_hand_traj = None
        self._current_traj = None
        if self._fake_feedback_timer is not None:
            self._fake_feedback_timer.shutdown()
            self._fake_feedback_timer = None

    def get_next_hand_target(self, at_time):
        prev_wp = self._current_hand_traj.points[0]
        for wp in self._current_hand_traj.points:
            if wp.time_from_start.to_sec() > at_time:
                time_period = wp.time_from_start.to_sec() - prev_wp.time_from_start.to_sec()
                interpolation_t = at_time - prev_wp.time_from_start.to_sec()
                prev_positions = numpy.array(prev_wp.positions)
                next_positions = numpy.array(wp.positions)
                positions = prev_positions + interpolation_t / time_period * (next_positions - prev_positions)
                configuration = dict(zip(self._current_hand_traj.joint_names, positions))
                time_left = wp.time_from_start.to_sec() - at_time
                return configuration, time_left
            prev_wp = wp
        configuration = dict(zip(self._current_hand_traj.joint_names,
                                 self._current_hand_traj.points[-1].positions))
        time_left = 0.0
        return configuration, time_left


