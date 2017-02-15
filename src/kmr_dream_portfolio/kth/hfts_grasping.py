from manipulation_dreambed.MethodTypes import ArmPlanner, ArmController, GraspPlanner, GraspController
from manipulation_dreambed.MethodTypes import GraspResult as ContextGraspResult
from manipulation_dreambed.Context import Pose as ContextPose
from manipulation_dreambed.Context import ConfigurationWrapper as ContextConfiguration
from manipulation_dreambed import ROSUtils
import actionlib
import rospy
import numpy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
from control_msgs.msg import JointTolerance
from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kmr_dream_portfolio.kth.utils.robotiq_interface import RobotiqHandInterface
from hfts_grasp_planner.srv import PlanArmMotion, PlanArmMotionRequest, PlanGraspMotion, PlanGraspMotionRequest, PlanGrasp, PlanGraspRequest
from hfts_grasp_planner.srv import AddObject, AddObjectRequest, RemoveObject, RemoveObjectRequest



class SceneBuilderInterface(object):
    def __init__(self, add_object_service_name, remove_object_service_name):
        self._add_object_service = rospy.ServiceProxy(add_object_service_name, AddObject)
        self._remove_object_service = rospy.ServiceProxy(remove_object_service_name, RemoveObject)

    def set_object_pose(self, object_info):
        add_object_request = AddObjectRequest()
        add_object_request.object_identifier = object_info.name
        add_object_request.class_identifier = object_info.object_class
        add_object_request.pose = ROSUtils.toROSPose(object_info.pose, bStamped=True)
        response = self._add_object_service(add_object_request)
        return response.success

    def set_robot_pose(self, robot_info):
        add_object_request = AddObjectRequest()
        add_object_request.object_identifier = robot_info.name
        add_object_request.class_identifier = robot_info.name
        add_object_request.pose = ROSUtils.toROSPose(robot_info.pose, bStamped=True)
        response = self._add_object_service(add_object_request)
        # TODO we should also set the configuration here
        return response.success

    def cleanup_scene(self):
        remove_object_request = RemoveObjectRequest()
        remove_object_request.object_identifier = 'all'
        response = self._remove_object_service(remove_object_request)
        if not response.success:
            raise RuntimeError('Could not clean-up planning scene!')

    def synchronize_scene(self, context):
        """ Synchronizes the scene information stored in the context and the world state
            in our OpenRAVE world."""
        scene_info = context.getSceneInformation()
        self.cleanup_scene()
        b_success = self.set_robot_pose(scene_info.getRobotInfo())
        for object_name in scene_info.getObjectNames():
            tb_success = self.set_object_pose(scene_info.getObjectInfo(object_name))
            b_success = tb_success and b_success
        if not b_success:
            raise RuntimeError('Could not synchronize OpenRAVE planning scene with context')


class BiRRTMotionPlanner(ArmPlanner):
    def __init__(self, service_name, add_object_service_name, remove_object_service_name):
        self._arm_service_name = service_name
        self._add_object_service_name = add_object_service_name
        self._remove_object_service_name = remove_object_service_name
        self._plan_arm_motion_service = None
        self._scene_interface = None

    def getName(self):
        return "BiRRTMotionPlanner"

    def getParameters(self, role, paramPrefix):
        if role is not 'ArmPlanner':
            raise ValueError('[BiRRTMotionPlanner::getParameters] Parameters for non implemented role requested: ' + str(role))
        # TODO do we have parameters for this method?
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
        return None

    def initialize(self):
        rospy.loginfo('Waiting for service %s to come up' % self._arm_service_name)
        rospy.wait_for_service(self._arm_service_name)
        self._plan_arm_motion_service = rospy.ServiceProxy(self._arm_service_name, PlanArmMotion)
        self._scene_interface = SceneBuilderInterface(self._add_object_service_name,
                                                      self._remove_object_service_name)

    def allocateResources(self, roles=None):
        pass

    def releaseResources(self, roles=None):
        pass

    def destroy(self):
        pass

    def hasResourceConflict(self, activeRoles):
        return False

    def supportsBatchProcessing(self):
        return False

    def executeBatch(self, startContext, batchInput, parameters):
        raise NotImplementedError('[BiRRTMotionPlanner::executeBatch] Batch processing not supported.')

    def planArmTrajectory(self, goal, context, paramPrefix, parameters):
        # TODO are there any parameters we can set here?
        self._scene_interface.synchronize_scene(context)
        plan_arm_request = PlanArmMotionRequest()
        start_configuration = context.getRobotInformation().configuration
        if type(goal) is not ContextPose:
            raise NotImplementedError('[BiRRTMotionPlanner::planArmTrajectory] Goals of type %s not supported.' % str(type(goal)))
        plan_arm_request.target_pose = ROSUtils.toROSPose(goal, bStamped=True)
        plan_arm_request.start_configuration = ROSUtils.toJointState(start_configuration)
        response = self._plan_arm_motion_service(plan_arm_request)
        if response.planning_success:
            import IPython
            IPython.embed()
            return ROSUtils.toContextTrajectory(response.trajectory)
        return None


class HFTSGraspPlanner(GraspPlanner):
    #TODO wrap service call to openrave node that runs HFTS only
    def __init__(self, service_name):
        self._grasp_planner_service = None
        self._service_name = service_name

    def getName(self):
        return "HFTSGraspPlanner"

    def getParameters(self, role, paramPrefix):
        if role is not 'GraspPlanner':
            raise ValueError('[HFTSGraspPlanner::getParameters] Parameters for non implemented role requested: ' + str(role))
        # TODO do we have parameters for this method?
        return {}

    def getConditionals(self, role, paramPrefix):
        if role is not 'GraspPlanner':
            raise ValueError('[HFTSGraspPlanner::getConditionals] Conditionals for non implemented role requested: ' + str(role))
        # TODO do we have parameters for this method?
        return {}

    def getForbiddenConfigurations(self, role, paramPrefix):
        if role is not 'GraspPlanner':
            raise ValueError('[HFTSGraspPlanner::getForbiddenConfigurations] Conditionals for non implemented role requested: ' + str(role))
        # TODO do we have parameters for this method?
        return {}

    def initialize(self):
        rospy.loginfo('Waiting for service %s to come up' % self._service_name)
        rospy.wait_for_service(self._service_name)
        self._grasp_planner_service = rospy.ServiceProxy(self._service_name, PlanGrasp)

    def allocateResources(self, roles=None):
        pass

    def releaseResources(self, roles=None):
        pass

    def destroy(self):
        self._grasp_planner_service = None

    def hasResourceConflict(self, activeRoles):
        return False

    def supportsBatchProcessing(self):
        return False

    def executeBatch(self, startContext, batchInput, parameters):
        raise NotImplementedError('[HFTSGraspPlanner::executeBatch] Batch processing not supported.')

    def planGrasp(self, object, context, paramPrefix, parameters):
        object_info = context.getSceneInformation().getObjectInfo(object)
        plan_grasp_request = PlanGraspRequest()
        plan_grasp_request.object_identifier = object_info.object_class
        response = self._grasp_planner_service(plan_grasp_request)
        if response.planning_success:
            grasp_result = ContextGraspResult()
            grasp_result.configuration = ROSUtils.toContextConfiguration(response.hand_configuration)
            grasp_result.pose = ROSUtils.toContextPose(response.grasp_pose)
            # TODO approach direction?
            return grasp_result
        return None


class IntegratedHFTSPlanner(GraspPlanner, GraspController, ArmPlanner, ArmController):
    # TODO wrap service calls to openrave node to plan both ArmMotion and Grasps
    def __init__(self, arm_trajectory_action_name, robotiq_hand_command_topic,
                 robotiq_hand_state_topic, arm_joint_names, hand_joint_names,
                 integrated_hfts_service_name, arm_service_name,
                 add_object_service_name, remove_object_service_name):
        self._arm_trajectory_action_name = arm_trajectory_action_name
        self._robotiq_hand_command_topic_name = robotiq_hand_command_topic
        self._robotiq_hand_state_topic_name = robotiq_hand_state_topic
        self._arm_joint_names = arm_joint_names
        self._hand_joint_names = hand_joint_names
        self._integrated_hfts_service_name = integrated_hfts_service_name
        self._arm_service_name = arm_service_name
        self._add_object_service_name = add_object_service_name
        self._remove_object_service_name = remove_object_service_name
        self._synched_hand_arm_controller = None
        self._integrated_hfts_service = None
        self._plan_arm_motion_service = None
        self._scene_interface = None

    def getName(self):
        return "IntegratedHFTSPlanner"

    def getParameters(self, role, paramPrefix):
        if role not in ['ArmPlanner', 'GraspPlanner', 'GraspController', 'ArmController']:
            raise ValueError('[IntegratedHFTSPlanner::getParameters] Parameters for non implemented role requested: ' + str(role))
        # TODO return parameters definition
        # closing_offset_scissor_joint, closing_offset_finger_2_joint_1
        return {}

    def getConditionals(self, role, paramPrefix):
        if role not in ['ArmPlanner', 'GraspPlanner', 'GraspController', 'ArmController']:
            raise ValueError('[IntegratedHFTSPlanner::getConditionals] Conditionals for non implemented role requested: ' + str(role))
        # TODO do we have parameters for this method?
        return {}

    def getForbiddenConfigurations(self, role, paramPrefix):
        if role not in ['ArmPlanner', 'GraspPlanner', 'GraspController', 'ArmController']:
            raise ValueError('[IntegratedHFTSPlanner::getForbiddenConfigurations] Conditionals for non implemented role requested: ' + str(role))
        # TODO do we have parameters for this method?
        return {}

    def initialize(self):
        rospy.loginfo('Waiting for service %s to come up' % self._integrated_hfts_service_name)
        rospy.wait_for_service(self._integrated_hfts_service_name)
        rospy.loginfo('Waiting for service %s to come up' % self._arm_service_name)
        rospy.wait_for_service(self._arm_service_name)
        rospy.loginfo('Waiting for service %s to come up' % self._add_object_service_name)
        rospy.wait_for_service(self._add_object_service_name)
        rospy.loginfo('Waiting for service %s to come up' % self._remove_object_service_name)
        rospy.wait_for_service(self._remove_object_service_name)
        self._synched_hand_arm_controller = SynchedHandArmController(self._arm_trajectory_action_name,
                                                                     self._robotiq_hand_command_topic_name,
                                                                     self._robotiq_hand_state_topic_name,
                                                                     self._arm_joint_names,
                                                                     self._hand_joint_names)
        self._scene_interface = SceneBuilderInterface(self._add_object_service_name, self._remove_object_service_name)
        self._integrated_hfts_service = rospy.ServiceProxy(self._integrated_hfts_service_name, PlanGraspMotion)
        self._plan_arm_motion_service = rospy.ServiceProxy(self._arm_service_name, PlanArmMotion)

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

    def supportsBatchProcessing(self):
        # TODO return True
        return False

    def executeBatch(self, startContext, batchInput, parameters):
        # TODO
        raise NotImplementedError('IMPLEMENT ME')

    def planGrasp(self, object, context, paramPrefix, parameters):
        # If this function is called, it means we are only asked to plan a grasp
        self._scene_interface.synchronize_scene(context)
        trajectory, pose = self._call_hfts_planner(object, context, paramPrefix, parameters)
        return self._make_grasp_result(trajectory, pose)

    def planArmTrajectory(self, goal, context, paramPrefix, parameters):
        # If this function is called, it means we are only asked to plan an arm motion
        self._scene_interface.synchronize_scene(context)
        plan_arm_request = PlanArmMotionRequest()
        start_configuration = context.getRobotInformation().configuration
        if type(goal) is not ContextPose:
            raise NotImplementedError('[IntegratedHFTSPlanner::planArmTrajectory] Goals of type %s not supported.' % str(type(goal)))
        plan_arm_request.target_pose = ROSUtils.toROSPose(goal, bStamped=True)
        plan_arm_request.start_configuration = ROSUtils.toJointState(start_configuration)
        response = self._plan_arm_motion_service(plan_arm_request)
        if response.planning_success:
            return ROSUtils.toContextTrajectory(response.trajectory)
        return None

    def executeArmTrajectory(self, trajectory, context, paramPrefix, parameters):
        # If this function is called, it means we are only asked to execute an arm motion
        ros_traj = ROSUtils.toROSTrajectory(trajectory)
        ros_traj.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.01)
        self._synched_hand_arm_controller.execute_arm_trajectory(ros_traj, block=True)
        return True

    def startGraspExecution(self, grasp, context, paramPrefix, parameters):
        """ Closes the hand to grasp an object. """
        input_to_hand = {}
        for (key, value) in grasp.hand_configuration.iteritems():
            closing_offset = parameters[paramPrefix + 'closing_offset_' + key]
            input_to_hand[key] = value + closing_offset

        input_to_hand['delta_t'] = 0.0
        input_to_hand['reduced_dofs'] = True
        self._synched_hand_arm_controller.hand_interface.set_goal_configuration(*input_to_hand)
        # TODO return value based on what simulator claims?
        return True

    def stopGraspExecution(self, grasp, context, paramPrefix, parameters):
        return True

    def _call_hfts_planner(self, object_name, context, paramPrefix, parameters):
        # TODO set parameters
        object_info = context.getSceneInformation().getObjectInfo(object_name)
        hfts_planner_request = PlanGraspMotionRequest()
        hfts_planner_request.object_identifier = object_name
        hfts_planner_request.model_identifier = object_info.object_class
        hfts_planner_request.object_pose = ROSUtils.toROSPose(object_info.pose, bStamped=True)
        start_config = context.getRobotInformation().configuration
        hfts_planner_request.start_configuration = ROSUtils.toJointState(start_config)
        response = self._integrated_hfts_service(hfts_planner_request)
        if not response.planning_success:
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
        grasp_result = ContextGraspResult()
        grasp_result.configuration = ContextConfiguration(dict(zip(grasp_joint_names, grasp_joint_values)))
        grasp_result.pose = ROSUtils.toContextPose(pose)
        return grasp_result


class SynchedHandArmController(object):
    def __init__(self, arm_trajectory_action_name,
                 robotiq_hand_command_topic, robotiq_hand_state_topic,
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
        self.hand_interface = RobotiqHandInterface(robotiq_hand_command_topic, robotiq_hand_state_topic)
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


