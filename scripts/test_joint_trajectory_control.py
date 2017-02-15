#! /usr/bin/python
import rospy
import IPython
from hfts_grasp_planner.srv import PlanArmMotionRequest, PlanArmMotion, PlanGraspMotion, PlanGraspMotionRequest
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped
from kmr_dream_portfolio.kth.hfts_grasping import SynchedHandArmController


def remove_joints_from_traj(joint_names, trajectory):
    indices = []
    num_joints = len(trajectory.joint_names)
    for name_idx in range(num_joints):
        if trajectory.joint_names[name_idx] in joint_names:
            indices.append(name_idx)
    trajectory.joint_names = [trajectory.joint_names[i] for i in range(num_joints) if i not in indices]
    for point in trajectory.points:
        point.positions = [point.positions[i] for i in range(len(point.positions)) if i not in indices]
        point.velocities = [point.velocities[i] for i in range(len(point.velocities)) if i not in indices]
        point.accelerations = [point.accelerations[i] for i in range(len(point.accelerations)) if i not in indices]


def call_service(service, request):
    response = service(request)
    remove_joints_from_traj(['hand_JF20', 'hand_JF2V'], response.trajectory)
    return response

if __name__ == "__main__":
    rospy.init_node('test_joint_trajectory_control')
    move_arm_service = rospy.ServiceProxy('/hfts_integrated_planner_node/plan_arm_motion',
                                          PlanArmMotion)
    grasp_move_arm_service = rospy.ServiceProxy('/hfts_integrated_planner_node/plan_fingertip_grasp_motion',
                                                PlanGraspMotion)
    traj_pub = rospy.Publisher('/kmr/lbr_joint_trajectory_controller/command',
                               JointTrajectory, queue_size=1)
    pose = PoseStamped()
    pose.pose.position.x = 0.87
    pose.pose.position.y = 0.35
    pose.pose.position.z = 1.8
    arm_move_request = PlanArmMotionRequest(target_pose=pose)
    grasp_move_request = PlanGraspMotionRequest(object_identifier='crayola')
    hand_arm_controller = SynchedHandArmController(arm_trajectory_action_name='/kmr/lbr_joint_trajectory_controller/follow_joint_trajectory',
                                                   robotiq_hand_command_topic='/robotiq_hand/command',
                                                   robotiq_hand_state_topic='/robotiq_hand/joint_states',
                                                   arm_joint_names=['lbrAxis1', 'lbrAxis2', 'lbrAxis3',
                                                                    'lbrAxis4', 'lbrAxis5', 'lbrAxis6', 'lbrAxis7'],
                                                   hand_joint_names=['scissor_joint', 'finger_1_joint_1',
                                                                     'finger_2_joint_1', 'finger_middle_joint_1'])
    IPython.embed()
