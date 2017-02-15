from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy


def to_ros_trajectory(trajectory):
    points = []
    for wp in trajectory.waypoints:
        time_stamp = rospy.Duration(secs=wp.timestamp[0], nsecs=wp.timestamp[1])
        jtp = JointTrajectoryPoint(positions=wp.positions, velocities=wp.velocities,
                                   accelerations=wp.accelerations, time_from_start=time_stamp)
        points.append(jtp)
    joint_traj = JointTrajectory(joint_names=trajectory.joint_names, points=points)
    return joint_traj
