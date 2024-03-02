import os
import time
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from tf import transformations as T
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Odometry

from telemoma.utils.transformations import euler_to_quat, quat_to_euler, add_angles, quat_diff
from telemoma.utils.ros_utils import Publisher, Listener, create_twist_command
from hsrb_interface import Robot, exceptions
from tracikpy import TracIKSolver


def process_odom(message):
    processed_odom = {}

    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    processed_odom['pose'] = np.array(
        [position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w])

    linear = message.twist.twist.linear
    angular = message.twist.twist.angular
    processed_odom['velocity'] = np.array([linear.x, linear.y, linear.z, angular.x, angular.y, angular.z])
    return processed_odom

class HSR:

    def __init__(self,
                    head_policy=None,
                    base_enabled=False,
                    torso_enabled=False,
                    arm_enabled=True):
        

        self.head_enabled = head_policy is not None
        self.base_enabled = base_enabled
        self.torso_enabled = torso_enabled
        self.arm_enabled = arm_enabled

        self.gripper_max = 0.13487309570568914
        self.gripper_min = -0.06273195173431098
        self.gripper_cmd_max = 1.2
        self.gripper_cmd_min = -0.5

        while not rospy.is_shutdown():
            try:
                self.robot = Robot()
                self.whole_body = self.robot.try_get('whole_body')
                self.gripper = self.robot.try_get('gripper')
                self.base = self.robot.try_get('omni_base')

                dir_path = os.path.dirname(os.path.realpath(__file__))
                self.ik_solver = TracIKSolver(dir_path+"/../../urdf/hsrb4s.urdf", "base_footprint", "hand_palm_link",
                                              timeout=0.025, epsilon=5e-4, solve_type="Distance")

                break
            except (exceptions.ResourceNotFoundError, exceptions.RobotConnectionError) as e:
                rospy.logerr_throttle(1, 'Failed to obtain resource: {}\nRetrying...'.format(e))

        self.reset_pose = {
            "arm": {'arm_flex_joint': 0.0,
                    'arm_lift_joint': 0.0,
                    'arm_roll_joint': 0.0,
                    'wrist_flex_joint': -1.57,
                    'wrist_roll_joint': 0.0},
            "head": {'head_pan_joint': 0.0,
                     'head_tilt_joint': -0.2}
        }

        self.odom_listener = Listener(input_topic_name='/hsrb/odom',
                                      input_message_type=Odometry,
                                      post_process_func=process_odom)

        self.base_writer = Publisher('/hsrb/command_velocity', Twist)
        self.arm_writer = Publisher('/hsrb/arm_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.lin_scale = 0.2
        self.ang_scale = 0.2

        self.reference_odom = self.odom_listener.get_most_recent_msg()

    @property
    def eef_pose(self):
        # start_time = time.time()
        p = self.whole_body.joint_positions
        joint_positions = np.zeros(self.ik_solver.number_of_joints)
        for joint in range(3, self.ik_solver.number_of_joints):
            joint_positions[joint] = p[self.ik_solver.joint_names[joint]]
        print (joint_positions)
        eef_matrix = self.ik_solver.fk(joint_positions)
        q = T.quaternion_from_matrix(eef_matrix)
        t = T.translation_from_matrix(eef_matrix)
        # print ("eef fk time: ", time.time() - start_time, t, q)

        return np.concatenate((t, q))

    @property
    def gripper_state(self):
        dist = self.gripper.get_distance()

        return (dist - self.gripper_min) / (self.gripper_max - self.gripper_min)

    def get_delta_pose(self):
        # returns a 3d vector corresponding to change in position in x, y (2d) and change in angle in z (1d)
        current_odom = self.odom_listener.get_most_recent_msg()

        current_pos = current_odom['pose'][:3]
        reference_pos = self.reference_odom['pose'][:3]
        delta_pos = current_pos - reference_pos

        current_quat = current_odom['pose'][3:]
        reference_quat = self.reference_odom['pose'][3:]
        delta_quat = quat_diff(current_quat, reference_quat)
        delta_euler = quat_to_euler(delta_quat)

        return np.array([delta_pos[0], delta_pos[1], delta_euler[2]])

    def process_action(self, action):
        # convert deltas to absolute positions
        pos_delta, euler_delta, gripper = action[:3], action[3:6], action[6]
        cur_pose = self.eef_pose
        cur_pos, cur_euler = cur_pose[:3], quat_to_euler(cur_pose[3:])

        target_pos = cur_pos + pos_delta
        target_euler = add_angles(euler_delta, cur_euler)
        target_quat = euler_to_quat(target_euler)

        return target_pos, target_quat, gripper

    def step(self, action):

        if self.arm_enabled:
            pos, quat, gripper_act = self.process_action(action['left'])

            # trac-ik
            ee_matrix = T.quaternion_matrix(quat)
            ee_matrix = np.dot(T.translation_matrix(pos), ee_matrix)
            ik_solution = self.ik_solver.ik(ee_matrix, qinit=np.zeros(self.ik_solver.number_of_joints))

            if ik_solution is None:
                print('No IK solution for ', pos, quat, self.eef_pose)
            else:
                base_pose, joint_positions = ik_solution[:3], ik_solution[3:]
                joint_goal = {
                    'arm_lift_joint': joint_positions[0],
                    'arm_flex_joint': joint_positions[1],
                    'arm_roll_joint': joint_positions[2],
                    'wrist_flex_joint': joint_positions[3],
                    'wrist_roll_joint': joint_positions[4]
                }
                arm_traj = JointTrajectory()
                arm_traj.joint_names = joint_goal.keys()

                arm_p = JointTrajectoryPoint()
                arm_p.positions = joint_positions
                arm_p.velocities = np.zeros(len(joint_positions))
                arm_p.time_from_start = rospy.Duration(0.5)
                arm_traj.points = [arm_p]
                self.arm_writer.write(arm_traj)

            if abs(gripper_act - self.gripper_state) > 0.2:
                print (gripper_act, self.gripper_state)
                gripper_cmd = gripper_act*(self.gripper_cmd_max - self.gripper_cmd_min) + self.gripper_cmd_min
                if gripper_act == 0:
                    self.gripper.apply_force(0.6, sync=False)
                else:
                    self.gripper.command(gripper_cmd, sync=False)
        
        if self.base_enabled:
            base_pose = action['base']
            if base_pose is None:
                self.base_writer.write(create_twist_command(np.zeros(3), np.zeros(3)))
            else:
                lin_cmd = np.zeros(3)
                lin_cmd[:2] = base_pose[:2]
                lin_cmd = self.lin_scale * lin_cmd

                ang_cmd = np.zeros(3)
                if abs(base_pose[2]) > 0.25:
                    ang_cmd[2] = base_pose[2]
                    ang_cmd = self.ang_scale * ang_cmd

                self.base_writer.write(create_twist_command(lin_cmd, ang_cmd))

    def reset(self, reset_arms=True):
        if reset_arms:
            if self.reset_pose is not None:
                self.whole_body.move_to_joint_positions(goals=self.reset_pose["arm"])

        if self.reset_pose is not None:
            self.whole_body.move_to_joint_positions(goals=self.reset_pose["head"])

        input('Reset complete. Press ENTER to continue')

