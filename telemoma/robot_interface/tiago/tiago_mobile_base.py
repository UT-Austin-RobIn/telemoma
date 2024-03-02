import numpy as np

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from telemoma.utils.ros_utils import Publisher, Listener, create_twist_command
from telemoma.utils.transformations import quat_diff, quat_to_euler

class TiagoBase:
    
    def __init__(self, base_enabled) -> None:
        self.base_enabled = base_enabled
        
        self.setup_listener()
        self.setup_actors()

    def setup_listener(self):
        def process_odom(message):
            processed_odom = {}

            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            processed_odom['pose'] = np.array([position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w])

            linear = message.twist.twist.linear
            angular = message.twist.twist.angular
            processed_odom['velocity'] = np.array([linear.x, linear.y, linear.z, angular.x, angular.y, angular.z])
            return processed_odom

        self.odom_listener = Listener(
                                        input_topic_name='/mobile_base_controller/odom',
                                        input_message_type=Odometry,
                                        post_process_func=process_odom)
        

        def process_scan(message):
            min_val = message.range_min
            max_val = message.range_max

            ranges = np.array(message.ranges)
            ranges = np.clip(message.ranges, min_val, max_val)
            return ranges

        self.scan = Listener(
                                input_topic_name='/scan',
                                input_message_type=LaserScan,
                                post_process_func=process_scan       
                                )
        self.reference_odom = self.odom_listener.get_most_recent_msg()

    def setup_actors(self):
        raise NotImplementedError

    def step(self, action):
        raise NotImplementedError
    
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

    def get_velocity(self):
        # returns a 3d vector corresponding to linear velocity in x, y (2d) and angular velocity in z (1d)
        current_velocity =  self.odom_listener.get_most_recent_msg()['velocity']

        return np.array([current_velocity[0], current_velocity[1], current_velocity[5]])
    
    def get_scan(self):
        return self.scan.get_most_recent_msg()

class TiagoBaseVelocityControl(TiagoBase):

    def setup_actors(self):
        self.base_writer = None
        if self.base_enabled:
            self.base_writer = Publisher('/mobile_base_controller/cmd_vel', Twist)

        self.lin_scale = 0.2
        self.ang_scale = 0.2
    
    def step(self, velocity):
        # 3d input where velocity[:2] correspond to linear velocity in x, y and velocity[2] corresponds to angular velocity in z

        if self.base_enabled:
            if velocity is None:
                self.base_writer.write(create_twist_command(np.zeros(3), np.zeros(3)))
                return
            
            lin_cmd = np.zeros(3)
            # if np.linalg.norm(velocity[:2]) > 0.2:
            lin_cmd[:2] = velocity[:2]
            lin_cmd = self.lin_scale * lin_cmd #/np.linalg.norm(lin_cmd)

            ang_cmd = np.zeros(3)
            # if abs(velocity[2]) > 0.25:
            ang_cmd[2] = velocity[2]
            ang_cmd = self.ang_scale * ang_cmd #/np.linalg.norm(ang_cmd)
            
            self.base_writer.write(create_twist_command(lin_cmd, ang_cmd))

    