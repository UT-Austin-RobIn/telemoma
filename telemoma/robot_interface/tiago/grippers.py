import rospy
from std_msgs.msg import Header, Bool
from control_msgs.msg  import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from telemoma.utils.ros_utils import Listener, Publisher

# gripper command between 0.0-1.0
class PALGripper:

    def __init__(self, side):
        self.side = side

        self.gripper_min = 0.0
        self.gripper_max = 0.09
        
        def process_gripper(message):
            return message.actual.positions[0]

        self.gripper_reader = Listener(input_topic_name=f'/parallel_gripper_{self.side}_controller/state', input_message_type=JointTrajectoryControllerState, post_process_func=process_gripper)
        self.gripper_writer = Publisher(f'/parallel_gripper_{self.side}_controller/command', JointTrajectory)

    def get_state(self):
        dist =  self.gripper_reader.get_most_recent_msg()

        # normalize gripper state and return
        return (dist - self.gripper_min)/(self.gripper_max - self.gripper_min)

    def create_gripper_command(self, dist):
        message = JointTrajectory()
        message.header = Header()
        message.joint_names = ['parallel_gripper_joint']
        point = JointTrajectoryPoint(positions=[dist], time_from_start = rospy.Duration(0.5))
        message.points.append(point)
        
        return message

    def step(self, gripper_act):
        # unnormalize gripper action
        gripper_act = gripper_act*(self.gripper_max - self.gripper_min) + self.gripper_min
        gripper_cmd = self.create_gripper_command(gripper_act)

        self.gripper_writer.write(gripper_cmd)


# gripper command between 0.0-1.0
class RobotiqGripper2F:

    def __init__(self, side):
        self.side = side

        self.gripper_min = 0.0
        self.gripper_max = None # determined in subclass
        
        def process_gripper(message):
            return message.actual.positions[0]

        self.gripper_reader = Listener(input_topic_name=f'/gripper_{self.side}_controller/state', input_message_type=JointTrajectoryControllerState, post_process_func=process_gripper)
        self.gripper_writer = Publisher(f'/gripper_{self.side}_controller/command', JointTrajectory)
    
        self.is_grasped = Listener(input_topic_name=f'/gripper_{self.side}/is_grasped', input_message_type=Bool, post_process_func=lambda x: x.data)

    def get_state(self):
        dist =  self.gripper_reader.get_most_recent_msg()

        # normalize gripper state and return
        return (self.gripper_max - dist)/(self.gripper_max - self.gripper_min)

    def create_gripper_command(self, dist):
        message = JointTrajectory()
        message.header = Header()
        message.joint_names = [f'gripper_{self.side}_finger_joint']
        point = JointTrajectoryPoint(positions=[dist], time_from_start = rospy.Duration(0.5))
        message.points.append(point)
        
        return message

    def step(self, gripper_act):
        if self.is_grasped.get_most_recent_msg() and gripper_act < self.get_state():
            gripper_act = self.get_state()
            # print('grasped')

        # unnormalize gripper action
        gripper_act = self.gripper_max - gripper_act*(self.gripper_max - self.gripper_min)
        gripper_cmd = self.create_gripper_command(gripper_act)
        self.gripper_writer.write(gripper_cmd)

class RobotiqGripper2F_140(RobotiqGripper2F):
    def __init__(self, side):
        super().__init__(side)
        self.gripper_max = 0.7

class RobotiqGripper2F_85(RobotiqGripper2F):
    def __init__(self, side):
        super().__init__(side)
        self.gripper_max = 0.8



