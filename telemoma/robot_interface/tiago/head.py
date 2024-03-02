from geometry_msgs.msg import PoseStamped

from telemoma.utils.ros_utils import Publisher, create_pose_command
from telemoma.utils.camera_utils import Camera

class TiagoHead:

    def __init__(self, head_policy) -> None:
        self.head_enabled = head_policy is not None
        self.head_policy = head_policy

        self.img_topic = "/xtion/rgb/image_raw"
        self.depth_topic = "/xtion/depth/image_raw"
        self.head_camera = Camera(img_topic=self.img_topic, depth_topic=self.depth_topic)

        self.setup_actors()

    def setup_actors(self):
        self.head_writer = None
        if self.head_enabled:
            self.head_writer = Publisher('/whole_body_kinematic_controller/gaze_objective_xtion_optical_frame_goal', PoseStamped)

    def write(self, trans, quat):
        if self.head_enabled:
            self.head_writer.write(create_pose_command(trans, quat))
    
    def get_camera_obs(self):
        return self.head_camera.get_camera_obs()
    
    def step(self, env_action):
        pos, quat = self.head_policy.get_action(env_action)
        if pos is None:
            return
        self.write(pos, quat)

    def reset_step(self, env_action):
        pos, quat = self.head_policy.get_action(env_action, euler=False)
        if pos is None:
            return
        self.write(pos, quat)

class TiagoHeadPolicy:

    def get_action(self, env_action, euler=True):
        '''
            if euler is true then env_action[arm] is expected to be a 7 dimensional vector -> pos(3), rot(3), grip(1) 
            otherwise, rot(4) is expected as a quat
        '''
        raise NotImplementedError

class FollowHandPolicy(TiagoHeadPolicy):

    def __init__(self, arm='right'):
        super().__init__()
        assert arm in ['right', 'left']

        self.arm = arm
    
    def get_action(self, env_action, euler=True):
        if env_action[self.arm] is None:
            return None, None
        
        position = env_action[self.arm][:3]
        return position, [0, 0, 0, 1]
    

class LookAtFixedPoint(TiagoHeadPolicy):

    def __init__(self, point) -> None:
        super().__init__()

        self.point = point
    
    def get_action(self, env_action, euler=True):
        position = self.point[:3]

        return position, [0, 0, 0, 1]

        


