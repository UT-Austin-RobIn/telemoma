import gym
import time
import numpy as np
from collections import OrderedDict

import rospy

from telemoma.robot_interface.hsr.hsr_core import HSR
from telemoma.utils.general_utils import AttrDict

class HSRGym(gym.Env):

    def __init__(self,
                    frequency=10,
                    head_policy=None,
                    base_enabled=True,
                    torso_enabled=False,
                    arm_enabled=True,
                    external_cams={}):
        
        super(HSRGym).__init__()

        self.frequency = frequency
        self.base_enabled = base_enabled
        self.torso_enabled = torso_enabled
        self.arm_enabled = arm_enabled

        self.hsr = HSR(
                        head_policy=head_policy,
                        base_enabled=base_enabled,
                        torso_enabled=torso_enabled,
                        arm_enabled=arm_enabled,
                    )

        self.cameras = OrderedDict()
        for cam_name in external_cams.keys():
            self.cameras[cam_name] = external_cams[cam_name]

        self.steps = 0

    @property
    def observation_space(self):
        ob_space = OrderedDict()

        ob_space['left'] = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(3+4,),
        )

        ob_space['right'] = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(3+4,),
        )

        ob_space['gripper'] = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(1,),
        )

        ob_space['base'] = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(2+1,) # 2d x, y position delta, 1d z orientation delta
        )

        for cam in self.cameras.keys():
            
            ob_space[f'{cam}_image'] = gym.spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=self.cameras[cam].img_shape,
            ) 

            ob_space[f'{cam}_depth'] = gym.spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=self.cameras[cam].depth_shape,
            )

        return gym.spaces.Dict(ob_space)

    @property
    def action_space(self):
        act_space = OrderedDict()
        
        if self.arm_enabled:
            act_space['arm'] = gym.spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(3+4+int(self.right_gripper_enabled)),
            )

        if self.base_enabled:
            act_space['base'] = gym.spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(3,), # 2d x, y linear velocity, 1d z angular velocity
            )

        return gym.spaces.Dict(act_space)


    def _observation(self):
        observations = AttrDict({
            'left': np.r_[self.hsr.eef_pose, self.hsr.gripper_state],
            'right': np.array([0, 0, 0, 0, 0, 0, 1]),
            'base': np.array(self.hsr.get_delta_pose())
        })

        for cam in self.cameras.keys():
            observations[f'{cam}_image'] = np.array(self.cameras[cam].get_img())
            observations[f'{cam}_depth'] = np.array(self.cameras[cam].get_depth())

        return observations

    def reset(self, *args, **kwargs):
        self.start_time = None
        self.end_time = None
        self.steps = 0

        self.hsr.reset(*args, **kwargs)
        
        return self._observation()
    
    def step(self, action):

        if action is not None:
            self.hsr.step(action)
        
        self.end_time = time.time()
        if self.start_time is not None:
            # print('Idle time:', 1/self.frequency - (self.end_time-self.start_time))
            rospy.sleep(max(0., 1/self.frequency - (self.end_time-self.start_time)))
        self.start_time = time.time()

        obs = self._observation()
        rew = 0
        done = False
        info = {}

        self.steps += 1

        return obs, rew, done, info