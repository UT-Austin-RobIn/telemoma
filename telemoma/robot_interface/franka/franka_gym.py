import gym
import time
import sys
import numpy as np
from collections import OrderedDict

import os
deoxys_path = os.path.join(os.path.expanduser("~"), "deoxys_control/deoxys")
sys.path.append(deoxys_path)

from deoxys.franka_interface import FrankaInterface
from deoxys.utils.config_utils import get_default_controller_config
from deoxys import config_root
from deoxys.utils import YamlConfig
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()

from telemoma.utils.general_utils import AttrDict
from telemoma.utils.transformations import rmat_to_quat

class FrankaGym(gym.Env):

    def __init__(self,
                    frequency=10,
                    arm_enabled=True,
                    cameras={}):
        
        super(FrankaGym).__init__()

        self.frequency = frequency
        self.arm_enabled = arm_enabled

        config_path = os.path.join(deoxys_path , "config/charmander.yml")
        self.robot_interface = FrankaInterface(config_path, use_visualizer=False)

        self.controller_type = "OSC_POSE"
        # self.controller_type = "OSC_POSITION"
        # self.controller_type = "OSC_YAW" #"OSC_POSE"
        self.controller_cfg = get_default_controller_config(controller_type=self.controller_type)
        self.cameras = cameras

        self.steps = 0
        self.gripper_max_width = 0.08

    @property
    def observation_space(self):
        # no observation space for testing
        ob_space = OrderedDict()

        ob_space['right'] = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(7,),
        )

        for cam in self.cameras:
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
            act_space['right'] = gym.spaces.Box(
                low=-1.0,
                high=1.0,
                shape=(7,1),
            )

        return gym.spaces.Dict(act_space)

    def get_ee_pose(self):
        while (len(self.robot_interface._state_buffer) == 0) or \
                (len(self.robot_interface._gripper_state_buffer) == 0):
            print('Waiting for robot_interface state buffer...')
            time.sleep(0.01)
        last_state = self.robot_interface._state_buffer[-1]
        last_gripper_state = self.robot_interface._gripper_state_buffer[-1]

        ee_pose = np.array(last_state.O_T_EE).reshape(4, 4)
        pos = ee_pose[-1, :3]
        quat = rmat_to_quat(ee_pose[:3, :3])

        return np.r_[pos, quat, np.array(last_gripper_state.width)/self.gripper_max_width]

    def _observation(self):
        
        ee_pose = self.get_ee_pose()

        observations = AttrDict({
            'left': np.array([0, 0, 0, 0, 0, 0, 1, 1]),
            'right': ee_pose,
            'base': np.array([0, 0, 0])
        })
        # print(np.array(last_gripper_state.width)/0.08)

        for cam in self.cameras.keys():
            observations[f'{cam}_image'] = np.array(self.cameras[cam].get_img())
            observations[f'{cam}_depth'] = np.array(self.cameras[cam].get_depth())

        return observations

    def reset_arms(self):
        controller_cfg = YamlConfig(config_root + f"/joint-position-controller.yml").as_easydict()

        controller_type = "JOINT_POSITION"

        # Golden resetting joints
        reset_joint_positions = [
            0.09162008114028396,
            -0.19826458111314524,
            -0.01990020486871322,
            -2.4732269941140346,
            -0.01307073642274261,
            2.30396583422025,
            0.8480939705504309,
        ]

        # This is for varying initialization of joints a little bit to
        # increase data variation.
        reset_joint_positions = [
            e + np.clip(np.random.randn() * 0.005, -0.005, 0.005)
            for e in reset_joint_positions
        ]
        action = reset_joint_positions + [-1.0]

        while True:
            if len(self.robot_interface._state_buffer) > 0:
                logger.info(f"Current Robot joint: {np.round(self.robot_interface.last_q, 3)}")
                logger.info(f"Desired Robot joint: {np.round(self.robot_interface.last_q_d, 3)}")

                if (np.max(np.abs(np.array(self.robot_interface._state_buffer[-1].q)
                            - np.array(reset_joint_positions))) < 1e-3):
                    break
            self.robot_interface.control(
                controller_type=controller_type,
                action=action,
                controller_cfg=controller_cfg,
            )

    def reset(self, reset_arms=False, *args, **kwargs):
        self.start_time = None
        self.end_time = None
        self.steps = 0

        if reset_arms:
            self.reset_arms()
            time.sleep(1)
        self.robot_interface.reset()
        
        return self._observation()
    
    def apply_constraints(self, action):

        ee_pose = self.get_ee_pose()
        
        # default workspace constraints
        min_xyz = np.array([0.3, -0.3, 0.025])
        max_xyz = np.array([0.75, 0.3, 0.4])
        
        min_constraint_violation = np.less(ee_pose[:3], min_xyz)
        max_constraint_violation = np.less(max_xyz, ee_pose[:3])
        for i, (lc, gc) in enumerate(zip(min_constraint_violation, max_constraint_violation)):
            if lc:
                action[i] = max(0.05, action[i])
            if gc:
                action[i] = min(-0.05, action[i])

        return action
    
    def step(self, action):
        action = action.right.copy()
        if action is not None:
            action[-1] = 1 if action[-1] < 0.5 else -1
            action = self.apply_constraints(action)
            
            self.robot_interface.control(
                controller_type=self.controller_type,
                action=action,
                controller_cfg=self.controller_cfg,
            )
        
        self.end_time = time.time()
        if self.start_time is not None:
            # print('Idle time:', 1/self.frequency - (self.end_time-self.start_time))
            time.sleep(max(0., 1/self.frequency - (self.end_time-self.start_time)))
        self.start_time = time.time()

        obs = self._observation()
        rew = 0
        done = False
        info = {}

        self.steps += 1

        return obs, rew, done, info

