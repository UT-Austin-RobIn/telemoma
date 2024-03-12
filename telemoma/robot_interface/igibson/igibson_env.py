import gym
import time
import numpy as np
from typing import Tuple
from collections import OrderedDict
from telemoma.utils.general_utils import AttrDict
from telemoma.human_interface.teleop_core import TeleopObservation, TeleopAction

from igibson.simulator import Simulator
from igibson.render.mesh_renderer.mesh_renderer_settings import MeshRendererSettings
from igibson.scenes.empty_scene import EmptyScene

class iGibsonEnv(gym.Env):

    def __init__(self, robot_name) -> None:
        self.robot_name = robot_name
        self.teleop_frequency = 10
        self.render_frequency = 20

        gui = 'ig'
        # Infer what GUI(s) to use
        render_mode, use_pb_gui = None, None
        if gui == "ig":
            render_mode, use_pb_gui = "gui_interactive", False
        elif gui == "pb":
            render_mode, use_pb_gui = "headless", True
        else:
            raise ValueError("Unknown GUI: {}".format(gui))
        
        self.s = Simulator( 
                            physics_timestep=1/120.,
                            render_timestep=1/self.render_frequency,
                            mode=render_mode,
                            use_pb_gui=use_pb_gui,
                            image_width=512,
                            image_height=512,
                            rendering_settings=MeshRendererSettings(enable_pbr=False)
                        )

        # Load scene
        scene = EmptyScene(floor_plane_rgba=[0.6, 0.6, 0.6, 1])
        self.s.import_scene(scene)

        self.start_time = None
    
    @property
    def observation_space(self):
        ob_space = OrderedDict()

        ob_space['right'] = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(3+4+1,), # pos, quat, grasp
            dtype=float
        )

        ob_space['left'] = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(3+4+1,), # pos, quat, grasp
            dtype=float
        )

        ob_space['base'] = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(2+1,), # 2d x, y position delta + 1d z rotation delta
            dtype=float
        )

        ob_space['torso'] = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(1,), # torso qpos
            dtype=float
        )

        return gym.spaces.Dict(ob_space)
    
    @property
    def action_space(self):
        act_space = OrderedDict()

        act_space['right'] = gym.spaces.Box(
            low=-1,
            high=1,
            shape=(3+3+1,), # 3d cartesian position delta + 3d cartesian orientationd delta + 1d grasp
            dtype=float
        )

        act_space['left'] = gym.spaces.Box(
            low=-1,
            high=1,
            shape=(3+3+1,), # 3d cartesian position delta + 3d cartesian orientationd delta + 1d grasp
            dtype=float
        )

        act_space['base'] = gym.spaces.Box(
            low=-1,
            high=1,
            shape=(3,), # 2d x, y linear velocity + 1d z angular velocity
            dtype=float
        )

        act_space['torso'] = gym.spaces.Box(
            low=-1,
            high=1,
            shape=(1,), # torso delta
            dtype=float
        )

    def _observation(self) -> TeleopObservation:
        obs = AttrDict(self.get_proprioception())
        return obs

    def reset(self) -> TeleopObservation:
        # Reset the robot
        self.robot.set_position([0, 0, 0])
        self.robot.reset()
        self.robot.keep_still()
        self.robot.untuck()

        self.s.viewer.initial_pos = [-0.5, 0, 1.9]
        self.s.viewer.initial_view_direction = [0.6, 0, -0.6]
        self.s.viewer.reset_viewer()

        self.step_count = 0
        self.start_time = None

        return self._observation()

    def get_proprioception(self):
        raise NotImplementedError
    
    def preprocess_action(self, action):
        raise NotImplementedError

    def step(self, action: TeleopAction) -> Tuple[TeleopObservation, float, bool, dict]:

        action = self.preprocess_action(action)
        self.robot.apply_action(action)

        self.s.step()
        self.step_count += 1
        self.s.step()

        self.end_time = time.time()
        if self.start_time is not None:
            print('Idle time:', 1/self.teleop_frequency - (self.end_time-self.start_time))
            time.sleep(max(0., 1/self.teleop_frequency - (self.end_time-self.start_time)))
        self.start_time = time.time()

        obs = self._observation()

        return obs, 0, False, {}

    def close(self):
        self.s.disconnect()
