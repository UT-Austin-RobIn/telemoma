import numpy as np
from telemoma.robot_interface.igibson.igibson_env import iGibsonEnv
from igibson.robots import REGISTERED_ROBOTS
from telemoma.utils.transformations import quat_to_euler, quat_diff

class FetchEnv(iGibsonEnv):

    def __init__(self):
        super().__init__(robot_name="Fetch")

        # Load robot
        self.robot = REGISTERED_ROBOTS[self.robot_name](
            action_type="continuous",
            action_normalize=True,
            controller_config={
                    'camera': {'name': 'JointController'}, 
                    'base': {'name': 'DifferentialDriveController'}, 
                    'arm_0': {'name': 'InverseKinematicsController'},
                    'gripper_0': {'name': 'JointController'},
                },
        )
        self.s.import_object(self.robot)
        self.min_gripper = 0.0
        self.max_gripper = 0.05

        self.robot_reference = self.robot._get_proprioception_dict()

    def rescale_gripper(self, gripper_qpos):
        return (gripper_qpos - self.min_gripper)/(self.max_gripper-self.min_gripper)
    
    def get_proprioception(self):
        proprio = self.robot._get_proprioception_dict()

        current_pos = proprio['robot_pos']
        reference_pos = self.robot_reference['robot_pos']
        delta_pos = current_pos - reference_pos
        
        current_quat = proprio['robot_quat']
        reference_quat = self.robot_reference['robot_quat']
        delta_quat = quat_diff(current_quat, reference_quat)
        delta_euler = quat_to_euler(delta_quat)

        gripper_state = self.rescale_gripper(proprio['gripper_0_qpos'][0])
        cleaned_proprio = dict(
            base=np.r_[delta_pos[:2], delta_euler[2]],
            right=np.r_[proprio['eef_0_pos'], proprio['eef_0_quat'], gripper_state],
            left=None,
            torso=proprio['trunk_qpos'],
        )

        return cleaned_proprio
    
    def preprocess_action(self, action):
        gripper_action = (2*action['right'][-1] - 1)*np.array([1, 1])

        # lin_vel 1, ang_vel 1, cam 2, right arm 6, right gripper 2
        return np.r_[0.3*action['base'][0], -0.2*action['base'][2], [0, 0], action['right'][:-1], gripper_action]