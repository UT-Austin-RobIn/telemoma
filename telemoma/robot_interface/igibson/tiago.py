import numpy as np
from telemoma.robot_interface.igibson.igibson_env import iGibsonEnv
from igibson.robots import REGISTERED_ROBOTS
from telemoma.utils.transformations import quat_to_euler, quat_diff

class TiagoEnv(iGibsonEnv):

    def __init__(self):
        super().__init__(robot_name="Tiago")

        # Load robot
        self.robot = REGISTERED_ROBOTS[self.robot_name](
            action_type="continuous",
            action_normalize=True,
            controller_config={
                    'camera': {'name': 'JointController'}, 
                    'base': {'name': 'JointController'}, 
                    'arm_0': {'name': 'InverseKinematicsController'},
                    'gripper_0': {'name': 'JointController'},
                    'arm_1': {'name': 'InverseKinematicsController'},
                    'gripper_1': {'name': 'JointController'},
                },
        )
        self.s.import_object(self.robot)
        self.min_gripper = 0
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

        gripper_left_state = self.rescale_gripper(proprio['gripper_left_qpos'][0])
        gripper_right_state = self.rescale_gripper(proprio['gripper_right_qpos'][0])
        cleaned_proprio = dict(
            base=np.r_[delta_pos[:2], delta_euler[2]],
            left=np.r_[proprio['eef_left_pos'], proprio['eef_left_quat'], gripper_left_state],
            right=np.r_[proprio['eef_right_pos'], proprio['eef_right_quat'], gripper_right_state],
            torso=proprio['trunk_qpos'],
        )

        return cleaned_proprio
    
    def preprocess_action(self, action):
        action['left'][-1] = 2*action['left'][-1] - 1
        action['right'][-1] = 2*action['right'][-1] - 1

        # lin_vel 2, ang_vel 1, 2 cam, left arm 6, left gripper 1, right arm 6, right gripper 1
        return np.r_[0.2*action['base'], [0, 0], action['left'],  action['right']]