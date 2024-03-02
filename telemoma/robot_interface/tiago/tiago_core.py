import time
import rospy

from telemoma.robot_interface.tiago.grippers import PALGripper, RobotiqGripper2F_140, RobotiqGripper2F_85
from telemoma.robot_interface.tiago.head import TiagoHead
from telemoma.robot_interface.tiago.tiago_mobile_base import TiagoBaseVelocityControl
from telemoma.robot_interface.tiago.tiago_torso import TiagoTorso
from telemoma.robot_interface.tiago.tiago_arms import TiagoArms

class Tiago:
    gripper_map = {'pal': PALGripper, 'robotiq2F-140': RobotiqGripper2F_140, 'robotiq2F-85': RobotiqGripper2F_85}

    def __init__(self,
                    head_policy=None,
                    base_enabled=False,
                    torso_enabled=False,
                    right_arm_enabled=True,
                    left_arm_enabled=True,
                    right_gripper_type=None,
                    left_gripper_type=None):
        

        self.head_enabled = head_policy is not None
        self.base_enabled = base_enabled
        self.torso_enabled = torso_enabled
        
        self.head = TiagoHead(head_policy=head_policy)
        self.base = TiagoBaseVelocityControl(base_enabled=base_enabled)
        self.torso = TiagoTorso(torso_enabled=torso_enabled)
        self.arms = {
            'right': TiagoArms(right_arm_enabled, side='right'),
            'left': TiagoArms(left_arm_enabled, side='left'),
        }

        # set up grippers
        self.gripper = {'right': None, 'left': None}
        for side in ['right', 'left']:
            gripper_type = right_gripper_type if side=='right' else left_gripper_type
            if gripper_type is not None:
                self.gripper[side] = self.gripper_map[gripper_type](side)
        
        self.reset_pose = {
                'right': [0.43, -0.81, 1.60, 1.78, 1.34, -0.49, 1.15, 1],
                'left': [0.43, -0.81, 1.60, 1.78, 1.34, -0.49, 1.15, 1],
                'torso': 0.15
            }

    @property
    def right_gripper_pos(self):
        if self.gripper['right'] is None:
            return None
        return self.gripper['right'].get_state()

    @property
    def left_gripper_pos(self):
        if self.gripper['left'] is None:
            return None
        return self.gripper['left'].get_state()

    def step(self, action):
        
        for side in ['right', 'left']:
            if action[side] is None:
                continue
            
            arm_action = action[side][:6]
            gripper_action = action[side][6]

            self.arms[side].step(arm_action)
            
            if self.gripper[side] is not None:
                self.gripper[side].step(gripper_action)

        if self.head_enabled:
            self.head.step(action)
        
        if self.base_enabled:
            self.base.step(action['base'])

        if self.torso_enabled and (self.torso is not None):
            self.torso.step(action['torso'])

    def reset(self, reset_arms=True):
        for side in ['right', 'left']:
            if (self.reset_pose[side] is not None) and (self.arms[side].arm_enabled):
                self.gripper[side].step(self.reset_pose[side][-1])

                if reset_arms:
                    print(f'resetting {side}...{time.time()}')
                    self.arms[side].reset(self.reset_pose[side][:-1])
                    rospy.sleep(1)

        if self.head_enabled:
            self.head.reset_step(self.reset_pose)

        if ('torso' in self.reset_pose.keys()) and (self.torso is not None):
            self.torso.reset(self.reset_pose['torso'])
        
        rospy.sleep(0.5)

        input('Reset complete. Press ENTER to continue')