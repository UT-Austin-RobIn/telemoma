import numpy as np
from pynput import keyboard
from telemoma.human_interface.teleop_core import BaseTeleopInterface, TeleopAction, TeleopObservation

class KeyboardInterface(BaseTeleopInterface):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.actions = self.get_default_action()
        # we want to scale down the movement speed to have better control for arms
        self.arm_speed_scaledown = kwargs.get("arm_speed_scaledown", 1.0)
        # store intermediate arm rotation euler angles
        self.left_delta_rpy = np.zeros(3)
        self.right_delta_rpy = np.zeros(3)
        
    def start(self) -> None:
        # start the keyboard subscriber
        self.data_thread = keyboard.Listener(on_press=self._update_internal_data, on_release=self._on_release)
        self.data_thread.start()  

    def stop(self) -> None:
        self.data_thread.join()

    def _on_release(self, event):
        left_gripper, right_gripper = self.actions.left[6], self.actions.right[6]
        self.actions = self.get_default_action()
        self.actions.left[6], self.actions.right[6] = left_gripper, right_gripper

    def _update_internal_data(self, event) -> None:
        try:
            key = event.char 
        except:
            key = event.name 
        # update Left hand
        if key == 'w':
            self.actions.left[0] = self.arm_speed_scaledown
        elif key == 's':
            self.actions.left[0] = -self.arm_speed_scaledown
        elif key == 'a':
            self.actions.left[1] = self.arm_speed_scaledown
        elif key == 'd':
            self.actions.left[1] = -self.arm_speed_scaledown
        elif key == 'e':
            self.actions.left[2] = self.arm_speed_scaledown
        elif key == 'q':
            self.actions.left[2] = -self.arm_speed_scaledown
        elif key == 'c':
            self.actions.left[3] = self.arm_speed_scaledown
        elif key == 'x':
            self.actions.left[3] = -self.arm_speed_scaledown
        elif key == 't':
            self.actions.left[4] = self.arm_speed_scaledown
        elif key == 'b':
            self.actions.left[4] = -self.arm_speed_scaledown
        elif key == 'z':
            self.actions.left[5] = self.arm_speed_scaledown
        elif key == 'v':
            self.actions.left[5] = -self.arm_speed_scaledown
        elif key == '4':
            self.actions.left[6] = (self.actions.left[6] + 1) % 2    
        # Update right hand
        elif key == 'i':
            self.actions.right[0] = self.arm_speed_scaledown
        elif key == 'k':
            self.actions.right[0] = -self.arm_speed_scaledown
        elif key == 'j':
            self.actions.right[1] = self.arm_speed_scaledown
        elif key == 'l':
            self.actions.right[1] = -self.arm_speed_scaledown
        elif key == 'o':
            self.actions.right[2] = self.arm_speed_scaledown
        elif key == 'u':
            self.actions.right[2] = -self.arm_speed_scaledown
        elif key == ',':
            self.actions.right[3] = self.arm_speed_scaledown
        elif key == 'm':
            self.actions.right[3] = -self.arm_speed_scaledown
        elif key == 'p':
            self.actions.right[4] = self.arm_speed_scaledown
        elif key == ';':
            self.actions.right[4] = -self.arm_speed_scaledown
        elif key == 'n':
            self.actions.right[5] = self.arm_speed_scaledown
        elif key == '.':
            self.actions.right[5] = -self.arm_speed_scaledown
        elif key == '7':
            self.actions.right[6] = (self.actions.right[6] + 1) % 2 
        # update base positions 
        elif key == 'up':
            self.actions.base[0] = 1.0
        elif key == 'down':
            self.actions.base[0] = -1.0
        elif key == '[':
            self.actions.base[1] = 1.0
        elif key == ']':
            self.actions.base[1] = -1.0
        elif key == 'left':
            self.actions.base[2] = 1.0
        elif key == 'right':
            self.actions.base[2] = -1.0
        elif key == '-':
            self.actions.torso = 1.0
        elif key == '+':
            self.actions.torso = -1.0
        return True

    def get_action(self, obs: TeleopObservation) -> TeleopAction:
        return self.actions
