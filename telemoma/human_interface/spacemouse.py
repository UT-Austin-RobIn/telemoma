import numpy as np
from importlib import import_module
from telemoma.human_interface.teleop_core import BaseTeleopInterface, TeleopAction, TeleopObservation
from telemoma.utils.general_utils import run_threaded_command

class SpaceMouseInterface(BaseTeleopInterface):
    def __init__(self, *args, **kwargs) -> None:
        try:
            self.pyspacemouse = import_module('pyspacemouse')
        except ModuleNotFoundError:
            raise ModuleNotFoundError("[SpaceMousePolicy] Please install pyspacemouse to use SpaceMouseSystem")
        super().__init__(*args, **kwargs)
        self.actions = TeleopAction()
        self.data_thread = None
        self.controllable_robot_parts = ["right", "left", "base"]
        self.cur_control_idx = 0
        # we want to scale down the movement speed to have better control for arms
        self.arm_speed_scaledown = kwargs.get("arm_speed_scaledown", 1.0)

    def start(self) -> None:
        """
        Start the space mouse connection and the data thread
        """
        assert self.pyspacemouse.open(button_callback=self._button_callback), \
            "[SpaceMousePolicy] Cannot connect to space mouse!"
        run_threaded_command(self._update_internal_data)

    def stop(self) -> None:
        """
        Stop the space mouse connection and the data thread
        """
        print("Stopping SpaceMouse connection...")
        self.pyspacemouse.close()

    def _update_internal_data(self) -> None:
        """
        Thread that stores the the spacemouse input to self.raw_data
        """
        while True:
            self.raw_data = self.pyspacemouse.read()

    def _button_callback(self, _, buttons) -> None:
        """
        Callback function for space mouse button press
        """
        if buttons[0]:
            # left button pressed, switch controlling part
            self.cur_control_idx = (self.cur_control_idx + 1) % len(self.controllable_robot_parts)
            print(f"Now controlling robot part {self.controllable_robot_parts[self.cur_control_idx]}")
        elif buttons[1]:
            # right button pressed, switch gripper open/close state if we are controlling one
            if self.controllable_robot_parts[self.cur_control_idx] in {"left", "right"}:
                arm = self.controllable_robot_parts[self.cur_control_idx]
                self.actions[arm][6] = (self.actions[arm][6] + 1) % 2

    def get_action(self, obs: TeleopObservation) -> TeleopAction:
        """
        Get the action of a body part
        """
        self.actions.base = np.zeros(3)
        if self.raw_data:
            controlling_robot_part = self.controllable_robot_parts[self.cur_control_idx]
            # update controlling part pose
            if controlling_robot_part == "base":
                self.actions.base[0] = self.raw_data.y
                self.actions.base[1] = -self.raw_data.x
                self.actions.base[2] = -self.raw_data.yaw
                self.actions.torso = self.raw_data.z
            else:
                self.actions[controlling_robot_part][:3] = np.array(       
                    [self.raw_data.y, -self.raw_data.x, self.raw_data.z]
                ) * self.arm_speed_scaledown
                self.actions[controlling_robot_part][3:6] = np.array(
                    [self.raw_data.roll, self.raw_data.pitch, -self.raw_data.yaw]
                ) * self.arm_speed_scaledown
        return self.actions