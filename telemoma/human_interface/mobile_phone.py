import socket
import numpy as np

import time
import copy
from telemoma.human_interface.teleop_core import BaseTeleopInterface, TeleopAction, TeleopObservation
from telemoma.utils.general_utils import run_threaded_command
from telemoma.utils.transformations import quat_diff, quat_to_euler

BUFFER_SIZE = 1024

class iPhoneReader:

    def __init__(self, address, port) -> None:
        ADDRESS = address
        PORT = port

        self.connection_timeout = 10

        self.socket = {
            'right': None,
            'left': None,
        }
        self.connection = {
            'right': None,
            'left': None,
        }
        self.address = {
            'right': None,
            'left': None,
        }

        for side in ['right', 'left']:
            if PORT[side] is None:
                continue

            self.socket[side] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket[side].settimeout(0.1)
            self.socket[side].bind((ADDRESS, PORT[side]))
            self.socket[side].listen(1)

            start_time = time.time()
            timeout = True
            while self.connection_timeout > time.time() - start_time:
                try:
                    self.connection[side], self.address[side] = self.socket[side].accept()
                    timeout = False
                    break
                except socket.timeout:
                    print(f'Connecting to {side} iPhone....{time.time()-start_time}')

            if timeout:
                self.connection[side] = None
        
        for side in ['right', 'left']:
            if self.connection[side] is None:
                print(f'==> Could not connect to {side} phone with post number {PORT[side]}', 'red')
    
    def get_pose(self):
        
        pose_dict = {
            'right': None,
            'left': None,
        }
        for side in ['right', 'left']:
            if self.connection[side] is None:
                continue
            try:
                data = self.connection[side].recv(BUFFER_SIZE)
                if len(data) == 0:
                    continue
            except ConnectionResetError as e:
                continue
            except socket.timeout:
                continue

            data_strings = data.decode('utf8')
            if '<' in data_strings[1:]:
                idx = data_strings[1:].find('<') + 1
                data_strings = data_strings[:idx]
            data_strings = data_strings.split(' ')

            command = data_strings[0]
            try:
                params = np.array([float(s) for s in data_strings[1:]])  
                pose_dict[side] = dict(
                    command=command,
                    pos=params[:3] if command in ['<Start>', '<Track>'] else np.zeros(3),
                    quat=np.r_[params[4:7], params[3]] if command in ['<Start>', '<Track>'] else np.r_[np.zeros(3), 1]
                )
            except ValueError as e:
                print(f'Failed to parse command: {data_strings}')
        
        return pose_dict

class MobilePhonePolicy(BaseTeleopInterface):
    def __init__(self, address: str, port: dict[str, int] = None) -> None:

        self.position = None
        self.rotation = None
        self.start_input_position = None
        self.start_input_rotation = None
        
        self.position_gain = 1
        self.rotation_gain = 1
        self.coordinate_change = np.array([[0, 0, -1],
                                            [-1, 0, 0],
                                            [0, 1, 0]])
        
        self.target_gripper = {'right': 1, 'left': 1}
        
        self.phone_reader = iPhoneReader(address, port)
        self.reset_state()
    
    def start(self):
        run_threaded_command(self._update_internal_state)

    def stop(self):
        self.running = False

    def reset_state(self) -> None:
        self._state = {
            'right': {
                "pos": None,
                "quat": None,
                "movement_enabled": False,
                "controller_on": True,
                "prev_gripper": False,
                "gripper_toggle": False,
                "command": None,
            },

            'left': {
                "pos": None,
                "quat": None,
                "movement_enabled": False,
                "controller_on": True,
                "prev_gripper": False,
                "gripper_toggle": False,
                "command": None,
            },
            'buttons': {}
        }

        self.reset_origin = {'right': False, 'left': False}
        self.robot_origin = {'right': None, 'left': None}
        self.phone_origin = {'right': None, 'left': None}

    def _update_internal_state(self):
        while True:
            pose = self.phone_reader.get_pose()

            for side in pose:
                if pose[side] is None:
                    self._state[side]["movement_enabled"] = False
                    self._state[side]["controller_on"] = False
                    continue
                else:
                    self._state[side]["controller_on"] = True
                command, pos, quat = pose[side]['command'], pose[side]['pos'], pose[side]['quat']

                self._state[side]["movement_enabled"] = command in ['<Start>', '<Track>'] 
                self.reset_origin[side] = self.reset_origin[side] or (command == '<Start>')

                new_gripper = command == '<Gripper>'
                self._state[side]["gripper_toggle"] = ((not self._state[side]["prev_gripper"]) and new_gripper) or self._state[side]["gripper_toggle"]
                self._state[side]["prev_gripper"] = new_gripper
                
                self._state[side]['command'] = command
                self._state[side]['pos'] = pos @ self.coordinate_change.T * self.position_gain
                self._state[side]['quat'] = np.r_[quat[:3] @ self.coordinate_change.T, quat[3]]

    def _calculate_action(self, robot_obs: dict[str, np.ndarray], side: str) -> np.ndarray:
        # Read Sensor #
        phone_state = copy.deepcopy(self._state[side])
        self._state[side]["gripper_toggle"] = False

        delta_action = np.zeros(6)
        if phone_state['movement_enabled']:        
            # Read Observation
            robot_pos = np.array(robot_obs["cartesian_position"][:3])
            robot_quat = robot_obs["cartesian_position"][3:]

            # Reset Origin On Release #
            if self.reset_origin[side]:
                self.robot_origin[side] = {"pos": robot_pos, "quat": robot_quat}
                self.phone_origin[side] = {"pos": phone_state["pos"], "quat": phone_state["quat"]}
                self.reset_origin[side] = False

            if self.robot_origin[side] is None or self.phone_origin[side] is None:
                return None

            # Calculate Positional Action #
            robot_pos_offset = robot_pos - self.robot_origin[side]["pos"]
            target_pos_offset = (phone_state["pos"] - self.phone_origin[side]["pos"])
            pos_action = target_pos_offset - robot_pos_offset

            # Calculate Euler Action #
            robot_quat_offset = quat_diff(robot_quat, self.robot_origin[side]["quat"])
            target_quat_offset = quat_diff(phone_state["quat"], self.phone_origin[side]["quat"])
            quat_action = quat_diff(target_quat_offset, robot_quat_offset)
            euler_action = quat_to_euler(quat_action)
            
            delta_action = np.concatenate((pos_action, euler_action))

        if phone_state["gripper_toggle"]:
            self.target_gripper[side] = 1 - int(robot_obs["gripper_position"] > 0.5)

        # Prepare Return Values #
        action = np.concatenate([delta_action, [self.target_gripper[side]]])
        action = action.clip(-1, 1)
        
        return action
    
    def get_action(self, obs: TeleopObservation) -> TeleopAction:
        action = self.get_default_action()

        for arm in ['right', 'left']:
            eef_data = obs[arm]
            if eef_data is None:
                continue
            robot_obs = {'cartesian_position': eef_data[:-1], 'gripper_position': eef_data[-1]}
            new_action = self._calculate_action(robot_obs, arm)
            if new_action is not None:
                action[arm] = new_action

        return action
    

if __name__=='__main__':
    reader = iPhoneReader()
    while 100:
        print(reader.get_pose())
        time.sleep(0.1)