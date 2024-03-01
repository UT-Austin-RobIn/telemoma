from telemoma.human_interface import INTERFACE_MAP
from telemoma.human_interface.teleop_core import BaseTeleopInterface, TeleopAction, TeleopObservation
from telemoma.utils.general_utils import AttrDict

class TeleopPolicy:
    def __init__(self, config: AttrDict) -> None:
        self.config = config
        
        self.controllers = {
            'left': config.arm_left_controller,
            'right': config.arm_right_controller,
            'base': config.base_controller,
            'torso': config.torso_controller
        }
        for controller in self.controllers.values():
            if controller is not None:
                assert controller in INTERFACE_MAP, 'Other controllers not implemented.'

        self.interfaces: dict[str, BaseTeleopInterface] = {}
        for part in self.controllers:
            if (self.controllers[part] is not None) and (self.controllers[part] not in self.interfaces):
                self.interfaces[self.controllers[part]] = INTERFACE_MAP[self.controllers[part]](**config.interface_kwargs[self.controllers[part]])

        if ('oculus' not in self.interfaces) and config.get('use_oculus', False):
            self.interfaces['oculus'] = INTERFACE_MAP['oculus']()

    def start(self) -> None:
        for interface in self.interfaces.values():
            if interface is not None:
                interface.start()

    def stop(self) -> None:
        for interface in self.interfaces.values():
            if interface is not None:
                interface.stop()

    def get_default_action(self) -> TeleopAction:
        return TeleopAction()

    def get_action(self, obs: TeleopObservation) -> TeleopAction:
        interface_action = {}
        action = self.get_default_action()
    
        for interface in self.interfaces:
            interface_action[interface] = self.interfaces[interface].get_action(obs)

            for extra_key in interface_action[interface].extra:
                action.extra[extra_key] = interface_action[interface].extra[extra_key]

        for part in self.controllers:
            if (self.controllers[part] is not None) and interface_action[self.controllers[part]][part] is not None:
                action[part] = interface_action[self.controllers[part]][part]
        
        return action
    
    def stop(self):
        for interface in self.interfaces.values():
            if interface is not None:
                interface.stop()