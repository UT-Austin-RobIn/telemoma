from telemoma.configs.base_config import teleop_config

teleop_config.arm_right_controller = 'mobile_phone'
teleop_config.interface_kwargs.mobile_phone = dict(address = '192.168.0.183', port = {'right': 6786, 'left': None})