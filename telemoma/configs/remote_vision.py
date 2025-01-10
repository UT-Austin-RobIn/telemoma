from telemoma.configs.base_config import teleop_config

# from telemoma.utils.camera_utils import Camera
# cam = Camera(img_topic="/camera/color/image_raw", depth_topic="/camera/aligned_depth_to_color/image_raw")

# teleop_config.arm_left_controller = 'remote_vision'
teleop_config.arm_right_controller = 'remote_vision'
# teleop_config.base_controller = 'remote_vision'
# teleop_config.torso_controller = 'remote_vision'
teleop_config.use_oculus = False
teleop_config.interface_kwargs.remote_vision = dict(set_ref=True)