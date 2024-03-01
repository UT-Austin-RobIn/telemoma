from telemoma.configs.base_config import teleop_config
from telemoma.utils.camera_utils import Camera, RealSenseCamera

# cam = Camera(img_topic="/body_tracking_camera/color/image_raw", depth_topic="/body_tracking_camera/aligned_depth_to_color/image_raw")
cam = RealSenseCamera()

teleop_config.arm_left_controller = 'oculus'
teleop_config.arm_right_controller = 'oculus'
teleop_config.base_controller = 'vision'
teleop_config.torso_controller = 'vision'
teleop_config.interface_kwargs.vision = dict(camera=cam, set_ref=True)