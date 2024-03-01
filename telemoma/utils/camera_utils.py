import cv2
import numpy as np
import pyrealsense2 as rs
from typing import Dict

class RealSenseCamera:
    def __init__(self, *args, **kwargs) -> None:
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        p = self.pipeline.start(config)
        depth_intr = p.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        rgb_intr = p.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self._img_shape  = [rgb_intr.width, rgb_intr.height, 3]
        self._depth_shape = [depth_intr.width, depth_intr.height, 1]
        self.align = rs.align(rs.stream.color)

    def stop(self):
        self.pipeline.stop()

    def get_img(self) -> np.ndarray:
        return self.get_camera_obs()['image']
    
    def get_depth(self) -> np.ndarray:
        return self.get_camera_obs()['depth']

    def get_camera_obs(self) -> Dict[str, np.ndarray]:
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        depth_image = np.expand_dims(np.asarray(depth_frame.get_data()), -1).astype(int) if depth_frame else None
        color_frame = aligned_frames.get_color_frame()
        color_image = np.asarray(color_frame.get_data()).astype(int) if color_frame else None
        return {
            'image': color_image,
            'depth': depth_image,
        }
    
    @property
    def img_shape(self):
        return self._img_shape

    @property
    def depth_shape(self):
        return self._depth_shape
    
                
try:
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, CameraInfo
    from telemoma.utils.ros_utils import Listener

    def img_processing(data):
        br = CvBridge()
        img = cv2.cvtColor(br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)
        return np.array(img).astype(int)

    def depth_processing(data):
        br = CvBridge()
        img = br.imgmsg_to_cv2(data)
        return np.expand_dims(np.array(img), -1).astype(int)

    def flip_img(img):
        return np.flip(np.array(img).astype(int), axis=[0, 1])

    def uncompress_image(data):
        np_arr = np.fromstring(data.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        return np.array(img).astype(int)

    def uncompress_depth(data):
        np_arr = np.fromstring(data.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        return np.expand_dims(np.array(img), -1).astype(int)

    # Handle when no depth is available
    class Camera:

        def __init__(self, img_topic, depth_topic, input_message_type=Image, camera_info_topic=None, img_post_proc_func=None, depth_post_proc_func=None, *args, **kwargs) -> None:
            self.img_topic = img_topic
            self.depth_topic = depth_topic

            self.img_listener = Listener(
                                input_topic_name=self.img_topic,
                                input_message_type=input_message_type,
                                post_process_func=img_processing if img_post_proc_func is None else img_post_proc_func
                            )
            self.depth_listener = Listener(
                                    input_topic_name=self.depth_topic,
                                    input_message_type=input_message_type,
                                    post_process_func=depth_processing if depth_post_proc_func is None else depth_post_proc_func
                                )
            
            self._img_shape  = self.img_listener.get_most_recent_msg().shape
            self._depth_shape = self.depth_listener.get_most_recent_msg().shape

            self.camera_info = None
            if camera_info_topic is not None:
                info_listener = Listener(
                                    input_topic_name=camera_info_topic,
                                    input_message_type=CameraInfo
                                )
                
                self.camera_info = info_listener.get_most_recent_msg()

        def get_img(self):
            return self.img_listener.get_most_recent_msg()
        
        def get_depth(self):
            return self.depth_listener.get_most_recent_msg()

        def get_camera_obs(self):
            return {
                'image': self.get_img(),
                'depth': self.get_depth(),
            }
        
        @property
        def img_shape(self):
            return self._img_shape

        @property
        def depth_shape(self):
            return self._depth_shape
        
        def stop(self):
            pass    

except:
    pass