from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import cv2
import time

from telemoma.utils.general_utils import run_threaded_command
import copy
from telemoma.human_interface.teleop_core import BaseTeleopInterface
from telemoma.utils.vision_teleop_utils import Body

import socket
from telemoma.utils.remote_utils import SocketConnection

HOST="ADD HOST IP" # add host IP address
PORT=6787

class RemoteVisionTeleopClient(BaseTeleopInterface):
    
    def __init__(self, camera, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.camera = camera
        self.latest_pose = {
                'image': self.get_processed_image(),
                'depth': self.get_processed_depth(),
                'body':  None
            }
        self.latest_pose_stamp = time.time()

        self.detector = solutions.holistic.Holistic(
                                                    static_image_mode=False,
                                                    model_complexity=1,
                                                    smooth_landmarks=True,
                                                    enable_segmentation=False,
                                                    smooth_segmentation=False,
                                                    refine_face_landmarks=False,
                                                    min_detection_confidence=0.5,
                                                    min_tracking_confidence=0.5
                                                )

    def start(self) -> None:
        # setup socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((HOST, PORT))
        self.client_socket = SocketConnection(client_socket)
        
        run_threaded_command(self._update_internal_state)

    def stop(self) -> None:
        print("Stopping HumanKpt interface...")
        self.camera.stop()

    def get_processed_image(self):
        image = self.camera.get_img().astype(np.uint8)
        image = np.flip(image, axis=1)
        return image

    def get_processed_depth(self):
        depth = self.camera.get_depth()
        depth = np.flip(depth, axis=1)
        return depth

    def _update_internal_state(self, hz=50):

        while True:
            image = self.get_processed_image()
            depth = self.get_processed_depth()
            
            _image = np.copy(image)
            _image.flags.writeable = False
            _image = cv2.cvtColor(_image, cv2.COLOR_BGR2RGB)
            pose_results = self.detector.process(_image)
            if pose_results.pose_landmarks is not None:
                self.latest_pose = {
                    'image': image,
                    'depth':  depth,
                    'body': Body(image, depth, pose_results)
                }

            else:
                self.latest_pose = {
                    'image': image,
                    'depth':  depth,
                    'body': None
                }
            self.latest_pose_stamp = time.time()

            self.client_socket.send({'data': self.latest_pose['body'], 'timestamp': self.latest_pose_stamp})
    
    def get_detection_results(self):
        return copy.deepcopy(self.latest_pose)

    def get_overlayed_img(self, base='image'):
        assert base in ['image', 'depth']
        pose = self.get_detection_results()

        if pose['body'] is None:
            current_image = pose[base].astype(np.float32)
            if base == 'image':
                current_image /= 255
            return current_image
        
        pose_landmarks = pose['body'].pose_landmarks

        if base == 'depth':
            im = np.copy(pose[base][:, :, :1]).repeat(3, axis=-1)
            im = 255*np.clip(im, 0, 4000)/4000
        else:
            im = np.copy(pose[base][:, :, :3])

        if pose['body'].hip_line is not None:
            for l in pose['body'].hip_line:
                im = cv2.circle(im, (l[1], l[0]), radius=10, color=(0, 0, 255), thickness=-1)

        annotated_image = np.ascontiguousarray(im, dtype=np.uint8)

        # Draw the pose landmarks.
        pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        pose_landmarks_proto.landmark.extend([
                        landmark_pb2.NormalizedLandmark(x=landmark[0], y=landmark[1], z=landmark[2]) for landmark in pose_landmarks])
        solutions.drawing_utils.draw_landmarks(
            annotated_image,
            pose_landmarks_proto,
            solutions.pose.POSE_CONNECTIONS,
            solutions.drawing_styles.get_default_pose_landmarks_style())

        for hand in ['right', 'left']:
            hand_landmarks = pose['body'].hand_landmarks[hand]
            if hand_landmarks is None:
                continue

            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            hand_landmarks_proto.landmark.extend([
                            landmark_pb2.NormalizedLandmark(x=landmark[0], y=landmark[1], z=landmark[2]) for landmark in hand_landmarks])
            solutions.drawing_utils.draw_landmarks(
                annotated_image,
                hand_landmarks_proto,
                solutions.hands.HAND_CONNECTIONS,
                solutions.drawing_styles.get_default_hand_landmarks_style(),
                solutions.drawing_styles.get_default_hand_connections_style())

        annotated_image = annotated_image.astype(np.float32)/255
        return annotated_image

if __name__=='__main__':
    import time
    from telemoma.utils.camera_utils import RealSenseCamera

    cam = RealSenseCamera()
    detector = RemoteVisionTeleopClient(camera=cam, set_ref=True)
    detector.start()

    while 1:
        img = detector.get_overlayed_img(base='image')
        depth = detector.get_overlayed_img(base='depth')
        if img is not None:
            cv2.imshow('img', img)
            cv2.imshow('depth', depth)
            cv2.waitKey(1)
            