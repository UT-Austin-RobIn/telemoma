from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import cv2
import time

from telemoma.utils.general_utils import run_threaded_command
import copy
from telemoma.utils.transformations import euler_to_quat, quat_diff, quat_to_euler, rmat_to_quat, quat_to_rmat, rmat_to_euler, euler_to_rmat
from telemoma.human_interface.teleop_core import BaseTeleopInterface, TeleopAction, TeleopObservation
from telemoma.utils.vision_teleop_utils import body_joint2idx, Body
from collections import deque

def convert_hip_to_robot_frame(hip_pos, hip_rot):
    hip_pos = np.array([hip_pos[2], hip_pos[0], hip_pos[1]])

    euler_angles = rmat_to_euler(hip_rot, degrees=False)
    euler_angles = np.array([euler_angles[2], euler_angles[0], -euler_angles[1]]) # making euler angle negative is a hack
    hip_rot = euler_to_rmat(euler_angles, degrees=False)

    return hip_pos, hip_rot

def convert_hand_to_robot_frame(hand_pos, hand_rot):
    hand_pos = np.array([-hand_pos[2], hand_pos[0], -hand_pos[1]])

    euler_angles = rmat_to_euler(hand_rot, degrees=False)
    euler_angles = np.array([-euler_angles[2], euler_angles[0], -euler_angles[1]])
    hand_rot = euler_to_rmat(euler_angles, degrees=False)

    return hand_pos, hand_rot

class VisionTeleopPolicy(BaseTeleopInterface):
    
    def __init__(self, camera, set_ref=True, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.camera = camera
        self.set_ref = set_ref
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


        self.robot_reference = None
        self.reference_pose = {
            'hip_absolute_reference': np.array([0,0,0]),
            'hip_rotation_reference': np.array([[1, 0, 0],
                                                [0, -1, 0],
                                                [0, 0, -1]])
        }

        self.reference_delay = 15
        self.reference_queue = deque(maxlen=self.reference_delay)
        self.hand_pose_queue = []

    def start(self) -> None:
        run_threaded_command(self._update_internal_state)
        if self.set_ref:
            self.set_reference(timer=10)

    def stop(self) -> None:
        print("Stopping HumanKpt interface...")
        self.camera.stop()

    def get_processed_image(self):
        image = self.camera.get_img().astype(np.uint8)
        # image = np.flip(image, axis=1)
        return image

    def get_processed_depth(self):
        depth = self.camera.get_depth()
        # depth = np.flip(depth, axis=1)
        return depth

    def _update_internal_state(self, hz=50):

        while True:
            start_time = time.time()
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
            time.sleep(max(0, 1/hz - (time.time() - start_time))) # compute time is higher so no sleep
    
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

    def get_hip_position_and_rotation(self, relative=True, reference=None):
        if relative:
            assert self.reference_pose is not None, 'Please set the reference pose'

        detection = self.get_detection_results()
        if detection['body'] is None:
            return None, None

        right_hip = detection['body'].pose_landmarks[body_joint2idx['right hip']]
        left_hip = detection['body'].pose_landmarks[body_joint2idx['left hip']]
        hip = (right_hip + left_hip) / 2

        right_shoulder = detection['body'].pose_landmarks[body_joint2idx['right shoulder']]
        left_shoulder = detection['body'].pose_landmarks[body_joint2idx['left shoulder']]
        neck = (right_shoulder + left_shoulder)/ 2    

        # calculate unit vectors of root joint
        root_u = left_hip - right_hip
        root_u = root_u/np.sqrt(np.sum(np.square(root_u)))
        root_v = neck - hip
        root_v = root_v - np.dot(root_u, root_v)*root_u
        root_v = root_v/np.sqrt(np.sum(np.square(root_v)))
        root_w = np.cross(root_u, root_v)
        
        #Make the rotation matrix
        root_rotation = np.array([root_u, root_v, root_w]).T
        hip_absolute = detection['body'].unnormalize_hip(hip)

        if relative:
            if reference is None:
                reference = self.reference_pose
            if reference['hip_absolute_reference'] is None:
                return None, None
            
            reference_position = reference['hip_absolute_reference']
            reference_rotation = reference['hip_rotation_reference']

            # calculate the delta in the camera frame and then rotate it to fit the body frame
            hip_absolute = reference_rotation @ (hip_absolute - reference_position)
            root_rotation = root_rotation @ np.linalg.inv(reference_rotation) 

        return hip_absolute, root_rotation
    
    def get_hand_position_rotation_and_grip(self, side='right'):
        detection = self.get_detection_results()
        if detection['body'] is None or detection['body'].hand_landmarks[side] is None:
            return None, None, None
        
        reference_pos, reference_rot = self.reference_pose['body'].get_hand_transform_wrt_hip(side)
        wrist_pos, wrist_rot = detection['body'].get_hand_transform_wrt_hip(side)

        wrist_pos = wrist_pos - reference_pos
        
        reference_quat = rmat_to_quat(reference_rot)
        wrist_quat = rmat_to_quat(wrist_rot)
        delta_quat = quat_diff(wrist_quat, reference_quat)
        wrist_rot = quat_to_rmat(delta_quat)

        return wrist_pos, wrist_rot, detection['body'].get_grip(side)

    def get_reference(self):
        reference_pose = {}
        reference_pose['hip_absolute_reference'], reference_pose['hip_rotation_reference'] = self.get_hip_position_and_rotation(relative=False)
        
        # hip position is delta but rotation is global
        reference_pose['hip_rotation_reference'] = np.array([[1, 0, 0],
                                                            [0, -1, 0],
                                                            [0, 0, -1]])

        reference_pose['body'] = self.get_detection_results()['body']

        return reference_pose

    def set_reference(self, timer=10):
        start_time = time.time()
        while (time.time() - start_time) < timer:
            print(timer - (time.time() - start_time))

            image = self.get_overlayed_img(base='image')
            if image is not None:
                cv2.imshow('a', image)
                cv2.waitKey(1)
        
        self.reference_pose = self.get_reference()

        self.reference_queue.append(self.reference_pose) # human reference

        print('Setting reference...')
        time.sleep(2)

        cv2.destroyAllWindows()

    def get_nav_action(self, obs: TeleopObservation) -> np.ndarray:
        self.reference_queue.append(self.get_reference())

        human_reference = self.reference_queue[0]
        hip_pos, hip_rot = self.get_hip_position_and_rotation(relative=True, reference=human_reference)
        if hip_pos is None:
            return np.zeros(3)
        
        hip_pos, hip_rot = convert_hip_to_robot_frame(hip_pos, hip_rot)

        pos_action = hip_pos*2
        base_delta_euler = np.zeros(3)
        base_delta_euler[2] = obs.base[2]
        base_delta_quat = euler_to_quat(base_delta_euler)
        target_delta_quat = rmat_to_quat(hip_rot)
        quat_action = quat_diff(target_delta_quat, base_delta_quat)
        euler_action = quat_to_euler(quat_action)

        action = np.array([pos_action[0], pos_action[1], euler_action[2]])
        action = action.clip(-1, 1)

        return action
    
    def get_torso_action(self, obs: TeleopObservation) -> np.ndarray:
        detection = self.get_detection_results()
        if (detection['body'] is None) or ('torso' not in obs):
            return None

        body_torso_reference = self.reference_pose['body'].get_hip_dist_from_foot()
        body_torso_current = detection['body'].get_hip_dist_from_foot()
        robot_torso_reference = self.robot_reference.torso
        robot_torso_current = obs.torso

        target_torso_offset = body_torso_current - body_torso_reference
        robot_torso_offset = robot_torso_current - robot_torso_reference

        return np.clip(target_torso_offset - robot_torso_offset, -0.02, 0.02) 
    
    def get_hand_action(self, obs: TeleopObservation, side='right') -> np.ndarray:
        robot_eef = obs[side]
        
        hand_delta_pos, hand_delta_rot, grip = self.get_hand_position_rotation_and_grip(side=side)
        if robot_eef is None or hand_delta_pos is None:
            return None
        
        hand_delta_pos, hand_delta_rot = convert_hand_to_robot_frame(hand_delta_pos, hand_delta_rot)

        robot_pos, robot_quat = robot_eef[:3], robot_eef[3:7]
        robot_reference_pos, robot_reference_quat = self.robot_reference[side][:3], self.robot_reference[side][3:7]

        robot_pos_offset = robot_pos - robot_reference_pos
        target_pos_offset = hand_delta_pos
        pos_action = target_pos_offset - robot_pos_offset

        # Calculate Euler Action #
        robot_quat_offset = quat_diff(robot_quat, robot_reference_quat)
        target_quat_offset = rmat_to_quat(hand_delta_rot)
        quat_action = quat_diff(target_quat_offset, robot_quat_offset)
        euler_action = quat_to_euler(quat_action)

        # Prepare Return Values #
        action = np.r_[pos_action, euler_action, [grip]]
        action = action.clip(-1, 1)
        
        return action

    def get_action(self, obs: TeleopObservation) -> TeleopAction:
        action = self.get_default_action()
        if self.robot_reference is None:
            self.robot_reference = obs
        action.base = self.get_nav_action(obs)
        action.torso = self.get_torso_action(obs)
        action.left = self.get_hand_action(obs, side='left')
        action.right = self.get_hand_action(obs, side='right')

        action.extra['overlayed_image'] = self.get_overlayed_img(base='image')
        return action

if __name__=='__main__':
    import time
    from telemoma.utils.camera_utils import RealSenseCamera

    cam = RealSenseCamera()
    detector = VisionTeleopPolicy(camera=cam, set_ref=True)
    detector.start()

    for i in range(500):
        img = detector.get_overlayed_img(base='image')
        depth = detector.get_overlayed_img(base='depth')
        if img is not None:
            cv2.imshow('img', img)
            cv2.imshow('depth', depth)
            cv2.waitKey(1)
            