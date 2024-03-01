import numpy as np
import time

class Body:

    def __init__(self, img, depth, detection_results, body_array=None) -> None:

        if body_array is not None:
            self.read_from_array(body_array)
            return
        
        self.pose_landmarks, self.visibility = self.parse_kpts(detection_results.pose_landmarks.landmark, get_visibility=True)
        self.real_hip_depth = self.get_real_hip_depth(depth)

        self.hand_landmarks = {'right': None, 'left': None}
        self.real_hand_depth = {'right': None, 'left': None}
        self.hand_points = {'right': None, 'left': None}
        for hand in ['right', 'left']:
            self.hand_landmarks[hand] = detection_results.right_hand_landmarks if hand == 'right' else detection_results.left_hand_landmarks
            if self.hand_landmarks[hand] is not None:
                self.hand_landmarks[hand] = self.parse_kpts(self.hand_landmarks[hand].landmark)
                self.real_hand_depth[hand], self.hand_points[hand] = self.get_real_hand_depth(depth, self.hand_landmarks[hand])

        self.image_w_scale, self.image_h_scale = self.get_unnormalization_scales(img)
        self.grip_count = 5

    def get_unnormalization_scales(self, image):
        left_hip = self.pose_landmarks[body_joint2idx['left hip']]
        right_hip = self.pose_landmarks[body_joint2idx['right hip']]
        hip = (right_hip + left_hip) / 2

        right_shoulder = self.pose_landmarks[body_joint2idx['right shoulder']]
        left_shoulder = self.pose_landmarks[body_joint2idx['left shoulder']]
        neck = (right_shoulder + left_shoulder) / 2

        # assuming hip to neck length is around 0.5m and always normal to ground
        self.hip2neck_dist = abs(hip[1] - neck[1])
        frame_height_scale = 0.5 / self.hip2neck_dist
        frame_width_scale = frame_height_scale*image.shape[1]/image.shape[0]

        return frame_width_scale, frame_height_scale
    
    def parse_kpts(self, pose_kpts, get_visibility=False):
        coordinates = []
        visibility = []
        for landmark in list(pose_kpts):
            coordinates.append([landmark.x, landmark.y, landmark.z])
            if get_visibility:
                visibility.append(landmark.visibility)
        if get_visibility:
            return np.array(coordinates), np.array(visibility)
        return np.array(coordinates)
    
    def get_avg_depth_of_points(self, depth, points):
        points_depth = []
        for p in points:
            d = depth[p[0], p[1]]
            points_depth.append(d)

        return np.mean(points_depth)/1000 # in meters

    def get_real_hip_depth(self, depth):
        left_hip = self.pose_landmarks[body_joint2idx['left hip']]
        right_hip = self.pose_landmarks[body_joint2idx['right hip']]

        hip_line = np.linspace(start=[left_hip[1], left_hip[0]], stop=[right_hip[1], right_hip[0]], num=20, endpoint=True)
        img_h, img_w = depth.shape[:2]

        hip_line[:, 0] = np.clip(img_h*hip_line[:, 0], 0, img_h-1)
        hip_line[:, 1] = np.clip(img_w*hip_line[:, 1], 0, img_w-1)
        hip_line = hip_line.astype(int)
        self.hip_line = hip_line

        return self.get_avg_depth_of_points(depth, hip_line)
    
    def get_real_hand_depth(self, depth, hand_kpts):
        palm_kpt_list = ['wrist', 'thumb_cmc', 'thumb_mcp','index_finger_mcp', 'middle_finger_mcp', 'ring_finger_mcp', 'pinky_mcp']
        _hand_points = []
        for joint in palm_kpt_list:
            _hand_points.append(hand_kpts[hand_joint2idx[joint]])
        
        _hand_points = np.array(_hand_points)
        hand_points = np.copy(_hand_points)

        img_h, img_w = depth.shape[:2]

        hand_points[:, 0] = np.clip(img_h*_hand_points[:, 1], 0, img_h-1)
        hand_points[:, 1] = np.clip(img_w*_hand_points[:, 0], 0, img_w-1)
        hand_points = hand_points.astype(int)
    
        return self.get_avg_depth_of_points(depth, hand_points), hand_points
    
    def get_hip_rotation(self):
        right_hip = self.pose_landmarks[body_joint2idx['right hip']]
        left_hip = self.pose_landmarks[body_joint2idx['left hip']]
        hip = (right_hip + left_hip) / 2

        right_shoulder = self.pose_landmarks[body_joint2idx['right shoulder']]
        left_shoulder = self.pose_landmarks[body_joint2idx['left shoulder']]
        neck = (right_shoulder + left_shoulder) / 2

        # calculate unit vectors of root joint
        root_u = left_hip - right_hip
        root_u = root_u/np.sqrt(np.sum(np.square(root_u)))
        root_v = neck - hip
        root_v = root_v - np.dot(root_u, root_v)*root_u
        root_v = root_v/np.sqrt(np.sum(np.square(root_v)))
        root_w = np.cross(root_u, root_v)
        
        #Make the rotation matrix
        hip_rotation = np.array([root_u, root_v, root_w]).T

        return hip_rotation

    def get_hand_transform_wrt_hip(self, side):
        hip_position = (self.pose_landmarks[body_joint2idx['right hip']] + self.pose_landmarks[body_joint2idx['left hip']])/2
        hip_rotation = np.eye(3)

        wrist = self.hand_landmarks[side][hand_joint2idx['wrist']]
        index = self.hand_landmarks[side][hand_joint2idx['index_finger_mcp']]
        pinky = self.hand_landmarks[side][hand_joint2idx['pinky_mcp']]

        root_u = hip_rotation @ (index - wrist)
        root_u = root_u/np.sqrt(np.sum(np.square(root_u)))
        root_v = hip_rotation @ (pinky - wrist)
        root_v = root_v - np.dot(root_v, root_u)*root_u
        root_v = root_v/np.sqrt(np.sum(np.square(root_v)))
        root_w = np.cross(root_u, root_v)
        root_w = root_w

        #Make the rotation matrix
        hand_rotation = np.array([root_u, root_v, root_w]).T

        hand_position = self.unnormalize_hands(wrist - hip_position, side)

        return hand_position, hand_rotation
    
    def get_grip(self, side):
        if self.hand_landmarks[side] is None:
            return None
        
        index_tip = self.hand_landmarks[side][hand_joint2idx['index_finger_tip']]
        thumb_tip = self.hand_landmarks[side][hand_joint2idx['thumb_tip']]

        if (np.linalg.norm(index_tip - thumb_tip) / self.hip2neck_dist) > 0.15:
            self.grip_count += 1
        else:
            self.grip_count = 0

        return self.grip_count >= 5 
    
    def get_hip_dist_from_foot(self):
        right_ankle = self.pose_landmarks[body_joint2idx['right ankle']]
        left_ankle = self.pose_landmarks[body_joint2idx['left ankle']]
        mid_ankle = (right_ankle + left_ankle)/2

        right_hip = self.pose_landmarks[body_joint2idx['right hip']]
        left_hip = self.pose_landmarks[body_joint2idx['left hip']]
        hip = (right_hip + left_hip) / 2

        dist = (mid_ankle[1] - hip[1])*self.image_h_scale
        return dist
    
    def unnormalize_hip(self, hip_pos):
        hip_pos_real = np.copy(hip_pos)
        hip_pos_real[0] = (hip_pos[0]-0.5)*self.image_w_scale
        hip_pos_real[2] = self.real_hip_depth

        return hip_pos_real
    
    def unnormalize_hands(self, hand_pos, side):
        hand_pos_real = np.copy(hand_pos)
        hand_pos_real[0] = (hand_pos[0])*self.image_w_scale
        hand_pos_real[1] = (hand_pos[1])*self.image_h_scale
        hand_pos_real[2] =  self.real_hand_depth[side] - self.real_hip_depth #(hand_pos[2])*self.image_w_scale

        return hand_pos_real
    
    def get_as_array(self):
        pose_landmarks_flattened = self.pose_landmarks.flatten()
        hand_landmarks_flattened = {
            'right': np.r_[1, self.hand_landmarks['right'].flatten(), self.real_hand_depth['right']] if self.hand_landmarks['right'] is not None else np.zeros(1),
            'left': np.r_[1, self.hand_landmarks['left'].flatten(), self.real_hand_depth['left']] if self.hand_landmarks['left'] is not None else np.zeros(1)
        }
        
        body_array = np.r_[pose_landmarks_flattened, self.real_hip_depth,
                            hand_landmarks_flattened['right'], hand_landmarks_flattened['left'],
                            self.image_w_scale, self.image_w_scale, self.hip2neck_dist,
                            time.time()]
        # print(body_array.shape)
        return body_array

    def read_from_array(self, body_array: np.ndarray):
        self.pose_landmarks = body_array[:99].reshape(33, 3)
        self.real_hip_depth = body_array[99]

        self.hand_landmarks = {'right': None, 'left': None}
        self.real_hand_depth = {'right': None, 'left': None}
        cur = 100

        # load right hand info
        if body_array[cur] == 0:
            cur += 1
        else:
            self.hand_landmarks['right'] = body_array[cur+1 : cur+64].reshape(21, 3)
            self.real_hand_depth['right'] = body_array[cur + 64]
            cur += 65
        
        # load left hand info
        if body_array[cur] == 0:
            cur += 1
        else:
            self.hand_landmarks['left'] = body_array[cur+1 : cur+64].reshape(21, 3)
            self.real_hand_depth['left'] = body_array[cur + 64]
            cur += 65

        self.image_w_scale, self.image_h_scale, self.hip2neck_dist = body_array[cur], body_array[cur + 1], body_array[cur + 2]

body_joint2idx = {
    'nose': 0,
    'left eye (inner)': 1,
    'left eye': 2,
    'left eye (outer)': 3,
    'right eye (inner)': 4,
    'right eye': 5,
    'right eye (outer)': 6,
    'left ear': 7,
    'right ear': 8,
    'mouth (left)': 9,
    'mouth (right)': 10,
    'left shoulder': 11,
    'right shoulder': 12,
    'left elbow': 13,
    'right elbow': 14,
    'left wrist': 15,
    'right wrist': 16,
    'left pinky': 17,
    'right pinky': 18,
    'left index': 19,
    'right index': 20,
    'left thumb': 21,
    'right thumb': 22,
    'left hip': 23,
    'right hip': 24,
    'left knee': 25,
    'right knee': 26,
    'left ankle': 27,
    'right ankle': 28,
    'left heel': 29,
    'right heel': 30,
    'left foot index': 31,
    'right foot index': 32,
}

hand_joint2idx = {
    'wrist': 0,
    'thumb_cmc': 1,
    'thumb_mcp': 2,
    'thumb_ip': 3,
    'thumb_tip': 4,
    'index_finger_mcp': 5,
    'index_finger_pip': 6,
    'index_finger_dip': 7,
    'index_finger_tip': 8,
    'middle_finger_mcp': 9,
    'middle_finger_pip': 10,
    'middle_finger_dip': 11,
    'middle_finger_tip': 12,
    'ring_finger_mcp': 13,
    'ring_finger_pip': 14,
    'ring_finger_dip': 15,
    'ring_finger_tip': 16,
    'pinky_mcp': 17,
    'pinky_pip': 18,
    'pinky_dip': 19,
    'pinky_tip': 20,
}

body_skeleton_links = frozenset([(0, 1), (1, 2), (2, 3), (3, 7), (0, 4), (4, 5),
                            (5, 6), (6, 8), (9, 10), (11, 12), (11, 13),
                            (13, 15), (15, 17), (15, 19), (15, 21), (17, 19),
                            (12, 14), (14, 16), (16, 18), (16, 20), (16, 22),
                            (18, 20), (11, 23), (12, 24), (23, 24), (23, 25),
                            (24, 26), (25, 27), (26, 28), (27, 29), (28, 30),
                            (29, 31), (30, 32), (27, 31), (28, 32)])