import tf
import rospy
from threading import Lock

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, Twist, Vector3

def create_pose_command(trans, quat):
    header = Header(stamp=rospy.Time.now(), frame_id='base_footprint')
    pose = Pose(position=Point(trans[0], trans[1], trans[2]), orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
    message = PoseStamped(header=header, pose=pose)

    return message

def create_twist_command(linear, angular):
    twist = Twist(linear=Vector3(*linear), angular=Vector3(*angular))
    return twist

class Publisher:

    def __init__(self, pub_name, pub_message_type, queue_size=5):
        self.publisher = rospy.Publisher(pub_name, pub_message_type, queue_size=queue_size)
    
    def write(self, message):
        self.publisher.publish(message)
        
class Listener:
    
    def __init__(self, input_topic_name, input_message_type, post_process_func=None):
        self.inputlock = Lock()
        self.input_topic_name = input_topic_name
        self.input_message_type = input_message_type
        self.post_process_func = post_process_func

        self.most_recent_message = None
        self.init_listener()

    def callback(self, data):
        with self.inputlock:
            self.most_recent_message = data
    
    def init_listener(self):
        rospy.Subscriber(self.input_topic_name, self.input_message_type, self.callback)
    
    def get_most_recent_msg(self):
        while (self.most_recent_message is None) and (not rospy.is_shutdown()):
            print(f'Waiting for topic {self.input_topic_name} to publish.')
            rospy.sleep(0.02)
        
        data = self.most_recent_message if self.post_process_func is None else self.post_process_func(self.most_recent_message)
        return data
    
class TFTransformListener:
    
    def __init__(self, base_link):
        self.listener = tf.TransformListener()    
        self.base_link = base_link
        rospy.sleep(1)

    def _get_transform(self, rel_link, base_link):
        try:
            (trans, rot) = self.listener.lookupTransform(base_link, rel_link, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('TF Connecting...')
            trans, rot = None, None

        return trans, rot

    def get_transform(self, target_link, base_link=None):
        if base_link is None:
            base_link = self.base_link
        trans, rot = self._get_transform(target_link, base_link)
        return trans, rot