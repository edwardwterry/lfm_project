#! usr/bin/env python
import rospy
from lfm.msg import Action, AprilTagDetectionArray, AprilTagDetection
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_inverse, quaternion_multiply
import numpy as np
import math

num_rows = 3
num_cols = 3
grid_size = np.array([num_rows, num_cols])
ready_for_action = False

initial_scene = rospy.get_param('blocks')
id_list = [initial_scene[tag]['id'] for tag in range(len(initial_scene))]

initial_link_prob = 0.5
displacement_threshold = 3 # mm
pos = {}
rot = {}
rot_rel = {}
displacement = {}
tags_in_scene = []

direction_map = {'N': 0.0, 'W': 90.0, 'S': 180.0, 'E': 270.0}

link_prob = np.ones([(2 * num_rows - 1), (2 * num_cols - 1)])
for i in range(link_prob.shape[0]):
    for j in range(link_prob.shape[1]):
        if (i % 2 == 1 and j % 2 == 1):
            link_prob[i,j] = 0.0
        elif ((i + j) % 2 == 1):
            link_prob[i,j] = initial_link_prob

def capture_scene():
    return pos, rot

def get_absolute_displacement():
    for tag in tags_in_scene:
        displacement[tag] = math.sqrt((after_snapshot[0].x - before_snapshot[0].x) ** 2 + (after_snapshot[0].y - before_snapshot[0].y) ** 2)
    return displacement

def get_relative_displacement(tag1, tag2):
    return displacement[tag2] - displacement[tag1]

def get_moved_blocks():
    return displacement >= displacement_threshold

def transform_cam_to_arm(cam):
    R = rospy.get_param('R')
    t = rospy.get_param('t')
    print R, t
    return R * cam + t; 

def detectionsClbk(msg):
    detections_raw = msg.detections
    indices_raw = [detections_raw[i].id[0] for i in range(len(detections_raw))]
    tags_in_scene = indices_raw
    for i in range(len(indices_raw)):
        pos[indices_raw[i]] = transform_cam_to_arm(detections_raw[i].pose.pose.pose.position)*1000.0 # assumes that R and t are populated, converts m to mm
        rot[indices_raw[i]] = detections_raw[i].pose.pose.pose.orientation

def readyForActionClbk(msg):
    global ready_for_action
    ready_for_action = True
    
def get_base_orientation():
    anchor_tag_id = rospy.get_param('blocks')[0]['id']
    return rot[anchor_tag_id]

def get_base_position():
    anchor_tag_id = rospy.get_param('blocks')[0]['id']
    return pos[anchor_tag_id]

def get_relative_rotations():
    anchor_rot = get_base_orientation()
    for tag in tags_in_scene:
        rot_rel[tag] = euler_from_quaternion(quaternion_multiply(quaternion_inverse(rot[tag]), anchor_rot))[2]

    print rot_rel

def transform_arm_to_local(arm):
    return 

def calc_reward():
    return

def send_action():
    msg = Action()
    msg.target_tag = 19
    msg.dist = 5.0
    msg.angle = direction_map['N'] + rot_rel[msg.target_tag]
    action_pub.publish(msg)

def run():
    action_pub = rospy.Publisher('/action_request', Action, queue_size=1)
    detections_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, detectionsClbk)
    ready_for_action_sub = rospy.Subscriber('/ready_for_action', Bool, readyForActionClbk)
    rospy.init_node('py_lfm', anonymous=True)
    rate = rospy.Rate(10)
    global ready_for_action
    before_snapshot = capture_scene()
    after_snapshot = before_snapshot
    while not rospy.is_shutdown():
        if (ready_for_action):
            after_snapshot = before_snapshot
            before_snapshot = capture_scene()
            update_scene_params()
            send_action()
            ready_for_action = False
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass