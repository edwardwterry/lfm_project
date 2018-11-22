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

# class LinkedBlocks():
#   """
#   Just a utility class for bookkeeping the probabilities for each link

#   Instance variables
#   ----------
#   num_blocks : Number of blocks in the grid, this is essentially 16
#   prob: a dictionary that keeps track of the connection probabilities
#   links: a dictionary that keeps track of the adjacent blocks for a given block_id
#   """
#   def __init__(self, num_blocks):
#     self._num_blocks = num_blocks
#     self._prob = {}
#     self._links = declare_connections()

#   def set_probability(self, i, j, prob):
#     """
#     DO NOT MODIFY!

#     Sets the connection probability for the link connecting blocks with block_ids
#     i and j to the value specified by prob

#     Parameters
#     ----------
#     i : index of one block in the link
#     j : index of the other block in the link (the ordering doesn't matter)
#     """
#     self._prob[tuple(sorted((i, j)))] = prob

#   def get_probability(self, i, j):
#     """
#     DO NOT MODIFY!

#     Returns the current connection probability for the link connecting blocks 
#     with block_ids i and j. Note that the ordering of i and j does not matter.
#     """
#     return self._prob[tuple(sorted((i, j)))]

#   def get_all_links(self):
#     """
#     DO NOT MODIFY!

#     Returns a list of all links in the grid.
#     """
#     return self._prob.keys()

#   def get_links(self, i):
#     """
#     DO NOT MODIFY!

#     Returns a list of the adjacent blocks for a given block_id, which may
#     or may not be connected
#     """
#     return self._links[i]

#   def get_link_entropy(self):
#     """
#     Returns a dictionary, where the key is the link and the value is
#     the entropy of the distribution of whether the link is connected.
#     """
#     entropy = {}
#     for link, prob in self._prob.iteritems():
#       # TODO:
#       raise NotImplementedError
#     return entropy

#   def compute_observation(self, block_id, state_pre, state_post, block_mappings):
#     """
#     DO NOT MODIFY!

#     Computes the high level observation, whether a given block has moved (or not)

#     Parameters
#     ----------
#     block_id : Corresponds to the block that was moved
#     state_pre : V-REP data structure of the state before block was moved
#     state_post : V-REP data structure of the state before block was moved
#     block_mappings : V-REP data structure mapping grid definition to that of V-REP

#     Returns
#     ----------
#     observation: A dictionary where the key is the block_id and value is whether
#     that block moved (boolean)
#     """
#     observation = {}

#     for block_id_adj in xrange(1, self._num_blocks+1):
#       adj_idx = int(block_mappings.flatten()[block_id_adj-1]) - 1
#       dist_moved = np.linalg.norm(np.array(state_post['Block'][adj_idx][:2]) - np.array(state_pre['Block'][adj_idx][:2]))
#       if dist_moved > THRESHOLD_MOVED:
#         print('When moving block {}, adjacent block {} moved {}'.format(block_id, block_id_adj, dist_moved))
#         observation[block_id_adj] = True
#       else:
#         print('When moving block {}, adjacent block {} was stationary, it moved {} (m)'.format(block_id, block_id_adj, dist_moved))
#         observation[block_id_adj] = False
#     return observation

#   def is_converged(self):
#     """
#     DO NOT MODIFY!

#     Returns True, if the probabilities have converged (i.e. below certain thresholds)
#     and False otherwise.
#     """
#     prob = np.array(self._prob.values())
#     return np.all(np.logical_or(prob<THRESHOLD_LOWER_BOUND, prob>THRESHOLD_UPPER_BOUND))

# def run_experiment():
