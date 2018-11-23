#! usr/bin/env python

import os, sys
import time
import copy

import argparse
import numpy as np
import matplotlib.pyplot as plt
import math
import random as rand
import rospy
from lfm.msg import Action, AprilTagDetectionArray, AprilTagDetection
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion, quaternion_inverse, quaternion_multiply

# Hyperparameters and utility functions
THRESHOLD_MOVED = 0.02
THRESHOLD_CONNECTION_PROB = 0.5
THRESHOLD_LOWER_BOUND = 0.1
THRESHOLD_UPPER_BOUND = 0.9
SHIFT_LENGTH=0.05

sequence_complete = False
sequence_initiated = False

np.random.seed(0)

def is_within_range(val, min_lim, max_lim):
  return val <= max_lim and val >= min_lim

def declare_connections(blocks):
  """
  DO NOT MODIFY!
  Function to declare links in a 4x4 grid of blocks
  - A block is defined by its block id
  - A link is defined by a tuple of the two blocks it is connecting

  Returns
  ----------
  A dictionary, where the key is the link and value is a list of adjacent blocks
  """

  joined = {}
  adjacent = {}
  id_list = [blocks[tag]['id'] for tag in range(len(blocks))]
  directions = {'N': (-1, 0), 'W': (0, -1), 'S': (1, 0), 'E': (0, 1)} # delta(r, c)
  for entry in range(len(id_list)):
    block_joined = []
    block_adjacent = []
    for dir in directions:
      coord = (blocks[entry]['row'], blocks[entry]['col'])
      delta = directions[dir]
      for entry_inner in range(len(id_list)):
        if ((blocks[entry_inner]['row'] == coord[0] + delta[0]) and \
            (blocks[entry_inner]['col'] == coord[1] + delta[1])):
          block_adjacent.append(blocks[entry_inner]['id'])
          if blocks[entry][dir] == 1:
            block_joined.append(blocks[entry_inner]['id'])

    adjacent[blocks[entry]['id']] = block_adjacent
    joined[blocks[entry]['id']] = block_joined

  return adjacent, joined

class Perception():
  def __init__(self, master_tag_id):
    self.detections_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detectionsClbk)
    self.master_tag_id = master_tag_id
    self.pos = {}
    self.rot = {}
    self.tags_in_scene = []
    self.pos_rel = {}
    self.rot_rel = {}
    self.displacement = {}   
    self.R = np.array([rospy.get_param('R')]).reshape(3,3)
    self.t = np.array([rospy.get_param('t')]).reshape(3,1)
    self.start_scene = {}
    self.end_scene = {}

  def detectionsClbk(self, msg):
    detections_raw = msg.detections
    indices_raw = [detections_raw[i].id[0] for i in range(len(detections_raw))]
    self.tags_in_scene = indices_raw
    for i in range(len(indices_raw)):
        _pos = detections_raw[i].pose.pose.pose.position
        _pos_vector = np.array([_pos.x, _pos.y, _pos.z])
        self.pos[indices_raw[i]] = self.transform_cam_to_arm(_pos_vector) # assumes that R and t are populated, converts m to mm
        self.rot[indices_raw[i]] = detections_raw[i].pose.pose.pose.orientation

  def capture_scene(self):
      return self.pos, self.rot

  def transform_cam_to_arm(self, cam):
      cam = np.array([cam]).reshape(3,1)
      return np.matmul(self.R, cam) + self.t

  def get_base_orientation(self):
      try:
        return self.rot[self.master_tag_id]
      except:
        pass

  def get_base_position(self):
      try:
        return self.pos[self.master_tag_id]
      except:
        pass

  def get_relative_rotations(self):
      anchor_rot = self.get_base_orientation()
      for tag in tags_in_scene:
          rot_rel[tag] = euler_from_quaternion(quaternion_multiply(quaternion_inverse(rot[tag]), anchor_rot))[2]

  def transform_arm_to_master(self, arm):
      master_orientation = self.get_base_orientation
      master_position = self.get_base_position
      th = euler_from_quaternion(quaternion_multiply(quaternion_inverse(master_orientation), [0,0,0,1]))[2] # TBA
      H = np.array([[1, 0, 0, loc[0]],[0, cos(th), sin(th), loc[1]],[0, -sin(th), cos(th), loc[2]],[0,0,0,1]])
      return np.matmul(H, arm) # TBA dimensions

  def calculate_displacement(self):
    delta = {}
    for tag in self.start_scene: # 0th element contains position data
      try: 
        if tag in self.end_scene:
          print self.start_scene[tag][0], self.end_scene[tag][0]
          print self.start_scene[tag][1], self.end_scene[tag][1]
          delta[tag] = math.sqrt((self.end_scene[tag][0] - self.start_scene[tag][0]) ** 2 \
                                    + (self.end_scene[tag][1] - self.start_scene[tag][1]) ** 2)
      except:
        print('Tag ', tag, ' can''t be found in end scene')
        pass
    print "Tag displacement:\n", delta
    return delta # this is probably in meters

  def save_current_scene(self, scene_type):
    if scene_type == "start":
      # https://stackoverflow.com/questions/11941817/how-to-avoid-runtimeerror-dictionary-changed-size-during-iteration-error
      self.start_scene = { k : v for k,v in self.pos.iteritems() if v.any()}
    elif scene_type == "end":
      self.end_scene = { k : v for k,v in self.pos.iteritems() if v.any()}

class Control():
  def __init__(self):
    pass
    self.action_pub = rospy.Publisher('/action_request', Action, queue_size = 100)
    self.direction_map = {'N': 0.0, 'W': 90.0, 'S': 180.0, 'E': 270.0}

  def send_action(self, target_tag, dist, angle):#, master_orientation, rel_to_master=False):
    action_msg = Action()
    action_msg.target_tag = target_tag
    action_msg.dist = dist
    action_msg.angle = angle# + rel_to_master*master_orientation/180.0*math.pi # assumes master_orientation is in radians
    while self.action_pub.get_num_connections() == 0:
      rospy.loginfo("Waiting for subscriber to connect")
      rospy.sleep(0.25)
    self.action_pub.publish(action_msg)
    print "Published action!"

class Scene():
  """
  Just a utility class for bookkeeping the probabilities for each link

  Instance variables
  ----------
  num_blocks : Number of blocks in the grid, this is essentially 16
  prob: a dictionary that keeps track of the connection probabilities
  links: a dictionary that keeps track of the adjacent blocks for a given block_id
  """
  def __init__(self, blocks):
    self._num_blocks = len(blocks)
    self._prob = {}
    self._adjacent, _joined = declare_connections(blocks)
    self.block_ids = [blocks[tag]['id'] for tag in range(len(blocks))]
    self.direction_map = {'N': 0.0, 'W': 90.0, 'S': 180.0, 'E': 270.0}
    self.num_actions = 6
    self.action_count = 0
    self.target_tags = []
    self.dists = []
    self.angles = []
    self.populate_action_sequence()

  def populate_action_sequence(self):
    max_dist = 40 # mm
    self.target_tags = np.random.choice(self.block_ids, self.num_actions)
    self.dists = np.random.rand(self.num_actions) * max_dist
    self.angles = np.random.choice(self.direction_map.values(), self.num_actions)
    print self.target_tags, self.dists, self.angles

  def get_next_action(self):
    try:
      out_1 = self.target_tags[self.action_count]
      out_2 = self.dists[self.action_count]
      out_3 = self.angles[self.action_count]
      self.action_count = self.action_count + 1
      return out_1, out_2, out_3
    except:
      pass

  def set_probability(self, i, j, prob):
    """
    DO NOT MODIFY!

    Sets the connection probability for the link connecting blocks with block_ids
    i and j to the value specified by prob

    Parameters
    ----------
    i : index of one block in the link
    j : index of the other block in the link (the ordering doesn't matter)
    """
    self._prob[tuple(sorted((i, j)))] = prob

  def get_probability(self, i, j):
    """
    DO NOT MODIFY!

    Returns the current connection probability for the link connecting blocks 
    with block_ids i and j. Note that the ordering of i and j does not matter.
    """
    return self._prob[tuple(sorted((i, j)))]

  def get_all_links(self):
    """
    DO NOT MODIFY!

    Returns a list of all links in the grid.
    """
    return self._prob.keys()

  def get_links(self, i):
    """
    DO NOT MODIFY!

    Returns a list of the adjacent blocks for a given block_id, which may
    or may not be connected
    """
    return self._adjacent[i]

  def get_link_entropy(self):
    """
    Returns a dictionary, where the key is the link and the value is
    the entropy of the distribution of whether the link is connected.
    """
    entropy = {}
    for link, prob in self._prob.iteritems():
      # TODO:
      raise NotImplementedError
    return entropy

  def choose_action(self):
    # TODO PAULO
    return 

  def compute_observation(self, block_id, state_pre, state_post, block_mappings):
    """
    DO NOT MODIFY!

    Computes the high level observation, whether a given block has moved (or not)

    Parameters
    ----------
    block_id : Corresponds to the block that was moved
    state_pre : V-REP data structure of the state before block was moved
    state_post : V-REP data structure of the state before block was moved
    block_mappings : V-REP data structure mapping grid definition to that of V-REP

    Returns
    ----------
    observation: A dictionary where the key is the block_id and value is whether
    that block moved (boolean)
    """
    observation = {}

    for block_id_adj in xrange(1, self._num_blocks+1):
      adj_idx = int(block_mappings.flatten()[block_id_adj-1]) - 1
      dist_moved = np.linalg.norm(np.array(state_post['Block'][adj_idx][:2]) - np.array(state_pre['Block'][adj_idx][:2]))
      if dist_moved > THRESHOLD_MOVED:
        print('When moving block {}, adjacent block {} moved {}'.format(block_id, block_id_adj, dist_moved))
        observation[block_id_adj] = True
      else:
        print('When moving block {}, adjacent block {} was stationary, it moved {} (m)'.format(block_id, block_id_adj, dist_moved))
        observation[block_id_adj] = False
    return observation

  def update(self, delta):

      for link in self.get_all_links():
        (block_1, block_2) = link
        prob_prior = self.get_probability(block_1, block_2)

        # Compute posterior of link connection given data
        # TODO: - update prob_posterior correctly, based on the observation and prior from previous iteration
        prob_posterior = 0.0

        # Update connection probability
        self.set_probability(block_1, block_2, prob_posterior)

  def is_converged(self):
    """
    DO NOT MODIFY!

    Returns True, if the probabilities have converged (i.e. below certain thresholds)
    and False otherwise.
    """
    prob = np.array(self._prob.values())
    return np.all(np.logical_or(prob<THRESHOLD_LOWER_BOUND, prob>THRESHOLD_UPPER_BOUND))

def seqCompleteClbk(msg):
  global sequence_complete
  sequence_complete = True

def seqInitiatedClbk(msg):
  global sequence_initiated
  sequence_initiated = True

def run():
  """
  TODO: Complete the implementation for this function

  Parameters
  ----------
  handles : V-REP data structure
  block_mappings : V-REP data structure
  use_random_policy : Pass in this flag in commandline args to select
    random actions at each iteration
  """
  actionSequence = []
  averageEntropy = []
  count_iter = 0

  # rot_rel_to_master = False

  global sequence_complete, sequence_initiated

  blocks = rospy.get_param('blocks')
  num_blocks = len(blocks)
  scene = Scene(blocks)
  control = Control()
  perc = Perception(blocks[0]['id'])
  seq_complete_sub = rospy.Subscriber('/sequence_complete', Bool, seqCompleteClbk)
  seq_initiated_sub = rospy.Subscriber('/sequence_initiated', Bool, seqInitiatedClbk)
  master_orientation = perc.get_base_orientation()
  ready_for_next_action = True

  # Initialize connection probabilities to 0.5 (prior):
  for block_id in scene.block_ids:
    for block_id_adj in scene.get_links(block_id):
      scene.set_probability(block_id, block_id_adj, 0.5)

  # # Terminate when probability values have converged
  while not rospy.is_shutdown():
    while not scene.is_converged() or count_iter < scene.action_count:
      if ready_for_next_action:
        target_tag, dist, angle = scene.get_next_action()
        actionSequence.append([target_tag, dist, angle])
        control.send_action(target_tag, dist, angle)
        ready_for_next_action = False

      #   entropy = scene.get_link_entropy()
      #   print entropy
      #   averageEntropy.append(np.mean(entropy.values()))

      if sequence_initiated:
        print "Capturing start scene..."
        perc.save_current_scene("start")
        print "Captured start scene"
        sequence_initiated = False

      if sequence_complete:
        count_iter += 1
        sequence_complete = False
        ready_for_next_action = True
        print "Capturing end scene..."
        perc.save_current_scene("end")
        print "Captured end scene"
        delta = perc.calculate_displacement()
        # scene.update(delta)

    # # Print the connections found
    # for link, p in linked_blocks._prob.items():
    #   if p > THRESHOLD_CONNECTION_PROB:
    #     print('Link found between block {} and {}'.format(link[0], link[1]))

    # print('Action Sequence executed: ', actionSequence)
    # print('Converged after {} iterations, final Entropy: {}'.format(count_iter, averageEntropy[-1]))

  # def main():
    # ready_for_action_sub = rospy.Subscriber('/ready_for_action', Bool, readyForActionClbk)

    # global ready_for_action
    # before_snapshot = capture_scene()
    # after_snapshot = before_snapshot
    # while not rospy.is_shutdown():
      # run()
    #   run()
    #     # if (ready_for_action):
    #     #     after_snapshot = before_snapshot
    #     #     before_snapshot = capture_scene()
    #     #     update_scene_params()
    #     #     send_action()
    #     #     ready_for_action = False
    #     rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hw1_adapted', anonymous=True)
    rate = rospy.Rate(10)
    try:
        run()
    except rospy.ROSInterruptException:
        pass

