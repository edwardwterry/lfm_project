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
np.set_printoptions(precision=3)
# Hyperparameters and utility functions
THRESHOLD_MOVED = 0.005
THRESHOLD_CONNECTION_PROB = 0.5
THRESHOLD_LOWER_BOUND = 0.1
THRESHOLD_UPPER_BOUND = 0.9
SHIFT_LENGTH=0.05

# https://stackoverflow.com/questions/2084508/clear-terminal-in-python
import os
os.system('cls' if os.name == 'nt' else 'clear')

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
    self.cx = 0.0
    self.cy = 0.0

  def detectionsClbk(self, msg):
    detections_raw = msg.detections
    indices_raw = [detections_raw[i].id[0] for i in range(len(detections_raw))]
    self.tags_in_scene = indices_raw
    for i in range(len(indices_raw)):
        _pos = detections_raw[i].pose.pose.pose.position
        _pos_vector = np.array([_pos.x, _pos.y, _pos.z])
        self.pos[indices_raw[i]] = self.transform_cam_to_arm(_pos_vector) # assumes that R and t are populated, converts m to mm (?)
        self.rot[indices_raw[i]] = detections_raw[i].pose.pose.pose.orientation

  def capture_centroid(self):
    _cx = 0.0
    _cy = 0.0
    for i in range(len(self.tags_in_scene)):
      _cx = _cx + self.pos[self.tags_in_scene[i][0]]
      _cy = _cy + self.pos[self.tags_in_scene[i][1]]
    self.cx = _cx / len(self.tags_in_scene)
    self.cy = _cy / len(self.tags_in_scene)

  def distWeight(self):
      pos = self.pos
      weightDist = [math.sqrt((self.cx-pos[_][0])**2 + (self.cy-pos[_][1])**2) for _ in self.pos]
      # normalize it
      normWeight = [_/sum(weightDist) for _ in weightDist]
      print('WEIGHT:',weightDist)
      print('Normalized Weight:', normWeight)
      return normWeight

  def capture_scene(self):
      return self.pos.squeeze(), self.rot

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
    _rot = self.get_base_orientation()
    _pos = self.get_base_position()
    try:
      _arm = arm.squeeze()
      _arm = np.array([_arm[0], _arm[1], _arm[2], 1.0]).reshape(4,1)
      quat_input = [_rot.x, _rot.y, _rot.z, _rot.w]
      th = euler_from_quaternion(quaternion_multiply(quaternion_inverse(quat_input), [0,1,0,0]))[2] # TBA
      H = np.array([[math.cos(th), -math.sin(th), 0, _pos[0]], [math.sin(th), math.cos(th), 0, _pos[1]], [0, 0, 1, _pos[2]], [0,0,0,1]])
      # H = np.array([[1, 0, 0, _pos[0]],[0, math.cos(th), -math.sin(th), _pos[1]],[0, math.sin(th), math.cos(th), 0.0*_pos[2]],[0,0,0,1]])
      # print _arm
      print np.matmul(H, _arm) 
      # return np.matmul(H, arm) # TBA dimensions
    except:
      pass

  def calculate_displacement(self):
    delta = {}
    for tag in self.start_scene: # 0th element contains position data
      try: 
        if tag in self.end_scene:
          # print self.start_scene[tag][0], self.end_scene[tag][0]
          # print self.start_scene[tag][1], self.end_scene[tag][1]
          delta[tag] = math.sqrt((self.end_scene[tag][0] - self.start_scene[tag][0]) ** 2 \
                                    + (self.end_scene[tag][1] - self.start_scene[tag][1]) ** 2)
      except:
        print('Tag ', tag, ' can''t be found in end scene')
        pass
    # print "Tag displacement:\n", delta
    return delta # this is in meters

  def save_current_scene(self, scene_type):
    if scene_type == "start":
      # https://stackoverflow.com/questions/11941817/how-to-avoid-runtimeerror-dictionary-changed-size-during-iteration-error
      self.start_scene = { k : v for k,v in self.pos.iteritems() if v.any()}
    elif scene_type == "end":
      self.end_scene = { k : v for k,v in self.pos.iteritems() if v.any()}

class Control():
  def __init__(self):
    # pass
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

# class Reps():
#   def __init__(self):
#     pass
#     self.goal_locations = {'pt': {(1, -1), (1, 0), (1, 1)}, 'line': {(1, 0)}, 'c1': {(1, 0)}, 'c2': {(2, 0)}, 'c3': {(3, 0)}}
  
#   def calc_goal_pos(self, config):


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
    self.block_ids = np.sort([blocks[tag]['id'] for tag in range(len(blocks))])
    self.block_index_map = {}
    for i in range(self._num_blocks):
      self.block_index_map[i] = self.block_ids[i]    
    print self.block_index_map
    self.direction_map = {'N': 0.0, 'W': 90.0, 'S': 180.0, 'E': 270.0}
    self.num_actions = 6
    self.action_count = 0
    self.target_tags = []
    self.dists = []
    self.angles = []
    self.E = np.tril(np.full((self._num_blocks,self. _num_blocks),1.0),-1) # initiate it with highest entropy
    self.L = []
    self.sureLink = []
    self.moved=[]
    self.move_distance = 10.0 # mm

  def populate_action_sequence(self):
    max_dist = 40 # mm
    self.target_tags = np.random.choice(self.block_ids, self.num_actions)
    self.dists = np.random.rand(self.num_actions) * max_dist
    self.angles = np.random.choice(self.direction_map.values(), self.num_actions)
    # print self.target_tags, self.dists, self.angles

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

  # def get_link_entropy(self):
  #   """
  #   Returns a dictionary, where the key is the link and the value is
  #   the entropy of the distribution of whether the link is connected.
  #   """
  #   entropy = {}
  #   for link, prob in self._prob.iteritems():
  #     # TODO:
  #     raise NotImplementedError
  #   return entropy

  # def choose_action(self):
  #   # TODO PAULO
  #   return 

  # def compute_observation(self, block_id, state_pre, state_post, block_mappings):
  #   """
  #   DO NOT MODIFY!

  #   Computes the high level observation, whether a given block has moved (or not)

  #   Parameters
  #   ----------
  #   block_id : Corresponds to the block that was moved
  #   state_pre : V-REP data structure of the state before block was moved
  #   state_post : V-REP data structure of the state before block was moved
  #   block_mappings : V-REP data structure mapping grid definition to that of V-REP

  #   Returns
  #   ----------
  #   observation: A dictionary where the key is the block_id and value is whether
  #   that block moved (boolean)
  #   """
  #   observation = {}

  #   for block_id_adj in xrange(1, self._num_blocks+1):
  #     adj_idx = int(block_mappings.flatten()[block_id_adj-1]) - 1
  #     dist_moved = np.linalg.norm(np.array(state_post['Block'][adj_idx][:2]) - np.array(state_pre['Block'][adj_idx][:2]))
  #     if dist_moved > THRESHOLD_MOVED:
  #       print('When moving block {}, adjacent block {} moved {}'.format(block_id, block_id_adj, dist_moved))
  #       observation[block_id_adj] = True
  #     else:
  #       print('When moving block {}, adjacent block {} was stationary, it moved {} (m)'.format(block_id, block_id_adj, dist_moved))
  #       observation[block_id_adj] = False
  #   return observation

  def update_moved(self, delta):
    print delta
    # print np.array([delta])
    # self.moved = [1 if delta[self.block_index_map[index]] > THRESHOLD_MOVED else 0 for index in range(len(self.block_index_map))]
    print self.block_index_map
    # print self.block_index_map[0]
    print delta[0]
    print delta[self.block_index_map.values()[2]]
    self.moved = [1 if delta[index] > THRESHOLD_MOVED else 0 for index in self.block_index_map.values()]
    print self.moved

  # def is_converged(self):
  #   """
  #   DO NOT MODIFY!

  #   Returns True, if the probabilities have converged (i.e. below certain thresholds)
  #   and False otherwise.
  #   """
  #   prob = np.array(self._prob.values())
  #   return np.all(np.logical_or(prob<THRESHOLD_LOWER_BOUND, prob>THRESHOLD_UPPER_BOUND))

  # Generates matrix of prior probabilities that each link combination is active
  def genLink(self): # generates ma
      L = np.full((self._num_blocks, self._num_blocks), THRESHOLD_CONNECTION_PROB)
      self.L = np.tril(L,-1)
      print self.L

  def bestAction(self): # TO DO this is just a baseline: Develop actual action policy
      #publish to robot next part to push
      maxEnt = np.argmax(self.E) # link with highest entropy
      highLink = np.unravel_index(maxEnt, self.E.shape) # unravel it into a position tuple
      # print highLink
      target_tag_index = np.random.choice(highLink) # pick a random part of the high link
      target_tag = self.block_index_map[target_tag_index] # pick a random part of the high link
      dist = self.move_distance
      angle = np.random.choice(self.direction_map.values()) # pick a random direction

  ### quadrants
#       x = self.target[0]
#       y = self.target[1]
#       if  x < self.cx and y < self.cy:
#           self.direction = 'N'
#       elif x < self.cx and y > self.cy:
#           self.direction = 'E'
#       elif x > self.cx and y > self.cy:
#           self.direction = 'S'
#       else:
#           self.direction = 'W'
# #        self.direction = np.random.choice(self.action) # pick a random direction

      self.action_count = self.action_count + 1
      print "Target tag: ", target_tag
      print "Distance: ", dist, " mm"
      print "Angle: ", angle, "deg"
      return target_tag, dist, angle

  def updateBelief(self):
      # iterate through lower triangular portion of matrix L and update probabilitiies links
      for i in range(self._num_blocks):
          for j in range(0,i): # iterate through row of part just moved
              # check if link is not already determined
              if (i,j) not in self.sureLink:
                  probPrior = self.L[i][j]
                  # these are Bayesian updated similar to HW1. TODO: update these ratios

                  pIsJsL1 = 0.5
                  pIsJsL0 = 0.5
                  pImJsL1 = 0.4
                  pIsJmL1 = 0.4
                  pImJsL0 = 0.6
                  pIsJmL0 = 0.6
                  pImJmL1 = 0.6
                  pImJmL0 = 0.4

                  if (self.moved[i] == True and self.moved[j] == True):
                      # probPos = 3 * probPrior / (2 + probPrior)
                      probPos = pImJmL1*probPrior/(pImJmL1*probPrior + pImJmL0*(1-probPrior))
                  elif (self.moved[i] == False and self.moved[j] == False):
                      probPos = probPrior # nothing was learned here
                  else:
                      # probPos = 2 * probPrior / (3 -  probPrior)
                      probPos = pImJsL1*probPrior/(pImJsL1*probPrior + pImJsL0*probPrior)
                  # update L matrix
                  self.L[i][j] = probPos
                  # adds to set link with high certainty to capture that relation
                  # to avoid updating it and moving towards convergence
                  if probPos > THRESHOLD_UPPER_BOUND or probPos < THRESHOLD_LOWER_BOUND:
                      self.sureLink.append((i,j))
                      if probPos > THRESHOLD_UPPER_BOUND:
                        print '\n !!!! PARTS ARE JOINED: ', self.block_index_map[i], ' and ', self.block_index_map[j], '\n'
                      if probPos < THRESHOLD_LOWER_BOUND:
                        print '\n **** PARTS ARE SEPARATE: ', self.block_index_map[i], ' and ', self.block_index_map[j], '\n'                      
      print('Updated Link Probabilities:')
      print(self.L)
      
  # update entropy matrix    
  def updateEntropy(self, weight):
      for i in range(0,self._num_blocks):
          for j in range(0,i):
              p = self.L[i][j]
              entropy = -p * math.log(p, 2) - (1-p) * math.log(1-p, 2)
              self.E[i][j] = entropy
              self.E = np.tril(self.E,-1) # extracts lower triangular
      print('Entropy Matrix (lower triangular part # vs part #)')
      print self.E
      # update it considering distance of each part to to center 
      # pre-multiplying Entropy matrix by weight vector
      # weight = self.distWeight()
      for i in range(0,len(self.E)):
          for j in range(0,len(self.E[0])):
              self.E[i][j] = self.E[i][j]*weight[i]
      print('Entropy updated with weighted distance')
      print(self.E)

  def is_converged(self):
    return np.all(np.bitwise_or(self.L<THRESHOLD_LOWER_BOUND, self.L>THRESHOLD_UPPER_BOUND))

def seqCompleteClbk(msg):
  global sequence_complete
  sequence_complete = True

def seqInitiatedClbk(msg):
  global sequence_initiated
  sequence_initiated = True

def run(policy):
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
  first_move = True
  
  scene.genLink()

  while not rospy.is_shutdown():
    # pass
    # print perc.pos.get(18)
    # perc.transform_arm_to_master(perc.pos.get(18))
    while not scene.is_converged():
      if ready_for_next_action:
        # target_tag, dist, angle = scene.get_next_action() # used for demo
        if policy == 'max_entropy':
          target_tag, dist, angle = scene.bestAction()
        actionSequence.append([target_tag, dist, angle])
        control.send_action(target_tag, dist, angle)
        ready_for_next_action = False

      if sequence_initiated:
        print "Capturing start scene..."
        perc.save_current_scene("start")
        rospy.loginfo("Captured start scene")
        if first_move:
          # perc.capture_centroid()
          first_move = False
        sequence_initiated = False

      if sequence_complete:
        sequence_complete = False
        ready_for_next_action = True
        print "Capturing end scene..."
        perc.save_current_scene("end")
        rospy.loginfo("Captured end scene")
        delta = perc.calculate_displacement()
        scene.update_moved(delta)
        scene.updateBelief()
        scene.updateEntropy(perc.distWeight())
        # raw_input()
    
    print "Completed in ", scene.action_count, " iterations"
    quit()

if __name__ == '__main__':
    rospy.init_node('hw1_adapted', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--policy', default = 'max_entropy')

    args = parser.parse_args()
    rate = rospy.Rate(10)
    try:
        run(args.policy)
    except rospy.ROSInterruptException:
        pass

