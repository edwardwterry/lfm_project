# -*- coding: utf-8 -*-
import random
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam

EPISODES = 10000

import matplotlib.pyplot as plt
np.random.seed(0)

class Env():
    def __init__(self):
        self.grid_edges = 4
        self.num_blocks = 2
        self.reward_per_move = -2.0
        # self.reward_sep = 1.0
        self.reward_grid = np.array([[2.0,1.0,1.0,2.0],[3.0,0.0,0.0,3.0],[1.0,0.0,0.0,1.0],[2.0,1.0,1.0,2.0]]) # requires 4x4
        self.reward_noop_sel = 0.0
        self.reward_adjacent = -1.0
        self.reward_offgrid = -2.0
        self.max_num_moves = 4
        self.overboard = False 
        self.grid_state = np.zeros((self.grid_edges, self.grid_edges))
        self.block_pos_orig = {1: (1, 1), 2: (1, 2)}
        self.block_pos = {1: (1, 1), 2: (1, 2)}
        self.move_count = 0
        self.action_map = {     'N':  (-1,  0), # (r, c)
                                # 'NW': (-1, -1),
                                'W':  ( 0, -1),
                                # 'SW': ( 1, -1),
                                'S':  ( 1,  0),
                                # 'SE': ( 1,  1),
                                'E':  ( 0,  1),
                                # 'NE': (-1,  1),
                                'NO_SEL': ( 0,  0)}
                                # 'NO_REQ': ( 0,  0)}
        # self.reward_captured = {1: False, 2: False, 3: False, 4: False}
        self.done = False

    def reset(self):
        self.grid_state = np.zeros((self.grid_edges, self.grid_edges))
        for block, pos in self.block_pos_orig.items():
            self.block_pos[block] = (pos[0], pos[1])
            self.grid_state[pos[0]][pos[1]] = block
            # self.reward_captured[block] = False

        self.move_count = 0
        self.done = False

        return self.grid_state

    def calc_reward(self, action):
        _reward = 0
        # rewards for action taken
        if action == 'NO_SEL':
            _reward = _reward + self.reward_noop_sel
        else:
            _reward = _reward + self.reward_per_move

        # rewards for location 
        for block in self.block_pos:
            _reward = _reward + self.reward_grid[self.block_pos[block][0]][self.block_pos[block][1]]

        # rewards for adjacency
        for block in self.block_pos:
            neigh = []
            for dir in self.action_map.keys():
                action = self.action_map[dir]            
                if not dir == 'NO_SEL':
                    _next = (self.block_pos[block][0] + action[0], self.block_pos[block][1] + action[1])
                    if self.on_grid(_next) and self.grid_state[_next[0]][_next[1]] == 0:
                        neigh.append(False)
                    elif not self.on_grid(_next):
                        neigh.append(False)
                    else: # there's another neighboring block
                        neigh.append(True)
            if any(v == True for v in neigh):
                _reward = _reward + self.reward_adjacent
        # if action == 'I_TERM' or self.done == True:
        #     for block in self.block_pos:
        #         free = []
        #         for dir in self.action_map.keys():
        #             action = self.action_map[dir]
        #             if not dir == 'NO_SEL':# and not dir == 'NO_REQ':
        #                 _next = (self.block_pos[block][0] + action[0], self.block_pos[block][1] + action[1])
        #                 if self.on_grid(_next) and self.grid_state[_next[0]][_next[1]] == 0:
        #                     free.append(True)
        #                 elif not self.on_grid(_next):
        #                     free.append(True)
        #                 else: # there's another neighboring block
        #                     free.append(False)
        #         if all(v == True for v in free):
        #             self.reward_captured[block] = True
        #             _reward = _reward + self.reward_sep

        #     if all(v == True for v in self.reward_captured.values()):
        #         self.done = True
        #         _reward = _reward + self.reward_end
        if self.overboard:
            _reward = _reward + self.reward_offgrid
        return _reward

    def on_grid(self, coord):
        row = coord[0]
        col = coord[1]
        if (row < 0 or 
            row >= self.grid_edges or
            col < 0 or 
            col >= self.grid_edges):
            return False
        else:
            return True

    def move(self, block, action):
        self.overboard = False
        if action == 'NO_SEL':
            return
        next_coord = (self.block_pos[block][0] + self.action_map[action][0], self.block_pos[block][1] + self.action_map[action][1])
        if not self.on_grid(next_coord): # move would take you over the edge
            self.block_pos[block] = (self.block_pos[block][0], self.block_pos[block][1])
            self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = block   
            self.overboard = True 
        elif (self.on_grid(next_coord) and self.grid_state[next_coord[0]][next_coord[1]] == 0): # move is on board and neighbor is free
            self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = 0
            self.block_pos[block] = (self.block_pos[block][0] + self.action_map[action][0], self.block_pos[block][1] + self.action_map[action][1])
            self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = block   
        else: # move is on board, and neighbor is occupied
            # how many neighbors in a row?
            occupants = []
            while self.on_grid(next_coord):
                occupant = int(self.grid_state[next_coord[0]][next_coord[1]])
                occupants.append(occupant)
                next_coord = (next_coord[0] + self.action_map[action][0], next_coord[1] + self.action_map[action][1])
            # https://stackoverflow.com/questions/6039425/sequence-find-function-in-python
            if 0 in occupants:
                num_nonzero_neighbors = occupants.index(0)-1 # [0 1 2 0] and 1,E gives 1 # [1 2 3 0] and 1,E gives 2
                for i in range(num_nonzero_neighbors, -1, -1):
                    # furthest block away
                    _block = occupants[i]
                    self.move(_block, action) # recursion!
                self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = 0
                self.block_pos[block] = (self.block_pos[block][0] + self.action_map[action][0], self.block_pos[block][1] + self.action_map[action][1])
                self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = block                   
            else: # gridlock, can't move at all
                self.block_pos[block] = (self.block_pos[block][0], self.block_pos[block][1])
                self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = block   

    def print_grid(self):
        print (self.grid_state)

    def step(self, block, action):
        self.move_count = self.move_count + 1
        # print ("Move #", self.move_count)
        self.move(block, action)
        if self.move_count == self.max_num_moves:
            self.done = True
        # print "Action: ", action, " on block ", block
        # self.print_grid()
        
        return self.grid_state, self.block_pos, self.calc_reward(action), self.done

# if __name__ == "__main__":
#     env = Env()
#     env.reset()
#     max_num_episodes = 1000
#     avail_actions = [k for k in env.action_map.keys()]# if not k=='NO_REQ']
#     avail_actions.append('I_TERM')
#     reward_array = []
#     episode = 0
#     while episode < max_num_episodes:
#         done = False
#         reward_total = 0
#         # print "\nNew episode!"
#         while not done:
#             state, block_pos, reward, done = env.step(np.random.choice(env.num_blocks)+1, np.random.choice(avail_actions))
#             # print reward
#             reward_total = reward_total + reward
#         episode = episode + 1
#         env.reset()
#         reward_array.append(reward_total)
#     # plt.scatter(range(max_num_episodes),reward_array)
#     # plt.show()
#     # print np.mean(reward_array)



class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=1000)
        self.gamma = 0.95    # discount rate
        self.epsilon = 0.8  # exploration rate
        self.epsilon_min = 0.05
        self.epsilon_decay = 0.9998
        self.learning_rate = 0.001
        self.model = self._build_model()
        # self.action_map = ((1, 'N'), (1, 'W'), (1, 'S'), (1, 'E'),
        #                    (2, 'N'), (2, 'W'), (2, 'S'), (2, 'E'), (1, 'NO_SEL'))
        self.action_map = ((1, 'W'), (1, 'E'), (2, 'W'), (2, 'E'), (1, 'NO_SEL'))

    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        model = Sequential()
        model.add(Dense(50, input_dim=self.state_size, activation='relu'))
        model.add(Dense(30, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss='mse',
                      optimizer=Adam(lr=self.learning_rate))
        return model

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        act_values = self.model.predict(state)
        # return np.argmax(act_values[0])  # returns action
        return np.argmax(act_values[0])  # returns action

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        # print (minibatch)
        for state, action, reward, next_state, done in minibatch:
            target = reward
            if not done:
                target = (reward + self.gamma *
                          np.amax(self.model.predict(next_state)[0]))
            target_f = self.model.predict(state)
            target_f[0][action] = target
            # print (target_f)
            # mask = np.zeros((1, self.action_size))
            # mask[0][action] = 1
            # print (mask*target_f)
            self.model.fit(state, target_f, epochs=1, verbose=0)
            # self.model.fit(state, mask*target_f, epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def load(self, name):
        self.model.load_weights(name)

    def save(self, name):
        self.model.save_weights(name)


if __name__ == "__main__":
    env = Env()
    state_size = len(env.grid_state.ravel())
    action_size = len(env.action_map)
    agent = DQNAgent(state_size, action_size)
    # agent.load("./save/cartpole-dqn.h5")
    # done = False
    batch_size = 32
    reward_total = []

    for e in range(EPISODES):
        state = env.reset()
        state = np.reshape(state, [1, state_size])
        done = False
        episode_reward = 0
        while not done:
            action = agent.act(state)
            next_state, _, reward, done = env.step(agent.action_map[action][0], agent.action_map[action][1])
            episode_reward = episode_reward + reward
            next_state = np.reshape(next_state, [1, state_size]) # turns into a row vector
            agent.remember(state, action, reward, next_state, done)
            state = next_state
            if done:
                reward_total.append(episode_reward)
                # print("\n", episode_reward)
                # print(next_state.reshape((4,4)))
                episode_reward = 0
                if e%50 == 0:
                    print("episode: {}/{}, reward: {:.2}, std: {:.2}, e: {:.2}"
                        .format(e, EPISODES, np.mean(reward_total), np.std(reward_total), agent.epsilon))
                    reward_total = []
                if e%250 == 0 and e > 0:
                    agent.save(str(e))
        if len(agent.memory) > batch_size:
            agent.replay(batch_size)
        # if e % 10 == 0:
        #     agent.save("./save/cartpole-dqn.h5")