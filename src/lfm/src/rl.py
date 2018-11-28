import numpy as np
np.random.seed(0)

class Env():
    def __init__(self):
        self.grid_edges = 4
        self.num_blocks = 4
        self.reward_per_move = -1.0
        self.reward_sep = 1.0
        self.reward_end = 10.0
        self.reward_noop_sel = 0.0
        self.max_num_moves = 10
        self.grid_state = np.zeros((self.grid_edges, self.grid_edges))
        self.block_pos_orig = {1: (1, 1), 2: (1, 2), 3: (2, 1), 4: (2, 2)}
        self.block_pos = {1: (1, 1), 2: (1, 2), 3: (2, 1), 4: (2, 2)}
        # self.block_pos = {1: (0, 0), 2: (2, 3), 3: (3, 0), 4: (3, 3)}
        self.move_count = 0
        self.action_map = {     'N':  (-1,  0), # (r, c)
                                # 'NW': (-1, -1),
                                'W':  ( 0, -1),
                                # 'SW': ( 1, -1),
                                'S':  ( 1,  0),
                                # 'SE': ( 1,  1),
                                'E':  ( 0,  1),
                                # 'NE': (-1,  1),
                                'NO_SEL': ( 0,  0),
                                'NO_REQ': ( 0,  0)}
        self.reward_captured = {1: False, 2: False, 3: False, 4: False}
        self.done = False

    def reset_env(self):
        self.grid_state = np.zeros((self.grid_edges, self.grid_edges))
        for block, pos in self.block_pos_orig.items():
            self.block_pos[block] = (pos[0], pos[1])
            self.grid_state[pos[0]][pos[1]] = block
            self.reward_captured[block] = False

        self.move_count = 0
        self.done = False

    def calc_reward(self, action):
        _reward = 0
        if action == 'NO_SEL':
            _reward = _reward + self.reward_noop_sel
        else:
            _reward = _reward + self.reward_per_move
        if action == 'I_TERM' or self.done == True:
            for block in self.block_pos:
                free = []
                for dir in self.action_map.keys():
                    action = self.action_map[dir]
                    if not dir == 'NO_SEL' and not dir == 'NO_REQ':
                        _next = (self.block_pos[block][0] + action[0], self.block_pos[block][1] + action[1])
                        if self.on_grid(_next) and self.grid_state[_next[0]][_next[1]] == 0:
                            free.append(True)
                        elif not self.on_grid(_next):
                            free.append(True)
                        else: # there's another neighboring block
                            free.append(False)
                if all(v == True for v in free):
                    self.reward_captured[block] = True
                    _reward = _reward + self.reward_sep

            if all(v == True for v in self.reward_captured.values()):
                self.done = True
                _reward = _reward + self.reward_end

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
        if action == 'NO_SEL':
            return
        next_coord = (self.block_pos[block][0] + self.action_map[action][0], self.block_pos[block][1] + self.action_map[action][1])
        if not self.on_grid(next_coord): # move would take you over the edge
            self.block_pos[block] = (self.block_pos[block][0] + self.action_map['NO_REQ'][0], self.block_pos[block][1] + self.action_map['NO_REQ'][1])
            self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = block   
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
                num_nonzero_neighbors = occupants.index(0)-1 # [0 1 2 0] and 1,E gives 1 # [1 2 3 0] and 1, E gives 2
                for i in range(num_nonzero_neighbors, -1, -1):
                    # furthest block away
                    _block = occupants[i]
                    self.move(_block, action) # recursion!
                self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = 0
                self.block_pos[block] = (self.block_pos[block][0] + self.action_map[action][0], self.block_pos[block][1] + self.action_map[action][1])
                self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = block                   
            else: # gridlock, can't move at all
                self.block_pos[block] = (self.block_pos[block][0] + self.action_map['NO_REQ'][0], self.block_pos[block][1] + self.action_map['NO_REQ'][1])
                self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = block   

    def print_grid(self):
        print self.grid_state

    def act(self, block, action):
        if ((self.move_count + 1) <= self.max_num_moves) and (not action == 'I_TERM'):
            self.move_count = self.move_count + 1
            self.move(block, action)
        else:
            self.done = True
        # print "Action: ", action, " on block ", block
        # self.print_grid()
        
        return self.grid_state, self.block_pos, self.calc_reward(action), self.done

if __name__ == "__main__":
    env = Env()
    env.reset_env()
    max_num_episodes = 10000
    avail_actions = [k for k in env.action_map.keys() if not k=='NO_REQ']
    avail_actions.append('I_TERM')
    reward_array = []
    episode = 0
    while episode < max_num_episodes:
        done = False
        reward_total = 0
        # print "\nNew episode!"
        while not done:
            state, block_pos, reward, done = env.act(np.random.choice(env.num_blocks)+1, np.random.choice(avail_actions))
            # print reward
            reward_total = reward_total + reward
        episode = episode + 1
        env.reset_env()
        reward_array.append(reward_total)
    print np.mean(reward_array)