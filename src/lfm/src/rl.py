import numpy as np

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
        self.block_pos = {1: (1, 1), 2: (1, 2), 3: (2, 1), 4: (2, 2)}
        self.move_count = 0
        self.action_map = {     'N':  (-1,  0), # (r, c)
                                'NW': (-1, -1),
                                'W':  ( 0, -1),
                                'SW': ( 1, -1),
                                'S':  ( 1,  0),
                                'SE': ( 1,  1),
                                'E':  ( 0,  1),
                                'NE': (-1,  1),
                                'NO_SEL': ( 0,  0),
                                'NO_REQ': ( 0,  0)}
        self.reward_captured = {1: False, 2: False, 3: False, 4: False}
        self.done = False

    def reset_env(self):
        self.grid_state = np.zeros((self.grid_edges, self.grid_edges))
        for block, pos in self.block_pos.items():
            self.grid_state[pos[0]][pos[1]] = block

        self.move_count = 0
        # self.reward_total = 0        
        pass

    def calc_reward(self, action):
        reward = 0
        if action == 'NO_SEL':
            reward = reward + self.reward_noop_sel
        else:
            reward = reward + self.reward_per_move
        for block in self.block_pos:
            if not self.reward_captured[block]:
                free = []
                for action in self.action_map.values():
                    _next = (self.block_pos[block][0] + action[0], self.block_pos[block][1] + action[1])
                    if self.on_grid(_next) and self.grid_state[_next[0]][_next[1]] == 0:
                        free.append(True)
                    elif not self.on_grid(_next):
                        free.append(True)
                if all(v == True for v in free):
                    self.reward_captured[block] = True
                    reward = reward + self.reward_sep
            
        if all(v == True for v in self.reward_captured):
            self.done = True
            reward = reward + self.reward_end

        return reward

    def on_grid(self, coord):
        row = coord[0]
        col = coord[1]
        # print coord
        if (row < 0 or 
            row >= self.grid_edges or
            col < 0 or 
            col >= self.grid_edges):
            return False
        else:
            return True

    def move(self, block, action):
        next_coord = (self.block_pos[block][0] + self.action_map[action][0], self.block_pos[block][1] + self.action_map[action][1])
        if not self.on_grid(next_coord): # move would take you over the edge
            self.block_pos[block] = (self.block_pos[block][0] + self.action_map['NO_REQ'][0], self.block_pos[block][1] + self.action_map['NO_REQ'][1])
            self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = block   
        elif (self.on_grid(next_coord) and self.grid_state[next_coord[0]][next_coord[1]] == 0): # move is on board and neighbor is free
            self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = 0
            self.block_pos[block] = (self.block_pos[block][0] + self.action_map[action][0], self.block_pos[block][1] + self.action_map[action][1])
            # print self.block_pos[block]
            self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = block   
        else: # move is on board, and neighbor is occupied
            # how many neighbors in a row?
            occupants = []
            while self.on_grid(next_coord):
                occupant = int(self.grid_state[next_coord[0]][next_coord[1]])
                # print int(occupant)
                occupants.append(occupant)
                next_coord = (next_coord[0] + self.action_map[action][0], next_coord[1] + self.action_map[action][1])
            # print occupants
            # https://stackoverflow.com/questions/6039425/sequence-find-function-in-python
            if 0 in occupants:
                num_nonzero_neighbors = occupants.index(0)-1 # [0 1 2 0] and 1,E gives 1 # [1 2 3 0] and 1, E gives 2
                # print num_nonzero_neighbors
                for i in range(num_nonzero_neighbors, -1, -1):
                    # furthest block away
                    _block = occupants[i]
                    # print _block
                    self.move(_block, action) # recursion!
                self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = 0
                self.block_pos[block] = (self.block_pos[block][0] + self.action_map[action][0], self.block_pos[block][1] + self.action_map[action][1])
                self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = block                   
            else: # gridlock, can't move at all
                self.block_pos[block] = (self.block_pos[block][0] + self.action_map['NO_REQ'][0], self.block_pos[block][1] + self.action_map['NO_REQ'][1])
                self.grid_state[self.block_pos[block][0]][self.block_pos[block][1]] = block   

    def episode_done(self):
        return self.done or self.move_count >= self.max_num_moves

    def print_grid(self):
        print self.grid_state

    def act(self, block, action):
        self.move_count = self.move_count + 1
        self.move(block, action)
        self.print_grid()
        
        return self.grid_state, self.calc_reward(action), self.episode_done()

if __name__ == "__main__":
    env = Env()
    env.reset_env()
    block = 1#int(raw_input())
    action = 'E'#raw_input()
    env.act(block, action)