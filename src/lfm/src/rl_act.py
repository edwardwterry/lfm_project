# -*- coding: utf-8 -*-
#! usr/bin/env python3
import rospy
import numpy as np
from keras.models import Sequential
from keras.layers import Dense

class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.model = self._build_model()
        # self.action_map = ((1, 'N'), (1, 'W'), (1, 'S'), (1, 'E'),
        #                    (2, 'N'), (2, 'W'), (2, 'S'), (2, 'E'), (1, 'NO_SEL'))
        self.action_map = ((1, 'W'), (1, 'E'), (2, 'W'), (2, 'E'), (1, 'NO_SEL'))
        self.state_sub = rospy.Subscriber("/grid_state", Int32MultiArray, self.update_grid_state_clbk)
        self.grid_state = np.zeros((4,4)).ravel()
        self.ready_to_act = False
        self.ready_to_publish = False
        self.best_action_msg = Int32()

    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        model = Sequential()
        model.add(Dense(50, input_dim=self.state_size, activation='relu'))
        model.add(Dense(30, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss='mse',
                      optimizer=Adam(lr=self.learning_rate))
        return model

    def act(self, state):
        act_values = self.model.predict(self.grid_state)
        self.ready_to_act = False
        self.ready_to_publish = True
        self.best_action_msg.data = np.argmax(act_values[0])

    def load(self, name):
        self.model.load_weights(name + '.h5')

    def update_grid_state_clbk(self, msg):
        self.grid_state = np.array(msg.data).reshape((4,4)).ravel()
        self.ready_to_act = True

action_pub = rospy.Publisher("/nn_action", Int32)

if __name__ == "__main__":
    state_size = 16
    action_size = 5 # L, R x 2, + no_op
    agent = DQNAgent(state_size, action_size)

    in_file = "TODO"
    agent.load(in_file)
    while not rospy.is_shutdown():
        if agent.ready_to_act:
            agent.act()
            agent.ready_to_act = False
        if agent.ready_to_publish:
            # wait for sub
            action_pub.publish(agent.best_action_msg)
            agent.ready_to_publish = False
