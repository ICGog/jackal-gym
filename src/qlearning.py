'''
Inspired from: https://bitbucket.org/theconstructcore/drone_training/src/master/
'''

import random

class QLearning:

    def __init__(self, actions, epsilon, alpha, gamma):
        self.q = {}             # Stores (state, action)-> q mappings
        self.epsilon = epsilon  # exploration constant
        self.alpha = alpha      # Learning rate
        self.gamma = gamma      # Discount factor
        self.actions = actions  # Available actions

    def get_q(self, state, action):
        return self.q.get((state, action), 0.0)

    def learnQ(self, state, action, reward, value):
        '''
        Q-learning:
            Q(s, a) = (1 - alpha) * Q(s, a) + alpha * value(s, a)
        '''
        old_quality = self.q.get((state, action), None)
        if old_quality is None:
            self.q[(state, action)] = reward
        else:
            self.q[(state, action)] = (1 - self.alpha) * old_quality + self.alpha * value

    def choose_action(self, state, return_q=False):
        qs = [self.get_q(state, action) for action in self.actions]
        max_q = max(qs)

        if random.random() < self.epsilon:
            mag = max(abs(min(qs)), abs(max_q))
            # Add random values to all action
            qs = [qs[i] + random.random() * mag - 0.5 * mag for i in range(len(self.actions))]
            max_q = max(qs)

        if qs.count(max_q) > 1:
            max_q_indices = [i for i in range(len(self.actions)) if qs[i] == max_q]
            index = random.choice(max_q_indices)
        else:
            index = qs.index(max_q)

        action = self.actions[index]
        if return_q:
            return action, q
        return action

    def learn(self, state, action, reward, next_state):
        max_next_q = max([self.get_q(next_state, action) for action in self.actions])
        self.learnQ(state, action, reward, reward + self.gamma * max_next_q)
