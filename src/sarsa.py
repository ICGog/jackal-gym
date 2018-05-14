import random

class Sarsa:

    def __init__(self, actions, epsilon, alpha, gamma):
        self.q = {}             # Stores (state, action)-> q mappings
        self.epsilon = epsilon  # Exploration constant
        self.alpha = alpha      # Learning rate
        self.gamma = gamma      # Discount factor
        self.actions = actions  # Available actions

    def get_q(self, state, action):
        return self.q.get((state, action), 0.0)

    def learn_q(self, state, action, reward, value):
        old_quality = self.q.get((state, action), None)
        if old_quality is None:
            self.q[(state, action)] = reward
        else:
            self.q[(state, action)] = old_quality + self.alpha * (value - old_quality)

    def choose_action(self, state):
        if random.random() < self.epsilon:
            action = random.choice(self.actions)
        else:
            qs = [self.get_q(state, action) for action in self.actions]
            max_q = max(qs)
            if qs.count(max_q) > 1:
                max_q_indices = [i for i in range(len(self.actions)) if qs[i] == max_q]
                index = random.choice(max_q_indices)
            else:
                index = qs.index(max_q)
            action = self.actions[index]
        return action

    def learn(self, state, action, reward, next_state, next_action):
        next_q = self.get_q(next_state, next_action)
        self.learn_q(state, action, reward, reward + self.gamma * next_q)
