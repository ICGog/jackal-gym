#!/usr/bin/env python

import gym
import numpy
import random
import rospy
import rospkg
import sys
import time

import jackal_env
import qlearning


def main(argv):
    rospy.init_node('jackal_gym', anonymous=True)
    env = gym.make('jackal-v0')

    rospack = rospkg.RosPack()
    training_dir = rospack.get_path('jackal-gym') + '/training_results'
    env = gym.wrappers.Monitor(env, training_dir, force=True)

    alpha = rospy.get_param('/alpha')
    epsilon = rospy.get_param('/epsilon')
    gamma = rospy.get_param('/gamma')
    epsilon_discount = rospy.get_param('/epsilon_discount')
    num_episodes = rospy.get_param('/num_episodes')
    num_steps = rospy.get_param('/num_steps')

    last_time_steps = numpy.ndarray(0)

    q_learning = qlearning.QLearning(range(env.action_space.n), epsilon, alpha, gamma)

    highest_reward = 0

    for episode in range(num_episodes):
        rospy.loginfo('Starting episode %s' % (episode))

        cumulated_reward = 0
        done = False
        if q_learning.epsilon > 0.05:
            q_learning.epsilon *= epsilon_discount

        observation = env.reset()
        state = ''.join(map(str, observation))

        for step in range(num_steps):
            rospy.loginfo('Step %d' % (step))
            action = q_learning.choose_action(state)
            observation, reward, done, info = env.step(action)
            cumulated_reward += reward
            highest_reward = max(highest_reward, cumulated_reward)
            next_state = ''.join(map(str, observation))

            q_learning.learn(state, action, reward, next_state)

            if done is True:
                last_time_steps = numpy.append(last_time_steps, [int(step + 1)])
                break
            else:
                state = next_state

        rospy.loginfo('Finished episode ' + str(episode))

    time_steps = last_time_steps.tolist()
    time_steps.sort()
    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 10 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-10:]) / len(l[-10:])))

    env.close()


if __name__ == '__main__':
    main(sys.argv)
