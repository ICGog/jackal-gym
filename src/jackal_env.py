#!/usr/bin/env python

'''
Inspired from: https://bitbucket.org/theconstructcore/drone_training/src/master/
'''

import gym
import rospy
import numpy as np
import time
import tf

from gazebo_simulation import GazeboSimulation
from geometry_msgs.msg import Twist, Pose
from gym import utils, spaces
from gym.envs.registration import register
from gym.utils import seeding
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Register the environment in gym.
reg = register(
    id='jackal-v0',
    entry_point='jackal_env:JackalEnv',
    timestep_limit=100)

class JackalEnv(gym.Env):

    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.speed = rospy.get_param('/speed')
        # Get goal
        self.desired_pose = Pose()
        self.desired_pose.position.x = rospy.get_param('/desired_pose/x')
        self.desired_pose.position.y = rospy.get_param('/desired_pose/y')
        self.desired_pose.position.z = rospy.get_param('/desired_pose/z')
        # Get training parameters
        self.step_sleep = rospy.get_param("/step_sleep")
        self.max_incl = rospy.get_param("/max_incl")
        # Start simulator
        self.simulator = GazeboSimulation()

        self.action_space = spaces.Discrete(4)
        self.reward_range = (-np.inf, np.inf)

        self._seed()


    def reset(self):
        # TODO(ionel): Fix! The robot stops receiving commands if we reset the
        # simulator.
        #self.simulator.reset()
        self.simulator.resume()

        rate = rospy.Rate(10)
        while self.cmd_vel_pub.get_num_connections() == 0:
            rate.sleep()

        # Reset Jackal to initial state.
        current_init_pose, imu = self.take_observation()
        self._smallest_dist = self._distance(current_init_pose.position,
                                             self.desired_pose.position)

        vel_cmd = Twist()
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(vel_cmd)
        time.sleep(self.step_sleep)

        pose, imu = self.take_observation()
        state = [pose.position.x]

        self.simulator.pause()

        return state


    def step(self, action):
        rospy.loginfo('Stepping ' + str(action))
        vel_cmd = Twist()
        if action == 0:
            # Move forward
            vel_cmd.linear.x = self.speed
            vel_cmd.angular.z = 0.0
        elif action == 1:
            # Move backwards
            vel_cmd.linear.x = -self.speed
            vel_cmd.angular.z = 0.0
        elif action == 2:
            # Move Left
            vel_cmd.linear.x = 0.05
            vel_cmd.angular.z = self.speed
        elif action == 3:
            # Move right
            vel_cmd.linear.x = 0.05
            vel_cmd.angular.z = -self.speed
        else:
            print('Unknown action type')

        self.simulator.resume()
        self.cmd_vel_pub.publish(vel_cmd)
        time.sleep(self.step_sleep)
        pose, imu = self.take_observation()
        self.simulator.pause()

        reward, done = self._process_data(pose, imu)

        if action == 0:
            reward += 50
        elif action == 1 or action == 2:
            reward += 10
        else:
            reward -= 30

        state = [pose.position.x]

        return state, reward, done, {}


    def take_observation(self):
        odometry = rospy.wait_for_message('/odometry/filtered', Odometry)
        pose = odometry.pose.pose
        imu = rospy.wait_for_message('/imu/data', Imu)
        return pose, imu


    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]


    def _distance(self, start_pos, end_pos):
        start_array = np.array((start_pos.x, start_pos.y, start_pos.z))
        end_array = np.array((end_pos.x, end_pos.y, end_pos.z))
        return np.linalg.norm(start_array - end_array)

    def _distance_reward(self, pose):
        current_dist = self._distance(pose.position, self.desired_pose.position)
        reward = 0
        if current_dist < self._smallest_dist:
            reward = 50
            self._smallest_dist = current_dist
        elif current_dist > self._smallest_dist:
            reward = -50
        return reward


    def _process_data(self, pose, imu):
        done = False
        euler = tf.transformations.euler_from_quaternion([imu.orientation.x,
                                                          imu.orientation.y,
                                                          imu.orientation.z,
                                                          imu.orientation.w])

        roll = euler[0]
        pitch = euler[1]

        if roll < -self.max_incl or roll > self.max_incl or pitch < -self.max_incl or pitch > self.max_incl:
            done = True
            reward = -100
        else:
            reward = self._distance_reward(pose)

        return reward, done
