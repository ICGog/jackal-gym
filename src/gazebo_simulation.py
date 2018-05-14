#!/usr/bin/env python

import logging
import rospy
from std_srvs.srv import Empty

class GazeboSimulation():

    def __init__(self):
        self._pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self._unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self._reset = rospy.ServiceProxy('/gazebo/reset_world', Empty)

    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self._pause()
        except rospy.ServiceException, e:
            logging.error('Could not pause simulation: %s', e)

    def resume(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self._unpause()
        except rospy.ServiceException, e:
            logging.error('Could not resume simulation: %s', e)

    def reset(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self._reset()
        except rospy.ServiceException, e:
            logging.error('Could not reset simulation: %s', e)
