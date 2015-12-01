#!/usr/bin/env python

# This file is released under the MIT license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2015, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.
#

import numpy as np
import random
from contextlib import contextmanager

import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
import heightmap.srv

import constants

from suspension_interface_node import SuspensionInteface

class ToFSuspensionController:
    def __init__(self):
        rospy.init_node('tof_suspension_controller', anonymous=True, log_level=rospy.DEBUG)

        self.suspension_interface = SuspensionInteface()
        self.suspension_interface.start()

        rospy.loginfo("Waiting for service 'heightmap_query'...")
        rospy.wait_for_service('heightmap')
        self.heightmap_query_srv = rospy.ServiceProxy('heightmap', heightmap.srv.Query)

        rospy.loginfo("Started")

    def get_heightmap(self, tf_frame_id):
        rospy.logdebug("Getting heightmap for tf frame: {}".format(tf_frame_id))

        corner = PointStamped()
        corner.header.frame_id = tf_frame_id
        corner.point.x = 0
        corner.point.y = 0
        corner.header.stamp = rospy.Time(0)
        # Note we use x_size=1 because we use a strip as the rover
        # cannot move sideways directly y_samples chosen arbitrarily
        res = self.heightmap_query_srv(corner=corner,
                                       x_size=1,
                                       y_size=constants.ARM_LENGTH,
                                       x_samples=1,
                                       y_samples=constants.SAMPLES)
        return np.array(res.map).reshape(res.y_samples, res.x_samples)

    def get_heightmaps(self):
        tf_frames = []
        for motor in self.suspension_interface.motor_interfaces:
            tf_frames.append('rover_amalia_leg_wheel_' + motor.name)
        return [self.get_heightmap(tf_frame_id) for tf_frame_id in tf_frames]

    def _find_theta(self, height, hmap, eps=constants.DEFAULT_EPSILON):
        thetas = np.linspace(0.1, np.pi/2, constants.SAMPLES)
        for th in thetas:
            # compute the wheel x position
            x = constants.ARM_LENGTH * np.sin(th)
            i = int(np.floor(x))
            # compute the wheel height from the requested height
            y = height - constants.ARM_LENGTH * np.cos(th)
            # distance from terrain:
            e = np.abs(hmap[i] - y)
            if e < eps:
                # ok, we have found a good angle. But are the
                # constraint satisfied?
                testx = np.arange(0, i)
                cotan_th = 1 / np.tan(th)
                # test if the arm is always above the terrain surface
                A = (-testx * cotan_th + height)
                B = hmap[0:i]
                tests =  A  > B
                if np.alltrue(tests):
                    # the arm does not collide with the terrain, return the angle as solution
                    rospy.logdebug("Solution for height {} is {} - eps: {} ".format(height, th, e))
                    rospy.logdebug("x is {}  y is {} - i : {} ".format(x, y, i))
                    return th
        # no solution found for the given height
        return -1

    def find_legs_solution(self, heightmaps, eps=constants.DEFAULT_EPSILON):
        """find a solution and publish the result for all the four wheels"""
        min_height = [np.min(hmap) for hmap in heightmaps]
        min_height = min(min_height)

        max_height =  [np.max(hmap) for hmap in heightmaps]
        max_height = max(max_height)
        max_height = max(max_height, max_height + constants.ARM_LENGTH)

        heights = np.linspace(min_height, max_height, constants.SAMPLES)
        heights = heights[::-1]
        thetas = []
        for height in heights:
            # test if it is possible to find a valid angle for every wheel
            thetas = np.array([self._find_theta(height, hmap, eps) for hmap in heightmaps])
            if np.alltrue(thetas > 0):
                # a solution for all the four wheel was found, so we can set these angles
                for i, motor in enumerate(self.suspension_interface.motor_interfaces):
                    if thetas[i] == -1: return False
                    motor.suspension_command(thetas[i])
                return True
        return False

# TODO: maybe we should have it in a util, that others can use too?
@contextmanager
def measure_time(task_description):
    init_time = rospy.get_rostime()
    yield
    end_time = rospy.get_rostime()
    elapsed_time = end_time - init_time

    rospy.loginfo("Time elapsed to {}: {} secs".format(task_description, elapsed_time.secs))


## TODO Catch and log exceptions?
if __name__ == '__main__':
    controller = ToFSuspensionController()
    rate = rospy.Rate(40)

    try:
        while not rospy.is_shutdown():
            with measure_time("get the heightmaps"):
                heightmaps = controller.get_heightmaps()

            with measure_time("search for a solution"):
                if not controller.find_legs_solution(heightmaps):
                    rospy.logwarn("Failed to find a feasible solution!")

            rate.sleep()

    except rospy.ROSInterruptException:
        # Usually thrown by the user terminating the process (e.g. by
        # pressing C-c)
        pass