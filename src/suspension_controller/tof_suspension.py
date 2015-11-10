#!/usr/bin/env python3

# This file is released under the MIT license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2015, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.
#

## TODO merge this code with the other suspension based on range finder from Mattia's work

import noise
from scipy.interpolate import UnivariateSpline as spline
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as interpolate
from mpl_toolkits.mplot3d import axes3d, Axes3D
import matplotlib.patches as patches
import random

%matplotlib inline

import rospy
#import tf

from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from heightmap.srv import Query

from suspension_controller.constants import constants

class ToFSuspensionController:
    def __init__(self):
        rpspy.init_node('tof_suspension_controller', anonymous=True, log_level=rospy.DEBUG)
        rospy.on_shutdown(self.__unpublish)

        self.__publish()

        rospy.loginfo("TOF SUSPENSION CONTROLLER")

    def __publish(self):
        self.wheel1_cmd_pub = rospy.Publisher('/suspension1/state', Float64)
        self.wheel2_cmd_pub = rospy.Publisher('/suspension2/state', Float64)
        self.wheel3_cmd_pub = rospy.Publisher('/suspension3/state', Float64)
        self.wheel4_cmd_pub = rospy.Publisher('/suspension4/state', Float64)

        rospy.wait_for_service('heightmap_query')
        self.heightmap_query_srv = rospy.ServiceProxy('heightmap_query', Query)

        #        self.tf_listener = tf.TransformListener()
        #        self.tf_broadcaster = tf.TransformBroadcaster()

    def __unpublish(self):
        #        self.heightmap_sub.unregister()
        self.wheel1_cmd_pub.unregister()
        self.wheel2_cmd_pub.unregister()
        self.wheel3_cmd_pub.unregister()
        self.wheel4_cmd_pub.unregister()

    def get_heightmap(self, x, y):
        corner = PointStamped()
        corner.point.x = x
        corner.point.y = y
        
        res = heightmap_query_srv(corner=corner)
        self.sizex =  res.x_samples
        self.sizey = res.y_samples
        self.heightmap = np.array(res.map, dtype=np.float64)

    def find_theta(self, height, hmap_x, hmap_y, eps=0.05):
        thetas = np.linspace(0.1, np.pi/2, 1000)
        dx = np.abs(hmap_x[1] - hmap_x[0])
        for th in thetas:
            # compute the wheel x position
            x = constants.ARM_LENGTH * np.sin(th)
            i = int(np.floor(x / dx))
            # compute the wheel height from the requested height
            y = self.req_height - constants.ARM_LENGTH * np.cos(th)
            # distance from terrain:
            e = np.abs(hmap_y[i] - y)
            if e < eps:
                # ok, we have found a good angle. But are the
                # constraint satisfied?
                testx = np.arange(0, i)*dx
                cotan_th = 1/np.tan(th)
                # test if the arm is always above the terrain surface
                A = (- testx*cotan_th + height)
                B = hmap_y[0:i]
                tests =  A  > B
                if np.alltrue(tests):
                    # the arm does not collide with the terrain, return the angle as solution
                    #print("Solution for height {} is {} - eps: {} ".format(self.req_height, th, e))
                    #print("x is {}  y is {} - i : {} ".format(x, y, i))
                    return th
        # no solution found for the given height
        return -1

    def find_legs_solution(self, heightmaps, eps=0.05):
        """find a solution and publish the result for all the four wheels"""
        min_height = [np.min(hmap[1]) for hmap in heightmaps]
        min_height = min(min_height)
        max_height =  [np.max(hmap[1]) for hmap in heightmaps]
        max_height = max(max_height)
        max_height = max(max_height, constants.ARM_LENGTH)
        heights = np.linspace(min_height, np.max(hmap[1])+constants.ARM_LENGTH, 100)
        heights = heights[::-1]
        thetas = []
        for height in heights:
            # test if it is possible to find a valid angle for every wheel
            thetas = np.array([self.find_theta(height, hmap[0], hmap[1], eps) for hmap in heightmaps])
            if np.alltrue(thetas > 0):
                # a solution for all the four wheel was found, so we can set these angles
                self.wheel1_cmd_pub(thetas[0])
                self.wheel2_cmd_pub(thetas[1])
                self.wheel3_cmd_pub(thetas[2])
                self.wheel4_cmd_pub(thetas[3])
                return True
        return False

class debug_draw:
    def __init__(self, model):
        self.model = model
    
    def draw_rect(ax, x, y, w, h):
        ax.add_patch(patches.Rectangle(
                                       (x, y),     # (x,y)
                                       w,          # width
                                       h,          # height
                                       fill=False
                                       )
                     )
    def draw_heightmap(ax, x, y):
        dx = np.abs(x[1] - x[0])
        for i in range(0, len(x)):
            top_n = np.floor(y[i]/dx)
            for n in range(0, int(top_n)):
                #for n in range(0, 4):
                self.draw_rect(ax, dx*i, n*dx, dx, dx)
            self.draw_rect(ax, dx*i, top_n*dx, dx, (y[i]-top_n*dx))
    
    def draw_leg(self, height, theta):
        """ draw a leg on the 2D terrain """
        end_point = (np.sin(theta)*constants.ARM_LENGTH, height-constants.ARM_LENGTH*np.cos(theta))
        plt.plot(end_point[0], end_point[1], 'ro', markersize=3)
        plt.plot([0, end_point[0]], [height, end_point[1]] , 'r-', markersize=3)


    def draw_legs_solution(self, heightmaps, eps=0.05):
        """ find a solution and draw the heightmap and the legs for all the four wheels"""
        min_height = [np.min(hmap[1]) for hmap in heightmaps]
        min_height = min(min_height)
        max_height =  [np.max(hmap[1]) for hmap in heightmaps]
        max_height = max(max_height)
        max_height = max(max_height, constants.ARM_LENGTH)
        heights = np.linspace(min_height, np.max(hmap[1])+constants.ARM_LENGTH, 100)
        heights = heights[::-1]
        thetas = []
        sol_found = False
        for height in heights:
            # test if it is possible to find a valid angle for every wheel
            thetas = np.array([self.model.find_theta(height, hmap[0], hmap[1], eps) for hmap in heightmaps])
            if np.alltrue(thetas > 0):
                # a solution for all the four wheel was found
                sol_found = True
                break
        fig = plt.figure(figsize=(10, 10))
        for i in range(0, 4):
            hmap = heightmaps[i]
            ax = fig.add_subplot(2,2, i+1)
            ax.set_title("leg {} theta: {} ".format(i, np.rad2deg(thetas[i])))
            ax.set_ylabel("height (m)")
            ax.axis([0, 0.6, 0, 0.6])
            self.draw_heightmap(ax, hmap[0], hmap[1])
            if sol_found > 0:
                self.draw_leg(height, thetas[i])

#### TODO add solution for 4 arms
if __name__ == '__main__':
    try:
        controller = ToFSuspensionController()
        for arm in range(1, 4):
            pass
        while not rospy.is_shutdown():
            controller.find_theta()
            rospy.Rate(40).sleep()
        
        rospy.spin()
    except rospy.ROSInterruptException: pass

