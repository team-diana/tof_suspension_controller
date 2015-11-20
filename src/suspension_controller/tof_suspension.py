# This file is released under the MIT license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2015, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.
#

## TODO merge this code with the other suspension based on range finder from Mattia's work

import numpy as np
import random
from contextlib import contextmanager

import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
import heightmap.srv

import constants

class ToFSuspensionController:
    def __init__(self):
        rospy.init_node('tof_suspension_controller', anonymous=True, log_level=rospy.DEBUG)
        rospy.on_shutdown(self.unregister)

        self.wheel1_cmd_pub = rospy.Publisher('/suspension1/state', Float64, queue_size=100)
        self.wheel2_cmd_pub = rospy.Publisher('/suspension2/state', Float64, queue_size=100)
        self.wheel3_cmd_pub = rospy.Publisher('/suspension3/state', Float64, queue_size=100)
        self.wheel4_cmd_pub = rospy.Publisher('/suspension4/state', Float64, queue_size=100)

        rospy.loginfo("Waiting for service 'heightmap_query'...")
        rospy.wait_for_service('heightmap')
        self.heightmap_query_srv = rospy.ServiceProxy('heightmap', heightmap.srv.Query)

        rospy.loginfo("Started")

    def unregister(self):
        self.wheel1_cmd_pub.unregister()
        self.wheel2_cmd_pub.unregister()
        self.wheel3_cmd_pub.unregister()
        self.wheel4_cmd_pub.unregister()

    ## XXX: check these numbers with the rover
    WHEEL_NAMES = ['f_r', 'f_l', 'b_r', 'b_l']

    def get_heightmap(self, wheel_id):
        wheel_name = self.WHEEL_NAMES[wheel_id]

        corner = PointStamped()
        corner.point.x = 0
        corner.point.y = 0
        corner.header.frame_id = 'rover_amalia_leg_wheel_' + wheel_name

        rospy.logdebug("Getting heightmap for wheel: #%d [%s] => TF frame name: %s",
                       wheel_id, wheel_name, corner.header.frame_id)

        # Note we use x_size=1 because we use a strip as the rover
        # cannot move sideways directly y_samples chosen arbitrarily
        res = self.heightmap_query_srv(corner=corner,
                                       x_size=1,
                                       y_size=constants.ARM_LENGTH,
                                       x_samples=1,
                                       y_samples=1000)
        return np.array(res.map).reshape(res.y_samples, res.x_samples)

    def get_heightmaps(self):
        return [self.get_heightmap(i) for i in (1, 2, 3, 4)]

    def _find_theta(self, height, hmap, eps=0.05):
        thetas = np.linspace(0.1, np.pi/2, 1000)
        dx = 1
        # dx = np.abs(hmap_x[1] - hmap_x[0])
        for th in thetas:
            # compute the wheel x position
            x = constants.ARM_LENGTH * np.sin(th)
            i = int(np.floor(x / dx))
            # compute the wheel height from the requested height
            y = self.req_height - constants.ARM_LENGTH * np.cos(th)
            # distance from terrain:
            e = np.abs(hmap[i] - y)
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
                    rospy.logdebug("Solution for height {} is {} - eps: {} ".format(self.req_height, th, e))
                    rospy.logdebug("x is {}  y is {} - i : {} ".format(x, y, i))
                    return th
        # no solution found for the given height
        return -1

    def find_legs_solution(self, heightmaps, eps=0.05):
        """find a solution and publish the result for all the four wheels"""
        min_height = [np.min(hmap) for hmap in heightmaps]
        min_height = min(min_height)
        max_height =  [np.max(hmap) for hmap in heightmaps]
        max_height = max(max_height)
        max_height = max(max_height, constants.ARM_LENGTH)
        heights = np.linspace(min_height, np.max(hmap[1])+constants.ARM_LENGTH, 100)
        heights = heights[::-1]
        thetas = []
        for height in heights:
            # test if it is possible to find a valid angle for every wheel
            thetas = np.array([self._find_theta(height, hmap, eps) for hmap in heightmaps])
            if np.alltrue(thetas > 0):
                # a solution for all the four wheel was found, so we can set these angles
                self.wheel1_cmd_pub(thetas[0])
                self.wheel2_cmd_pub(thetas[1])
                self.wheel3_cmd_pub(thetas[2])
                self.wheel4_cmd_pub(thetas[3])
                return True
        return False


@contextmanager
def measure_time(task_description):
    init_time = rospy.get_rostime()
    yield
    end_time = rospy.get_rostime()
    elapsed_time = later - now

    rospy.loginfo("Time elapsed to {}: {} secs"
                  .format(task_description, elapsed_time.secs))


## TODO add solution for 4 arms
## TODO Catch and log exceptions?
def main():
    controller = ToFSuspensionController()
    rate = rospy.Rate(40).sleep()

    try:
        while not rospy.is_shutdown():
            with measure_time("get the heightmaps"):
                heightmaps = controller.get_heightmaps()

            with measure_time("find a solution"):
                controller.find_legs_solution(heightmaps)

            rate.sleep()

    except rospy.ROSInterruptException:
        # Usually thrown by the user terminating the process (e.g. by
        # pressing C-c)
        pass

if __name__ == '__main__':
    main()
