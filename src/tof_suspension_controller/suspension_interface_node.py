#!/usr/bin/env python

# This file is released under the MIT license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2015, Vincenzo Comito
# All rights reserved.
#

import rospy
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

from constants import GEAR_FACTOR, QUEUE_SIZE

class SuspensionMotorInteface:
    def __init__(self, motor_id, direction, name):
        self.motor_id = motor_id
        self.direction = direction
        self.state_sub = rospy.Subscriber("/suspension_motor{}/state".format(motor_id), JointState, self.motor_state_callback)
        self.state_pub = rospy.Publisher('/suspension{}/state'.format(motor_id), Float64, queue_size=QUEUE_SIZE)
        self.command_sub = rospy.Subscriber("/suspension{}/command".format(motor_id), Float64, self.command_callback)
        self.command_pub = rospy.Publisher('/suspension_motor{}/command'.format(motor_id), Float64, queue_size=QUEUE_SIZE)
        self.suspension_command_pub = rospy.Publisher("/suspension{}/command".format(motor_id), Float64, queue_size=QUEUE_SIZE)
        self.name = name

    def unregister(self):
        self.state_sub.unregister()
        self.state_pub.unregister()
        self.command_sub.unregister()
        self.command_pub.unregister()
        self.suspension_command_pub.unregister()

    def motor_state_callback(self, data):
        motor_angle = data.current_pos
        suspension_angle = motor_angle/GEAR_FACTOR * self.direction
        self.state_pub.publish(suspension_angle)
    
    def command_callback(self, data):
        suspension_angle = data.data
        motor_angle = suspension_angle * GEAR_FACTOR * self.direction
        self.command_pub.publish(motor_angle)

    def suspension_command(self, suspension_angle):
        self.suspension_command_pub.publish(suspension_angle)

## XXX: check the names of the wheels
class SuspensionInteface:
    def __init__(self):
        rospy.init_node('amalia_suspension_inteface', anonymous=True)
        rospy.on_shutdown(self.unregister)
        self.motor_interfaces = []
        self.motor_interfaces.append(SuspensionMotorInteface(1, 1, 'f_r'))
        self.motor_interfaces.append(SuspensionMotorInteface(2, -1, 'f_l'))
        self.motor_interfaces.append(SuspensionMotorInteface(3, 1, 'b_l'))
        self.motor_interfaces.append(SuspensionMotorInteface(4, -1, 'b_r'))
    
    def start(self):
        rospy.spin()

    def unregister(self):
        motor.unregister() for motor in self.motor_interfaces

if __name__ == '__main__':
    interface = SuspensionInteface()
    interface.start()