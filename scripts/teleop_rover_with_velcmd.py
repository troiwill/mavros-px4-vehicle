#!/usr/bin/env python

import math
import rospy

from geometry_msgs.msg import Twist

from mavros_px4_vehicle.px4_modes import PX4_MODE_OFFBOARD, PX4_MODE_LOITER
from mavros_px4_vehicle.px4_offboard_modes import SetVelocityCmdBuilder
from mavros_px4_vehicle.px4_vehicle import PX4Vehicle

def new_velocity_callback(vel_cmd):
    rover.set_velocity(vel_cmd)
    rospy.sleep(5)
    rover.set_velocity(SetVelocityCmdBuilder.build())

if __name__ == "__main__":
    # Connect to rover
    rospy.init_node("mavros_px4_rover")
    rospy.loginfo("Connecting to the rover.")
    rover = PX4Vehicle(auto_connect = True)

    # Arm rover
    rover.arm()
    rover.wait_for_status(rover.is_armed, True, 2)
    rover.set_velocity(SetVelocityCmdBuilder.build()) # Set velocity to 0
    rospy.sleep(2.)

    # Change to offboard mode so velocity commands can be sent
    rospy.loginfo("Changing to offboard mode.")
    rover.set_mode(PX4_MODE_OFFBOARD)

    # Create the subscriber
    rospy.Subscriber("vel_cmds", Twist, new_velocity_callback)
    
    rospy.spin()