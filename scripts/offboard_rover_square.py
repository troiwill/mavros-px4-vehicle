#!/usr/bin/env python

import math
import rospy

from mavros_px4_vehicle.px4_modes import PX4_MODE_OFFBOARD, PX4_MODE_LOITER
from mavros_px4_vehicle.px4_offboard_modes import SetVelocityCmdBuilder
from mavros_px4_vehicle.px4_vehicle import PX4Vehicle


def offboard_drive_square():
    # Create an instance and arm the rover.
    rospy.loginfo("Connecting to the rover.")
    rover = PX4Vehicle(auto_connect = True)
    rover.arm()
    rover.wait_for_status(rover.is_armed, True, 2)

    # Create the forward and turning velocity commands.
    zero_cmd = SetVelocityCmdBuilder.build()
    fwd_cmd = SetVelocityCmdBuilder.build(vz = 1.)
    right_turn_cmd = SetVelocityCmdBuilder.build(vz = 1., vx = math.radians(15.))
    rover.set_velocity(zero_cmd)
    rospy.sleep(2.)
    rospy.loginfo("Changing to offboard mode.")
    rover.set_mode(PX4_MODE_OFFBOARD)

    count = 0
    while count < 4:
        rospy.loginfo("Making the rover drive forward.")
        rover.set_velocity(fwd_cmd)
        rospy.sleep(5.)

        rospy.loginfo("Making the rover turn right #{}.\n".format(count + 1))
        rover.set_velocity(right_turn_cmd)
        rospy.sleep(6.)
        count = count + 1

    # "Stop" the vehicle.
    rover.set_velocity(zero_cmd)
    rospy.sleep(2.)

    # Make the vehicle "hold."
    rospy.loginfo("Changing to loiter mode.")
    rover.set_mode(PX4_MODE_LOITER)
    rospy.spin()
#end def

if __name__ == "__main__":
    rospy.init_node("offboard_drive_square")
    offboard_drive_square()
