#!/usr/bin/env python

import rospy

from mavros_px4_vehicle.experimental.avoidance import FlightAvoidance
from mavros_px4_vehicle.px4_offboard_modes import SetPositionWithYawCmdBuilder
from mavros_px4_vehicle.px4_modes import PX4_MODE_OFFBOARD
from mavros_px4_vehicle.px4_vehicle import PX4Vehicle


def offboard_avoidance_vertical():
    # Create a copter instance and arm the copter.
    rospy.loginfo("Connecting to the vehicle.")
    copter = PX4Vehicle(auto_connect = True)
    copter.arm()
    copter.wait_for_status(copter.is_armed, True, 2)

    # Request the copter to hover using local position commands.
    rospy.loginfo("Sending the set position commands.")
    cmd = SetPositionWithYawCmdBuilder.build(z = 2.)
    copter.set_pose2d(cmd)
    copter.sleep(2)

    rospy.loginfo("Changing to offboard mode.")
    copter.set_mode(PX4_MODE_OFFBOARD)
    copter.sleep(10.)

    goal = [7., 7., 2., 0.]
    rospy.loginfo("Flying to location {} with yaw {}".format(goal[:3], goal[-1]))
    avoidance = FlightAvoidance(px4_vehicle=copter, min_safe_altitude=5.,
        max_safe_altitude=10., avoid_mode=FlightAvoidance.AVOID_MODE_VERTICAL)
    avoidance.go_to(goal)

    rospy.loginfo("Reached goal.")
    rospy.sleep(4.)

    rospy.loginfo("Landing the copter.")
    copter.land(block=True)
    if copter.is_armed():
        copter.disarm()
    copter.disconnect()
#end def

if __name__ == "__main__":
    rospy.init_node("offboard_avoidance_vertical")
    offboard_avoidance_vertical()
    rospy.spin()
