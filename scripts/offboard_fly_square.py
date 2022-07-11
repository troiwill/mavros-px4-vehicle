#!/usr/bin/env python

import math
import rospy

from mavros_px4_vehicle.px4_modes import PX4_MODE_OFFBOARD, PX4_MODE_LOITER
from mavros_px4_vehicle.px4_offboard_modes import SetPositionWithYawCmdBuilder
from mavros_px4_vehicle.px4_vehicle import PX4Vehicle


def offboard_fly_square():
    # Create an instance and arm the drone.
    rospy.loginfo("Connecting to the drone.")
    drone = PX4Vehicle(auto_connect = True)
    drone.arm()
    drone.wait_for_status(drone.is_armed, True, 2)

    # Make the drone take off.
    drone.takeoff(block=True)

    # Create the forward and turning velocity commands.
    # zero_cmd = SetRawCmdBuilder.build()
    fwd_cmd = SetRawCmdBuilder.build(vx = 0.)
    # right_turn_cmd = SetRawCmdBuilder.build(yawrate = math.radians(15.))
    # drone.set_velocity(zero_cmd)
    rospy.sleep(2.)
    rospy.loginfo("Changing to offboard mode.")
    drone.set_mode(PX4_MODE_OFFBOARD)

    count = 0
    while count < 4:
        rospy.loginfo("Making the drone fly forward.")
        drone.set_posvelacc(fwd_cmd)
        rospy.sleep(5.)

        rospy.loginfo("Making the drone turn right #{}.\n".format(count + 1))
        # drone.set_velocity(zero_cmd)
        # rospy.sleep(2.)
        # drone.set_posvelacc(right_turn_cmd)
        rospy.sleep(6.)
        count = count + 1

    # "Stop" the vehicle.
    # drone.set_posvelacc(zero_cmd)
    rospy.sleep(2.)

    # Make the vehicle "hold."
    rospy.loginfo("Changing to loiter mode.")
    drone.set_mode(PX4_MODE_LOITER)
    rospy.sleep(4.)

    # Make the drone land.
    rospy.loginfo("Making the drone fly.")
    drone.land(block=True)

    rospy.spin()
#end def

if __name__ == "__main__":
    try:
        rospy.init_node("offboard_fly_square")
        offboard_fly_square()

    except:
        pass
