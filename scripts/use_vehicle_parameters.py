#!/usr/bin/env python

import rospy

from mavros_px4_vehicle.px4_modes import PX4_MODE_LOITER, PX4_MODE_OFFBOARD
from mavros_px4_vehicle.px4_offboard_modes import SetPositionWithYawCmdBuilder
from mavros_px4_vehicle.px4_vehicle import PX4Vehicle


def use_vehicle_parameters():
    # Create a copter instance.
    rospy.loginfo("Connecting to the vehicle.")
    copter = PX4Vehicle(auto_connect = True)

    # Set the parameters for slow and low takeoff.
    copter.set_param("MIS_TAKEOFF_ALT", 1.0)
    copter.set_param("MPC_XY_CRUISE", 3.5)

    # Arm the copter.
    rospy.loginfo("Copter is arming.")
    copter.arm()
    copter.wait_for_status(copter.is_armed, True, 2)

    # Make the vehicle takeoff.
    rospy.loginfo("Copter is taking off.")
    copter.takeoff(block=True)
    copter.wait_for_status(copter.in_hovering_mode, True, 2)
    copter.sleep(2.)

    # Send the pose command to the drone. If the set parameter works
    # correctly, the drone should remain at its current altitude and 
    # move 2 meters forward.
    rospy.loginfo("Sending pose2d command.")
    cmd = SetPositionWithYawCmdBuilder.build(x = 2., z = 1.)
    copter.set_pose2d(cmd)
    copter.sleep(1.)
    rospy.loginfo("Changing to offboard mode.")
    copter.set_mode(PX4_MODE_OFFBOARD)
    copter.sleep(3.)

    # Land the vehicle.
    rospy.loginfo("Landing the copter.")
    copter.land(block=True)
    if copter.is_armed():
        copter.disarm()
    copter.disconnect()
#end def

if __name__ == "__main__":
    rospy.init_node("use_vehicle_parameters")
    use_vehicle_parameters()
    rospy.spin()
