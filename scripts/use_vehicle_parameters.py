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

    rospy.loginfo("Sending pose2d command.")
    cmd = SetPositionWithYawCmdBuilder.build(x = 2., z = 1.)
    copter.set_pose2d(cmd)
    rospy.sleep(2)
    rospy.loginfo("Changing to offboard mode.")
    copter.set_mode(PX4_MODE_OFFBOARD)

    rospy.loginfo("Changing to hover mode.")
    copter.set_mode(PX4_MODE_LOITER)

    copter.land(block=True)
    copter.disarm()
    copter.disconnect()

    

# def offboard_hover():
#     # Create a copter instance and arm the copter.
#     rospy.loginfo("Connecting to the vehicle.")
#     copter = PX4Vehicle(auto_connect = True)
#     copter.arm()
#     copter.wait_for_status(copter.is_armed, True, 2)

#     # Request the copter to hover using local position commands.
#     rospy.loginfo("Sending the set position commands.")
#     cmd = SetPositionWithYawCmdBuilder.build(z = 2.)
#     copter.set_pose2d(cmd)
#     rospy.sleep(2)

#     rospy.loginfo("Changing to offboard mode.")
#     copter.set_mode(PX4_MODE_OFFBOARD)
#     rospy.sleep(10.)

#     rospy.loginfo("Changing to hover mode.")
#     copter.set_mode(PX4_MODE_LOITER)
    
# #end def

if __name__ == "__main__":
    rospy.init_node("use_vehicle_parameters")
    use_vehicle_parameters()
    rospy.spin()
