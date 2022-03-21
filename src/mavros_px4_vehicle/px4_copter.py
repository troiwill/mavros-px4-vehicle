import __future__

import rospy

import px4_modes
import px4_vehicle_base


class PX4MavRosCopter(px4_vehicle_base.PX4MavRosVehicleBase):
    r"""
    A class for controlling a copter with a PX4 autopilot via MAVROS.
    """

    def __init__(self, vehicle_name, delay = 3., timeout = None,
        auto_connect = False, use_vehicle_name_in_topics = False):
        super(PX4MavRosCopter, self).__init__(vehicle_name, delay, timeout,
            auto_connect, use_vehicle_name_in_topics)

    @staticmethod
    def allowable_modes():
        """
        Returns the list of allowable copter modes.
        """
        return [
            px4_modes.PX4_MODE_MANUAL, px4_modes.PX4_MODE_OFFBOARD,
            px4_modes.PX4_MODE_MISSION, px4_modes.PX4_MODE_LOITER,
            px4_modes.PX4_MODE_LAND, px4_modes.PX4_MODE_TAKEOFF
        ]
    #end def

    def set_hold_mode(self):
        """
        Stops the vehicle from moving.
        """
        if self.is_armed():
            # Change mode to loiter.
            self.__set_mode__(px4_modes.PX4_MODE_LOITER)

            rospy.loginfo("Vehicle is holding.")
            rospy.sleep(self.delay)
        
        else:
            rospy.logwarn("Vehicle is not armed.")
    #end def

    def is_holding(self):
        """
        Checks the FCU flight mode to determing if the copter is hovering.
        """
        return self.is_armed() \
            and self.state.mode == px4_modes.PX4_MODE_LOITER
    #end def

    def land(self):
        """
        Requests that the copter to land.
        """
        if self.is_armed():
            # Ask the copter to hold before landing.
            if not self.is_holding():
                self.set_hold_mode()

                rospy.logdebug("Waiting for vehicle to enter hold mode.")
                self.wait_for_event(self.is_holding, True, 2.)

            # Ask the copter to land.
            self.__set_mode__(px4_modes.PX4_MODE_LAND)

        else:
            rospy.logwarn("Vehicle is not armed.")
    #end def

    def takeoff(self):
        """
        Commands the copter to takeoff. Once takeoff is complete, the copter 
        hovers. If takeoff was successful, the function returns True. 
        Otherwise, it returns False.
        """
        if not self.is_armed():
            rospy.logwarn("Vehicle is not armed.")
            return

        # Use the auto takeoff feature.
        self.__set_mode__(px4_modes.PX4_MODE_TAKEOFF)

        # TODO: Add a timeout in case the copter never switches to HOVER.
        rospy.logdebug("Waiting for vehicle to enter hold mode.")
        self.wait_for_event(self.is_holding, True, 1.)
    #end def

    def __set_mode__(self, new_mode):
        """
        Sends a set_mode request to the PX4 flight controller. The response 
        specifies if the mode is known, parsed correctly, and SET_MODE was 
        sent. Note that the set mode must be usable for a PX4 copter 
        vehicle.
        """
        if not self.is_connected():
            rospy.logwarn("Vehicle is not connected.")
            return

        if not isinstance(new_mode, str):
            raise Exception("`new_mode` must be a string.")
        
        if new_mode not in PX4MavRosCopter.allowable_modes():
            rospy.logerr("Unrecognized `new_mode` = " + new_mode)
            return

        return super(PX4MavRosCopter, self).__set_mode__(new_mode)
    #end def
#end class
