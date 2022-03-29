import __future__

import mavros_msgs.msg
import mavros_msgs.srv
import rospy

import px4_offboard_modes
import px4_offboard_pub
import ros_handle


class PX4Vehicle:
    r"""
    An abstract class that defines a set of functions and variables used for 
    controlling a vehicle using PX4 and MAVROS.
    """

    def __init__(self, name = "", delay = 3., timeout = None,
        auto_connect = False):
        rospy.logdebug("Setting up vehicle.")
        self.name = name
        self.delay = delay
        self.timeout = timeout
        self.__is_connected = False

        self.__topic_prefix = "" if self.name == "" else ("/" + self.name)

        if auto_connect is True:
            self.connect()
    #end def

    @property
    def state(self):
        if self.is_connected():
            return self.__state_sub.data
        else:
            rospy.logwarn("Vehicle is not connected.")
            return None
    #end def

    def arm(self):
        """
        Arms the vehicle.
        """
        rv = False
        if self.is_connected():
            resp = self.__arm_service.call(True)
            rv = resp.success
            if not resp.success:
                rospy.logerr("Could not arm the vehicle!")
            else:
                rospy.loginfo("Vehicle armed!")
            rospy.sleep(self.delay)

        else:
            rospy.logwarn("Vehicle is not connected.")
        
        return rv
    #end def
    
    def connect(self):
        """
        Connects to the vehicle by setting up a ROS node and instantiating 
        a set of ROS services and subscribers.
        """
        if not self.is_connected():
            # Set up ROS publishers.
            rospy.logdebug("Setting up ROS publishers.")
            self.__offboard_pub = px4_offboard_pub.OffboardPublisher(
                self.__topic_prefix)

            # Set up ROS services.
            rospy.logdebug("Setting up ROS services.")
            self.__arm_service = ros_handle.RosServiceHandle(
                self.__topic_prefix + "/mavros/cmd/arming",
                mavros_msgs.srv.CommandBool)
            self.__set_mode_service = ros_handle.RosServiceHandle(
                self.__topic_prefix + "/mavros/set_mode",
                mavros_msgs.srv.SetMode)

            self.__wp_clear_service = ros_handle.RosServiceHandle(
                self.__topic_prefix + "/mavros/mission/clear",
                mavros_msgs.srv.WaypointClear)
            self.__wp_push_service = ros_handle.RosServiceHandle(
                self.__topic_prefix + "/mavros/mission/push",
                mavros_msgs.srv.WaypointPush)

            # Set up ROS subscribers.
            rospy.logdebug("Setting up ROS subscribers.")
            self.__state_sub = ros_handle.RosSubscribeHandle(
                self.__topic_prefix + "/mavros/state", mavros_msgs.msg.State)
            self.__state_sub.setup()

            self.__is_connected = True
        else:
            rospy.logwarn("Vehicle is already connected.")
    #end def

    def disarm(self):
        """
        Disarms the vehicle.
        """
        rv = False
        if self.is_armed():
            resp = self.__arm_service.call(False)
            rv = resp.success
            if not resp.success:
                raise Exception("Could not disarm!")
            else:
                rospy.loginfo("Vehicle disarmed!")
            rospy.sleep(self.delay)

        else:
            rospy.logwarn("Vehicle is not armed.")

        return rv
    #end def

    def disconnect(self):
        """
        Shuts down the "connection" to the vehicle.
        """
        if self.is_armed():
            rospy.logwarn("Disarm the vehicle first.")
            return

        if self.is_connected():
            rospy.logdebug("Terminating the offboard publisher.")
            self.__offboard_pub.terminate()
            del self.__offboard_pub

            rospy.logdebug("Destroying services and subscribers.")
            del self.__arm_service
            del self.__set_mode_service
            del self.__wp_clear_service
            del self.__wp_push_service
            del self.__state_sub
            del self.__topic_prefix

            rospy.loginfo("Resetting connect flag.")
            self.__is_connected = False

            rospy.sleep(5)
        else:
            rospy.logwarn("Vehicle is not connected.")
    #end def

    def get_mode(self):
        """
        Returns the vehicle's current mode if the vehicle is connected. 
        Otherwise, return None.
        """
        if self.is_connected():
            return self.__state_sub.data.mode
        else:
            rospy.logwarn("Vehicle is not connected.")
            return None
    #end def

    def is_armed(self):
        """
        Specifies if the vehicle is armed.
        """
        return self.is_connected() and self.__state_sub.data.armed
    #end def

    def is_connected(self):
        """
        Determines if this instance is set up to communicate with the vehicle.
        """
        return self.__is_connected and not rospy.is_shutdown()
    #end defs

    def set_mode(self, new_mode, wait_for_new_mode = False):
        """
        Sends a request to set the vehicle in a new mode.
        """
        resp = self.__set_mode_service.call(custom_mode=new_mode)
        if not resp.mode_sent:
            raise Exception("Could not set mode to " + new_mode)
        else:
            rospy.loginfo("Vehicle is set to new mode: " + new_mode)

        if wait_for_new_mode is True:
            self.wait_for_status(self.get_mode, new_mode, 4)
        rospy.sleep(self.delay)

        return resp.mode_sent
    #end def

    def set_position(self, position_cmd):
        """
        Sends a pose message to the FCU. The reference frame is FLU 
        (X Forward, Y Left, Z Up) for the vehicle's body. See 
        https://docs.px4.io/v1.12/en/ros/external_position_estimation.html#reference-frames-and-ros
        for more details.
        """
        self.__offboard_pub.set_cmd(position_cmd,
            px4_offboard_modes.CMD_SET_POSE_LCL)
    #end def

    def set_velocity(self, twist_cmd):
        """
        Sends a velocity message to the FCU. The reference frame is FLU 
        (X Forward, Y Left, Z Up) for the vehicle's body. See 
        https://docs.px4.io/v1.12/en/ros/external_position_estimation.html#reference-frames-and-ros
        for more details.
        """
        self.__offboard_pub.set_cmd(twist_cmd,
            px4_offboard_modes.CMD_SET_VEL)
    #end def

    def wait_for_status(self, status_fnc, expected_status, loop_rate):
        """
        A method that loops until the function `event_fnc` produces the
        expected condition. This method loops according to the loop rate.
        """
        check_rate = rospy.Rate(loop_rate)
        while self.is_connected() and status_fnc() != expected_status:
            check_rate.sleep()
    #end def
#end class
