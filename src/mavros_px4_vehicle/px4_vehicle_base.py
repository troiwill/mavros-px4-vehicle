import __future__
import px4_offboard_modes

import rospy
import geometry_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv
import tf.transformations

import abc

import px4_offboard_pub
import px4_modes
import ros_handle


class PX4MavRosVehicleBase:
    r"""
    An abstract class that defines a set of functions and variables used for 
    controlling a vehicle using PX4 and MAVROS.
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, vehicle_name, delay = 5., timeout = None,
        auto_connect = False, use_vehicle_name_in_topics = False):
        rospy.loginfo("Setting up vehicle named " + vehicle_name)
        self.__vehicle_name = vehicle_name
        self.__delay = delay
        self.__timeout = timeout

        self.__topic_prefix = ("/" + self.__vehicle_name) \
            if use_vehicle_name_in_topics else ""

        self.__is_connected = False

        if auto_connect is True:
            self.connect()
    #end def

    @property
    def delay(self):
        return self.__delay

    @property
    def state(self):
        if self.is_connected():
            return self.__state_sub.data
        else:
            rospy.logwarn("Vehicle is not connected.")
            return None

    @property
    def timeout(self):
        return self.__timeout

    @property
    def vehicle_name(self):
        return self.__vehicle_name

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
            rospy.loginfo("Vehicle armed!")
            rospy.sleep(self.delay)

        else:
            rospy.logwarn("Vehicle is not connected.")
        
        return rv
    #end def

    def __clear_mission_waypoints__(self):
        """
        Sends a request to clear the mission waypoints on the FCU.
        """
        rv = False
        if not self.is_connected():
            resp = self.__wp_clear_service.call()
            rv = resp.success
            if not resp.success:
                rospy.logerr("Could not clear mission waypoints.")
            rospy.loginfo("Waypoints cleared!")
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

            rospy.loginfo("Resetting connect flag.")
            self.__is_connected = False

            rospy.sleep(5)
        else:
            rospy.logwarn("Vehicle is not connected.")
    #end def

    @abc.abstractmethod
    def set_hold_mode(self):
        """
        A vehicle-dependent function that generally stops the vehicle from
        moving if it is a copter, rover, or boat.
        """
        raise NotImplementedError("This is an abstract method.")
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
    #end def

    @abc.abstractmethod
    def is_holding(self):
        """
        Checks the FCU flight mode to determine if the vehicle is holding.
        """
        raise NotImplementedError("This is an abstract method.")
    #end def

    def __send_new_mission_waypoints__(self, waypoints):
        """
        Sends a request to upload new mission waypoints to the FCU.
        """
        resp = self.__wp_push_service.call(0, waypoints)
        if not resp.success:
            raise Exception(self.__vehicle_name \
                + " could not send new waypoints.")

        rospy.loginfo(self.__vehicle_name \
            + ": sent new mission with {} waypoints!".format(
                resp.wp_transfered))
        rospy.sleep(self.delay)

        return resp.wp_transfered
    #end def

    @abc.abstractmethod
    def __set_mode__(self, new_mode):
        """
        Sends a request to set the vehicle in a new mode.
        """
        resp = self.__set_mode_service.call(custom_mode=new_mode)
        if not resp.mode_sent:
            raise Exception(self.__vehicle_name + " could not set mode to " \
                + new_mode)

        rospy.loginfo("Vehicle is set to new mode: " + new_mode)
        rospy.sleep(self.delay)

        return resp.mode_sent
    #end def

    def set_offboard_mode(self):
        """
        Changes the FCU to OFFBOARD mode.
        """
        rv = False
        if self.is_connected():
            rv = self.__set_mode__(px4_modes.PX4_MODE_OFFBOARD)

        else:
            rospy.logwarn("Vehicle is not connected.")

        return rv
    #end def

    def set_position(self, x = 0, y = 0, z = 0, t = 0):
        """
        Sends a pose message to the FCU. The reference frame is FLU 
        (X Forward, Y Left, Z Up) for the vehicle's body. See 
        https://docs.px4.io/v1.12/en/ros/external_position_estimation.html#reference-frames-and-ros
        for more details.
        """
        qt = tf.transformations.quaternion_from_euler(0., 0., t)
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.stamp = rospy.Time.now()

        pose.pose.position = geometry_msgs.msg.Point(x = x, y = y, z = z)
        pose.pose.orientation = geometry_msgs.msg.Quaternion(x = qt[0],
            y = qt[1], z = qt[2], w = qt[3])

        rv = self.__offboard_pub.set_cmd(pose,
            px4_offboard_modes.CMD_SET_POSE_LCL)
        if rv is False:
            rospy.logwarn("Failed to set offboard position message.")
        return rv
    #end def

    def set_velocity(self, vx = 0, vy = 0, vz = 0, wx = 0, wy = 0, wz = 0):
        """
        Sends a velocity message to the FCU. The reference frame is FLU 
        (X Forward, Y Left, Z Up) for the vehicle's body. See 
        https://docs.px4.io/v1.12/en/ros/external_position_estimation.html#reference-frames-and-ros
        for more details.
        """
        twist = geometry_msgs.msg.Twist()
        twist.linear = geometry_msgs.msg.Vector3(vx, vy, vz)
        twist.angular = geometry_msgs.msg.Vector3(wx, wy, wz)

        rv = self.__offboard_pub.set_cmd(twist,
            px4_offboard_modes.CMD_SET_VEL)
        if rv is False:
            rospy.logwarn("Failed to set offboard velocity message.")

        return rv
    #end def

    def start_new_waypoint_mission(self, waypoints):
        """
        Starts a new mission with a list of waypoints. This function assumes 
        the vehicle is already armed and/or flying.
        """
        rv = 0
        if self.is_armed():
            # Request for the vehicle to hold.
            self.set_hold_mode()
            
            # Clear waypoints from the FCU and add the new waypoints to the FCU.
            wps_cleared = self.__clear_mission_waypoints__()
            if not wps_cleared:
                return 0
            rv = self.__send_new_mission_waypoints__(waypoints)

            # Check if the vehicle is holding.
            self.wait_for_event(self.is_holding, True, loop_rate = 2.)

            # Execute the waypoint mission.
            self.__set_mode__(px4_modes.PX4_MODE_MISSION)

        else:
            rospy.logwarn("Vehicle is not armed.")

        return rv
    #end def

    def wait_for_event(self, event_fnc, condition, loop_rate):
        """
        A method that loops until the function `event_fnc` produces the
        expected condition. This method loops according to the loop rate.
        """
        check_rate = rospy.Rate(loop_rate)
        while self.is_connected() and event_fnc() != condition:
            check_rate.sleep()
    #end def
#end class
