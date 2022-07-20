import __future__

import geometry_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv
import numpy as np
import rospy

import px4_modes
import px4_offboard_modes
import px4_offboard_pub
import ros_handle


class PX4Vehicle:
    r"""
    A class that defines a set of functions and variables used for 
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
    def local_pose(self):
        if not self.is_connected():
            rospy.logwarn("Vehicle is not connected.")
        return self.__lcl_pose_sub.data
    #end def

    @property
    def local_vel(self):
        if not self.is_connected():
            rospy.logwarn("Vehicle is not connected.")
        return self.__lcl_vel_sub.data
    #end def

    @property
    def state(self):
        if not self.is_connected():
            rospy.logwarn("Vehicle is not connected.")
        return self.__state_sub.data
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

    def clear_waypoints(self):
        """
        Sends a request to clear the mission waypoints on the FCU.
        """
        resp = self.__wp_clear_service.call()
        if not resp.success:
            raise Exception("Could not clear mission waypoints.")
    
        rospy.loginfo("Waypoints cleared!")
        rospy.sleep(self.delay)

        return resp.success
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

            self.__param_set_service = ros_handle.RosServiceHandle(
                self.__topic_prefix + "/mavros/param/set",
                mavros_msgs.srv.ParamSet)
            self.__param_get_service = ros_handle.RosServiceHandle(
                self.__topic_prefix + "/mavros/param/get",
                mavros_msgs.srv.ParamGet)

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

            self.__lcl_pose_sub = ros_handle.RosSubscribeHandle(
                self.__topic_prefix + "/mavros/local_position/pose",
                geometry_msgs.msg.PoseStamped)
            self.__lcl_pose_sub.setup()

            self.__lcl_vel_sub = ros_handle.RosSubscribeHandle(
                self.__topic_prefix + "/mavros/local_position/velocity_local",
                geometry_msgs.msg.TwistStamped)
            self.__lcl_vel_sub.setup()

            self.__is_connected = True

            # Final setup calls.
            self.clear_waypoints()
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
            del self.__param_set_service
            del self.__param_get_service
            del self.__lcl_pose_sub
            del self.__lcl_vel_sub
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

    def get_param(self, param_id):
        """
        Returns the value of the parameter with id `param_id` if the vehicle 
        is connected. Otherwise, return None.
        """
        rv = None
        if self.is_connected():
            resp = self.__param_get_service.call(param_id=param_id)
            if resp.success == True:
                rospy.info("Vehicle return param {} = {}".format(
                    param_id, resp.value))
            else:
                rospy.logerr("Could not get param " + param_id)
            rospy.sleep(self.delay)
            rv = resp.value
        #end if
        return rv
    #end def

    def in_hovering_mode(self):
        """
        Checks if the vehicle is in loiter or hovering mode.
        """
        return self.get_mode() == px4_modes.PX4_MODE_LOITER
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

    def is_near_local(self, pose, distance):
        """
        Checks if the vehicle is within a certain distance of the specified
        position.
        """
        if self.is_connected():
            diff_xyz = np.array([
                self.local_pose.pose.position.x - pose.pose.position.x,
                self.local_pose.pose.position.y - pose.pose.position.y,
                self.local_pose.pose.position.z - pose.pose.position.z
            ])
            return np.sqrt(np.sum(np.square(diff_xyz))) <= distance
        else:
            rospy.logwarn("Vehicle is not connected.")
            return False
        #end if
    #end def

    def land(self, block = False):
        """
        Requests that the vehicle land.
        """
        if self.is_armed():
            self.set_mode(px4_modes.PX4_MODE_LAND)
            if block:
                self.wait_for_status(self.is_armed, False, 1)
        else:
            rospy.logerr("Vehicle is not armed.")
    #end def

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

    def set_param(self, param_id, value):
        """
        Sends a request to set the parameter on the vehicle.
        """
        rv = None
        if self.is_connected():
            pval = mavros_msgs.msg.ParamValue()
            if isinstance(value, float):
                pval.real = value
            else:
                pval.integer = value
            resp = self.__param_set_service.call(param_id=param_id,
                value=pval)
            if resp.success == True:
                rospy.loginfo("Vehicle param {} is now {}.".format(
                    param_id, value))
            else:
                rospy.logerr("Vehicle failed to set param {} to {}.".format(
                    param_id, value))
            rospy.sleep(self.delay)
            rv = resp.value
        #end if
        return rv
    #end def

    def set_pose2d(self, pose2d_cmd, block = False, thres = 0.1):
        """
        Sends a pose message to the FCU. The reference frame is FLU 
        (X Forward, Y Left, Z Up) for the vehicle's body. See 
        https://docs.px4.io/v1.12/en/ros/external_position_estimation.html#reference-frames-and-ros
        for more details.
        """
        self.__offboard_pub.set_cmd(pose2d_cmd,
            px4_offboard_modes.CMD_SET_POSE_LOCAL)
        
        while block and not self.is_near_local(pose2d_cmd, thres):
            rospy.sleep(0.2)
    #end def

    def set_posvelacc(self, pva_cmd):
        """
        Sends a pose message, a velocity message, or an acceleration 
        message to the FCU. The reference frame is FLU
        (X Forward, Y Left, Z Up) for the vehicle's body. See 
        https://docs.px4.io/v1.12/en/ros/external_position_estimation.html#reference-frames-and-ros
        for more details.
        """
        self.__offboard_pub.set_cmd(pva_cmd,
            px4_offboard_modes.CMD_SET_RAW_LOCAL)
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

    def set_waypoints(self, waypoints):
        """
        Sends one or more waypoints to the flight control unit. This 
        function assumes the vehicle is already armed and/or flying.
        """
        n_wps_sent = 0
        if len(waypoints) > 0 and self.is_armed():
            # Clear waypoints from the FCU and add the new waypoints
            # to the FCU.
            self.clear_waypoints()
            resp = self.__wp_push_service.call(0, waypoints)
            if not resp.success:
                raise Exception("Could not send new waypoints.")

            rospy.loginfo("Sent new mission with {} waypoints!".format(
                resp.wp_transfered))
            rospy.sleep(self.delay)
            n_wps_sent = resp.wp_transfered

        else:
            rospy.logwarn("Vehicle is not armed.")
        
        rv = (n_wps_sent == len(waypoints) and len(waypoints) > 0)
        return rv
    #end def

    def takeoff(self, block = False):
        """
        Requests that the vehicle takeoff is possible.
        """
        if self.is_armed():
            self.set_mode(px4_modes.PX4_MODE_TAKEOFF)
            if block:
                self.wait_for_status(self.in_hovering_mode, True, 2)
        else:
            rospy.logerr("Vehicle is not armed.")
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
