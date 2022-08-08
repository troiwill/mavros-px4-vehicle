import geometry_msgs.msg
import mavros_msgs.msg
import rospy
import threading

from mavros_px4_vehicle import px4_offboard_modes


class OffboardPublisher:
    r"""
    A class used for publishing offboard setpoint commands to the FCU.
    """

    def __init__(self, topic_prefix = ""):
        self.__can_send_new_msg_signal = threading.Event()
        self.__can_run = True
        self.__new_data = None
        self.__pub_index = None
        self.__publishers = dict()

        # Set up publishers.
        QUEUE_SIZE = 10
        self.__publishers[px4_offboard_modes.CMD_SET_RAW_LOCAL] = \
            rospy.Publisher(topic_prefix \
                + "/mavros/setpoint_raw/local", mavros_msgs.msg.PositionTarget,
                queue_size = QUEUE_SIZE)

        self.__publishers[px4_offboard_modes.CMD_SET_VEL] = rospy.Publisher(
            topic_prefix + "/mavros/setpoint_velocity/cmd_vel_unstamped",
            geometry_msgs.msg.Twist, queue_size = QUEUE_SIZE)

        self.__publishers[px4_offboard_modes.CMD_SET_POSE_LOCAL] = \
            rospy.Publisher(topic_prefix + "/mavros/setpoint_position/local",
            geometry_msgs.msg.PoseStamped, queue_size = QUEUE_SIZE)

        self.__can_send_new_msg_signal.set()
        self.__pub_thread = threading.Thread(target=self.__publish_loop__)
        self.__pub_thread.start()
    #end def

    @staticmethod
    def allowable_modes():
        """
        Returns a list of allowable modes.
        """
        return [
            px4_offboard_modes.CMD_SET_RAW_LOCAL,
            px4_offboard_modes.CMD_SET_POSE_LOCAL,
            px4_offboard_modes.CMD_SET_VEL
        ]

    def __is_okay__(self):
        """
        Checks if ROS is okay and if the publish loop can still run.
        """
        return self.__can_run and not rospy.is_shutdown()
    #end def

    def __publish_loop__(self):
        """
        A function that continually publishes messages to the FCU.
        """
        rate = rospy.Rate(30)
        data = None
        publisher = None

        while self.__is_okay__():
            if self.__can_send_new_msg_signal.is_set() == False:
                data = self.__new_data
                publisher = self.__publishers[self.__pub_index]
                self.__can_send_new_msg_signal.set()
            #end if

            if data is not None:
                publisher.publish(data)
            rate.sleep()
        #end while
    #end def

    def set_cmd(self, msg, cmd_set_value):
        """
        Sends a new MSG to the FCU.
        """
        if cmd_set_value not in OffboardPublisher.allowable_modes():
            rospy.logerr("Unrecognized cmd set value.")
            return False

        self.__can_send_new_msg_signal.wait()
        self.__new_data = msg
        self.__pub_index = cmd_set_value
        self.__can_send_new_msg_signal.clear()
        
        return True
    #end def

    def terminate(self):
        """
        Signals that the publisher needs to terminate.
        """
        # Stop the publish thread.
        self.__can_run = False
        self.__pub_thread.join()
        for key in self.__publishers.keys():
            self.__publishers[key].unregister()

        # Destroy the instance attributes.
        del self.__can_send_new_msg_signal
        del self.__new_data
        del self.__publishers
    #end def
#end class
