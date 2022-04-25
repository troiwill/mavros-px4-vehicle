import rospy
import abc


class RosHandleBase:
    r"""
    An abstract base class for interacting with ROS services, subscribers, 
    and publishers.
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, topic, object_type):
        self.topic = topic
        self.object_type = object_type
    #end def
#end class


class RosServiceHandle(RosHandleBase):
    r"""
    A class used to interact with ROS services.
    """

    def __init__(self, topic, request_type, call_timeout=None):
        super(RosServiceHandle, self).__init__(topic, request_type)
        self.call_timeout = call_timeout
    #end def

    def call(self, *args, **kwds):
        """
        Calls the ROS service at the corresponding topic. If arguments are 
        provided, they are passed along to the ROS service. If the service 
        fails to respond within a time limit, this function raises an
        exception.
        """
        try:
            rospy.logdebug("waiting for service " + self.topic)
            rospy.wait_for_service(self.topic, self.call_timeout)

            rospy.logdebug("calling service " + self.topic)
            service_call = rospy.ServiceProxy(self.topic, self.object_type)
        
        except rospy.ROSException as err:
            raise Exception("error with service ({}). message => {}".format(
                self.topic, str(err)))

        return service_call(*args, **kwds)
    #end def
#end class


class RosSubscribeHandle(RosHandleBase):
    r"""
    A class used to subscribe to ROS topics.
    """

    def __init__(self, topic, subscriber_type, queue_size = 10):
        super(RosSubscribeHandle, self).__init__(topic, subscriber_type)
        self.queue_size = queue_size
        self.data = None
    #def def

    def data_callback(self, msg):
        """
        Callback function for the ROS subscriber.
        """
        self.data = msg
    #end def

    def setup(self, initial_timeout = 3.):
        """
        Creates a ROS subscriber and ensures that the topic is producing data.
        """
        # Try to get one message using the ROS topic.
        self.data = None

        while self.data is None and not rospy.is_shutdown():
            try:
                self.data = rospy.wait_for_message(self.topic,
                    self.object_type, timeout = initial_timeout)
            except rospy.ROSException as err:
                raise Exception("No message for {}. Error message: {}".format(
                    self.topic, err))
        
        # Once we receive one message, create the subscriber.
        self.subscriber = rospy.Subscriber(self.topic,
            self.object_type, callback = self.data_callback,
            queue_size = self.queue_size)
    #end def

    def terminate(self):
        """
        Unregisters the subscriber.
        """
        self.subscriber.unregister()
    #end def
#end class
