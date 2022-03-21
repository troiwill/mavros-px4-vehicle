#!/usr/bin/env python

import rospy
import unittest

from mavros_px4_vehicle.px4_copter import PX4MavRosCopter


class TestCopterFeatures(unittest.TestCase):

    def test_connect(self):
        # Do initial set up and test for no connection.
        copter = PX4MavRosCopter("test")
        self.assertFalse(copter.is_connected(),
            "Vehicle should not be connected!")

        # Connect to the vehicle and test for connection.
        copter.connect()
        self.assertTrue(copter.is_connected(),
            "Vehicle should be connected!")

        # TODO: how to test for publisher and subscribers?

        copter.disconnect()
        del copter
    #end def

    def test_auto_connect(self):
        # Do initial set up and test for connection.
        copter = PX4MavRosCopter("test", auto_connect = True)
        self.assertEquals(copter.is_connected(), True,
            "Vehicle should be connected!")

        copter.disconnect()
        del copter
    #end def

    def test_disconnect(self):
        # Do initial set up and test for connection.
        copter = PX4MavRosCopter("test", auto_connect = True)
        self.assertTrue(copter.is_connected(),
            "Vehicle should be connected!")

        rospy.sleep(3.)
        copter.disconnect()
        self.assertFalse(copter.is_connected(),
            "Vehicle should NOT be connected!")

        copter.disconnect()
        del copter
    #end def

    def test_arm(self):
        copter = PX4MavRosCopter("test")
        copter.connect()

        # Arm the vehicle.
        rv = self.copter.arm()

        # Run tests.
        # self.assertTrue(rv, "Arm cmd not sent?")
        self.assertTrue(copter.state.armed,
            "Internal state says not armed?")
        self.assertTrue(copter.is_armed(),
            "Vehicle state is not armed?")

        # Disarm the vehicle.
        rv = copter.disarm()
        # self.assertTrue(rv, "Disarm cmd not sent?")
        self.assertFalse(copter.state.armed,
            "Internal state still says armed?")
        self.assertFalse(copter.is_armed(),
            "Vehicle state is still armed?")

        copter.disconnect()
        del copter
    #end def
#end class

if __name__ == "__main__":
    import rostest
    PKG = 'mavros_px4_vehicle'
    FNAME = "test_copter.py"

    rospy.init_node("test_copter_features")
    rostest.rosrun(PKG, FNAME, TestCopterFeatures)
#end if
