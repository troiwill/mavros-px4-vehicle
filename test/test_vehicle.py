#!/usr/bin/env python

import rospy
import unittest

from mavros_px4_vehicle.px4_vehicle import PX4Vehicle


class TestVehicleFeatures(unittest.TestCase):

    def test_connect(self):
        # Do initial set up and test for no connection.
        vehicle = PX4Vehicle()
        self.assertFalse(vehicle.is_connected(),
            "Vehicle should not be connected!")

        # Connect to the vehicle and test for connection.
        vehicle.connect()
        self.assertTrue(vehicle.is_connected(),
            "Vehicle should be connected!")

        vehicle.disconnect()
        del vehicle
    #end def

    def test_auto_connect(self):
        # Do initial set up and test for connection.
        vehicle = PX4Vehicle(auto_connect = True)
        self.assertEquals(vehicle.is_connected(), True,
            "Vehicle should be connected!")

        vehicle.disconnect()
        del vehicle
    #end def

    def test_disconnect(self):
        # Do initial set up and test for connection.
        vehicle = PX4Vehicle(auto_connect = True)
        self.assertTrue(vehicle.is_connected(),
            "Vehicle should be connected!")

        rospy.sleep(3.)
        vehicle.disconnect()
        self.assertFalse(vehicle.is_connected(),
            "Vehicle should NOT be connected!")

        vehicle.disconnect()
        del vehicle
    #end def

    def test_arm(self):
        vehicle = PX4Vehicle()
        vehicle.connect()

        # Arm the vehicle.
        rv = vehicle.arm()

        # Run tests.
        # self.assertTrue(rv, "Arm cmd not sent?")
        self.assertTrue(vehicle.state.armed,
            "Internal state says not armed?")
        self.assertTrue(vehicle.is_armed(),
            "Vehicle state is not armed?")

        # Disarm the vehicle.
        rv = vehicle.disarm()
        # self.assertTrue(rv, "Disarm cmd not sent?")
        self.assertFalse(vehicle.state.armed,
            "Internal state still says armed?")
        self.assertFalse(vehicle.is_armed(),
            "Vehicle state is still armed?")

        vehicle.disconnect()
        del vehicle
    #end def
#end class

if __name__ == "__main__":
    import rostest
    PKG = 'mavros_px4_vehicle'
    FNAME = "test_vehicle.py"

    rospy.init_node("test_vehicle_features")
    rostest.rosrun(PKG, FNAME, TestVehicleFeatures)
#end if
