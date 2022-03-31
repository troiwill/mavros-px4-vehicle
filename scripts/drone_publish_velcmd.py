#!/usr/bin/env python

import rospy

from mavros_px4_vehicle.px4_offboard_modes import SetVelocityCmdBuilder
from geometry_msgs.msg import Twist

def talker():
    # Create the publisher to vel_cmds
    pub = rospy.Publisher('vel_cmds', Twist, queue_size=1)
    rospy.init_node('mavros_px4_drone', anonymous=True)

    while not rospy.is_shutdown():
        fwd_cmd = SetVelocityCmdBuilder.build(vz = 1.) # Send command that sets velocity to 1
        pub.publish(fwd_cmd) # Publish the command to vel_cmds
        rospy.sleep(7)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass