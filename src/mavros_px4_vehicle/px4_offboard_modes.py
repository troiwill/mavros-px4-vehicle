import geometry_msgs.msg
import rospy
import tf.transformations


CMD_SET_VEL = 0
CMD_SET_POSE_LCL = 1

class SetPositionCmdBuilder():

    @staticmethod
    def build(x  = 0, y = 0, z = 0, hdg = 0):
        qt = tf.transformations.quaternion_from_euler(0., 0., hdg)
        cmd = geometry_msgs.msg.PoseStamped()
        cmd.header.stamp = rospy.Time.now()

        cmd.pose.position = geometry_msgs.msg.Point(x = x, y = y, z = z)
        cmd.pose.orientation = geometry_msgs.msg.Quaternion(x = qt[0],
            y = qt[1], z = qt[2], w = qt[3])

        return cmd
    #end def
#end class


class SetVelocityCmdBuilder:

    def build(vx  = 0, vy = 0, vz = 0, wx = 0, wy = 0, wz = 0):
        twist = geometry_msgs.msg.Twist()
        twist.linear = geometry_msgs.msg.Vector3(vx, vy, vz)
        twist.angular = geometry_msgs.msg.Vector3(wx, wy, wz)

        return twist
    #end def
#end class