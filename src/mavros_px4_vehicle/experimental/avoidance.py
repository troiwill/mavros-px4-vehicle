from mavros_px4_vehicle.px4_modes import PX4_MODE_LOITER
from mavros_px4_vehicle.px4_offboard_modes import SetPositionWithYawCmdBuilder
from mavros_px4_vehicle.px4_vehicle import PX4Vehicle
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R
from typing import Union, List


class FlightAvoidance:
    """
    A tool that helps a quadrotor fly to a goal while using a set of avoidance features.
    """
    AVOID_MODE_VERTICAL = 0
    AVOID_MODES = {"vertical": AVOID_MODE_VERTICAL}

    # def __init__(self, px4_vehicle: PX4Vehicle, min_safe_altitude: float, max_safe_altitude: float, 
    # avoid_mode: Union[str,int] = AVOID_MODE_VERTICAL):
    def __init__(self, px4_vehicle, min_safe_altitude, max_safe_altitude, 
    avoid_mode = AVOID_MODE_VERTICAL):
        # Sanity checks.
        if not isinstance(px4_vehicle, PX4Vehicle):
            raise Exception("px4 vehicle object is incorrect type.")

        if not isinstance(min_safe_altitude, (int, float)):
            raise Exception("Minimum safe altitude must be a number.")

        if min_safe_altitude < 1.:
            raise ValueError("Minimum safe altitude must be at least one meter.")

        if not isinstance(max_safe_altitude, (int, float)):
            raise Exception("Maximum safe altitude must be a number.")

        if max_safe_altitude <= min_safe_altitude:
            raise ValueError("Maximum safe altitude must be greater than minimum safe altitude.")

        if not isinstance(avoid_mode, (str, int)):
            raise Exception("Avoid mode must be an integer or string.")

        avoid_mode_int = avoid_mode if isinstance(avoid_mode, int) else FlightAvoidance.AVOID_MODES.get(avoid_mode, -1)
        if avoid_mode_int not in list(FlightAvoidance.AVOID_MODES.values()):
            raise ValueError("Unknown avoidance mode specified: {}".format(avoid_mode))

        self.__vehicle = px4_vehicle
        self.__min_safe_alt = float(min_safe_altitude)
        self.__max_safe_alt = float(max_safe_altitude)
        self.__avoid_mode = avoid_mode_int

    @property
    def min_safe_altitude(self):
        """
        Returns the minimum safe altitude.
        """
        return self.__min_safe_alt

    @property
    def max_safe_altitude(self):
        """
        Returns the maximum safe altitude.
        """
        return self.__max_safe_alt

    @property
    def avoidance_mode(self):
        """
        Returns the avoidance mode.
        """
        return self.__avoid_mode

    # def go_to(self, goal_pose: Union[np.ndarray, List[float]], thres: float = 0.1) -> None:
    def go_to(self, goal_pose, thres = 0.1):
        """
        Sets the new goal (3D position and yaw) that the drone should fly to.
        """
        # Sanity checks.
        gx, gy, gz, gyaw = goal_pose
        if gz < 1.:
            rospy.logerr("Currently unable to fly below one meter for safety.")
            return False

        if self.avoidance_mode == FlightAvoidance.AVOID_MODE_VERTICAL:
            curr_cmd = None
            try:
                # Ascend to the minimum safe altitude.
                current_pose = self.__vehicle.local_pose.pose
                x, y = current_pose.position.x, current_pose.position.y
                q = current_pose.orientation
                _, _, yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_rotvec()
                ascend_pose = [x, y, self.min_safe_altitude, yaw]

                # Go to and turn towards a point above the goal/destination.
                intermediary_yaw = np.arctan2(gy - y, gx - x) - yaw
                horz_end_pose = [gx, gy, self.min_safe_altitude, intermediary_yaw]

                # Descend to the goal/destination.
                goal_pose = [gx, gy, gz, gyaw]

                for cx, cy, cz, cyaw in [ ascend_pose, horz_end_pose, goal_pose ]:
                    curr_cmd = SetPositionWithYawCmdBuilder.build(x=cx, y=cy, z=cz, hdg=cyaw)
                    rospy.loginfo("Flying to pose: ({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(cx, cy, cz, cyaw))
                    self.__vehicle.set_pose2d(curr_cmd, block=True, thres=thres)
                    rospy.sleep(2.)
                    
                # Note the flight as a success.
                return True
            
            except:
                rospy.logerr("Vehicle failed to reach the destination.")
                
                # Set hover mode.
                self.__vehicle.set_mode(PX4_MODE_LOITER, True)

        else:
            rospy.logerr("Unhandled case for avoidance mode: {}".format(self.avoidance_mode))
        return False
