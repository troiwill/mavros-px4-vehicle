from mavros_px4_vehicle.px4_modes import PX4_MODE_LOITER, PX4_MODE_OFFBOARD
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
    def __init__(self, px4_vehicle, min_hover_altitude, min_cruise_altitude, max_cruise_altitude, 
    avoid_mode = AVOID_MODE_VERTICAL):
        # Sanity checks.
        if not isinstance(px4_vehicle, PX4Vehicle):
            raise Exception("px4 vehicle object is incorrect type.")

        if not isinstance(min_hover_altitude, (int, float)):
            raise Exception("Minimum hover altitude must be a number.")

        if not isinstance(min_cruise_altitude, (int, float)):
            raise Exception("Minimum cruise altitude must be a number.")

        if min_cruise_altitude < 0.75:
            raise ValueError("Minimum cruise altitude must be at least 0.75 meters.")

        if min_hover_altitude > min_cruise_altitude:
            raise ValueError("Minimum hover altitude must be less than or equal to minimum cruise altitude.")

        if not isinstance(max_cruise_altitude, (int, float)):
            raise Exception("Maximum cruise altitude must be a number.")

        if max_cruise_altitude <= min_cruise_altitude:
            raise ValueError("Maximum cruise altitude must be greater than minimum cruise altitude.")

        if not isinstance(avoid_mode, (str, int)):
            raise Exception("Avoid mode must be an integer or string.")

        avoid_mode_int = avoid_mode if isinstance(avoid_mode, int) else FlightAvoidance.AVOID_MODES.get(avoid_mode, -1)
        if avoid_mode_int not in list(FlightAvoidance.AVOID_MODES.values()):
            raise ValueError("Unknown avoidance mode specified: {}".format(avoid_mode))

        self.__vehicle = px4_vehicle
        self.__min_hover_alt = float(min_hover_altitude)
        self.__min_cruise_alt = float(min_cruise_altitude)
        self.__max_cruise_alt = float(max_cruise_altitude)
        self.__avoid_mode = avoid_mode_int

    @property
    def min_hover_altitude(self):
        """
        Returns the minimum hover altitude.
        """
        return self.__min_hover_alt

    @property
    def min_cruise_altitude(self):
        """
        Returns the minimum cruise altitude.
        """
        return self.__min_cruise_alt

    @property
    def max_cruise_altitude(self):
        """
        Returns the maximum cruise altitude.
        """
        return self.__max_cruise_alt

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
        if gz < self.min_hover_altitude:
            rospy.logerr("Currently unable to fly below {:.3f} meter(s) for safety.".format(self.min_hover_altitude))
            return False

        if self.avoidance_mode == FlightAvoidance.AVOID_MODE_VERTICAL:
            curr_cmd = None
            try:
                # Get the vehicle's current pose.
                current_pose = self.__vehicle.local_pose.pose
                x, y, z = current_pose.position.x, current_pose.position.y, current_pose.position.z
                q = current_pose.orientation
                _, _, yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_rotvec()

                # Set the vehicle to offboard mode using the current pose if necessary.
                if self.__vehicle.get_mode() != PX4_MODE_OFFBOARD:
                    offboard_hover_cmd = SetPositionWithYawCmdBuilder.build(x=x, y=y, z=z, hdg=yaw)
                    self.__vehicle.set_pose2d(offboard_hover_cmd)
                    rospy.sleep(1.)
                    self.__vehicle.set_mode(PX4_MODE_OFFBOARD, True)

                # Ascend to the minimum safe altitude.
                ascend_pose = [x, y, self.min_cruise_altitude, yaw]

                # Go to and turn towards a point above the goal/destination.
                intermediary_yaw = np.arctan2(gy - y, gx - x)
                horz_end_pose = [gx, gy, self.min_cruise_altitude, intermediary_yaw]

                # Descend to the goal/destination.
                goal_pose = [gx, gy, gz, gyaw]

                for cx, cy, cz, cyaw in [ ascend_pose, horz_end_pose, goal_pose ]:
                    curr_cmd = SetPositionWithYawCmdBuilder.build(x=cx, y=cy, z=cz, hdg=cyaw)
                    rospy.loginfo("Flying to pose: ({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(cx, cy, cz, cyaw))
                    self.__vehicle.set_pose2d(curr_cmd, block=True, thres=thres)
                    rospy.sleep(0.25)
                    
                # Note the flight as a success.
                return True
            
            except:
                rospy.logerr("Vehicle failed to reach the destination.")
                
                # Set hover mode.
                self.__vehicle.set_mode(PX4_MODE_LOITER, True)

        else:
            rospy.logerr("Unhandled case for avoidance mode: {}".format(self.avoidance_mode))
        return False
