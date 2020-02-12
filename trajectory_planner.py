"""!
! Trajectory planner.

TODO: build a trajectory generator and waypoint planner so it allows your state machine to iterate through the plan at
the desired command update rate.
"""
import numpy as np
import time

class TrajectoryPlanner():
    """!
    @brief      This class describes a trajectory planner.
    """

    def __init__(self, rexarm):
        """!
        @brief      Constructs a new instance.

        @param      rexarm  The rexarm
        """
        self.idle = True
        self.rexarm = rexarm
        self.initial_wp = None
        self.final_wp = None
        self.dt = 0.05 # command rate
        self.desired_speed = None

    def set_initial_wp(self):
        """!
        @brief      TODO: Sets the initial wp to the current position.
        """
        self.initial_wp = self.rexarm.get_positions()
        pass

    def set_final_wp(self, waypoint):
        """!
        @brief      TODO: Sets the final wp.

        @param      waypoint  The waypoint
        """
        self.final_wp = waypoint
        pass

    def go(self, max_speed=2.5):
        """!
        @brief      TODO Plan and execute the trajectory.

        @param      max_speed  The maximum speed
        """
        pass

    def stop(self):
        """!
        @brief      TODO Stop the trajectory planner
        """
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        """!
        @brief      TODO Calculate the time to get from initial to final waypoint.

        @param      initial_wp  The initial wp
        @param      final_wp    The final wp
        @param      max_speed   The maximum speed

        @return     The amount of time to get to the final waypoint.
        """
        joint_dist_to_cover = np.asarray(final_wp) - np.asarray(initial_wp)
        max_joint_dist_to_cover = np.max(joint_dist_to_cover)
        T = max_joint_dist_to_cover / max_speed
        return T
        pass

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        """!
        @brief      TODO generate a cubic spline

        @param      initial_wp  The initial wp
        @param      final_wp    The final wp
        @param      T           Amount of time to get from initial to final waypoint

        @return     The plan as num_steps x num_joints np.array
        """

        pass

    def execute_plan(self, plan, look_ahead=8):
        """!
        @brief      TODO: Execute the planed trajectory.

        @param      plan        The plan
        @param      look_ahead  The look ahead
        """
        pass
