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
        self.set_initial_wp()
        self.set_final_wp([-0.7832884252188035,-0.09436285675897427,-0.02531686437435887,-0.007677401401759543])
        T = self.calc_time_from_waypoints(self.initial_wp, self.final_wp, 0.25)
        plan = self.generate_cubic_spline(self.initial_wp, self.final_wp, T)
        self.execute_plan(plan)
        pass

    def stop(self):
        """!
        @brief      TODO Stop the trajectory planner
        """
        self.idle = True
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        """!
        @brief      TODO Calculate the time to get from initial to final waypoint.

        @param      initial_wp  The initial wp
        @param      final_wp    The final wp
        @param      max_speed   The maximum speed

        @return     The amount of time to get to the final waypoint.
        """
        joint_dist_to_cover = np.absolute(np.asarray(final_wp) - np.asarray(initial_wp))
        max_joint_dist_to_cover = np.max(joint_dist_to_cover)
        T = max_joint_dist_to_cover / max_speed
        print(joint_dist_to_cover)
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
        T0 = 0
        V0 = 0
        Vf = 0
        numSteps = int(T / self.dt)
        numJoints = len(initial_wp)
        result = np.zeros([numSteps, numJoints])
        M = self.getM(T0, T)
        print(T)
        M_inv = np.linalg.inv(M)
        parameters = []
        for i in range(numJoints):
            constraint = np.array([[initial_wp[i]],[V0],[final_wp[i]],[Vf]])
            parameter = np.dot(M_inv, constraint) 
            parameters.append(parameter)

        t = T0
        for i in range(numSteps):
            for j in range(numJoints):
                timeVector = np.array([[0],[t],[t**2],[t**3]])
                result[i,j] = np.dot(timeVector, parameters[j])
                t = t + dt
                
        return result

    def getM(self, T0, T):
        M = np.array([[1,T0,T0**2,T0**3],[0,1,2*T0,3*T0**2],[1,T,T**2,T**3],[0,1,2*T,3*T**2]])
        return M
        # return np.matrix('1 T0 T0**2 T0**3; 0 1 2*T0 3*T0**2; 1 T T**2 T**3; 0 1 2*T 3*T**2')

    def execute_plan(self, plan, look_ahead=8):
        """!
        @brief      TODO: Execute the planed trajectory.

        @param      plan        The plan
        @param      look_ahead  The look ahead
        """
        self.idle = False
        for q in plan:
            q_list = q.tolist()
            self.set_positions(q_list)
        pass
