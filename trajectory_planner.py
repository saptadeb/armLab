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
        T = self.calc_time_from_waypoints(self.initial_wp, self.final_wp, 1.5)
        (pose_plan, velocity_plan) = self.generate_cubic_spline(self.initial_wp, self.final_wp, T)
        self.execute_plan(pose_plan, velocity_plan)
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
        pose_plan = np.zeros([numSteps, numJoints])
        velocity_plan = np.zeros([numSteps, numJoints])
        M = self.getCubicCoeffs(T0, T)(T0, T)
        M_inv = np.linalg.inv(M)
        parameters = []
        for i in range(numJoints):
            constraint = np.array([[initial_wp[i]],[V0],[final_wp[i]],[Vf]])
            parameter = np.dot(M_inv, constraint)
            parameters.append(parameter)
        t = T0
        for i in range(numSteps):
            for j in range(numJoints):
                timeVector = np.array([[1],[t],[t**2],[t**3]])
                pose_plan[i,j] = np.dot(timeVector.T, parameters[j])
                velTimeVec = np.array([[0],[1],[2*t],[3*t**2]])
                velocity_plan[i,j] = np.dot(velTimeVec.T, parameters[j])
            t = t + self.dt

        return pose_plan, velocity_plan

    def getCubicCoeffs(self, T0, T):
        M = np.array([[1,T0,T0**2,T0**3],[0,1,2*T0,3*T0**2],[1,T,T**2,T**3],[0,1,2*T,3*T**2]])
        return M

    def generate_quintic_spline(self, initial_wp, final_wp, T):
        """!
        @brief      TODO generate a quintic spline

        @param      initial_wp  The initial wp
        @param      final_wp    The final wp
        @param      T           Amount of time to get from initial to final waypoint

        @return     The plan as num_steps x num_joints np.array
        """
        T0 = 0
        V0 = 0
        Vf = 0
        a0 = 0
        af = 0
        numSteps = int(T / self.dt)
        numJoints = len(initial_wp)
        pose_plan = np.zeros([numSteps, numJoints])
        velocity_plan = np.zeros([numSteps, numJoints])
        M = self.getQuinticCoeffs(T0, T)
        M_inv = np.linalg.inv(M)
        parameters = []
        for i in range(numJoints):
            constraint = np.array([[initial_wp[i]],[V0],[a0],[final_wp[i]],[Vf],[af]])
            parameter = np.dot(M_inv, constraint)
            parameters.append(parameter)
        t = T0
        for i in range(numSteps):
            for j in range(numJoints):
                timeVector = np.array([[1],[t],[t**2],[t**3],[t**4],[t**5]])
                pose_plan[i,j] = np.dot(timeVector.T, parameters[j])
                velTimeVec = np.array([[0],[1],[2*t],[3*t**2],[4*t**3],[5*t**4]])
                velocity_plan[i,j] = np.dot(velTimeVec.T, parameters[j])
            t = t + self.dt

        return pose_plan, velocity_plan

    def getQuinticCoeffs(self, T0, T):
        M = np.array(
                    [
                     [1,T0,T0**2,T0**3,T0**4,T0**5] , [0,1,2*T0,3*T0**2,4*T0**3,5*T0**4],
                     [0,0,2,6*T0,12*T0**2,20*T0**3] , [1,T,T**2,T**3,T**4,T**5],
                     [0,1,2*T,3*T**2,4*T**3,5*T**4] , [0,0,2,6*T,12*T**2,20*T**3]
                    ]
                    )
        return M

    def execute_plan(self, pose_plan, velocity_plan, look_ahead=8):
        """!
        @brief      TODO: Execute the planed trajectory.

        @param      plan        The plan
        @param      look_ahead  The look ahead
        """
        self.idle = False
        for q,v in zip(pose_plan,velocity_plan):
            q_list = q.tolist()
            v_list = v.tolist()
            self.rexarm.set_positions(q_list)
            self.rexarm.set_speeds(v_list)
        pass
