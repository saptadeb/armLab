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
        # print(f"initial_waypoints:{self.initial_wp}")
        pass

    def set_final_wp(self, waypoint):
        """!
        @brief      TODO: Sets the final wp.

        @param      waypoint  The waypoint
        """
        self.final_wp = waypoint
        # print(f"final_waypoint:{self.final_wp}")
        pass

    def go(self, max_speed=2.5):
        """!
        @brief      TODO Plan and execute the trajectory.

        @param      max_speed  The maximum speed
        """
        self.set_initial_wp()
        # self.set_final_wp([-0.01611073205641045,0.0038358879658120237,1.5765501589487174,-0.0025591338005868103])
        T = self.calc_time_from_waypoints(self.initial_wp, self.final_wp, 2.5)
        (pose_plan, velocity_plan) = self.generate_cubic_spline(self.initial_wp, self.final_wp, T)
        self.execute_plan(pose_plan, velocity_plan)
        time.sleep(T*2.5)
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
        # print(f"joint_dist_to_cover:{joint_dist_to_cover}")
        max_joint_dist_to_cover = np.max(joint_dist_to_cover)
        # print(f"max:{max_joint_dist_to_cover}")
        T = max_joint_dist_to_cover / max_speed
        # print(f"TimeTot:{T}")
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
        M = self.getM(T0, T)
        # print(f"time matrix:{M}")
        M_inv = np.linalg.inv(M)
        # print(f"time_mat_inv:{M_inv}")
        parameters = []
        for i in range(numJoints):
            constraint = np.array([[initial_wp[i]],[V0],[final_wp[i]],[Vf]])
            parameter = np.dot(M_inv, constraint) 
            parameters.append(parameter)
        # print(f"params:{parameters}")
        t = T0
        for i in range(numSteps):
            for j in range(numJoints):
                timeVector = np.array([[1],[t],[t**2],[t**3]])
                pose_plan[i,j] = np.dot(timeVector.T, parameters[j])
                velTimeVec = np.array([[0],[1],[2*t],[3*t**2]])
                velocity_plan[i,j] = np.dot(velTimeVec.T, parameters[j])
            t = t + self.dt
            # print(f"time:{t} | plan:{result[i,:]}")
              
        return pose_plan, velocity_plan

    def getM(self, T0, T):
        M = np.array([[1,T0,T0**2,T0**3],[0,1,2*T0,3*T0**2],[1,T,T**2,T**3],[0,1,2*T,3*T**2]])
        return M
        # return np.matrix('1 T0 T0**2 T0**3; 0 1 2*T0 3*T0**2; 1 T T**2 T**3; 0 1 2*T 3*T**2')

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
            # print(q_list)
        pass
