"""!
The state machine that implements the logic.
"""

import time
import numpy as np
from numpy import genfromtxt

class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
    """

    def __init__(self, rexarm, planner, kinect):
        """!
        @brief      Constructs a new instance.

        @param      rexarm   The rexarm
        @param      planner  The planner
        @param      kinect   The kinect
        """
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.waypoints = [
            [0.0,           0.0,            0.0,            0.0],
            [np.pi * 0.1,   0.0,            np.pi / 2,      0.0],
            [np.pi * 0.25,  np.pi / 2,      -np.pi / 2,     np.pi / 2],
            [np.pi * 0.4,   np.pi / 2,      -np.pi / 2,     0.0],
            [np.pi * 0.55,  0,              0,              0],
            [np.pi * 0.7,   0.0,            np.pi / 2,      0.0],
            [np.pi * 0.85,  np.pi / 2,      -np.pi / 2,     np.pi / 2],
            [np.pi,         np.pi / 2,      -np.pi / 2,     0.0],
            [0.0,           np.pi / 2,      np.pi / 2,      0.0],
            [np.pi / 2,     -np.pi / 2,     np.pi / 2,      0.0]]

    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

                    This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    TODO: Add states and funcitons as needed.
        """
        if self.next_state == "initialize_rexarm":
            self.initialize_rexarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute_tp":
            self.execute_tp()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()

        if self.next_state == "manual":
            self.manual()

        if self.next_state == "record":
            self.record()

        if self.next_state == "playback":
            self.playback()



    """Functions run for each state"""
    def record(self):
        torque_limit_cmds = [0] * len(self.rexarm._joints)
        for joint in self.rexarm._joints:
            joint.torque_limit = 0
        current_config = self.rexarm.get_positions()
        print (current_config)
        f = open("waypoints.csv", "a")
        for joint_num, angle in enumerate(current_config):
            if joint_num == len(current_config) - 1:
                f.write(str(angle) + '\n')
            else:
                f.write(str(angle) + ',')
        f.close()
        self.set_next_state("estop")

    def playback(self):
        waypoints = genfromtxt("waypoints.csv", delimiter=',')
        (num_wp, num_joints) = waypoints.shape
        for wp in range(num_wp):
            wp_list = waypoints[wp,:].tolist()
            self.rexarm.set_positions(wp_list)
            time.sleep(1)
        self.set_next_state("idle")

    def manual(self):
        """!
        @brief      Manually control the Rexarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()

    def execute(self):
        """!
        @brief      Go through all waypoints
        """
        self.status_message = "State: Execute - Executing motion plan"
        self.next_state = "idle"
        for wp in self.waypoints:
            # Ensure the correct number of joint angles
            full_wp = [0.0] * self.rexarm.num_joints
            full_wp[0:len(wp)] = wp
            # TODO: Set the positions and break if estop is needed
            self.rexarm.set_positions(wp)
            time.sleep(1)
            if (self.next_state == "estop"):
                self.set_next_state("estop")
                break


    def execute_tp(self):
        """!
        @brief      Go through all waypoints with the trajectory planner.
        """
        self.status_message = "State: Execute TP - Executing Motion Plan with trajectory planner"
        self.current_state = "execute"
        self.next_state = "idle"
        waypoints = genfromtxt("waypoints.csv", delimiter=',')
        (num_wp, num_joints) = waypoints.shape
        actual_joint_positions = []
        for wp in range(num_wp):
            print(f"waypoint:{wp}")
            wp_list = waypoints[wp,:].tolist()
            self.tp.set_final_wp(wp_list)
            self.tp.go()
            while( np.linalg.norm(np.asarray(wp_list) - np.asarray(self.rexarm.get_positions()))  > 0.1):
                time.sleep(0.01)
                print(f"waypoint:{np.asarray(wp_list)} | currPos:{np.asarray(self.rexarm.get_positions())} | Euclidean dist:{np.linalg.norm(np.asarray(wp_list) - np.asarray(self.rexarm.get_positions()))}")
            # TODO: Send the waypoints to the trajectory planner and break if estop
        self.set_next_state("idle")


    def calibrate(self):
        """!
        @brief      Gets the calibration clicks
        """
        self.current_state = "calibrate"
        self.next_state = "idle"

        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of shoulder motor"]
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False

        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False

        """TODO Perform camera calibration here"""
        print(self.kinect.rgb_click_points)
        print(self.kinect.depth_click_points)

        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)

    def initialize_rexarm(self):
        """!
        @brief      Initializes the rexarm.
        """
        self.current_state = "initialize_rexarm"

        if not self.rexarm.initialize():
            print('Failed to initialize the rexarm')
            self.status_message = "State: Failed to initialize the rexarm!"
            time.sleep(5)
        self.next_state = "idle"
