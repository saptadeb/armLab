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
        """!
        @brief      Record waypoints manually
        """
        self.status_message = "State: Record - Manually guide the rexarm and hit record"
        self.current_state = "record"

        # Disable torque to enable manual guiding
        torque_limit_cmds = [0] * len(self.rexarm._joints)
        for joint in self.rexarm._joints:
            joint.torque_limit = 0

        current_config = self.rexarm.get_positions()

        #Write current configuration as a csv row into a csv file
        f = open("waypoints.csv", "a")
        for joint_num, angle in enumerate(current_config):
            if joint_num == len(current_config) - 1:
                f.write(str(angle) + '\n')
            else:
                f.write(str(angle) + ',')
        f.close()
        self.check_and_log()
        self.set_next_state("estop")

    def playback(self):
        """
        @brief      Playback the recorded waypoints in waypoints.csv
        """
        self.status_message = "State: Playback - Playing the recorded moves within waypoints.csv"
        self.current_state = "playback"

        waypoints = genfromtxt("waypoints.csv", delimiter=',')
        (num_wp, num_joints) = waypoints.shape
        for wp in range(num_wp):
            wp_list = waypoints[wp,:].tolist()
            self.rexarm.set_positions(wp_list)
            time.sleep(1)

        self.check_and_log()
        self.set_next_state("idle")

    def manual(self):
        """!
        @brief      Manually control the Rexarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.check_and_log()

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.check_and_log()


    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.check_and_log()

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
            self.check_and_log()
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
        for wp in range(1,num_wp):
            print(f"waypoint:{wp}")
            wp_list = waypoints[wp,:].tolist()
            self.tp.set_final_wp(wp_list)
            if wp == 1:
                self.tp.go(is_final=False)
            elif wp == num_wp - 1:
                self.tp.go(is_init = False)
            else:
                self.tp.go(is_init = False, is_final=False)
            print(f"is_init: {self.tp.is_init} | is_final: {self.tp.is_final}")
            # self.tp.go()
            while( np.linalg.norm(np.asarray(wp_list) - np.asarray(self.rexarm.get_positions()))  > 0.15):
                time.sleep(0.01)
                self.check_and_log()
                #print(f"waypoint:{np.asarray(wp_list)} | currPos:{np.asarray(self.rexarm.get_positions())} | Euclidean dist:{np.linalg.norm(np.asarray(wp_list) - np.asarray(self.rexarm.get_positions()))}")
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
        # Use mouse clicks to get pixel locations of known locations in the workspace   
        # Repeat with the depth frame and use an affine transformation to register the two together.
        depth2rgb_affine = self.kinect.getAffineTransform(self.kinect.depth_click_points, self.kinect.rgb_click_points)
        self.kinect.depth2rgb_affine = depth2rgb_affine[0 : 2, :]
        self.kinect.depth2rgb_affine3 = depth2rgb_affine
        self.kinect.kinectCalibrated = True
        self.kinect.captureDepthFrame()

        # Load intrinsic data
        intrinsicFile = fopen('calibration.cfg', 'r')
        self.kinect.loadCameraCalibration()
        intrinsicFile.close()

        # Convert pixels to camera frame coordinates
        for i in range(i):
            self.kinect.cameraFramePoints[i] = self.kinect.pixel2Camera(self.kinect.rgb_click_points)
        # Find extrinsic matrix by affine transformation
        worldCoords = np.array([[-0.304,-0.310,0],[-0.304,0.298,0],[0.305,0.298,0],[0.305,-0.310,0],[0,0,0]])
        self.cameraIntrinsicMatrix = self.kinect.getAffineTransform(worldCoords, self.kinect.cameraFramePoints)


        # Using the intrinsic matrix you find for the RGB camera and the depth calibration function, create a
        # function that takes pixel coordinates from the image and returns workspace coordinates.



        # print(self.kinect.depth_click_points)
        # print(self.kinect.rgb_click_points)

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

    def check_and_log(self):
        if self.is_logging:
            currRexConfig = self.rexarm.get_positions()
            self.rexarm.csv_writer.writerow(currRexConfig)
