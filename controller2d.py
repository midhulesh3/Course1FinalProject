#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np
from statistics import mean


class Controller2D(object):
    def __init__(self, waypoints):
        self.vars = cutils.CUtils()
        self._current_x = 0
        self._current_y = 0
        self._current_yaw = 0
        self._current_speed = 0
        self._desired_speed = 0
        self._current_frame = 0
        self._current_timestamp = 0
        self._start_control_loop = False
        self._set_throttle = 0
        self._set_brake = 0
        self._set_steer = 0
        self._waypoints = waypoints
        self._conv_rad_to_steer = 180.0 / 70.0 / np.pi
        self._pi = np.pi
        self._2pi = 2.0 * np.pi
        self._min_distance_ind=0

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x = x
        self._current_y = y
        self._current_yaw = yaw
        self._current_speed = speed
        self._current_timestamp = timestamp
        self._current_frame = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx = 0
        min_dist = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                self._waypoints[i][0] - self._current_x,
                self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints) - 1:
            desired_speed = self._waypoints[min_idx][2]
            self._min_distance_ind = min_idx
        else:
            desired_speed = self._waypoints[-1][2]
        
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x = self._current_x
        y = self._current_y
        yaw = self._current_yaw
        v = self._current_speed
        self.update_desired_speed()
        v_desired = self._desired_speed
        t = self._current_timestamp
        waypoints = self._waypoints
        throttle_output = 0
        steer_output = 0
        brake_output = 0
         

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """

            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.

            # implimenting a PI controller for logitudinal control

            Pgain_throttle = 1.1  # this number is tunnable
            Igain_throttle = 1 / 8  # tunnable gain

            Pgain_brake = -1 / 20  # this number is tunnable,
            Igain_brake = -1 / 200  # tunnable gain

            throttle_output = 0
            brake_output = 0
            v_error = v_desired - v
            self.vars.create_var('accum_error_throttle', 0.0)
            self.vars.create_var('t_prev', 0.0)
            self.vars.create_var('accum_error_brake', 0.0)

            if v_error > 0:
                self.vars.accum_error_brake = 0  # reset brake I controller when switching
                brake_raw = 0

                time_diff = t - self.vars.t_prev
                self.vars.accum_error_throttle = self.vars.accum_error_throttle + v_error * time_diff  # integration of error
                throttle_raw = Pgain_throttle * v_error + Igain_throttle * self.vars.accum_error_throttle
            else:
                self.vars.accum_error_throttle = 0.75*self.vars.accum_error_throttle
                throttle_raw = 0

                time_diff = t - self.vars.t_prev
                self.vars.accum_error_brake = self.vars.accum_error_brake + v_error * time_diff  # integration of error
                brake_raw = Pgain_brake * v_error + Igain_brake * self.vars.accum_error_brake

            self.vars.t_prev = t
            throttle_output = throttle_raw
            brake_output = brake_raw
            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            l_front = 1.5  # distance of front axel from center point
            x_f = self._current_x + l_front * np.sin(
                yaw)  # translating refernce point to the front axel to implement stanley controller
            y_f = self._current_y + l_front * np.cos(yaw)
            total_points = len(self._waypoints)
            #local_len = total_points//30
            #local_line = np.polyfit(self._waypoints[0:2][0], self._waypoints[0:2][1], 1)
            cross_track_gain = -0.03
            required_yaw = []
            #temp_var = cross_track_gain*np.arctan2(self._waypoints[0][1]-y, self._waypoints[0][0]-x)
            #required_yaw.append(temp_var)
            for i in range(total_points-1):
                temp_var = np.arctan2(self._waypoints[i+1][1]-self._waypoints[i][1], self._waypoints[i+1][0]-self._waypoints[i][0]) # gives the orientation
                required_yaw.append(temp_var)
           #if required_yaw < 0:
            #    required_yaw = np.pi + required_yaw  #forcing 0 to pi output
            steer_for_cross_track = np.arctan((cross_track_gain*cross_track_error))
            if cross_track_error<3:
                steer_for_cross_track = 0
            mean_req_yaw = mean(required_yaw)
            steering_raw = mean_req_yaw - yaw




            # min_idx = 0
            # min_dist = float("inf")
            # for i in range(len(self._waypoints)):
            #     dist = np.linalg.norm(np.array([
            #         self._waypoints[i][0] - self._current_x,
            #         self._waypoints[i][1] - self._current_y]))
            #     if dist < min_dist:
            #         min_dist = dist
            #         min_idx = i
            # if min_idx < len(self._waypoints) - 1:
            #     x_target = self._waypoints[min_idx+1][0]
            #     y_target = self._waypoints[min_idx+1][1]
            # else:
            #     x_target = self._waypoints[-1][0]
            #     y_target = self._waypoints[-1][1]
            #
            # angle = np.arctan((y_target-y)/(x_target-x))
            #
            # if angle < 0:
            #     angle = np.pi + angle  #forcing 0 to pi output
            #
            # steering_raw = angle-yaw

            # Change the steer output with the lateral controller.

            if steering_raw > 1.22:
                steer_output = 1.22
            elif steering_raw < -1.22:
                steer_output = -1.22
            else:
                steer_output = steering_raw + steer_for_cross_track

            # steer_output = 0
            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)  # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)  # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
