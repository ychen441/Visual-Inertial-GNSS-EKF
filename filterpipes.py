"""An extended Kalman filter for computing intertial and visual-inertial sensor fusion.

Provide EKF for fusing data from IMU-GNSS or Camera-IMU-GNSS multi-sensor system. The currently-used 
motion model in EKF prediction stage is a 2-Dimensional vehicle moving in world coordinate. The filter 
will possibly be tested on an agricultural machinery for its availability.

Updated date: 25/03/23 by ychen441
"""

import math
import numpy as np
import time as time
import matplotlib.pyplot as plt


class EkfMethods:
    """Fuses IMU and GNSS data by inertial_pred (prediction) and 
    inertial_crc (correction) functions. Additional visual measurements
    are fused with the inertial data using vi_pred and vi_crc functions.

    Attributes: 
        var_a: system noise, accelerometer
        var_w: system noise, gyroscope
        var_x: measurement noise, x-axis GNSS
        var_y: measurement noise, y-axis GNSS
        var_a_m: measurement noise, accelerometer
        var_w_m: measurement noise, gyroscope

    Raises: 
         x and y used in the following functions may be changed according to the practical IMU setting.
    """

    def __init__(self, var_a, var_w, var_x, var_y, var_a_m, var_w_m):
        """Define system and measurement variances.
        """
        # Process noise in prediction stage
        self.noise_q = [[var_a, 0], [0, var_w]]
        # Measurement noise in correction stage
        self.noise_r = [[var_x, 0, 0, 0], [0, var_y, 0, 0], [0, 0, var_a_m, 0], [0, 0, 0, var_w_m]]

    def inertial_pred(self, x_body_pst, y_body_pst, v_x_pst, v_y_pst, a_x_pst, a_y_pst,
                      theta_pst, w_pst, cov_pst, T):
        """The prediction stage of Inertial EKF.

        Args:
          x_body_pst: x-axis position (body), time t-1
          y_body_pst: y-axis position (body), time t-1
          v_x_pst: velocity (x, body) time t-1
          v_y_pst: velocity (y, body) time t-1
          a_x_pst: acceleration (x, body), time t-1
          a_y_pst: acceleration (y, body), time t-1
          theta_pst: yaw angle, y-axis (body) vs x-axis (world), time t-1
          w_pst: angular velocity (yaw), time t-1
          T: sampling rate
          cov_pst: covariance matrix of the input state

        Returns:
          predicted state estimate miu_pred,
          predicted covariance estimate cov_pred
        """
        # Build a dynamic model with const accel and angular rate
        a_x_curr = a_x_pst  
        a_y_curr = a_y_pst  
        w_curr = w_pst 

        # Predictions in kinematics
        theta_curr = theta_pst + w_pst * T
        v_x_curr = v_x_pst + a_x_pst * T
        v_y_curr = v_y_pst + a_y_pst * T
        x_body_curr = x_body_pst + v_x_pst * T
        y_body_curr = y_body_pst + v_y_pst * T
        v_pst = np.sqrt(v_x_pst ** 2 + v_y_pst ** 2)  # Body velocity, time t-1
        v_curr = np.sqrt(v_x_curr ** 2 + v_y_curr ** 2)  # Body velocity, time t
        a_curr = np.sqrt(a_x_curr ** 2 + a_y_curr ** 2)  # Body acceleration

        # Predicted state estimate (6*1) miu_pred
        miu_pred = [x_body_curr, y_body_curr, v_curr, a_curr, theta_curr, w_curr]

        # State transition matrix (6*6) f_pred
        f = [[1, 0, np.cos(theta_pst) * T, 0, -v_pst * T * np.sin(theta_pst)],
             [0, 1, np.sin(theta_pst) * T, 0, v_pst * T * np.cos(theta_pst)],
             [0, 0, 1, T, 0, 0], [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 1, T], [0, 0, 0, 0, 0, 1]]
        """Here I use a really nasty way to compute the Jacobian, which is certainly not an appropriate 
        way to derive large-scale matrices in visual-inertial fusion (store landmarks). A possible 
        solution is to write a loop and find the elements' Jacobians in a row-to-column manner.
        """

        # Process noise covariance (6*6) q
        s = [[0, 0], [0, 0], [0, 0], [1, 0], [0, 0], [0, 1]]  # Shaping matrix
        q = s * self.noise_q * np.transpose(s)

        # Predicted covariance estimate (6*6) cov_pred
        cov_pred = f * cov_pst * np.transpose(f) + q

        return miu_pred, cov_pred

    def inertial_crc(self, x_gnss, y_gnss, a_x_msd, a_y_msd, w_msd,
                     x_body_temp, y_body_temp, v_temp, a_temp, theta_temp, w_temp, cov_temp):
        """The correction stage of Inertial EKF.
        ***Here I add some temporary variables to replace miu_pred and cov_pred to avoid error reports.

        Args:
          x_gnss: x-axis position (GNSS), time t
          y_gnss: y-axis position (GNSS), time t
          a_x_msd: acceleration measured (x, body), time t
          a_y_msd: acceleration measured (y, body), time t
          w_msd: angular velocity measured (yaw), time t

        Returns:
          corrected state estimate miu_crc,
          corrected covariance estimate cov_crc
        """
        # Observation matrix (4*1) z
        a_msd = np.sqrt(a_x_msd ** 2 + a_y_msd ** 2)
        z = [x_gnss, y_gnss, a_msd, w_msd]

        # Observation model matrix (4*6) h
        h = [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1]]

        # Innovation of measurement residual (4*1) y
        miu_temp = [x_body_temp, y_body_temp, v_temp, a_temp, theta_temp, w_temp]
        y = z - h * miu_temp

        # Kalman gain (6*4) k
        k = cov_temp * np.transpose(h) * np.linalg.inv(h * cov_temp * np.transpose(h) + self.noise_r)

        # Corrected state estimate (6*1) miu_crc
        miu_crc = miu_temp + k * y

        # Corrected covariance matrix (6*6) cov_crc
        cov_crc = (np.identity(6) - k * h) * cov_temp

        return miu_crc, cov_crc
