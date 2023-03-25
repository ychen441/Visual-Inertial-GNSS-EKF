# KalmanFilter-visual-inertial-GNSS
This is an extended Kalman filter for inertial-GNSS (currently) and visual-inertial-GNSS (finally) sensor fusion. The dynamic model now being used is 2-Dimensional since the filter will be tested on an agri-machinery or vehicle for self-navigation and realising precise agriculture. The hardware setup for the test includes a 6-DoF IMU (Bosch BMI055/085), an Intel D455 depth camera, and a GNSS module (NongXin AMG-PFZ202). Updates on different dimensions, complexities,and working scenarios of the system may be added in the future.  Agri-machinery for test: https://github.com/Jingxu-Li/Weed-Detector


# Log
**23/03/23** The first version of EKF for agri-machinery self-navigation, currently a framework. The next step is to tune the filter with IMU and GNSS module using realsense2 (https://github.com/IntelRealSense/librealsense) and pynmea2 (https://github.com/Knio/pynmea2). 
