# Visual-Inertial-GNSS-EKF
This is an extended Kalman filter for inertial-GNSS (currently) and visual-inertial-GNSS (finally) sensor fusion. The dynamic model now being used is 2-Dimensional, a bird's-eye view of our body inside world reference.<br /><br />
The filter will be tested on an agri-machinery (*https://github.com/Jingxu-Li/Weed-Detector*) for self-navigation and realising precise agriculture. The hardware setup for the test includes a 6-DoF IMU (Bosch BMI055/085), an Intel D455 depth camera, and a GNSS module (NongXin AMG-PFZ202). Updates on different dimensions, complexities,and working scenarios of the system may be added in the future.


# Log
**23/03/23** The first version of EKF for agri-machinery self-navigation, currently a framework. The next step is to tune the filter with IMU and GNSS module using realsense2 (*https://github.com/IntelRealSense/librealsense*) and pynmea2 (*https://github.com/Knio/pynmea2*).<br /><br />
**30/03/23** Update funcs for fetching absolute positions and inertial sensor inputs from GNSS module and camera (Intel D455), the data is then sent to their buffers separately. Temporal sychronisation of the entire multi-sensor system is written but not complete. Current idea of TimeSync is to set firstly the GNSS samples as global temporal reference, then search IMU_accel, IMU_gyro, and Camera_rgb (ongoing) around that moment, and finally interpolates data for equivalent value and sent them all to the filter. The idea is similar to Stephen's 2012 paper (*https://ieeexplore.ieee.org/abstract/document/6696917*).
