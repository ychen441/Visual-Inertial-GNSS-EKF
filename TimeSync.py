"""A pipeline trying to solve the temporal synchronisation issue in 
a multi-sensor system.

***Will possibly separate the GNSS receiver module (abs position) and the 
camera receiver module (visual and intertial info) and define them 
as two functions to be called in this TimeSync programme. Then it 
may be easy to grab elements from two buffers and do whatever we want.***

Update time: 29/03/23 by ychen441
"""

import io
import time
import serial
import pynmea2
from tryCam import rs2stream
from tryTime import TimeTricks
from datetime import time as dtime

# Stream initialisation
# Primarily check /dev/ttyS* in the terminal. Here I use a pseudo one.
ser = serial.Serial('/dev/ttyS0', 9600, timeout=5.0)
gnss_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

# Build buffers for storing data
gnss_buffer = []
acc_buffer = []
gyro_buffer = []
"""Read GNSS serial inputs. t_lcl obtained by GNSS is set as 
the gobal temporal reference of the entire multi-sensor system.
"""
for sentence in gnss_io:
    try:
        msg = pynmea2.parse(sentence)
        if isinstance(msg, pynmea2.types.talker.RMC):
            lon = msg.longitude
            if (lon < 0):
                lon *= -1  # Hemi S-to-N conversion
            lat = msg.latitude
            if (lat < 0):
                lat *= -1
            t_utc = msg.timestamp  # UTC time
            t_lcl = TimeTricks.getlcl(t_utc)  # UTC to local time
            """Only send effective GPS data yo the buffer (or keep all module 
            inputs and abandon those lat/lon-zero data when feeding them into 
            the filter?)
            """
            if lon != 0 and lat != 0:
                gnss_buffer.append([lon, lat, t_lcl])
    except serial.SerialException as e:
        #print('Device error: {}'.format(e))
        break
    except pynmea2.ParseError as e:
        #print('Parse error: {}'.format(e))
        continue
"""Read inertial inputs from camera."""
# Initialisation
enable_rgb = True
enable_IMU = True
width = 1280
height = 800
"""Suggeseted freq pairings are 200/400 Hz for gyro, 63/250 Hz for accel"""
accel_rate = 63
gyro_rate = 200
max_frame_num = 0  # Set maximum frames (if needed) that the camera accepts

try:
    cam = rs2stream(frame_width=width,
                    frame_height=height,
                    acc_framerate=accel_rate,
                    gyro_framerate=gyro_rate,
                    enable_rgb=False,
                    enable_imu=True)

    # Set a counter for frames and time
    counter = 0
    t_init = time.time()
    t_frame = t_init

    # Main, grab IMU accel and gyro data and send to buffers.
    while True:
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = cam.run_imu()
        counter += 1
        t_last = t_frame
        t_frame = time.time()
        # Feed 3-axis accelerations and angular rates to the buffer
        acc_buffer.append([acc_x, acc_y, acc_z, t_last])
        gyro_buffer.append([gyro_x, gyro_y, gyro_z, t_last])
        # Break if reaching maximum_frame_num
        if max_frame_num > 0:
            if counter == max_frame_num:
                break
        else:
            continue
            # or time.sleep(0.02)
finally:
    cam.shutdown()
