"""A class of helper functions, including an UTC-to-local 
time conversion, a longitude-latitude-to-meter distance 
conversion for now.

Update date: 03/04/23 by ychen441
"""

import numpy as np
from datetime import time as dtime


class helpers:
    """funcs that may help in temporal sync
    """

    def __init__(self) -> None:
        pass

    def getlcl(self, time_utc):
        """UTC-to-local time conversion
        
        Args:
          utc time from serial port in datatime.time()

        Returns:
          local time in datatime.time()
        """
        t_crt_h = time_utc.hour + 8  # Time zone's modified here.
        if t_crt_h >= 24:
            t_crt_h = t_crt_h - 24  # Avoid hrs-count overflow
        t_crt_m = time_utc.minute
        t_crt_s = time_utc.second
        t_crt_mius = time_utc.microsecond
        t_crt = dtime(t_crt_h, t_crt_m, t_crt_s, t_crt_mius)
        return t_crt


def Body2World(self, x_body_curr, y_body_curr, x_world_pst, y_world_pst,
               theta_curr):
    """Convert body coordinate to world coordinate. Kinetic states like 
        distance are then able to be calculated under an uniform frame.

        Args:
          x_body_curr: x-axis position, time t
          y_body_curr: y-axis position, time t
          x_world_pst: x-axis position, time t-1
          y_world_pst: y-axis position, time t-1
          theta_curr: yaw angle, y-axis (body) vs x-axis (world), time t

        Returns:
          2D position x and y in world frame

        Raises:
          Possibly introduce bias to this function after test since the real 
          vehicle is not a mass point on paper.
        """
    r = np.sqrt(x_body_curr**2 +
                y_body_curr**2)  # Distance between positions at time t and t+1
    phi = theta_curr + np.arctan2(x_body_curr,
                                  y_body_curr)  # Yaw in world frame

    # x and y-axis position of the vehicle in world frame
    x_world_curr = x_world_pst + r * np.cos(phi)
    y_world_curr = y_world_pst + r * np.sin(phi)

    return x_world_curr, y_world_curr


def Geo2Mtrs(self, lon_past, lat_past, lon_curr, lat_curr):
    """Use longitude and latitude to calculate distance between the last and the 
     current instance, based on Haversine formula.
     
     Args:
     lon_past: body's longitude, time t-1
     lat_past: body's latitude, time t-1
     lon_curr: body's longitude, time t
     lat_curr: body's latitude, time t
     
     Return:
     Distance d between time t-1 and t
     """
    earth_radius = 6317000  # in metres
    # Longitude/latitude degree-to-radius conversion
    lon1 = lon_past * np.pi / 180
    lat1 = lat_past * np.pi / 180
    lon2 = lon_curr * np.pi / 180
    lat2 = lat_curr * np.pi / 180

    deltaLon = lon2 - lon1
    deltaLat = lat2 - lat1
    a = np.power(
        np.sin(deltaLat / 2),
        2) + np.cos(lat1) * np.cos(lat2) * np.power(np.sin(deltaLon / 2), 2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    d = earth_radius * c

    return d
