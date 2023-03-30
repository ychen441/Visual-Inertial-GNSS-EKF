"""A class of functions related to TimeSync pipeline, including an UTC-to-local 
time conversion for now.

Update date: 28/03/23 by ychen441
"""

from datetime import time as dtime


class TimeTricks:
    """funcs that may help in temporal sync
    """

    def __init__(self) -> None:
        pass

    def getlcl(time_utc):
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


"""
t_tst = time(18, 15, 10)
t_now = TimeTricks.getlcl(t_tst)
print(t_tst, t_now)
"""
