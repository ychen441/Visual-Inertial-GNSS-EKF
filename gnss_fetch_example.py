import pynmea2
from classOfuncs import helpers


# Buffers for unsynchronised spatial measurements
gps_buffer = []
# Buffers for temporal measurements
t_std = []

geo = helpers()

gps_io = open('/home/chen/code/pynmea2/examples/data.log', encoding='utf-8')
for sentence in gps_io.readlines():
    try:
        msg = pynmea2.parse(sentence)
        if isinstance(msg, pynmea2.types.talker.RMC):
            lon_raw = msg.longitude
            lon = geo.Lon2Cartesian(lon_raw)
            lat_raw = msg.latitude
            lat = geo.Lat2Cartesian(lat_raw)
            t_epoch = msg.datetime.timestamp()
            #gps_buffer.append([lon_raw, lat_raw])
            gps_buffer.append([lon, lat])
            t_std.append(t_epoch)
    #except serial.SerialException as e:
        #print('Device error: {}'.format(e))
        #break
    except pynmea2.ParseError as e:
        #print('Parse error: {}'.format(e))
        continue
print("gps inputs:", gps_buffer)
print("gps time:", t_std)