from pymavlink import mavutil
import time


connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

global_origin = None 

while True:
    time.sleep(1)
    msg = connection.recv_match(type='GLOBAL_POSITION_INT')
    if msg is not None:
        global_origin = (msg.lat / 1e7, msg.lon / 1e7) 
        print(f"Global Origin: {global_origin}")

    msg = connection.recv_match(type='GPS_RAW_INT')
    if msg is not None and global_origin is not None:
        relative_lat = (msg.lat / 1e7) - global_origin[0]
        relative_lon = (msg.lon / 1e7) - global_origin[1]
        print(f"Relative Latitude: {relative_lat}, Relative Longitude: {relative_lon}")

