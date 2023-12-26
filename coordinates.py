from pymavlink import mavutil
import time

connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550') 

global_origin = None  

while True:
    msg = connection.recv_match(type='GLOBAL_POSITION_INT')
    if msg is not None:
        global_origin = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0)  
        print(f"Global Origin: {global_origin}")

    msg = connection.recv_match(type='GPS_RAW_INT')
    if msg is not None and global_origin is not None:
        relative_lat = ((msg.lat / 1e7) - global_origin[0] )* 1.113195e5
        relative_lon = ((msg.lon / 1e7) - global_origin[1]) * 1.113195e5
        relative_alt = (msg.alt / 1000.0) - global_origin[2]
        print(f"Relative X: {relative_lon}, Relative Y: {relative_lat}, Relative Z: {relative_alt}")
       # time.sleep(1)
