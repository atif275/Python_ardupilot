from pymavlink import mavutil


connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550') 
while True:
    msg = connection.recv_match(type='GPS_RAW_INT')
    if msg is not None:
        lat = msg.lat / 1e7  
        lon = msg.lon / 1e7  
        alt= msg.alt/1000
        print(f"Latitude: {lat}, Longitude: {lon}, alt:{alt}")