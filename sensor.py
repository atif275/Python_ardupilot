from pymavlink import mavutil


connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550') 

while True:
    msg = connection.recv_msg()
    if msg is not None:
        if msg.get_type() == 'GLOBAL_POSITION_INT':
        
            lat = msg.lat / 1e7  
            lon = msg.lon / 1e7  
            altitude_mm = msg.alt  
            alt = altitude_mm / 1000.0 
            agl = (msg.relative_alt / 1000.0 ) 
            gps_speed = msg.vx / 100.0  
            print(f"GPS - Latitude: {lat}, Longitude: {lon}, Altitude: {agl}, AGL: {agl}, GPS Speed: {gps_speed}")

        elif msg.get_type() == 'IMU':
          
            roll = msg.roll  
            pitch = msg.pitch  
            yaw = msg.yaw  
            print(f"IMU - Roll: {roll / 100.0} degrees, Pitch: {pitch / 100.0} degrees, Yaw: {yaw / 100.0} degrees")

        elif msg.get_type() == 'VFR_HUD':
     
            airspeed = msg.airspeed 
            throttle = msg.throttle 
            heading = msg.heading  
            print(f"VFR HUD - Airspeed: {airspeed} m/s, Throttle: {throttle}%, Heading: {heading} degrees")

        # elif msg.get_type() == 'DISTANCE_SENSOR':
       
        #     distance = msg.distance / 100.0  # Distance in meters
        #     print(f"Distance Sensor - Distance: {distance} meters")

        elif msg.get_type() == 'HEARTBEAT':
            
            mode = msg.custom_mode
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            print(f"Mode: {mode}, Arm/Disarm: {armed}")

        elif msg.get_type() == 'ATTITUDE':
        
            roll = msg.roll  
            pitch = msg.pitch 
            yaw = msg.yaw/100  
            # heading = msg.yaw  
            print(f"Attitude - Roll: {roll / 100.0} degrees, Pitch: {pitch / 100.0} degrees, yaw: {yaw} degrees")


        

    
        # if msg.get_type() == 'ATTITUDE':
        #     roll = msg.roll
        #     pitch = msg.pitch
        #     yaw = msg.yaw
        #     print(f"Attitude - Roll: {roll} radians, Pitch: {pitch} radians, Yaw: {yaw} radians")
