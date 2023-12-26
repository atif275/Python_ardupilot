from pymavlink import mavutil
import time
import json
mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
mav_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (mav_connection.target_system, mav_connection.target_component))

def mission(x, y ,alt):

    mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, mav_connection.target_system,
                        mav_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111111000), x, y, alt, 0, 0, 0, 0, 0, 0, 1.57, 0.5))

# the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
#                         the_connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000), int(-35.3629849 * 10 ** 7), int(149.1649185 * 10 ** 7), 10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))
def takeoff(altitude):
 
    mav_connection.wait_heartbeat()
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True) 
    print(msg)
    return msg.result
def gps_mission(lat,lon,alt):

    lat=-int(lat*1e7)
    lon=int(lon*1e7)
    msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0,
        0,
        0,
        6,
        3576,
        lat,
        lon,
        alt,
        0, 0, 0,
        0,0,0,0,0
    )
    # SET_POSITION_TARGET_GLOBAL_INT 0 0 0 6 3576 -353621474 1491651746 10 0 0 0 0 0 0 0 0
    mav_connection.mav.send(msg)


def read_waypoints_from_file(file_path):
    with open(file_path, 'r') as file:
        waypoints_data = json.load(file)

    for waypoint_name, waypoint_info in waypoints_data.items():
        command = waypoint_info["command"]
        print(f"command= {command}")
        x, y, alt = waypoint_info["params"][-3:]  # Assuming x, y, alt are the last three elements in the "params" list
        print(f"x={x}, y={y}, alt={alt}")
        wp_id = waypoint_info["wp_id"]
        print(f"wp_id= {wp_id}")
        mission(x, y, alt)
        # if command =='takeoff':
        #     alt=waypoint_info["params"][-1:]
        #     takeoff(alt)
        # elif command =='rel_go_to_wp'  :
        #     x, y, alt = waypoint_info["params"][-3:]  # Assuming x, y, alt are the last three elements in the "params" list
        #     print(f"x={x}, y={y}, alt={alt}")
        #     wp_id = waypoint_info["wp_id"]
        #     print(f"wp_id= {wp_id}")
        #     mission(x, y, alt)


        # elif command =='gps_go_to_wp'  :
        #     lat, lon, alt = waypoint_info["params"][-3:]  # Assuming x, y, alt are the last three elements in the "params" list
        #     print(f"lat={lat}, lon={lon}, alt={alt}")
        #     wp_id = waypoint_info["wp_id"]
        #     print(f"wp_id= {wp_id}")
        #     gps_mission(x, y, alt)    


        #time.sleep(4)





file_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/mission.txt'
read_waypoints_from_file(file_path)
#conversion of gps to drone refrence  
#conversion of drone refrence to gps 
#relative velocity mode speed wala only take input of x,y,z of velocity with key

# wp_1{

#     "command": 84,
#     wp_id 5,

#     "params": [
#                     0,
#                     0,
#                     0,
#                     0,
#                     x,
#                     y,
#                     alt
#                 ]


# }