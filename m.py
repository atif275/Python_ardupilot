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



# def read_waypoints_from_file(file_path):
#     with open(file_path, 'r') as file:
#         waypoints_data = json.load(file)

    # for waypoint_name, waypoint_info in waypoints_data.items():
    #     command = waypoint_info["command"]
    #     print(f"command= {command}")
    #     x, y, alt = waypoint_info["params"][-3:]  # Assuming x, y, alt are the last three elements in the "params" list
    #     print(f"x={x}, y={y}, alt={alt}")
    #     wp_id = waypoint_info["wp_id"]
    #     print(f"wp_id= {wp_id}")
    #     mission(x, y, alt)

def read_waypoints_from_file(file_path):
    with open(file_path, 'r') as file:
        file_content = file.read()
        print("File Content:", file_content)
        waypoints_data = json.loads(file_content)
        for waypoint_name, waypoint_info in waypoints_data.items():
            command = waypoint_info["command"]
            print(f"command= {command}")
            x, y, alt = waypoint_info["params"][-3:]  # Assuming x, y, alt are the last three elements in the "params" list
            print(f"x={x}, y={y}, alt={alt}")
            wp_id = waypoint_info["wp_id"]
            print(f"wp_id= {wp_id}")
            mission(x, y, alt)

file_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/mission.txt'
read_waypoints_from_file(file_path)        