

import argparse
import json
from typing import Dict, Optional
import time
#from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import math
#- Importing Tkinter: sudo apt-get install python-tk
import tkinter as tk


#-- Connect to the vehicle
print('Connecting...')
mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550') 
mav_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

gnd_speed = 5 
yaw_increment_degrees = -10  # Yaw angle increment for each step
yaw_increment_radians = math.radians(yaw_increment_degrees)
total_rotation_degrees = -50  # Total rotation desired
total_rotation_radians = math.radians(total_rotation_degrees)

chk=0
def arm_and_takeoff(altitude):

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    return msg.result
      

def set_velocity_body(vx, vy, vz):

    msg = mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, mav_connection.target_system,
                        mav_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b0000111111000111), 0, 0, 0, vx, vy, vz, 0, 0, 0, 1.57, 0.5))

    # msg = mav_connection.message_factory.set_position_target_local_ned_encode(
    #         0,
    #         0, 0,
    #         mavutil.mavlink.MAV_FRAME_BODY_NED,
    #         0b0000111111000111, #-- BITMASK  Consider only the velocities
    #         0, 0, 0,        
    #         vx, vy, vz,     
    #         0, 0, 0,        
    #         0, 0)
    #mav_connection.send_mavlink(msg)
    #mav_connection.flush()

def set_yaw():
    current_yaw = 0

    # msg = mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, mav_connection.target_system,
    #                     mav_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111111111), 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.57, yaw_rate))
    while current_yaw < total_rotation_radians:
        mav_connection.mav.command_long_send(
        mav_connection.target_system,          # Target system ID
        mav_connection.target_component,       # Target component ID
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # Command ID
        0,                             # Confirmation
        0,                             # Param 1 (Yaw angle in radians; set to 0 for relative angle)
        0,                             # Param 2 (Yaw speed in degrees per second; set to 0 for immediate execution)
        current_yaw,                    # Param 3 (Target yaw angle in degrees)
        0,                             # Param 4 (Unused)
        1,                             # Param 5 (Unused)
        0,                             # Param 6 (Unused)
        0                              # Param 7 (Unused)
        )
        current_yaw += yaw_increment_radians

        # Wait for a short duration between commands
        time.sleep(1)
    print("out of while")    


def change_mode(mav_connection, mode, autopilot='ardupilot', sub_mode='NONE'):
    if mode not in mav_connection.mode_mapping():
        print(f'Unknown mode : {mode}')
        print(f"available modes: {list(mav_connection.mode_mapping().keys())}")
        raise Exception('Unknown mode')
    mode_id = mav_connection.mode_mapping()[mode]
    # print(mode_id)
    sub_mode = 0
    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, sub_mode, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    # print(ack_msg)
    return ack_msg.result    
def arm(mav_connection, arm_command):
 
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    return msg.result  
def upload_qgc_mission(mission_file: str, the_connection: mavutil.mavlink_connection) -> Optional[bool]:
 
    mission = read_qgc_mission(mission_file)
    if not mission:
        return None

    wploader = []

    seq = 0  

    add_home_position_waypoint(wploader, mission, seq)
    seq += 1  

    for wp in mission["mission"]["items"]:
        print(wp)
        add_waypoint(wploader, the_connection, wp, seq)
        seq += 1  
    
    return upload_mission(the_connection, wploader)

def read_qgc_mission(mission_file: str) -> Dict:

    try:
        with open(mission_file, 'r') as file:
            mission = json.load(file)
    except FileNotFoundError:
        print(f"Mission file {mission_file} not found.")
        return {}

    return mission

def add_home_position_waypoint(wploader: list, mission: Dict, seq: int) -> None:

    wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
        0, 0, seq, 0, 16, 0, 0, 0, 0, 0, 0,
        int(mission["mission"]["plannedHomePosition"][0] * 10 ** 7), 
        int(mission["mission"]["plannedHomePosition"][1] * 10 ** 7), 
        int(mission["mission"]["plannedHomePosition"][2])
    ))

def add_waypoint(wploader: list, master: mavutil.mavlink_connection, wp: Dict, seq: int) -> None:

    wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
        master.target_system, master.target_component, seq, wp["frame"], wp["command"], 0, int(wp["autoContinue"]), 
        float(wp["params"][0]), float(wp["params"][1]), float(wp["params"][2]), 
        float(wp["params"][3]), int(wp["params"][4] * 10 ** 7), int(wp["params"][5] * 10 ** 7), 
        int(wp["params"][6])
    ))

def upload_mission(master: mavutil.mavlink_connection, wploader: list) -> bool:
  
    print('Clearing mission')
    master.mav.mission_clear_all_send(master.target_system, master.target_component)

    if not verify_ack(master, 'Error clearing mission'):
        return False

    master.waypoint_count_send(len(wploader))
    return send_waypoints(master, wploader)

def verify_ack(master: mavutil.mavlink_connection, error_msg: str) -> bool:

    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
    print(ack)
    if ack.type != 0:
        print(f'{error_msg}: {ack.type}')
        return False
    return True

def send_waypoints(master: mavutil.mavlink_connection, wploader: list) -> bool:
    for i in range(len(wploader)):
        msg = master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=3)
        if not msg:
            print('No waypoint request received')
            return False
        print(f'Sending waypoint {msg.seq}/{len(wploader)-1}')
        master.mav.send(wploader[msg.seq])

        if msg.seq == len(wploader)-1:
            break

    return verify_ack(master, 'Error uploading mission')


def key(event):
    if event.char == event.keysym: 
        if event.keysym == 'r':
            arm(mav_connection,1)
            
        elif event.keysym == 'd':
            arm(mav_connection,0)

        elif event.keysym == 'l':
            change_mode(mav_connection,'LAND')  

        elif event.keysym == 'g':
            change_mode(mav_connection,'GUIDED')   

        elif event.keysym == 'a':
            change_mode(mav_connection,'AUTO')     
        elif event.keysym == 's':
            change_mode(mav_connection,'STABILIZE')         
        elif event.keysym == 't':
            arm_and_takeoff(5)    

        elif event.keysym == 'm':
            upload_qgc_mission('plan.txt', mav_connection)  

        elif event.keysym == 'y':
            set_yaw()     

        elif event.keysym == 'p':
            set_velocity_body( -1, 0, 0)
            print("front")
            time.sleep(1)
            set_velocity_body( 0, -1, 0)
            print("right")
            time.sleep(1)
            set_velocity_body( 1, 0, 0)
            print("back")
            time.sleep(1)
            set_velocity_body( 0, 1, 0)
            print("left")
            time.sleep(1)
            

    else: 
        if event.keysym == 'Up':
            set_velocity_body( -gnd_speed, 0, 0)
        elif event.keysym == 'Down':
            set_velocity_body(gnd_speed, 0, 0)
        elif event.keysym == 'Left':
            set_velocity_body( 0, gnd_speed, 0)
        elif event.keysym == 'Right':
            set_velocity_body(0, -gnd_speed, 0)
    
    

#arm_and_takeoff(10)
root = tk.Tk()
print(">> Control the drone with the arrow keys. Press r for RTL mode")
root.bind_all('<Key>', key)
root.mainloop()
 
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
      