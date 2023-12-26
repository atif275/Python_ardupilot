import socket
import argparse
from pymavlink import mavutil
import json
from typing import Dict, Optional

mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
auto_alt_wp=0
def arm(arm_command):
    # mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
 
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    return msg.result

def takeoff(altitud):
    # mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitud)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True) 
    print(msg)
  
    return msg.result

def arm(arm_command):
    mav_connection.wait_heartbeat()
    
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    return msg.result


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


def upload_qgc_mission(mission_file: str, the_connection: mavutil.mavlink_connection) -> Optional[bool]:
 
    mission = read_qgc_mission(mission_file)
    if not mission:
        return None

    wploader = []

    seq = 0  

    add_home_position_waypoint(wploader, mission, seq)
    seq += 1  # Increase waypoint sequence

    for wp in mission["mission"]["items"]:
        print(wp)
        add_waypoint(wploader, the_connection, wp, seq)
        seq += 1  # Increase waypoint sequence for the next waypoint
    
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
    # if ack.type != 0:
    #     print(f'{error_msg}: {ack.type}')
    #     return False
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
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("127.0.0.1", 12344))  # Server IP and port
server_socket.listen(1)  # Listen for incoming connections

print("Server is listening...")

connection, address = server_socket.accept()
print(f"Connection from {address}")

while True:
    data = connection.recv(1024)
    if not data:
        break

    received_data = data.decode()
    print(f"Received from client: {received_data}")
    #print(f"Received from client[2]: {received_data[2]}")
    # Send an acknowledgment back to the client
    acknowledgment = "Message received: " + received_data
    connection.send(acknowledgment.encode())

    if received_data == "exit":
        break  # Exit the loop if the client sends 'exit'

    if received_data== "arm":
        result = arm(1)
      

    if received_data== "disarm":
        result = arm(0)

    if received_data== "takeoff":
        print("enterted")
        acknowledgment = "Enter altitude" 
        connection.send(acknowledgment.encode())
        alt = connection.recv(1024)
        altitude=alt.decode()
        int_alt=int(altitude)
        print("alitutde=")
        print(int_alt)
        result = takeoff(int_alt)
        print(f'Result of takeoff: {result}')

    if received_data== "GUIDED":
        #mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')   
        print("entered into guided mode") 
        mav_connection.wait_heartbeat()
        result = change_mode(mav_connection, 'GUIDED')
    if received_data== "LAND":
        #mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')   
        print("entered into LAND mode") 
        mav_connection.wait_heartbeat()
        result = change_mode(mav_connection, 'LAND')

    if received_data== "STABILIZE":
        #mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')   
        print("entered into STABILIZE mode") 
        mav_connection.wait_heartbeat()
        result = change_mode(mav_connection, 'STABILIZE')

    if received_data== "AUTO":
        #mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')   
        print("entered into AUTO mode") 
        mav_connection.wait_heartbeat()
        if auto_alt_wp==1:
            result=takeoff(1)
            auto_alt_wp=0
        

        mav_connection.wait_heartbeat()
        result = change_mode(mav_connection, 'AUTO')

    if received_data== "Mission":
        # Receive and write the file

        print("into misssion uploader file is being imorted\n")
        with open("received_file.txt", "wb") as file:
            print("opening file\n")
            y=0
            while y<4:
                data = connection.recv(1024)
                # if  data=="\nNULL":
                #     print("breaking\n")
                #     break
                # if not data:
                #     break
                
                #data=dataa.decode()
                #print("data=")
                #print(data)
                
                file.write(data)
                y+=1
                # y+=1
                
                #break
        print("breaked")        
        file.close()    
        with open("received_file.txt", "rb") as file:

            
            file_contents = file.read()

        
        file_contents_str = file_contents.decode('utf-8')

        
        print("Contents of the received file:")
        print(file_contents_str)  
        upload_qgc_mission("received_file.txt",mav_connection)
        auto_alt_wp=1
        print("uploaded mission")  

    




connection.close()
