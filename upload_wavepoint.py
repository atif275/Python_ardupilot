import argparse
import json
from typing import Dict, Optional

from pymavlink import mavutil

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

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Upload a mission created by QGroundControl to a vehicle.')
    parser.add_argument('--mission_file', type=str, default='plan2.txt',
                        help='Path to the mission file.')
    parser.add_argument('-c', '--connect', help="Connection string", default='udpin:localhost:14550')

    args = parser.parse_args()
    mav_connection = mavutil.mavlink_connection(args.connect)

    mav_connection.wait_heartbeat()
    print(f"Heartbeat from system (system {mav_connection.target_system} component {mav_connection.target_component})")
    upload_qgc_mission(args.mission_file, mav_connection)