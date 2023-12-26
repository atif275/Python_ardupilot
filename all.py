import argparse
from pymavlink import mavutil
import socket


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("127.0.0.1", 12340))  # Server IP and port
server_socket.listen(1)  # Listen for incoming connections

print("Server is listening...")

connection, address = server_socket.accept()
print(f"Connection from {address}")

received_data = b""  # Initialize an empty byte string to store received data

while True:
    data = connection.recv(1024)
    if not data:
        break
    print(f"Received from client: {data.decode()}")
    received_data += data  # Append the received data to the existing data
    break

# Now you can use the received_data for other purposes
print("Received data from the clientt: ", received_data.decode())
print("01")

connection.close()

print("e1")
def arm(mav_connection, arm_command):
 
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    return msg.result

main_mode_mapping_px4 = {
    # 'MANUAL': 0,
    # 'ALTCTL': 1,
    # 'POSCTL': 2,
    # 'AUTO': 3,
    # 'ACRO': 4,
    # 'OFFBOARD': 5,
    # 'STABILIZED': 6,
    # 'RATTITUDE': 7,
     0 : 'STABILIZE',
    1 : 'ACRO',
    2 : 'ALT_HOLD',
    3 : 'AUTO',
    4 : 'GUIDED',
    5 : 'LOITER',
    6 : 'RTL',
    7 : 'CIRCLE',
    8 : 'POSITION',
    9 : 'LAND',
    10 : 'OF_LOITER',
    11 : 'DRIFT',
    13 : 'SPORT',
    14 : 'FLIP',
    15 : 'AUTOTUNE',
    16 : 'POSHOLD',
    17 : 'BRAKE',
    18 : 'THROW',
    19 : 'AVOID_ADSB',
    20 : 'GUIDED_NOGPS',
    21 : 'SMART_RTL',
    22 : 'FLOWHOLD',
    23 : 'FOLLOW',
    24 : 'ZIGZAG',
    25 : 'SYSTEMID',
    26 : 'AUTOROTATE',
    27 : 'AUTO_RTL',
}
sub_mode_mapping_px4 = {
    'READY': 0,
    'TAKEOFF': 1,
    'HOLD': 2, 
    'MISSION': 3,
    'RETURN_TO_LAUNCH': 4,
    'LAND': 5,
    'FOLLOW_ME': 6,
}
def change_mode(master, mode, autopilot='ardupilot', sub_mode='NONE'):

    if autopilot == 'ardupilot':
        print("reached")
        if mode not in master.mode_mapping():
            print(f'Unknown mode : {mode}')
            print(f"available modes: {list(master.mode_mapping().keys())}")
            raise Exception('Unknown mode')
        
        # Get mode ID
        mode_id = master.mode_mapping()[mode]
        sub_mode = 0
    elif autopilot == 'px4':
        # Get mode ID
        mode_id = main_mode_mapping_px4[mode]
        sub_mode = sub_mode_mapping_px4[sub_mode]


    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode, sub_mode, 0, 0, 0, 0)
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(ack_msg)
    return ack_msg.result



def takeoff(mav_connection, altitude):
 
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True) 
    print(msg)
  
    return msg.result





if received_data.decode() == "arm":
    print("enter into arm state")
    mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
    print("enter into arm state")
    result = arm(mav_connection, 1)
if received_data.decode() == "disarm":
    mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
    print("enter into disarm state")
    result = arm(mav_connection, 0)

if received_data.decode() == "GUIDED":
    mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
    print("enterd in guided mode")
    result = change_mode(mav_connection,"GUIDED")

if received_data.decode() == "LAND":
    mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
    result = change_mode(mav_connection,"LAND")

if received_data.decode() == "STABILIZE":
    mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
    result = change_mode(mav_connection,"STABILIZE")

if received_data.decode() == "takeoff":
    mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
    alt=0
    print("enter altitude for take off")
    input(alt)
    result = takeoff(mav_connection, alt)
# if __name__ == "__main__":
#     parser = argparse.ArgumentParser(description='Send arm/disarm commands using MAVLink protocol.')
#     parser.add_argument('-c', '--connect', help="Connection string", default='udpin:localhost:14550')
#     parser.add_argument('-a', '--arm', type=int, choices=[0, 1], help="Arm/Disarm command", default=1)
#     args = parser.parse_args()

#     mav_connection = mavutil.mavlink_connection(args.connect)
#     result = arm(mav_connection, args.arm)
#     print(f'Result of arm/disarm command: {result}')