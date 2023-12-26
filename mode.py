import sys
from pymavlink import mavutil
import socket
import argparse

# server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server_socket.bind(("127.0.0.1", 12347))  # Server IP and port
# server_socket.listen(1)  # Listen for incoming connections

# print("Server is listening...")

# connection, address = server_socket.accept()
# print(f"Connection from {address}")

# received_data = b""  # Initialize an empty byte string to store received data

# while True:
#     data = connection.recv(1024)
#     if not data:
#         break
#     print(f"Received from client: {data.decode()}")
#     received_data += data  # Append the received data to the existing data
#     break

# # Now you can use the received_data for other purposes
# print("Received data from the clientt: ", received_data.decode())
# print("01")

# connection.close()

# print("e1")

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


# mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')    
# mav_connection.wait_heartbeat()
# print("Heartbeat from system (system %u component %u)" % (mav_connection.target_system, mav_connection.target_component))
# if received_data.decode()== "GUIDED":
#         result = change_mode(mav_connection, 'GUIDED')



    # if autopilot == 'ardupilot':
    #     print("reached")
    #     if mode not in master.mode_mapping():
    #         print(f'Unknown mode : {mode}')
    #         print(f"available modes: {list(master.mode_mapping().keys())}")
    #         raise Exception('Unknown mode')
        
    #     # Get mode ID
    #     mode_id = master.mode_mapping()[mode]
    #     sub_mode = 0
    # elif autopilot == 'px4':
    #     # Get mode ID
    #     mode_id = main_mode_mapping_px4[mode]
    #     sub_mode = sub_mode_mapping_px4[sub_mode]



    # master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    #                             0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, sub_mode, 0, 0, 0, 0)
    # ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    # print(ack_msg)
    # return ack_msg.result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Change mode of the drone')
    parser.add_argument('--mode', type=str, default='STABILIZE', help='Mode to change to')
    parser.add_argument("--sysid", type=int, default=1)
    args = parser.parse_args()

    the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
        (the_connection.target_system, the_connection.target_component))

    change_mode(the_connection, args.mode)
   