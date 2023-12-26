import argparse
from pymavlink import mavutil
import socket


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
def arm(mav_connection, arm_command):
 
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    return msg.result


print("e2")








# if received_data.decode() == "arm":
#     print("enter into arm statee")
#     mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
#     print("enter into arm state")
#     result = arm(mav_connection, 1)
# if received_data.decode() == "disarm":
#     mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
#     print("enter into disarm state")
#     result = arm(mav_connection, 0)




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Send arm/disarm commands using MAVLink protocol.')
    parser.add_argument('-c', '--connect', help="Connection string", default='udpin:localhost:14550')
    parser.add_argument('-a', '--arm', type=int, choices=[0, 1], help="Arm/Disarm command", default=1)
    args = parser.parse_args()

    mav_connection = mavutil.mavlink_connection(args.connect)
    result = arm(mav_connection, args.arm)
    print(f'Result of arm/disarm command: {result}')