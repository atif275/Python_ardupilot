import argparse
from pymavlink import mavutil

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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Send takeoff command with altitude using MAVLink protocol.')
    parser.add_argument('-c', '--connect', help="Connection string", default='udpin:localhost:14550')
    parser.add_argument("-takeoff", '--altitude', type=int, default=4)
    args = parser.parse_args()

    mav_connection = mavutil.mavlink_connection(args.connect)
    result = takeoff(mav_connection, args.altitude)
    print(f'Result of takeoff: {result}')
    