import time
from pymavlink import mavutil

# Connect to the SITL instance (adjust the IP and port as needed)
#master = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
mav_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
mav_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
        (mav_connection.target_system, mav_connection.target_component))

    # Wait for the connection to be established
    # while True:
    #     msg = master.recv_msg()
    #     if msg.get_type() == 'HEARTBEAT':
    #         print("Connection to SITL established.")
    #         break
    #     time.sleep(1)
print("gg")
mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    # Send an arming command (MAV_CMD_COMPONENT_ARM_DISARM)
# arming_command = mav_connection.mav.command_long_encode(
#         1, 1,                   # Target System ID and Target Component ID
#         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # Command
#         0,                       # Confirmation
#         1,                       # Arming command (1 to arm, 0 to disarm)
#         0, 0, 0, 0, 0, 0        # Parameters (not used for arming)
#     )

#master.mav.send(arming_command)


