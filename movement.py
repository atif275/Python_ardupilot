from pymavlink import mavutil
import time
mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
mav_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (mav_connection.target_system, mav_connection.target_component))


mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, mav_connection.target_system,
                        mav_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b010111111000), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.1))

# mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, mav_connection.target_system,
#                         mav_connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000), int(-35.3629849 * 10 ** 7), int(149.1649185 * 10 ** 7), 10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))

# mav_connection.mav.send(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT(10, mav_connection.target_system,
#                         mav_connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 0,0, 0, 0, 0, -35.36267574, 149.16605724, 5))
# mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
#                                          mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,0, 0, 0, 0, -35.36267574, 149.16605724, 5)

# msg_set_mode = mav_connection.mav.command_long_send(
#     mav_connection.target_system,
#     mav_connection.target_component,
#     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#     0,  # confirmation
#     mavutil.mavlink.MAV_MODE_AUTO_ARMED,  # Custom mode value for AUTO
#     0,  # param2 (custom sub-mode, set to 0 for default)
#     0,  # param3 (custom sub-mode, set to 0 for default)
#     0,  # param4 (custom sub-mode, set to 0 for default)
#     0,  # param5 (custom sub-mode, set to 0 for default)
#     0,  # param6 (custom sub-mode, set to 0 for default)
#     0   # param7 (custom sub-mode, set to 0 for default)
# )
# time.sleep(2)

# msg = mav_connection.mav.command_long_send(
#     mav_connection.target_system,
#     mav_connection.target_component,
#     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
#     0,  # confirmation
#     0,  # param1 (autocontinue)
#     0,  # param2
#     0,  # param3 (Hold Time, set to 0 for no hold time)
#     0,  # param4
#     -35.36267574,  # Latitude
#     149.16605724,  # Longitude
#     5  # Altitude in meters
# )
# lat=-int(35.36267574 *1e7)
# lon=int(149.1660572 *1e7)
# msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(
#     0,
#     0,
#     0,
#     6,
#     3576,
#     lat,
#     lon,
#     10,
#     0, 0, 0,
#     0,0,0,0,0
# )
# # SET_POSITION_TARGET_GLOBAL_INT 0 0 0 6 3576 -353621474 1491651746 10 0 0 0 0 0 0 0 0
# mav_connection.mav.send(msg)

#mav_connection.mav.send(msg)
# while 1:
#     msg = mav_connection.recv_match(
#         type='LOCAL_POSITION_NED', blocking=True)
#     print(msg)

#conversion of gps to drone refrence  
#conversion of drone refrence to gps 
#relative velocity mode speed wala only take input of x,y,z of velocity with key
