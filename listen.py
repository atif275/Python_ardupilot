from pymavlink import mavutil
print("prointing")
# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
print("\nprointing2")
# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("\nprointing3")
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

# while 1:
#     msg = the_connection.recv_match(type='ATTITUDE', blocking=True)
#     print(msg)