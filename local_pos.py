from pymavlink import mavutil
from math import radians, cos, sin

# Connection to the vehicle (replace 'udpin:0.0.0.0:14550' with your connection string)
connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
def ned_to_lla(ref_lat, ref_lon, ref_alt, ned):
    """
    Convert NED coordinates to LLA (Latitude, Longitude, Altitude).
    ref_lat, ref_lon, ref_alt: Reference point (origin) in LLA.
    ned: NED coordinates (North, East, Down).
    Returns: Calculated LLA coordinates.
    """
    earth_radius = 6378137.0  # Earth's radius in meters

    lat = ref_lat + (ned[0] / earth_radius) * (180.0 / 3.141592653589793)
    lon = ref_lon + (ned[1] / earth_radius) * (180.0 / 3.141592653589793) / cos(radians(ref_lat))
    alt = ref_alt - ned[2]  # Altitude is in the opposite direction (Down)

    return lat, lon, alt
ref_latitude = -35.3629849
ref_longitude = 149.1649185
ref_altitude = 583.99
while True:
    msg = connection.recv_msg()
    if msg is not None and msg.get_type() == 'LOCAL_POSITION_NED':
        # Accessing NED coordinates
        ned_coordinates = (msg.x, msg.y, msg.z)

        # Accessing reference coordinates (usually sent separately or known)
        # ref_latitude = msg.origin_lat
        # ref_longitude = msg.origin_lon
        # ref_altitude = msg.origin_alt
        # print(f"Referance Latitude: {ref_latitude} degrees")
        # print(f"Referance Longitude: {ref_longitude} degrees")
        # print(f"Referance Altitude: {ref_altitude} meters")
        # Calculate relative LLA coordinates
        relative_lat, relative_lon, relative_alt = ned_to_lla(ref_latitude, ref_longitude, ref_altitude, ned_coordinates)

        print(f"Relative Latitude: {relative_lat} degrees")
        print(f"Relative Longitude: {relative_lon} degrees")
        print(f"Relative Altitude: {relative_alt} meters")
