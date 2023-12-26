import cv2
from pymavlink import mavutil
import os
import time
import math
from PIL import Image
from PIL import ImageFile
# Connect to the MAVLink
mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
mav_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (mav_connection.target_system, mav_connection.target_component))


image_directory = '/tmp/camera_save_tutorial' 



def move_drone(vx, vy, vz, yaw_rate):
    mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, mav_connection.target_system,
                        mav_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b010111111000), vx, vy, vz, 0, 0, 0, 0, 0, 0, 0, yaw_rate))

def get_alt():
    msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT')
    if msg is not None:
            lat = msg.lat / 1e7  
            lon = msg.lon / 1e7  
            altitude_mm = msg.alt  
            alt = altitude_mm / 1000.
            agl = (msg.relative_alt / 1000.0 ) 
            gps_speed = msg.vx / 100.0  
            #print(f"GPS - Latitude: {lat}, Longitude: {lon}, Altitude: {agl}, AGL: {agl}, GPS Speed: {gps_speed}")
            print(f"alt={agl}")
            return agl
        
    #print(msg)
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
    #ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    # print(ack_msg)
    #return ack_msg.result               
pre_avg_y=0

region_size=5
while True:
    image_files = [f for f in os.listdir(image_directory) if f.endswith('.jpg')]
    if image_files:
        latest_image_file = max(image_files, key=lambda x: os.path.getctime(os.path.join(image_directory, x)))
        image_path = os.path.join(image_directory, latest_image_file)
        ImageFile.LOAD_TRUNCATED_IMAGES = True
        image = Image.open(image_path)

        width, height = image.size
    
        red_object_coordinates = []
 
        for x in range(width):
            for y in range(height):
             
                r, g, b = image.getpixel((x, y))
                
               
                if r > 180 and g < 100 and b < 100:
                    red_object_coordinates.append((x, y))
        

        center_x = width // 2
        center_y = height // 2
        
     
        if not red_object_coordinates:
            if(pre_avg_y>370):
                altt=get_alt()
                #altt+=0.6
                #altt-=0.5
                #print(f"landing at: {int(altt)}")
                move_drone(0,0,0,0)
                time.sleep(3)
                angle=math.radians(42.97)
                print(f"angle in radian {angle}")
                d=altt*math.tan(angle)
                print(f"distace={d}")

                move_drone(d-1.5,0,0,0)
                time.sleep(2)
                change_mode(mav_connection,'LAND')
                #move_drone(1,0,int(altt),0)

                break

            print("No red object detected")
            move_drone(0,0,0,0.2);
        else:

            # if(len(red_object_coordinates)>3500):
            #     print("stoping drone")
            #     print("ahead is object")
            #     #move_drone(0,0,0,0);

            # else:
                avg_x = sum(coord[0] for coord in red_object_coordinates) // len(red_object_coordinates)
                avg_y = sum(coord[1] for coord in red_object_coordinates) // len(red_object_coordinates)
                forward_backward = (center_y - height / 2) / (height / 2)
                left_right = (center_x - width / 2) / (width / 2)
                lx=center_x-40
                ly=center_y-40
                rx=center_x+40
                ry=center_y+40
                # lx=280
                # ly=200      
                # rx=360
                # ry=280
                pre_avg_y=avg_y
              
                position = ""
                if avg_x < center_x:
                    position += "left "
                elif avg_x > center_x:
                    position += "right "
                
                if avg_y < center_y:
                    position += "up"
                elif avg_y > center_y:
                    position += "down"
                
                result_details = {
                    "Average Coordinates": (avg_x, avg_y),
                    "lx,ly=":(lx,ly),
                    "rx,ry=":(rx,ry),
                    "Position relative to center": position,
                    "Number of red pixels": len(red_object_coordinates),
                    "width":(width),
                    "height":(height),
                    "center x,y":(center_x,center_y),
                    "forward_backward":(forward_backward),
                    "left_right":(left_right)
                }
                
                print(result_details)
                

                
                if ((avg_x >lx and avg_x < rx)):
                    print("moving forward")
                    move_drone(1.5,0,0,0)
                
                # elif(avg_y>ry and not red_object_coordinates) :
                    

                    # if avg_y < center_y:
                    #     print("moving up")
                    #     move_drone(0,0,-0.6,0);
                    # elif avg_y > center_y:
                    #     print("moving down")
                    #     move_drone(0,0,0.6,0);    
                # if(avg_y<ly and avg_y<ry):
                #     print("moving up")
                #     move_drone(0,0,-0.6,0);
                # elif(avg_y>ly and avg_y>ry):
                #     print("moving down")
                #     move_drone(0,0,0.6,0); 


                # r1,g1,b1 = image.getpixel((center_x, center_y))
                # if (r1 > 200 and g1 < 100 and b1 < 100):
                #             print("moving forward")
                #             move_drone(1,0,0,0)

                # center_region = [

                #     (x, y) for x in range(center_x - region_size, center_x + region_size)
                #     for y in range(center_y - region_size, center_y + region_size)
                # ]
                # print(f"center_region ={len(center_region)}")
                # red_center_pixels = [coord for coord in red_object_coordinates if coord in center_region]
                # print(f"red_center_pixels ={len(red_center_pixels)}")
                # if len(red_center_pixels) >= len(center_region) / 2:
                #     print("Moving forward")
                #     move_drone(1, 0, 0, 0)


                else:

                    if avg_x < center_x:
                        print("moving left")
                        move_drone(0,0,0,-0.2);
                        # time.sleep(1) 
                        # move_drone(0,0,0,0);
                        #position += "left "
                    elif avg_x > center_x:
                        print("moving right")
                        move_drone(0,0,0,0.2);

                    if(avg_y<ly):
                        print("moving up")
                        move_drone(0,0,-1,0);
                    # elif(avg_y>ry):
                    #     print("moving down")
                    #     move_drone(0,0,1,0);

                    # elif(avg_y>370 and avg_y<460):
                    #     altt=get_alt()
                    #     altt+=0.5
                    #     print(f"landing at: {altt}")
                    #     move_drone(0,0,int(altt),0)
                    #     break

                    
                        # time.sleep(1) 
                        # move_drone(0,0,0,0);
                        #position += "right "
                # if(len(red_object_coordinates)>8000):
                #     print("stoping drone")
                #     move_drone(0,0,0,0);


    else:
        print(f"No JPEG images found in {image_directory}")


    # time.sleep(1)            



