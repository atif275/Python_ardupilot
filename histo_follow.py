import cv2
from pymavlink import mavutil
import os
import numpy as np
import time
import math
from PIL import Image
from PIL import ImageFile
# Connect to the MAVLink
mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
mav_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (mav_connection.target_system, mav_connection.target_component))

temp=0

image_directory = '/tmp/camera_save_tutorial' 
def find_obj(template_path, scene_path):
    # Read images
    template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
    scene = cv2.imread(scene_path, cv2.IMREAD_GRAYSCALE)

    # Initialize SIFT detector
    sift = cv2.SIFT_create()

    # Find keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(template, None)
    kp2, des2 = sift.detectAndCompute(scene, None)

    # FLANN parameters
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)

    # FLANN-based matcher
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1, des2, k=2)

    # Ratio test as per Lowe's paper
    good_matches = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good_matches.append(m)
    average = []
    # Check if good_matches is not empty before calculating average coordinates
    if good_matches:
        # Accumulate coordinates of good matches
        total_x, total_y = 0, 0
        for match in good_matches:
            query_idx = match.queryIdx
            train_idx = match.trainIdx
            x, y = kp2[train_idx].pt
            total_x += x
            total_y += y

        # Calculate average coordinates
        average_x = total_x / len(good_matches)
        average_y = total_y / len(good_matches)
        average = average_x,average_y

        # Print and display results
        print(f"Average Pixel Coordinates: ({int(average[0])}, {int(average[1])})")
        return average
    else:
        print("No good matches found.")
        return -1

    # Draw matches
    # img_matches = cv2.drawMatches(template, kp1, scene, kp2, good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    # cv2.imshow('Matches', img_matches)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

def find_object(template_path, scene_path):
    # Read the template and scene images
    template = cv2.imread(template_path, cv2.IMREAD_COLOR)
    scene = cv2.imread(scene_path, cv2.IMREAD_COLOR)
    template = cv2.resize(template, (int(template.shape[1] * 0.5), int(template.shape[0] * 0.5)))

    # Convert images to grayscale
    template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    scene_gray = cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY)

    # Calculate histograms
    hist_template = cv2.calcHist([template_gray], [0], None, [256], [0, 256])
    hist_scene = cv2.calcHist([scene_gray], [0], None, [256], [0, 256])
    i=0
    print("hist of scene : ...........................")
    while(i<256):
       # hist_scene[i]=hist_scene[i]*109200
        print(int(hist_scene[i]))
       #hist_template[i]=hist_template[i]*29601
       
        i=i+1
    i=0
    print("hist of template : .......................... ")
    while(i<256):
        print(int(hist_template[i]))
        i=i+1
     
    #print(f"histogram of object : {hist_template}")
    #print(f"histogram of frame : {hist_scene}")
    # Normalize histograms
    cv2.normalize(hist_template, hist_template, 0, 1, cv2.NORM_MINMAX)
    cv2.normalize(hist_scene, hist_scene, 0, 1, cv2.NORM_MINMAX)

    # Apply histogram comparison methods
    methods = [cv2.HISTCMP_CORREL, cv2.HISTCMP_CHISQR, cv2.HISTCMP_INTERSECT]
    results = []

    for method in methods:
        result = cv2.compareHist(hist_template, hist_scene, method)
        results.append(result)

    # Get the index of the method with the maximum result
    best_method_index = np.argmax(results)

    # Find the location of the template in the scene using template matching
    res = cv2.matchTemplate(scene_gray, template_gray, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

    # Draw a rectangle around the detected area
    h, w = template_gray.shape
    if best_method_index == 0:  # Use histogram correlation for more accurate results
        top_left = max_loc
    else:  # Use template matching result
        top_left = max_loc

    bottom_right = (top_left[0] + w, top_left[1] + h)
    average_coordinates = ((top_left[0] + bottom_right[0]) // 2, (top_left[1] + bottom_right[1]) // 2)

    cv2.rectangle(scene, top_left, bottom_right, (0, 255, 0), 2)
    print(f"Average Pixel Coordinates: {average_coordinates}")

    # Display the result
    # cv2.imshow('Result', scene)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    output_directory='/Users/ATIFHANIF/Desktop/project/ardupilot/python/pic'
    if best_method_index == 0 or len(scene[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]) > 0:
        print(f"Average Pixel Coordinates: {average_coordinates}")
        filename= os.path.join(output_directory, f"result_{temp}.png")
        if not os.path.exists(filename):
            cv2.imwrite(filename, scene)
            print(f"Image saved as {filename}")
    
        return average_coordinates
    else:
        print("No object detected.")
        return -1


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
template_path='/Users/ATIFHANIF/Desktop/project/ardupilot/python/Archive/patch17.png'
region_size=5
while True:
    image_files = [f for f in os.listdir(image_directory) if f.endswith('.jpg')]
    if image_files:
        latest_image_file = max(image_files, key=lambda x: os.path.getctime(os.path.join(image_directory, x)))
        image_path = os.path.join(image_directory, latest_image_file)
        res=find_obj(template_path, image_path )
        print(f".....result ========={res}")


        ImageFile.LOAD_TRUNCATED_IMAGES = True
        image = Image.open(image_path)

        width, height = image.size
    
        # red_object_coordinates = []
 
        # for x in range(width):
        #     for y in range(height):
             
        #         r, g, b = image.getpixel((x, y))
                
               
        #         if r > 180 and g < 100 and b < 100:
        #             red_object_coordinates.append((x, y))
        

        center_x = width // 2
        center_y = height // 2
        
     
        if (res==-1):
            
            # if(pre_avg_y>370):
            #     altt=get_alt()
            #     #altt+=0.6
            #     #altt-=0.5
            #     #print(f"landing at: {int(altt)}")
            #     move_drone(0,0,0,0)
            #     time.sleep(3)
            #     angle=math.radians(42.97)
            #     print(f"angle in radian {angle}")
            #     d=altt*math.tan(angle)
            #     print(f"distace={d}")

            #     move_drone(d-1.5,0,0,0)
            #     time.sleep(2)
            #     change_mode(mav_connection,'LAND')
            #     #move_drone(1,0,int(altt),0)

            #     break

            print("No red object detected")
            move_drone(0,0,0,0.2);
        else:

            # if(len(red_object_coordinates)>3500):
            #     print("stoping drone")
            #     print("ahead is object")
            #     #move_drone(0,0,0,0);

            # else:
                avg_x = int(res[0])
                avg_y = int(res[1])
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
                    "Number of red pixels": 1,
                    "width":(width),
                    "height":(height),
                    "center x,y":(center_x,center_y),
                    "forward_backward":(forward_backward),
                    "left_right":(left_right)
                }
                
                print(result_details)
                # if(len(red_object_coordinates)>30000):
                #     print("object ahead")
                #     move_drone(0,0,0,0)
                #     time.sleep(1)
                #     move_drone(0,0,-1,0)
                #     time.sleep(1)
                #     move_drone(1,0,0,0)
                #     time.sleep(1)
                #     print("landing")
                #     change_mode(mav_connection,'LAND')
                #     break


                if ((avg_x >lx and avg_x < rx)):
                    if(avg_y>ry):
                        print("moving diagnoly")
                        move_drone(1,0,0.3,0)
                    elif(avg_y<ly):
                        print("moving up diagnolly")
                        move_drone(1,0,-0.3,0)
                    else:
                        print("moving forward")
                        move_drone(1,0,0,0)
                
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

                    # if(avg_y<ly):
                    #     print("moving up")
                    #     move_drone(0,0,-1,0);
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



