# # # # import cv2
# # # # import numpy as np
# # # # import os
# # # # from pymavlink import mavutil
# # # # import time

# # # # print('Connecting...')
# # # # mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# # # # mav_connection.wait_heartbeat()
# # # # print("Heartbeat from system (system %u component %u)" %
# # # #       (mav_connection.target_system, mav_connection.target_component))

# # # # # Function to set position relative to the current drone position
# # # # def set_position_relative(x, y, z):
# # # #     msg = mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
# # # #         10, mav_connection.target_system, mav_connection.target_component,
# # # #         mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b010111111000), x, y, z, 0, 0, 0, 0, 0, 0, 0, 0))

# # # # # Function to rotate the drone by setting yaw rate
# # # # def rotate_drone(yaw_rate):
# # # #     msg = mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
# # # #         10, mav_connection.target_system, mav_connection.target_component,
# # # #         mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b010111111000), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, yaw_rate))

# # # # # Directory where images are saved
# # # # image_dir = "/tmp/camera_save_tutorial"

# # # # # Main loop
# # # # while True:
# # # #     # Get the latest image file in the directory
# # # #     image_files = [f for f in os.listdir(image_dir) if f.endswith('.jpg')]
# # # #     if image_files:
# # # #         latest_image_file = max(image_files, key=lambda x: os.path.getctime(os.path.join(image_dir, x)))
# # # #         image_path = os.path.join(image_dir, latest_image_file)

# # # #         # Read the image
# # # #         image = cv2.imread(image_path)

# # # #         # Convert BGR to HSV
# # # #         hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# # # #         # Define the range of red color in HSV
# # # #         lower_red = np.array([0, 100, 100])
# # # #         upper_red = np.array([10, 255, 255])

# # # #         # Threshold the HSV image to get only red colors
# # # #         mask = cv2.inRange(hsv, lower_red, upper_red)

# # # #         # Find contours in the mask
# # # #         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# # # #         # Process each contour
# # # #         if contours:
# # # #             print("red found")
# # # #             for contour in contours:
# # # #                 # Get the bounding box of the contour
# # # #                 x, y, w, h = cv2.boundingRect(contour)

# # # #                 # Calculate normalized positions
# # # #                 norm_x = (x + w / 2 - image.shape[1] / 2) / (image.shape[1] / 2)
# # # #                 norm_y = (y + h / 2 - image.shape[0] / 2) / (image.shape[0] / 2)

# # # #                 # Adjust position based on object position
# # # #                 print(f"normx={norm_x}")
# # # #                 print(f"normy={norm_y}")
# # # #                 set_position_relative(-norm_x, norm_y, 0)  # Move forward relative to the current position

# # # #                 # Break out of the loop after processing the first contour
# # # #                 break
# # # #         else:
# # # #             # Rotate the drone if no red object is detected
# # # #             print("no red found")
# # # #             rotate_drone(0.5)  # Set the desired yaw rate

# # # #         # Show the processed image
# # # #         cv2.imshow('Object Detection', image)
# # # #         cv2.waitKey(1)

# # # #     # Introduce a delay to control the loop frequency
# # # #     time.sleep(1)

# # # # # Close the MAVLink connection
# # # # mav_connection.close()

# # # import cv2
# # # import numpy as np
# # # import os
# # # from pymavlink import mavutil
# # # import time

# # # print('Connecting...')
# # # mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# # # mav_connection.wait_heartbeat()
# # # print("Heartbeat from system (system %u component %u)" %
# # #       (mav_connection.target_system, mav_connection.target_component))

# # # # Function to set position relative to the current drone position
# # # def set_position_relative(x, y, z):
# # #     # Scaling factors (adjust as needed)
# # #     x_scale = 2.0  # Adjust as needed
# # #     y_scale = 2.0  # Adjust as needed
# # #     z_scale = 2.0  # Adjust as needed

# # #     # Apply scaling factors to adjust position
# # #     x *= x_scale
# # #     y *= y_scale
# # #     z *= z_scale

# # #     # Send MAVLink command to set position
# # #     msg = mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
# # #         10, mav_connection.target_system, mav_connection.target_component,
# # #         mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b010111111000), x, y, z, 0, 0, 0, 0, 0, 0, 0, 0))

# # # # Function to rotate the drone by setting yaw rate
# # # def rotate_drone(yaw_rate):
# # #     msg = mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
# # #         10, mav_connection.target_system, mav_connection.target_component,
# # #         mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b010111111000), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, yaw_rate))

# # # # Directory where images are saved
# # # image_dir = "/tmp/camera_save_tutorial"

# # # # Main loop
# # # while True:
# # #     # Get the latest image file in the directory
# # #     image_files = [f for f in os.listdir(image_dir) if f.endswith('.jpg')]
# # #     if image_files:
# # #         latest_image_file = max(image_files, key=lambda x: os.path.getctime(os.path.join(image_dir, x)))
# # #         image_path = os.path.join(image_dir, latest_image_file)

# # #         # Read the image
# # #         image = cv2.imread(image_path)

# # #         # Convert BGR to HSV
# # #         hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# # #         # Define the range of red color in HSV
# # #         lower_red = np.array([0, 100, 100])
# # #         upper_red = np.array([10, 255, 255])

# # #         # Threshold the HSV image to get only red colors
# # #         mask = cv2.inRange(hsv, lower_red, upper_red)

# # #         # Find contours in the mask
# # #         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# # #         # Process each contour
# # #         if contours:
# # #             for contour in contours:
# # #                 # Get the bounding box of the contour
# # #                 x, y, w, h = cv2.boundingRect(contour)

# # #                 # Calculate normalized positions
# # #                 norm_x = (x + w / 2 - image.shape[1] / 2) / (image.shape[1] / 2)
# # #                 norm_y = (y + h / 2 - image.shape[0] / 2) / (image.shape[0] / 2)

# # #                 # Adjust position based on object position
# # #                 set_position_relative(norm_x, norm_y, 0)  # Move forward relative to the current position

# # #                 # Break out of the loop after processing the first contour
# # #                 #break
# # #         else:
# # #             # Rotate the drone if no red object is detected
# # #             rotate_drone(0.5)  # Set the desired yaw rate

# # #         # Show the processed image
# # #         # cv2.imshow('Object Detection', image)
# # #         # cv2.waitKey(1)

# # #     # Introduce a delay to control the loop frequency
# # #     time.sleep(1)

# # # # Close the MAVLink connection
# # # mav_connection.close()
# # import cv2
# # from pymavlink import mavutil
# # import time
# # import math

# # # Connect to the MAVLink
# # mav_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')  # Update with your drone's connection details

# # # Set the video capture source
# # cap = cv2.VideoCapture('/tmp/camera_save_tutorial')  # Update with your camera source

# # # Define the lower and upper bounds of the red color (in HSV)
# # lower_red = (0, 100, 100)
# # upper_red = (10, 255, 255)

# # # Function to move the drone
# # def move_drone(x, y, z, vx, vy, vz, afx, afy, afz, yaw_rate):
# #     mav_connection.mav.send(
# #         mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
# #             10, mav_connection.target_system, mav_connection.target_component,
# #             mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b110111111000, x, y, z, vx, vy, vz, afx, afy, afz, 0, 0, yaw_rate
# #         )
# #     )

# # # Main loop
# # while True:
# #     ret, frame = cap.read()

# #     if not ret:
# #         print("Error reading frame")
# #         break

# #     # Convert the frame to HSV
# #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# #     # Threshold the image to get only red regions
# #     mask = cv2.inRange(hsv, lower_red, upper_red)

# #     # Find contours in the mask
# #     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# #     if contours:
# #         # Get the largest contour
# #         largest_contour = max(contours, key=cv2.contourArea)

# #         # Get the bounding box of the contour
# #         x, y, w, h = cv2.boundingRect(largest_contour)

# #         # Calculate the center of the bounding box
# #         center_x = x + w // 2
# #         center_y = y + h // 2

# #         # Get the width and height of the frame
# #         frame_width = cap.get(3)
# #         frame_height = cap.get(4)

# #         # Calculate the desired drone movement based on the position of the red object
# #         forward_backward = (center_y - frame_height / 2) / (frame_height / 2)
# #         left_right = (center_x - frame_width / 2) / (frame_width / 2)

# #         # Adjust the scaling factor for your specific drone's characteristics
# #         scaling_factor = 0.1

# #         # Calculate yaw rate for rotation
# #         yaw_rate = left_right * math.pi  # Adjust as needed

# #         # Move the drone
# #         move_drone(forward_backward * scaling_factor, left_right * scaling_factor, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, yaw_rate)

# #         # Display the frame with the bounding box
# #         cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
# #         cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

# #     # Display the frame
# #     cv2.imshow('Frame', frame)

# #     # Exit the loop if 'q' is pressed
# #     if cv2.waitKey(1) & 0xFF == ord('q'):
# #         break

# # # Release the video capture and close all windows
# # cap.release()
# # cv2.destroyAllWindows()
# import cv2
# from pymavlink import mavutil
# import os
# import time
# import math

# # Connect to the MAVLink
# mav_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')  # Update with your drone's connection details

# # Set the image directory path
# image_directory = '/tmp/camera_save_tutorial'  # Update with the path to your image directory

# # Define the lower and upper bounds of the red color (in HSV)
# lower_red = (0, 100, 100)
# upper_red = (10, 255, 255)

# # Function to move the drone
# def move_drone(x, y, z, vx, vy, vz, afx, afy, afz, yaw_rate):
#     msg = mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
#        10, mav_connection.target_system, mav_connection.target_component,
#          mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b010111111000), x, y, z, 0, 0, 0, 0, 0, 0, 0, yaw_rate))



# # Main loop
# while True:
#     # Get the latest image file in the directory
#     image_files = [f for f in os.listdir(image_directory) if f.endswith('.jpg')]
#     if image_files:
#         latest_image_file = max(image_files, key=lambda x: os.path.getctime(os.path.join(image_directory, x)))
#         latest_image_path = os.path.join(image_directory, latest_image_file)

#         # Read the image
#         frame = cv2.imread(latest_image_path)

#         if frame is None:
#             print(f"Error reading image from: {latest_image_path}")
#         else:
#             # Convert the frame to HSV
#             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#             # Threshold the image to get only red regions
#             mask = cv2.inRange(hsv, lower_red, upper_red)

#             # Find contours in the mask
#             contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#             if contours:
#                 # Get the largest contour
#                 largest_contour = max(contours, key=cv2.contourArea)

#                 # Get the bounding box of the contour
#                 x, y, w, h = cv2.boundingRect(largest_contour)

#                 # Calculate the center of the bounding box
#                 center_x = x + w // 2
#                 center_y = y + h // 2

#                 # Get the width and height of the frame
#                 frame_width = frame.shape[1]
#                 frame_height = frame.shape[0]

#                 # Calculate the desired drone movement based on the position of the red object
#                 forward_backward = (center_y - frame_height / 2) / (frame_height / 2)
#                 left_right = (center_x - frame_width / 2) / (frame_width / 2)

#                 # Adjust the scaling factor for your specific drone's characteristics
#                 scaling_factor = 0.1

#                 # Calculate yaw rate for rotation
#                 yaw_rate = left_right * math.pi  # Adjust as needed

#                 # Move the drone
#                 move_drone(forward_backward * scaling_factor, left_right * scaling_factor, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, yaw_rate)

#                 # Display the frame with the bounding box
#                 cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                 cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

#             # Display the frame
#             cv2.imshow('Frame', frame)

#             # Wait for a key press and close the window
#             cv2.waitKey(1)

#     else:
#         print(f"No JPEG images found in {image_directory}")

#     # Introduce a delay if needed
#     time.sleep(1)
import cv2
from pymavlink import mavutil
import os
import time
import math

# Connect to the MAVLink
mav_connection = mavutil.mavlink_connection('udpin:localhost:14550')
mav_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (mav_connection.target_system, mav_connection.target_component))


# Set the image directory path
image_directory = '/tmp/camera_save_tutorial'  # Update with the path to your image directory

# Define the lower and upper bounds of the red color (in HSV)
lower_red = (0, 100, 100)
upper_red = (10, 255, 255)

# Function to move the drone
def move_drone(vx, vy, vz, yaw_rate):
    mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, mav_connection.target_system,
                        mav_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b010111111000), vx, vy, vz, 0, 0, 0, 0, 0, 0, 0, yaw_rate))

kp = 0.0
prev_x = 0.0
# Main loop
while True:
    # Get the latest image file in the directory
    image_files = [f for f in os.listdir(image_directory) if f.endswith('.jpg')]
    if image_files:
        latest_image_file = max(image_files, key=lambda x: os.path.getctime(os.path.join(image_directory, x)))
        latest_image_path = os.path.join(image_directory, latest_image_file)

        # Read the image
        frame = cv2.imread(latest_image_path)

        if frame is None:
            print(f"Error reading image from: {latest_image_path}")
        else:
            # Convert the frame to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Threshold the image to get only red regions
            mask = cv2.inRange(hsv, lower_red, upper_red)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Get the largest contour
                largest_contour = max(contours, key=cv2.contourArea)

                # Get the bounding box of the contour
                x, y, w, h = cv2.boundingRect(largest_contour)

                # Calculate the center of the bounding box
                center_x = x + w // 2
                center_y = y + h // 2

                # Get the width and height of the frame
                frame_width = frame.shape[1]
                frame_height = frame.shape[0]
 
                # Calculate the desired drone movement based on the position of the red object
                forward_backward = (center_y - frame_height / 2) / (frame_height / 2)
                left_right = (center_x - frame_width / 2) / (frame_width / 2)
                print(f"forward_backward={forward_backward}, left_right={left_right}")
                # Adjust the scaling factor for your specific drone's characteristics
                scaling_factor = 10 - kp * abs(forward_backward) # Adjust as needed
               
                print(f"scaling factor ={scaling_factor}")
                # Convert to meters
                x_meters = forward_backward * scaling_factor
                y_meters = left_right * scaling_factor
                max_x_change = 0.3
                if abs(x_meters - prev_x) > max_x_change:
                    x_meters = prev_x + max_x_change if x_meters > prev_x else prev_x - max_x_change
                elif abs(x_meters - prev_x) > -max_x_change:
                    x_meters = prev_x + max_x_change if x_meters > prev_x else prev_x - max_x_change
                # Update the previous x value
                prev_x = x_meters
                # Calculate yaw rate for rotation
                yaw_rate = left_right * math.pi  # Adjust as needed
                print(f"x={x_meters}, y={y_meters}, yaw_rate={yaw_rate}")
                # Move the drone with MAV_FRAME_BODY_OFFSET_NED
                move_drone(x_meters, y_meters,0.0, yaw_rate)

                # Display the frame with the bounding box
                #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # Display the frame
            cv2.imshow('Frame', frame)

            # Wait for a key press and close the window
            #cv2.waitKey(1)

    else:
        print(f"No JPEG images found in {image_directory}")

    # Introduce a delay if needed
    time.sleep(1)



