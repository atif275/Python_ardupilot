# import cv2
# import numpy as np
# import os

# def resize_image(image, ratio=0.5):
#     height, width = image.shape[:2]
#     new_width = int(ratio * width)
#     new_height = int(ratio * height)
#     resized_image = cv2.resize(image, (new_width, new_height))
#     return resized_image

# def compute_hog(image):
#     sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=3)
#     sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=3)

#     magnitude, angle = cv2.cartToPolar(sobelx, sobely)

#     return magnitude, angle

# def save_image(image, filename, directory):
#     path = os.path.join(directory, filename)
#     cv2.imwrite(path, image)
#     print(f"Image saved: {path}")

# def detect_object(frame_image, object_image, save_directory):
#     frame_gray = cv2.cvtColor(frame_image, cv2.COLOR_BGR2GRAY)
#     object_gray = cv2.cvtColor(object_image, cv2.COLOR_BGR2GRAY)

#     frame_resized = resize_image(frame_gray, ratio=0.5)
#     object_resized = resize_image(object_gray, ratio=0.5)

#     # Save resized and grayscaled images to the specified directory
#     save_image(frame_resized, 'frame_resized_gray.jpg', save_directory)
#     save_image(object_resized, 'object_resized_gray.jpg', save_directory)

#     object_magnitude, _ = compute_hog(object_resized)

#     (winW, winH) = (object_resized.shape[1], object_resized.shape[0])
#     for y in range(0, frame_resized.shape[0] - winH, 5):
#         for x in range(0, frame_resized.shape[1] - winW, 5):
#             window = frame_resized[y:y + winH, x:x + winW]
#             window_magnitude, _ = compute_hog(window)

#             correlation = np.sum(window_magnitude * object_magnitude) / np.sqrt(np.sum(window_magnitude ** 2) * np.sum(object_magnitude ** 2))

#             threshold = 0.5

#             if correlation > threshold:
#                 cv2.rectangle(frame_image, (x, y), (x + winW, y + winH), (0, 255, 0), 2)
#                 avg_x = (x + x + winW) // 2
#                 avg_y = (y + y + winH) // 2
#                 return frame_image, avg_x, avg_y

#     return None, None, None

# # Specify the directory to save images
# save_directory = "/Users/ATIFHANIF/Desktop/project/ardupilot/python"


# frame_image = cv2.imread('/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame.jpg')
# object_image = cv2.imread('/Users/ATIFHANIF/Desktop/project/ardupilot/python/object2.jpg')

# result_image, avg_x, avg_y = detect_object(frame_image, object_image, save_directory)

# if result_image is not None:
#     cv2.imshow('Object Detection Result', result_image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
#     print(f"Average Coordinates: ({avg_x}, {avg_y})")
# else:
#     print("Object not found in the frame.")
# import cv2
# import numpy as np
# import os

# def detect_and_draw_template(template_path, frame_path, output_directory):
#     # Read the template and frame images
#     template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
#     frame = cv2.imread(frame_path)
    # # Resize the template to 256x128
    # template = cv2.resize(template, (512, 256))

    # # Resize the frame to 256x128
    # frame = cv2.resize(frame, (512, 256))

#     # Convert the frame to grayscale (assuming it's not already in grayscale)
#     frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Use template matching to find the template in the frame
#     result = cv2.matchTemplate(frame_gray, template, cv2.TM_CCOEFF_NORMED)
#     min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

#     # Define a threshold for template matching
#     threshold = 0.2
#     print(f"Template dimensions: {template.shape}")
#     print(f"Frame dimensions: {frame.shape}")
#     print(f"Max value: {max_val}")
#     print(f"Min value: {min_val}")
#     print(f"Max loc: {max_loc}")
#     print(f"min loc: {min_loc}")
#     print(f"Threshold: {threshold}")

#     if max_val >= threshold:
#         # Get the coordinates of the matched region
#         top_left = max_loc
#         h, w = template.shape

#         # Draw a green rectangle around the matched region
#         bottom_right = (top_left[0] + w, top_left[1] + h)
#         cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

#         # Save the output image with the green rectangle in the specified directory
#         output_path = os.path.join(output_directory, 'output_image_with_rectangle.jpg')
#         cv2.imwrite(output_path, frame)
#         print(f"Template found and saved at: {output_path}")
#     else:
#         print("Template not found in the frame.")

# # Example usage
# template_image_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/object4.jpg'
# frame_image_path ='/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame1.jpg'
# output_directory = '/Users/ATIFHANIF/Desktop/project/ardupilot/python'

# detect_and_draw_template(template_image_path, frame_image_path, output_directory)

#############################################
import cv2
import numpy as np
import os

def detect_and_draw_template(template_path, frame_path, output_directory):
    # Read the template and frame images
    template = cv2.imread(template_path)
    frame = cv2.imread(frame_path)
     # Resize the template to 256x128
    #template = cv2.resize(template, (512, 256))

    # Resize the frame to 256x128
    #frame = cv2.resize(frame, (512, 256))

    # Resize the template to maintain aspect ratio
    #template = cv2.resize(template, (int(template.shape[1] * 0.5), int(template.shape[0] * 0.5)))

    # Convert images to grayscale
    template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Create HOG descriptor
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # Detect objects using HOG
    template_locations, _ = hog.detectMultiScale(template_gray)
    frame_locations, _ = hog.detectMultiScale(frame_gray)
    print(template_locations)
    print(frame_locations)

    # Draw rectangles around detected regions
    for (x, y, w, h) in template_locations:
        print(f"x = {x} y = {y}  w = {w} h = {h}")
        cv2.rectangle(template, (x, y), (x + w, y + h), (0, 255, 0), 2)

    for (x, y, w, h) in frame_locations:
        print(f"xx = {x} yy = {y}  ww= {w} hh = {h}")
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Save the output images
    output_template_path = os.path.join(output_directory, 'output_template_with_rectangle.jpg')
    cv2.imwrite(output_template_path, template)
    print(f"Template with rectangle saved at: {output_template_path}")

    output_frame_path = os.path.join(output_directory, 'output_frame_with_rectangle.jpg')
    cv2.imwrite(output_frame_path, frame)
    print(f"Frame with rectangle saved at: {output_frame_path}")

# Example usage
template_image_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/object5.jpg'
frame_image_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame1.jpg'
output_directory = '/Users/ATIFHANIF/Desktop/project/ardupilot/python'

detect_and_draw_template(template_image_path, frame_image_path, output_directory)


# import cv2
# import numpy as np
# import os

# def sliding_window(image, step_size, window_size):
#     for y in range(0, image.shape[0] - window_size[1], step_size):
#         for x in range(0, image.shape[1] - window_size[0], step_size):
#             yield (x, y, image[y:y + window_size[1], x:x + window_size[0]])

# def detect_and_draw_template(template_path, frame_path, output_directory):
#     # Read the template and frame images
#     template = cv2.imread(template_path)
#     frame = cv2.imread(frame_path)
#     template = cv2.resize(template, (512, 256))

#     # Resize the frame to 256x128
#     frame = cv2.resize(frame, (512, 256))
#     # Convert images to grayscale
#     template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
#     frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Resize the template to maintain aspect ratio
#     template_gray = cv2.resize(template_gray, (int(template_gray.shape[1] * 0.5), int(template_gray.shape[0] * 0.5)))

#     # Create HOG descriptor
#     hog = cv2.HOGDescriptor()
#     hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

#     # Detect objects using HOG with sliding windows on the template
#     template_locations = []
#     for (x, y, window) in sliding_window(template_gray, step_size=16, window_size=(64, 128)):
#         if window.shape[0] != 128 or window.shape[1] != 64:
#             continue

#         # Convert the window to 8-bit unsigned integer
#         window = np.uint8(window)

#         descriptor = hog.compute(window)
#         _, result = hog.detect(descriptor)

#         if result > 0.5:  # Threshold for detection confidence
#             template_locations.append((x, y, window.shape[1], window.shape[0]))

#     # Detect objects using HOG with sliding windows on the frame
#     frame_locations = []
#     for (x, y, window) in sliding_window(frame_gray, step_size=16, window_size=(64, 128)):
#         if window.shape[0] != 128 or window.shape[1] != 64:
#             continue

#         # Convert the window to 8-bit unsigned integer
#         window = np.uint8(window)

#         descriptor = hog.compute(window)
#         _, result = hog.detect(descriptor)

#         if result > 0.5:  # Threshold for detection confidence
#             frame_locations.append((x, y, window.shape[1], window.shape[0]))

#     # Draw rectangles around detected regions
#     for (x, y, w, h) in template_locations:
#         cv2.rectangle(template, (x, y), (x + w, y + h), (0, 255, 0), 2)

#     for (x, y, w, h) in frame_locations:
#         cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

#     # Save the output images
#     output_template_path = os.path.join(output_directory, 'output_template_with_rectangle.jpg')
#     cv2.imwrite(output_template_path, template)
#     print(f"Template with rectangle saved at: {output_template_path}")

#     output_frame_path = os.path.join(output_directory, 'output_frame_with_rectangle.jpg')
#     cv2.imwrite(output_frame_path, frame)
#     print(f"Frame with rectangle saved at: {output_frame_path}")

# # Example usage
# template_image_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/object4.jpg'
# frame_image_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame1.jpg'
# output_directory = '/Users/ATIFHANIF/Desktop/project/ardupilot/python'

# detect_and_draw_template(template_image_path, frame_image_path, output_directory)



# import cv2
# import numpy as np
# import os

# def sliding_window(image, step_size, window_size):
#     for y in range(0, image.shape[0] - window_size[1], step_size):
#         for x in range(0, image.shape[1] - window_size[0], step_size):
#             yield (x, y, image[y:y + window_size[1], x:x + window_size[0]])

# def detect_and_draw_template(template_path, frame_path, output_directory):
#     # Read the template and frame images
#     template = cv2.imread(template_path)
#     frame = cv2.imread(frame_path)
#     template = cv2.resize(template, (512, 256))

#     # Resize the frame to 256x128
#     frame = cv2.resize(frame, (512, 256))
#     # Convert images to grayscale
#     template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
#     frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Resize the template to maintain aspect ratio
#     template_gray = cv2.resize(template_gray, (int(template_gray.shape[1] * 0.5), int(template_gray.shape[0] * 0.5)))

#     # Create HOG descriptor
#     hog = cv2.HOGDescriptor()
#     hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

#     # Detect objects using HOG with sliding windows on the template
#     template_locations = []
#     for (x, y, window) in sliding_window(template_gray, step_size=16, window_size=(64, 128)):
#         if window.shape[0] != 128 or window.shape[1] != 64:
#             continue

#         # Convert the window to 8-bit unsigned integer
#         window = np.uint8(window)

#         # Compute the HOG descriptor for the window
#         descriptor = hog.compute(window)

#         # Ensure the image is of the correct type for HOG detection
#         window = cv2.cvtColor(window, cv2.COLOR_GRAY2BGR)

#         # Use SVM to get the confidence score
#         result = hog.detect(window)

#         if result and result[0][0] > 0.5:  # Threshold for detection confidence
#             template_locations.append((x, y, window.shape[1], window.shape[0]))

#     # Detect objects using HOG with sliding windows on the frame
#     frame_locations = []
#     for (x, y, window) in sliding_window(frame_gray, step_size=16, window_size=(64, 128)):
#         if window.shape[0] != 128 or window.shape[1] != 64:
#             continue

#         # Convert the window to 8-bit unsigned integer
#         window = np.uint8(window)

#         # Compute the HOG descriptor for the window
#         descriptor = hog.compute(window)

#         # Ensure the image is of the correct type for HOG detection
#         window = cv2.cvtColor(window, cv2.COLOR_GRAY2BGR)

#         # Use SVM to get the confidence score
#         result = hog.detect(window)
#         print(f"result == {result}")

#         if result and result[0][0] > 0.5:  # Threshold for detection confidence
#             frame_locations.append((x, y, window.shape[1], window.shape[0]))

#     # Draw rectangles around detected regions
#     for (x, y, w, h) in template_locations:
#         cv2.rectangle(template, (x, y), (x + w, y + h), (0, 255, 0), 2)

#     for (x, y, w, h) in frame_locations:
#         cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

#     # Save the output images
#     output_template_path = os.path.join(output_directory, 'output_template_with_rectangle.jpg')
#     cv2.imwrite(output_template_path, template)
#     print(f"Template with rectangle saved at: {output_template_path}")

#     output_frame_path = os.path.join(output_directory, 'output_frame_with_rectangle.jpg')
#     cv2.imwrite(output_frame_path, frame)
#     print(f"Frame with rectangle saved at: {output_frame_path}")

# # Example usage
# template_image_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/object4.jpg'
# frame_image_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame1.jpg'
# output_directory = '/Users/ATIFHANIF/Desktop/project/ardupilot/python'

# detect_and_draw_template(template_image_path, frame_image_path, output_directory)
# import cv2
# import numpy as np
# import os

# def detect_and_draw_template(template_path, frame_path, output_directory):
#     # Read the template and frame images
#     template = cv2.imread(template_path)
#     frame = cv2.imread(frame_path)
#     template = cv2.resize(template, (256, 128))

#     # Resize the frame to 256x128
#     frame = cv2.resize(frame, (256, 128))

#     # Convert images to grayscale
#     template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
#     frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Create ORB detector
#     orb = cv2.ORB_create()

#     # Find the keypoints and descriptors with ORB
#     kp1, des1 = orb.detectAndCompute(template_gray, None)
#     kp2, des2 = orb.detectAndCompute(frame_gray, None)

#     # Use Brute Force Matcher with Hamming distance
#     bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

#     # Match descriptors
#     matches = bf.match(des1, des2)

#     # Sort them in ascending order of distance
#     matches = sorted(matches, key=lambda x: x.distance)

#     # Get the coordinates of keypoints in both images
#     template_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
#     frame_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

#     # Find homography matrix
#     H, _ = cv2.findHomography(template_pts, frame_pts, cv2.RANSAC)

#     # Define the corners of the template
#     h, w = template_gray.shape
#     corners = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)

#     # Project the corners into the frame
#     transformed_corners = cv2.perspectiveTransform(corners, H)

#     # Draw a rectangle around the detected object in the frame
#     frame_with_rectangle = frame.copy()
#    # print(frame_with_rectangle)
#     #cv2.polylines(frame_with_rectangle, [np.int32(transformed_corners)], True, (0, 255, 0), 2)

#     # Save the output image
#     output_frame_path = os.path.join(output_directory, 'output_frame_with_rectangle.jpg')
#     cv2.imwrite(output_frame_path, frame_with_rectangle)
#     print(f"Frame with rectangle saved at: {output_frame_path}")

# # Example usage
# template_image_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/object5.jpg'
# frame_image_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame1.jpg'
# output_directory = '/Users/ATIFHANIF/Desktop/project/ardupilot/python'

# detect_and_draw_template(template_image_path, frame_image_path, output_directory)

# import cv2
# import numpy as np
# import os

# def detect_and_draw_template(template_path, frame_path, output_directory):
#     # Read the template and frame images
#     template = cv2.imread(template_path)
#     frame = cv2.imread(frame_path)
#     template = cv2.resize(template, (256, 128))

#     # Resize the frame to 256x128
#     frame = cv2.resize(frame, (256, 128))

#     # Convert images to grayscale
#     template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
#     frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Create ORB detector
#     orb = cv2.ORB_create()

#     # Find the keypoints and descriptors with ORB
#     kp1, des1 = orb.detectAndCompute(template_gray, None)
#     kp2, des2 = orb.detectAndCompute(frame_gray, None)

#     # Use Brute Force Matcher with Hamming distance
#     bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

#     # Match descriptors
#     matches = bf.match(des1, des2)

#     # Sort them in ascending order of distance
#     matches = sorted(matches, key=lambda x: x.distance)

#     # Get the coordinates of keypoints in both images
#     template_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
#     frame_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

#     # Find the bounding box around the matched keypoints
#     template_rect = cv2.boundingRect(template_pts)
#     frame_rect = cv2.boundingRect(frame_pts)

#     # Draw rectangles around the detected object in the frame
#     frame_with_rectangle = frame.copy()
#     cv2.rectangle(frame_with_rectangle, frame_rect[:2], (frame_rect[0]+frame_rect[2], frame_rect[1]+frame_rect[3]), (0, 255, 0), 2)

#     # Save the output image
#     output_frame_path = os.path.join(output_directory, 'output_frame_with_rectangle.jpg')
#     cv2.imwrite(output_frame_path, frame_with_rectangle)
#     print(f"Frame with rectangle saved at: {output_frame_path}")

# # Example usage
# template_image_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/object4.jpg'
# frame_image_path = '/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame1.jpg'
# output_directory = '/Users/ATIFHANIF/Desktop/project/ardupilot/python'

# detect_and_draw_template(template_image_path, frame_image_path, output_directory)

# frame_image = cv2.imread('/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame.jpg')
# object_image = cv2.imread('/Users/ATIFHANIF/Desktop/project/ardupilot/python/object.jpg')

# '/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame.jpg'
# '/Users/ATIFHANIF/Desktop/project/ardupilot/python/object.jpg'