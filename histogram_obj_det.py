# # from PIL import Image

# # def convert_to_grayscale(image):
# #     return image.convert("L")

# # def calculate_histogram(image):
# #     return image.histogram()

# # def calculate_difference(hist1, hist2):
# #     return sum(abs(x - y) for x, y in zip(hist1, hist2))

# # def object_detection(object_image, frame_image, segment_size):
# #     # Convert images to grayscale
# #     object_gray = convert_to_grayscale(object_image)
# #     frame_gray = convert_to_grayscale(frame_image)

# #     # Calculate histograms
# #     object_hist = calculate_histogram(object_gray)
    
# #     frame_width, frame_height = frame_gray.size

# #     # Loop through segments
# #     for x in range(0, frame_width - segment_size, segment_size):
# #         for y in range(0, frame_height - segment_size, segment_size):
# #             # Extract segment from the frame
# #             segment = frame_gray.crop((x, y, x + segment_size, y + segment_size))

# #             # Calculate histogram for the segment
# #             segment_hist = calculate_histogram(segment)

# #             # Calculate difference between histograms
# #             difference = calculate_difference(object_hist, segment_hist)
# #             print(f"difference=  {difference}")
# #             # You can set a threshold for matching
# #             threshold = 2500000

# #             if difference < threshold:
# #                 print(f"Object found at position: ({x}, {y})")

# #             # else:
# #             #     print("no object detected")

# # # Load images
# # object_image = Image.open("/Users/ATIFHANIF/Desktop/project/ardupilot/python/object.jpg")
# # frame_image = Image.open("/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame.jpg")

# # # Set the segment size
# # segment_size = 50

# # # Perform object detection
# # object_detection(object_image, frame_image, segment_size)
# from PIL import Image, ImageDraw

# def convert_to_grayscale(image, save_path=None):
#     gray_image = image.convert("L")
#     if save_path:
#         gray_image.save(save_path)
#     return gray_image

# def calculate_histogram(image, save_path=None):
#     hist = image.histogram()
#     if save_path:
#         with open(save_path, 'w') as file:
#             for i, value in enumerate(hist):
#                 file.write(f"{i}: {value}\n")
#     return hist

# def calculate_difference(hist1, hist2):
#     return sum(abs(x - y) for x, y in zip(hist1, hist2))

# def draw_green_square(draw, x, y, size):
#     draw.rectangle([x, y, x + size, y + size], outline="green", width=2)

# def object_detection(object_image, frame_image, segment_size, save_path="/Users/ATIFHANIF/Desktop/project/ardupilot/python"):
#     # Convert images to grayscale and save if needed
#     object_gray = convert_to_grayscale(object_image, f"{save_path}/object_gray.jpg")
#     frame_gray = convert_to_grayscale(frame_image, f"{save_path}/frame_gray.jpg")

#     # Calculate histograms and save if needed
#     object_hist = calculate_histogram(object_gray, f"{save_path}/object_histogram.txt")
    
#     frame_width, frame_height = frame_gray.size

#     # Create a copy of the frame image for drawing
#     frame_with_square = frame_image.copy()
#     draw = ImageDraw.Draw(frame_with_square)

#     # Loop through segments
#     for x in range(0, frame_width - segment_size, segment_size):
#         for y in range(0, frame_height - segment_size, segment_size):
#             # Extract segment from the frame
#             segment = frame_gray.crop((x, y, x + segment_size, y + segment_size))

#             # Calculate histogram for the segment and save if needed
#             segment_hist = calculate_histogram(segment, f"{save_path}/segment_{x}_{y}_histogram.txt")

#             # Calculate difference between histograms
#             difference = calculate_difference(object_hist, segment_hist)

#             # You can set a threshold for matching
#             threshold = 1000

#             if difference < threshold:
#                 print(f"Object found at position: ({x}, {y})")
                
#                 # Draw a green square around the detected object
#                 draw_green_square(draw, x, y, segment_size)

#     # Save the frame image with the green square
#     frame_with_square.save(f"{save_path}/frame_with_square.jpg")

# # Load images
# object_image = Image.open("/Users/ATIFHANIF/Desktop/project/ardupilot/python/object.jpg")
# frame_image = Image.open("/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame.jpg")

# # Set the segment size
# segment_size = 50

# # Perform object detection
# object_detection(object_image, frame_image, segment_size)
from PIL import Image, ImageDraw
import os

def convert_to_grayscale(image, save_path=None):
    gray_image = image.convert("L")
    if save_path:
        gray_image.save(save_path)
    return gray_image

def calculate_histogram(image, save_path=None):
    hist = image.histogram()
    if save_path:
        with open(save_path, 'w') as file:
            for i, value in enumerate(hist):
                file.write(f"{i}: {value}\n")
    return hist

def calculate_difference(hist1, hist2):
    return sum(abs(x - y) for x, y in zip(hist1, hist2))

def draw_green_square(draw, x, y, width, height):
    draw.rectangle([x, y, x + width, y + height], outline="green", width=2)

def object_detection(object_image, frame_image, save_path="/Users/ATIFHANIF/Desktop/project/ardupilot/python"):
    # Convert images to grayscale and save if needed
    object_gray = convert_to_grayscale(object_image, f"{save_path}/object_gray.jpg")
    frame_gray = convert_to_grayscale(frame_image, f"{save_path}/frame_gray.jpg")

    # Calculate and save histograms
    object_hist = calculate_histogram(object_gray, f"{save_path}/histograms/object_histogram.txt")
    frame_hist = calculate_histogram(frame_gray, f"{save_path}/histograms/frame_histogram.txt")
    
    frame_width, frame_height = frame_gray.size

    # Create a copy of the frame image for drawing
    frame_with_square = frame_image.copy()
    draw = ImageDraw.Draw(frame_with_square)

    # Calculate difference between histograms for the entire frame
    difference = calculate_difference(object_hist, frame_hist)

    # You can set a threshold for matching
    threshold = 1000

    if difference < threshold:
        print("Object found in the frame")
        
        # Draw a green square around the detected object
        draw_green_square(draw, 0, 0, frame_width, frame_height)

        # Save the frame image with the green square
        frame_with_square.save(f"{save_path}/frame_with_square.jpg")
    else:
        print("Object not found in the frame")

# Load images
object_image = Image.open("/Users/ATIFHANIF/Desktop/project/ardupilot/python/object.jpg")
frame_image = Image.open("/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame.jpg")

# Perform object detection
object_detection(object_image, frame_image)
