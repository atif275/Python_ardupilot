from PIL import Image

def detect_red_object(image_path):
    # Open the image
    image = Image.open(image_path)
    
    # Get the width and height of the image
    width, height = image.size
    
    # Initialize variables to store information about the red object
    red_object_coordinates = []
    
    # Iterate through each pixel of the image
    for x in range(width):
        for y in range(height):
            # Get the RGB values of the current pixel
            r, g, b = image.getpixel((x, y))
            
            # Check if the pixel is red (you may need to adjust the thresholds)
            if r > 180 and g < 100 and b < 100:
                red_object_coordinates.append((x, y))
    
    # Calculate the center of the image
    center_x = width // 2
    center_y = height // 2
    
    # Analyze the position of the red object relative to the center
    if not red_object_coordinates:
        return "No red object detected"
    
    # Get the average coordinates of the red object
    avg_x = sum(coord[0] for coord in red_object_coordinates) // len(red_object_coordinates)
    avg_y = sum(coord[1] for coord in red_object_coordinates) // len(red_object_coordinates)
    forward_backward = (center_y - height / 2) / (height / 2)
    left_right = (center_x - width / 2) / (width / 2)
    lx=center_x-40
    ly=center_y-40
    rx=center_x+40
    ry=center_y+40
    # Determine the position of the red object relative to the center
    position = ""
    if ((avg_x >lx and avg_x < rx)):
        position = "forward "
    
    if avg_x < center_x:
        position += "left "
    elif avg_x > center_x:
        position += "right "
    
    if avg_y < center_y:
        position += "up"
    elif avg_y > center_y:
        position += "down"
    
    # Provide other details about the red object
    result_details = {
        "Average Coordinates": (avg_x, avg_y),
        "Position relative to center": position,
        "Number of red pixels": len(red_object_coordinates),
        "width":(width),
        "height":(height),
        "center x,y":(center_x,center_y),
        "forward_backward":(forward_backward),
        "left_right":(left_right)
    }
    
    return result_details

# Example usage
image_path = "/tmp/camera_save_tutorial/default_iris_demo_iris_demo_gimbal_small_2d_tilt_link_camera(1)-9581.jpg"
result = detect_red_object(image_path)
print(result)
