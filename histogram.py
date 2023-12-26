from PIL import Image, ImageDraw
import numpy as np
import os

def convert_to_grayscale(image_path, save_path):
    image = Image.open(image_path).convert('L')
    image.save(save_path)

def create_histogram(image_path, save_folder):
    image = Image.open(image_path).convert('L')
    image_array = np.array(image)
    height, width = image_array.shape

    histogram_details = []

    for i in range(0, height-60, 60):
        for j in range(0, width-40, 40):
            segment = image_array[i:i+60, j:j+40]
            histogram = np.histogram(segment.flatten(), bins=256, range=[0, 256])[0]
            histogram_details.append((i, j, histogram))
            
            # Save histogram details to a text file
            with open(os.path.join(save_folder, f"histogram_{i}_{j}.txt"), 'w') as f:
                f.write("\n".join(map(str, histogram)))


    return histogram_details

def compare_histograms(object_histogram, frame_histogram):
    correlation = np.correlate(object_histogram, frame_histogram)
    return correlation[0]

def detect_object(frame_image_path, object_image_path, histogram_details):
    object_histogram = np.histogram(np.array(Image.open(object_image_path).convert('L')).flatten(), bins=256, range=[0, 256])[0]
    # frame_with_square = frame_image_path.copy()
    # draw = ImageDraw.Draw(frame_with_square)
    for i, j, frame_histogram in histogram_details:
        correlation = compare_histograms(object_histogram, frame_histogram)
        print(f"differece= {correlation}")
        
        if correlation < threshold:  # You can set a threshold value based on experimentation
            print(f"Object found at ({i}, {j})")
            #draw_green_square(draw, x, y, segment_size)
            #draw_green_square(draw, i, j, 60)
            draw_square(frame_image_path, i, j)


def draw_green_square(draw, x, y, size):
    draw.rectangle([x, y, x + size, y + size], outline="green", width=2)

def draw_square(image_path, i, j):
    image = Image.open(image_path)
    square_color = 255  # Use an integer for grayscale images
    draw = ImageDraw.Draw(image)
    draw.rectangle([j, i, j+40, i+60], outline="green")
    image.save("result_image.jpg")

if __name__ == "__main__":
    base_path = "/Users/ATIFHANIF/Desktop/project/ardupilot/python"
    
    object_image_path = os.path.join(base_path, "object.jpg")
    frame_image_path = os.path.join(base_path, "frame.jpg")

    object_grayscale_path = os.path.join(base_path, "object_grayscale.jpg")
    frame_grayscale_path = os.path.join(base_path, "frame_grayscale.jpg")
    histogram_folder = os.path.join(base_path, "histograms")

    convert_to_grayscale(object_image_path, object_grayscale_path)
    convert_to_grayscale(frame_image_path, frame_grayscale_path)

    os.makedirs(histogram_folder, exist_ok=True)

    histogram_details = create_histogram(frame_grayscale_path, histogram_folder)

    threshold = 1000  # You can adjust the threshold based on experimentation

    detect_object(frame_grayscale_path, object_grayscale_path, histogram_details)
