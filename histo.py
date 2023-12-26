import cv2
import numpy as np
import os

# Example usage
def find_object(template_path, scene_path):
    # Read the template and scene images
    template = cv2.imread(template_path, cv2.IMREAD_COLOR)
    scene = cv2.imread(scene_path, cv2.IMREAD_COLOR)

    # Convert images to grayscale
    template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    scene_gray = cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY)

    # Calculate histograms
    hist_template = cv2.calcHist([template_gray], [0], None, [256], [0, 256])
    hist_scene = cv2.calcHist([scene_gray], [0], None, [256], [0, 256])

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
    cv2.rectangle(scene, top_left, bottom_right, (0, 255, 0), 2)

    # Display the result
    cv2.imshow('Result', scene)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


find_object('/Users/ATIFHANIF/Desktop/project/ardupilot/python/patch2.png', '/Users/ATIFHANIF/Desktop/project/ardupilot/python/scene.webp')
#find_object('./object6.jpg', './frame2.jpg')
