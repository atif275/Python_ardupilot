import cv2
import numpy as np
import os

# def find_object(template_path, scene_path, output_directory):
#     # Read the template and scene images
#     template = cv2.imread(template_path, cv2.IMREAD_COLOR)
#     scene = cv2.imread(scene_path, cv2.IMREAD_COLOR)

#     # Convert images to grayscale
#     template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
#     scene_gray = cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY)

#     # Calculate histograms
#     hist_template = cv2.calcHist([template_gray], [0], None, [256], [0, 256])
#     hist_scene = cv2.calcHist([scene_gray], [0], None, [256], [0, 256])

#     # Normalize histograms
#     cv2.normalize(hist_template, hist_template, 0, 1, cv2.NORM_MINMAX)
#     cv2.normalize(hist_scene, hist_scene, 0, 1, cv2.NORM_MINMAX)

#     # Apply histogram comparison methods
#     methods = [cv2.HISTCMP_CORREL, cv2.HISTCMP_CHISQR, cv2.HISTCMP_INTERSECT]
#     results = []

#     for method in methods:
#         result = cv2.compareHist(hist_template, hist_scene, method)
#         results.append(result)

#     # Get the index of the method with the maximum result
#     best_method_index = np.argmax(results)

#     # Find the location of the template in the scene
#     if best_method_index == 0:  # Use histogram correlation for more accurate results
#         res = cv2.matchTemplate(scene_gray, template_gray, cv2.TM_CCOEFF_NORMED)
#         min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
#         top_left = max_loc
#     else:  # Use template matching result
#         top_left = (0, 0)  # Use a default value for top_left if template matching is not selected

#     # Draw a rectangle around the detected area
#     h, w = template_gray.shape
#     bottom_right = (top_left[0] + w, top_left[1] + h)
#     cv2.rectangle(scene, top_left, bottom_right, (0, 255, 0), 2)

    # Save the result image in the specified directory
    # output_path = os.path.join(output_directory, 'result_image.jpg')
    # cv2.imshow('Result', scene)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # cv2.imwrite(output_path, scene)

def find_object(template_path, scene_path, output_directory):
    # Read the template and scene images
    template = cv2.imread(template_path, cv2.IMREAD_COLOR)
    scene = cv2.imread(scene_path, cv2.IMREAD_COLOR)
    # template = cv2.resize(template, (int(template.shape[1] * 0.5), int(template.shape[0] * 0.5)))
    # scene = cv2.resize(scene, (int(scene.shape[1] * 0.5), int(scene.shape[0] * 0.5)))

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
    #output_path = os.path.join(output_directory, 'result_image.png')
    # cv2.imshow('Result', scene)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    #cv2.imwrite(output_path, scene)
    # Display the result
    cv2.imshow('Result', scene)
    #cv2_imshow(scene)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


# Example usage
output_directory = '/Users/ATIFHANIF/Desktop/project/ardupilot/python'
#find_object('template_image.jpg', 'scene_image.jpg', output_directory)

# Example usage
find_object('/Users/ATIFHANIF/Desktop/project/ardupilot/python/object2.png', '/Users/ATIFHANIF/Desktop/project/ardupilot/python/frame3.png' , output_directory)
