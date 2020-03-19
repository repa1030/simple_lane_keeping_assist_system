# ======================================================
# Copyright (C) 2020 repa1030
# This program and the accompanying materials
# are made available under the terms of the MIT license.
# ======================================================

import cv2
import sys
import numpy as np
import os
sys.path.append('../')
from LaneDetector import lane_detector

# Pure Pursuit Settings
look_ahead_ratio = 1.0          # Default: 1.0 (smoothing of steering)
max_look_ahead_distance = 7.0   # Default: 7.0 [m] (if -1, it is calculated automatically)

# CV Settings
use_quadratic_interpol = False  # Default: False (don't use this)
offset_to_clipping_plane = 3.83 # Default: 3.83 [m] (base link to bottom border of image) 
pixel_to_cm = 0.25              # Default: 0.25 (change resolution of line image)
padding_x = 100                 # Default: 150 [cm] (padding on both sides)
padding_y = 700                 # Default: 700 [cm] (padding to forward)

# Draw Settings
draw_line_prediction = True     # Default: True (draw predicted line)
draw_waypoint = True            # Default: True (draw next waypoint on image)
draw_thickness = 3              # Default: 3 (thickness of lines and waypoint)
waypoint_radius = 4             # Default: 4 (radius of the drawn waypoint)
color_right_line = [0, 255, 0]  # Default: [0, 255, 0] [B G R]
color_left_line = [255, 0, 0]   # Default: [255, 0, 0] [B G R]
color_pred_line = [0, 0, 255]   # Default: [0, 0, 255] [B G R]
color_waypoint = [0, 140, 255]  # Default: [0, 140, 255] [B G R]

src_p0 = np.float32([[150, 540], [808, 540], [571, 344], [389, 344]]) # UL, UR, OR, OL
src_p1 = np.float32([[155, 337], [525, 337], [410, 255], [275, 255]]) # UL, UR, OR, OL 
src_p2 = np.float32([[0, 880], [760, 880], [510, 700], [240, 700]]) # UL, UR, OR, OL 
src_p3 = np.float32([[90, 341], [425, 341], [350, 260], [230, 260]]) # UL, UR, OR, OL 
src_p4 = np.float32([[140, 340], [490, 340], [345, 255], [255, 255]]) # UL, UR, OR, OL
src_p5 = np.float32([[105, 341], [365, 341], [335, 270], [260, 270]]) # UL, UR, OR, OL 
src_p6 = np.float32([[150, 2157], [2915, 2157], [2630, 1680], [1500, 1680]]) # UL, UR, OR, OL 

src_points = [['test0', src_p0],
              ['test1', src_p1],
              ['test2', src_p2],
              ['test3', src_p3],
              ['test4', src_p4],
              ['test5', src_p5],
              ['test6', src_p6]]

dst_p = np.float32([[int((0 + padding_x) * pixel_to_cm), int((1000 + padding_y) * pixel_to_cm)],
                        [int((300 + padding_x) * pixel_to_cm), int((1000 + padding_y) * pixel_to_cm)], 
                        [int((300 + padding_x) * pixel_to_cm), int((0 + padding_y) * pixel_to_cm)], 
                        [int((0 + padding_x) * pixel_to_cm), int((0 + padding_y) * pixel_to_cm)]])
warp_size = (int((300 + 2 * padding_x) * pixel_to_cm), int((1000 + padding_y) * pixel_to_cm))

i = 0
img_folder = 'Images'
for file in os.listdir(img_folder):
    file_str = str(file)
    file_path = os.path.join(img_folder, file)
    img = cv2.imread(img_folder + '/' + file_str)
    for obj in src_points:
        if file_str.find(obj[0]) is not -1:
            src_p = obj[1]
            break

    ld = lane_detector.SimpleLaneDetector(src_p, dst_p, warp_size, look_ahead_ratio,
                                            max_look_ahead_distance, pixel_to_cm, 
                                            use_quadratic_interpol, draw_line_prediction,
                                            draw_waypoint, color_right_line, color_left_line, 
                                            color_pred_line, color_waypoint, draw_thickness,
                                            waypoint_radius)
    (output, wp) = ld.detectLaneLines(img, 0)
    warp_mat = cv2.getPerspectiveTransform(src_p, dst_p)
    image = cv2.warpPerspective(img, warp_mat, warp_size)

    print(file_str + '\t' + str(wp))
    cv2.imwrite('Warped/' + file_str + '_warped.jpg', image)
    cv2.imwrite('Results/' + file_str + '_detection.jpg', output)

    i += 1



