# ======================================================
# Copyright (C) 2020 repa1030
# This program and the accompanying materials
# are made available under the terms of the MIT license.
# ======================================================

import numpy as np
import cv2

class SimpleLaneDetector:
    
    # constructor of class
    def __init__(self, src_points, dst_points, warp_img_size, look_ahead_ratio,
                    max_look_ahead_dst, pixel_to_cm, use_quadratic_interpolation,
                    draw_line_prediction, draw_waypoint, color_right_line, 
                    color_left_line, color_predicted_line, color_waypoint, 
                    draw_thickness, waypoint_radius):
        self.lane_width = 0.0
        self.i = 0
        self.warp_mat = cv2.getPerspectiveTransform(src_points, dst_points)
        self.warp_mat_inv = cv2.getPerspectiveTransform(dst_points, src_points)
        self.warp_size = warp_img_size
        self.la_ratio = look_ahead_ratio
        self.max_la = max_look_ahead_dst
        self.px2cm = pixel_to_cm
        self.use_quadratic = use_quadratic_interpolation
        self.line_prediction = draw_line_prediction
        self.draw_wp = draw_waypoint
        self.color_r = color_right_line
        self.color_l = color_left_line
        self.color_p = color_predicted_line
        self.color_wp = color_waypoint
        self.draw_thick = draw_thickness
        self.wp_radius = waypoint_radius

    # this is for drawing quadratic interpolation (unstable)
    def drawPolynom(self, img, detect, wp, min_y, max_y, width, height, poly_left, poly_right, dist=3):
        line_img = np.zeros_like(img).astype(np.uint8)
        i = int(min_y)
        if detect == 1:   
            while i > int(max_y):
                p1l = (int(poly_left(i)), int(i))
                p2l = (int(poly_left(i-dist)), int(i-dist))
                p1r = (int(poly_right(i)), int(i)) 
                p2r = (int(poly_right(i-dist)), int(i-dist))
                cv2.line(line_img, p1l, p2l, self.color_l, self.draw_thick)
                cv2.line(line_img, p1r, p2r, self.color_r, self.draw_thick)
                i -= dist
        elif detect == 2:
            while i > int(max_y):
                p1l = (int(poly_left(i)), int(i))
                p2l = (int(poly_left(i-dist)), int(i-dist))
                cv2.line(line_img, p1l, p2l, self.color_l, self.draw_thick)
                i -= dist
        elif detect == 3:
            while i > int(max_y):
                p1r = (int(poly_right(i)), int(i)) 
                p2r = (int(poly_right(i-dist)), int(i-dist))
                cv2.line(line_img, p1r, p2r, self.color_r, self.draw_thick)
                i -= dist
        if self.draw_wp:
            cv2.circle(line_img, (int(width/2 - (wp[1] * self.px2cm * 100.0)), int(height - (wp[0] * self.px2cm * 100.0))), self.wp_radius, self.color_wp, -1)
        return line_img

    # this function draws the final lane lines on the image
    def drawLines(self, img, lines, height, width, wp):
        line_img = np.zeros_like(img).astype(np.uint8)
        for line in lines:
            for x1, y1, x2, y2, color in line:
                cv2.line(line_img, (x1, y1), (x2, y2), color, self.draw_thick)
        if self.draw_wp:
            cv2.circle(line_img, (int(width/2.0 - (wp[1] * self.px2cm * 100.0)), int(height - (wp[0] * self.px2cm * 100.0))), self.wp_radius, self.color_wp, -1)
        return line_img

    # this function combines line and original image
    def combineImages(self, line_img, orig_img, width, height):
        line_img = cv2.warpPerspective(line_img, self.warp_mat_inv, (width, height))
        result = cv2.addWeighted(orig_img, 1, line_img, 0.8, 0.0)
        return result

    # this function calculates the next waypoint
    def calcWaypoint(self, x_left, x_right, y, width):
        wp = [0, 0, 0]
        # changing from openCV coord system to sensor coord system
        wp[0] = y
        wp[1] = (width - x_right - x_left) / 2.0 / self.px2cm / 100.0
        self.lane_width = x_right - x_left 
        return wp

    # this function calculates the next waypoint if only one lane is detected
    def calcWaypointWithOneLine(self, x, y, lane, width, det_lane):
        wp = [0, 0, 0]
        # left line is known
        if det_lane == 2:
            x_th = x + lane
            wp[1] = (width - x - x_th) / 2.0 / self.px2cm / 100.0
        #right line is known
        else:
            x_th = x - lane
            wp[1] = (width - x_th - x) / 2.0 / self.px2cm / 100.0
        wp[0] = y
        return wp

    # this is the pipeline for the lane detection
    def detectLaneLines(self, img_orig, velo):
        # waypoint (x=longitudinal, y=lateral, z=0) [m]
        wp = [0, 0, 0]
        # 0 -> no lines detected, 
        # 1 -> both lines detected,
        # 2 -> left line detected
        # 3 -> right line detected
        det_lines = 0
        height_orig = img_orig.shape[0]
        width_orig = img_orig.shape[1]
        warped = cv2.warpPerspective(img_orig, self.warp_mat, self.warp_size)
        height = warped.shape[0]
        width = warped.shape[1]
        
        gray_image = cv2.cvtColor(warped, cv2.COLOR_RGB2GRAY) 
        cannyed_image = cv2.Canny(gray_image, 100, 200)
        lines = cv2.HoughLinesP(
            cannyed_image,
            rho=6,
            theta=np.pi / 60,
            threshold=160,
            lines=np.array([]),
            minLineLength=40,
            maxLineGap=25
        )

        left_line_x = []
        left_line_y = []
        right_line_x = []
        right_line_y = []

        # Check if there are any hough lines
        if lines is None:
            return img_orig, wp

        # Add hough lines to right or left line
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x1 < (width/2) and x2 < (width/2):
                    left_line_x.extend([x1, x2])
                    left_line_y.extend([y1, y2])
                elif x1 > (width/2) and x2 > (width/2):
                    right_line_x.extend([x1, x2])
                    right_line_y.extend([y1, y2])
                else:
                    continue

        min_y = int(height)
        max_y = int(height - (self.max_la * self.px2cm * 100))
        if max_y < 0:
            max_y = 0

        # Get left line polynom
        if not left_line_x or not left_line_y:
            left_line_det = False
        else:
            if self.use_quadratic:
                poly_left = np.poly1d(np.polyfit(
                    left_line_y,
                    left_line_x,
                    deg=2
                ))
            else:
                poly_left = np.poly1d(np.polyfit(
                    left_line_y,
                    left_line_x,
                    deg=1
                ))
                left_x_start = int(poly_left(max_y))
                left_x_end = int(poly_left(min_y))
            left_line_det = True

        # Get right line polynom
        if not right_line_x or not right_line_y:
            right_line_det = False
        else:
            if self.use_quadratic:
                poly_right = np.poly1d(np.polyfit(
                    right_line_y,
                    right_line_x,
                   deg=2
                ))
            else:
                poly_right = np.poly1d(np.polyfit(
                    right_line_y,
                    right_line_x,
                   deg=1
                ))
                right_x_start = int(poly_right(max_y))
                right_x_end = int(poly_right(min_y))
            right_line_det = True
        
        # calculate look ahead distance
        la_dst = velo * self.la_ratio
        if la_dst > self.max_la:
            la_dst = self.max_la
        la_x = height - (la_dst * 100.0 * self.px2cm)

        # Check how many lines are detected and calculate waypoint
        if left_line_det and right_line_det:
            if not self.use_quadratic:            
                available_lines = [
                    [[left_x_start, max_y, left_x_end, min_y, self.color_l]],
                    [[right_x_start, max_y, right_x_end, min_y, self.color_r]]
                    ]
            det_lines = 1
            wp = self.calcWaypoint(int(poly_left(la_x)), int(poly_right(la_x)), la_dst, width)
        elif left_line_det:
            poly_right = []
            det_lines = 2
            if not self.use_quadratic:
                if self.line_prediction and self.lane_width > 0.0:
                    available_lines = [
                        [[left_x_start, max_y, left_x_end, min_y, self.color_l]],
                        [[int(left_x_start + self.lane_width), max_y, int(left_x_end + self.lane_width), min_y, self.color_p]]
                        ]
                else:
                    available_lines = [
                        [[left_x_start, max_y, left_x_end, min_y, self.color_l]]
                        ]
            if self.lane_width > 0.0:
                wp = self.calcWaypointWithOneLine(int(poly_left(la_x)), la_dst, self.lane_width, width, det_lines)
        elif right_line_det:
            poly_left= []
            det_lines = 3
            if not self.use_quadratic:
                if self.line_prediction and self.lane_width > 0.0:
                    available_lines = [
                        [[int(right_x_start - self.lane_width), max_y, int(right_x_end - self.lane_width), min_y, self.color_p]],
                        [[right_x_start, max_y, right_x_end, min_y, self.color_r]]
                        ]
                else:
                    available_lines = [
                        [[right_x_start, max_y, right_x_end, min_y, self.color_r]]
                        ]
            if self.lane_width > 0.0:
                wp = self.calcWaypointWithOneLine(int(poly_right(la_x)), la_dst, self.lane_width, width, det_lines)
        else:
            poly_right = []
            poly_left = []
            if not self.use_quadratic:
                available_lines = []

        # draw lines on empty line image
        if self.use_quadratic:
            line_image = self.drawPolynom(
                warped, det_lines, wp, 
                min_y, max_y, 
                width, height,
                poly_left, poly_right
            )
        else:
            line_image = self.drawLines(
                warped, available_lines,
                height,
                width,
                wp
            )

        # warp line image and combine it with original image        
        output_image = self.combineImages(line_image, img_orig, width_orig, height_orig)

        # Return result
        return output_image, wp
