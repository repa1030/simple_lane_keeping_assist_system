# ======================================================
# Copyright (C) 2020 repa1030
# This program and the accompanying materials
# are made available under the terms of the MIT license.
# ======================================================

from TcpServer import tcp_server
from PurePursuit import pure_pursuit
from LaneDetector import lane_detector
import numpy as np
import cv2

if __name__=="__main__":

    ####################################################
    ################# Parameter Section ################
    ####################################################

    # General Settings
    display_detection = True        # Default: True
    display_warnings = False        # Default: False

    # TCP/IP Settings
    tcp_ip = '0.0.0.0'              # Default: '0.0.0.0'
    tcp_port = 5005                 # Default: 5005
    tcp_buffer_size = 1024          # Default: 1024 [bytes]
    tcp_time_out = 3.0              # Default: 3.0 [seconds]
    
    # Vehicle Settings
    command_velocity = 50.0         # Default: 50.0 [km/h]
    vehicle_mass = 1580.0           # Default: 1580.0 [kg]
    wheel_radius = 0.29             # Default: 0.29 [m]
    max_motor_torque = 1000.0       # Default: 1000.0 [Nm]
    max_brake_torque = 1500.0       # Default: 1500.0 [Nm]
    wheel_base = 2.44               # Default: 2.44 [m]
    
    # Pure Pursuit Settings
    look_ahead_ratio = 1.0          # Default: 1.0 (smoothing of steering)
    max_look_ahead_distance = 7.0   # Default: 7.0 [m] (if -1, it is calculated automatically)

    # CV Settings
    use_quadratic_interpol = False  # Default: False (don't use this)
    offset_to_clipping_plane = 3.83 # Default: 3.83 [m] (base link to bottom border of image) 
    pixel_to_cm = 0.25              # Default: 0.25 (change resolution of line image)
    padding_x = 150                 # Default: 150 [cm] (padding on both sides)
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
    
    ####################################################
    ############# End of Parameter Section #############
    ####################################################

    src_p = np.float32([[150, 540], [808, 540], [571, 344], [389, 344]]) # BottomL, BottomR, TopR, TopL
    dst_p = np.float32([[int((0 + padding_x) * pixel_to_cm), int((1000 + padding_y) * pixel_to_cm)],
                        [int((300 + padding_x) * pixel_to_cm), int((1000 + padding_y) * pixel_to_cm)], 
                        [int((300 + padding_x) * pixel_to_cm), int((0 + padding_y) * pixel_to_cm)], 
                        [int((0 + padding_x) * pixel_to_cm), int((0 + padding_y) * pixel_to_cm)]])
    warp_size = (int((300 + 2 * padding_x) * pixel_to_cm), int((1000 + padding_y) * pixel_to_cm))
    if max_look_ahead_distance is -1:
        max_look_ahead_distance = (1000 + padding_y - 100)
    ld = lane_detector.SimpleLaneDetector(src_p, dst_p, warp_size, look_ahead_ratio,
                                            max_look_ahead_distance, pixel_to_cm, 
                                            use_quadratic_interpol, draw_line_prediction,
                                            draw_waypoint, color_right_line, color_left_line, 
                                            color_pred_line, color_waypoint, draw_thickness,
                                            waypoint_radius)
    print ('Initialized Instance of Lane Detector.')
    pp = pure_pursuit.PurePursuit(command_velocity, vehicle_mass, wheel_radius, 
                                    max_motor_torque, max_brake_torque, wheel_base,
                                    offset_to_clipping_plane)
    print ('Initialized Instance of Pure Pursuit.')
    tcp = tcp_server.TcpServer(tcp_ip, tcp_port, tcp_buffer_size, tcp_time_out)
    print ('Initialized Instance of TCP Server.')
    print ('Waiting for Connection...')
    tcp.connectToClient()
    connection = True
    print ('Connection to Client is established.')
    is_info_displayed = False
    
    while True:
        # If error occured -> try reconnecting
        if (connection is False):
            tcp.reconnect()
        # Receive message containing the image and the current velocity
        (image, current_velocity, connection) = tcp.receiveMessage()
        # Check if the image and the velocity are valid 
        if (image is None) or (current_velocity is None):
            if display_warnings:
                print ('Warning: Image transfer failed.')
            connection = tcp.sendMessage(0.0, 0.0, 0.0)
            continue
        # Display info that lkas is running
        if not is_info_displayed:
            print ('Lane Following Programm is running.')
            is_info_displayed = True
        current_velocity = current_velocity * 1000.0 / (60.0 * 60.0)
        # Detect the lane and calculate the waypoint
        (line_image, waypoint) = ld.detectLaneLines(image, current_velocity)
        if display_detection:
            cv2.imshow('Detection', line_image)
            cv2.waitKey(10)
        # Calculate the control commands
        (steerCmd, brakeCmd, gasCmd) = pp.calcControlCommands(current_velocity, waypoint)
        # Send the control commands back to Unity
        connection = tcp.sendMessage(steerCmd, brakeCmd, gasCmd)
    
    tcp.closeConnection()
        
