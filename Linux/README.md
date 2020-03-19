# Linux Environment

The computing part of this lane keeping assist system is placed in the ubuntu environment. It is possible to execute this in a virtual machine or on a native ubuntu PC. The parts of the lane keeping assist system are described below.

## Calibration

You can find here the data for calibration of the current front center camera of the vehicle.

## Lane Detector

This detection is based on openCV functions:
- First, the perspective image is warped to an BEV image
- Then the BEV image is converted to a grayscale image
- After that, a canny filter is applied to the grayscale image
- Now, the Hough Line Transform "detects" lines in the image and save those lines into an array
- the lines are now sorted to left and right lane, depending on their location
- The polynom is created using the poly1d and polyfit function of numpy
- The look ahead distance for the next waypoint is calculated
- If only one of two lane lines were detected, the other one will be predicted
- The waypoint is calculated and the lane lines are saved in an array
- Plotting of lane lines
- Inverse warp of the lane line image
- Combining of the lane line image and the original image
- Return of the combined image and the waypoint

_Note: Calculation of waypoint is also possible with a single lane line. This function uses the last known lane width_

## Pure Pursuit

This script calculates the steering angle and pedal positions depending on the acceleration of the vehicle.
The pedal position are calculated depending on the current velocity, the target speed and the distance to the next waypoint.
The relation of the maximum acceleration respectively the maximum deceleration to the target acceleration is used for the pedal positions.
The pure pursuit algorithm for the steering angle is based on trigonometry functions and is described here: 
https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf

## TcpServer

This script contains the functions for connection to unity, reconnection to unity and sending of control commmands and receiving of images.

## Testing

This directory is the test environment of the lane detection. It is applied on different real world images.

## How To Use

1. Set the parameters in the file lkas.py (default values are recommend)
2. Start the unity exe file ("/Windows/UnityBuild/Lane Keeping Assist System.exe")
3. Execute ```python lkas.py``` before entering the scene
4. Start the scene (Select the scene from main menu)
