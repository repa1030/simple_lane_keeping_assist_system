# Lane Keeping Assist System
Project of University of Applied Science Karlsruhe, Lane Keeping Assist System in Unity using OpenCV.

## Windows Environment (/Windows)

The simulation runs on windows in Unity and streams images of the front center camera and the current velocity of the vehicle towards linux on a TCP/IP stream.

## Linux Environment (/Linux)

This is where the lane detection is done. After detecting the lane and calculating a waypoint,
the pure pursuit algorithm is used to calculate the steer, gas and brake command and these commands
are send back.

### Requirements

OS: Ubuntu 18.04  
Fixed IP Adress: 192.168.206.3  
Ubuntu-Packages: `sudo apt-get install python python-pip`  
pip-Packages: `pip install numpy opencv-python`  
