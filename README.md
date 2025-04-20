# ADA Pet Robotics Capstone Project

## Overview
This project is a working submission as a capstone project for the University of Washington Robotics Capstone Course CSE 481. Our goal is to design a robot that can help people with disabilities take care of their pets and build deeper connections with them.

## Features
- Web-based control interface for robot operation

## System Requirements
- Hello Robot Stretch robot with default ROS packages

## Project Structure
The repository is organized as follows:

[web_teleop](./web_teleop/): contains a launch file for all necessary robot driver for the web interface


## Installation
1. [ROBOT and PC] Clone the repository:  
    ```bash
    git clone https://github.com/yourusername/ada_pet_capstone.git
    cd ada_pet_capstone
    ```
2. [ROBOT] Install dependencies and build the project:  
    ```bash
    colcon build
    source install/setup.bash
    ```

## Usage
1. [ROBOT] Start the robot interface:
    ```bash
    ./launch_interface.sh
    ```
    
    This script will:
    - Source the ROS2 environment
    - Run system checks on the Stretch robot
    - Home the robot to a safe starting position
    - Launch the necessary robot drivers for teleop

2. [PC] Open index.html in a web browser
    ```bash
    cd src/ada_pet_capstone
    firefox index.html
    ```
3. [PC] Use the interface to control the robot:
    - Use the joystick for base movement
    - Click buttons to open/close the gripper
    - Control arm extension and lift height
    - Adjust wrist yaw, pitch, and roll
    - Home or stow the robot as needed

## Troubleshooting
- Realsense not found `[ERROR] [1608156657.316423905]: The requested device with is NOT found. Will Try again.`
    - Best solution I found so far is to completely power down the robot (unplug robot from power sources, not just a software restart), then starting it back up. This is a known issue with Realsense cameras. 
    - (kaikwan): I'm looking into ways where I can just restart a Realsense/device management process to prevent a restart, but haven't tried yet.
- Check browser console for any JavaScript errors

## Acknowledgments
- University of Washington CSE 481 Course Staff
- Hello Robot (Stretch Robot platform)

