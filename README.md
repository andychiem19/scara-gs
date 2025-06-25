# SCARA Robot Arm (Arduino / C++)

This project implements inverse kinematics and coordinated stepper motor control in Arduino to consistently and precisely move an end effector to a desired position. 

## Features

- Inverse kinematics to convert a desired (x,y) position in cm to stepper motor rotation steps
- Coordinated motor stepping function
- Electronic limiting

## Project Structure

- `main/`: Main branch of microcontroller code
- `develop/`: Actively developed, merged into main when compiled and tested

## Tools Used

- Arduino IDE
- Fritzing

## TODO
//Not actively being worked on
- Implement start position homing using limit switches
- Implement forward kinematics to determine current position of end effector
