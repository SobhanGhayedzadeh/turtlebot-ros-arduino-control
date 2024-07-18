# TurtleBot Control with ROS and Arduino

This repo provides a node for controlling a TurtleBot using ROS (Robot Operating System) with an Arduino Uno as the microcontroller. The setup uses ROSSerial for communication between ROS and the Arduino, and BTS7960 motor drivers to control the DC motors. The package includes nodes for sending velocity commands (cmd_vel_autonomous) to the Arduino, which then translates these commands into motor actions to drive the robot.

## Introduction

This TurtleBot was built for testing SLAM, navigation and path planning, making it an excellent platform for robotics research and development. It is powered by ROS, which provides a flexible framework for writing robot software, and it features a RealSense camera for advanced perception capabilities.


## 3D Model 

In this repository, you will find the 3D model of the TurtleBot setup. This model can be used for reference or modifications. 

**Real Picture**:

![Real Robot](/Documents/TURTLEBOT.jpg)

**3D Model**:

![3D Model](/Documents/TURTLEBOT_SolidWorks.gif)

## Hardware Components

- **TurtleBot Chassis**: The physical structure that houses all components.
- **Arduino Uno**: The microcontroller used to interface with ROS and control the motors.
- **2 DC Motors**: Provide the driving force for the TurtleBot.
- **2 BTS7960 Motor Drivers**: Ensure efficient and powerful control of the DC motors.
- **12 V Battery Pack**: Powers the entire system.
- **ROS-Compatible Computer**: Runs ROS and communicates with the Arduino.

## Wiring for BTS7960 Motor Drivers

To ensure correct wiring of the BTS7960 motor drivers, follow these instructions provided in this [wiring guide](https://forum.arduino.cc/t/dc-motor-driver-bts7960/606686) to connect the 12 V power supply, and then follow these instructions:

1. **Left Motor Driver to Arduino**:
   - `RPWM` to Arduino Pin 9
   - `LPWM` to Arduino Pin 11
   - `R_EN` and `L_EN` to Arduino Pin 10
   - `VCC` to Arduino 5V
   - `GND` to Arduino GND

2. **Right Motor Driver to Arduino**:
   - `RPWM` to Arduino Pin 6
   - `LPWM` to Arduino Pin 3
   - `R_EN` and `L_EN` to Arduino Pin 5
   - `VCC` to Arduino 5V
   - `GND` to Arduino GND


## Software Requirements
- [ROS](http://wiki.ros.org/ROS/Installation) (Robot Operating System) - noetic
- Arduino IDE
- Necessary ROS packages:
  - `rosserial_arduino`
  - `rosserial_client`

## Setup Instructions

### Arduino Setup
1. **Connect the hardware:**
    - Connect the DC motors to the BTS7960 motor driver.
    - Connect the BTS7960 to the Arduino Uno.
    - Follow the wiring connections up above.

2. **Upload the Arduino Code:**
    - Open the `turtleBOT.ino` file in the Arduino IDE.
    - Select the correct board and port from the Tools menu.
    - Upload the code to the Arduino Uno.

### ROS Setup
1. **Create a ROS Package:**
    ```sh
    cd ~/catkin_ws/src
    git clone https://github.com/SobhanGhayedzadeh/turtlebot-ros-arduino-control.git
    ```

2. **Add the Python Teleoperation Script:**
    - Ensure the script has executable permissions:
    ```sh
    chmod +x ~/catkin_ws/src/turtlebot_controller/scripts/teleop_key.py
    ```

4. **Install `rosserial` Packages:**
    ```sh
    sudo apt-get install ros-$(rosversion -d)-rosserial-arduino ros-$(rosversion -d)-rosserial
    ```

## Usage
1. **Start ROS Core:**
    ```sh
    roscore
    ```

2. **Start Teleoperation Node:**
    ```sh
    rosrun turtlebot_controller teleop_key.py
    ```

2. **Start ROSSerial to communicate with Arduino:**
    ```sh
    rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600

    fuser -k /dev/ttyACM0
    ```

3. **Control the TurtleBot3:**
    - Use the following keys to control the robot:
    ```
    Control Your TurtleBot
    ---------------------------
    Moving around:
            w
       a    s    d
            x

    w/x : increase/decrease linear velocity
    a/d : increase/decrease angular velocity

    space key, s : force stop

    CTRL-C to quit
    ```

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
