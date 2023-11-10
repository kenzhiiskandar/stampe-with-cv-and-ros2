# About the Project

This project aims to stamp a particular object with a correct orientation on a conveyor belt system using a custom Dynamixel-motored Robot.

https://github.com/kenzhiiskandar/stampe_conveyer_belt_ros2/assets/120554498/93bcf0f6-c90a-4e07-b2fc-a61b279affb1

The main system compromises of several sub-systems as follows:

**Object Detection and Orientation Analytics:**

The system employs a custom object detection model based on YOLOv8s, meticulously crafted to identify and track specific elements in real-time video feeds. The primary objects of interest include a white box, a red dot, and a blue dot, with the entire architecture running seamlessly on a Ubuntu 22.04 OS Personal Computer. Simultaneously, the red and blue dots are used to indicate the orientation of the white box.

**Triggered Action:**

The triggered action occurs when the right end of the white box's bounding box crosses a predefined yellow line on the webcam. It initiates the transmission of object coordinates to a Raspberry Pi via ROS2 Topic. 

**Robot Response:**

Upon receiving object coordinates in the ROS2 Topic, the system engages an inverse kinematics algorithm, moves a 6-degree-of-freedom robot to a precisely calculated offset position.

**Stamping Process:**

Once in position, the robot patiently awaits the approach of the stamped object before executing a stamping action. The stamping is achieved through a solenoid connected to the Raspberry Pi's GPIO.

**Return Action:**

After the stamping is done, the robot moves to the default coordinate position, and waits until the next box triggers the program.

## Hardware & Software Architectures
![image](https://github.com/kenzhiiskandar/stampe_conveyer_belt_ros2/assets/120554498/cbe6e14b-1db0-4e1f-b1c8-65a4596f51be)

### Computer Vision

The system employs a custom object detection model trained with YOLOv8. To visualize the detection results, a standard OpenCV is used to access an external webcam. This allows us to draw bounding boxes, lines, and change colors.

Since our program is built on ROS2, the display of frames is facilitated through image transport packages. For more details, please refer to the provided links.

* **Training a custom dataset using YOLOv8** https://blog.roboflow.com/how-to-train-yolov8-on-a-custom-dataset/

* **Webcam Real Time Object Detection using Open CV** https://dipankarmedh1.medium.com/real-time-object-detection-with-yolo-and-webcam-enhancing-your-computer-vision-skills-861b97c78993

* **Open CV in ROS2** https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/

### ROS2 

Our program is built on ROS2 Humble, taking advantage of the compatibility offered by Ubuntu 22.04 OS. The system operates using a straightforward ROS2 Publisher-Subscriber mechanism. Data dissemination is organized into three distinct topics: "box_coordinate," "red_dot_coordinate," and "blue_dot_coordinate."

To facilitate communication, we employ ROS2 Custom Interfaces, defining the structure of the messages shared across all topics. If you're new to ROS2 or need installation guidance, check out the provided link for detailed instructions and tutorials.

* **ROS2 Humble** https://docs.ros.org/en/humble/Tutorials.html

### Robot Control
The robotic arm's joints are powered by Dynamixel AX-12 and AX-18 motors, while the conveyor belt is driven by a Dynamixel MX-28AT motor. Python's Dynamixel library is employed to control and manage these motors seamlessly.

For comprehensive details on how to control these specific motor types, please refer to the provided link.

* **Dynamixel Python Library** https://github.com/ROBOTIS-GIT/DynamixelSDK

* **Dynamixel AX-12 Control** https://github.com/aakieu/ax12_control

### GPIO Control
The GPIO is used to actuate and dis-actuate the solenoid. The Raspberry Pi's GPIO suppors Python Development in Ubuntu. Refer to this link for more information. 

* **Raspberry Pi GPIO Python Library** https://pypi.org/project/RPi.GPIO/

### Mathematics

The coordinates captured by the external webcam are transformed from pixel coordinates to world coordinates. In this transformation, the (0,0,0) point is defined at the center of the robot arm. Following this, the program engages an inverse kinematics solution, utilizing the geometry of the 6-degree-of-freedom (6-DOF) robot.

This calculated solution precisely positions all the joints, aligning the robot to the defined offset coordinates. This process ensures accurate and controlled movements of the robotic arm.

## Getting Started

This is an example of how you may give instructions on setting up your project locally.

### Software Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* Ubuntu 22.04 OS https://ubuntu.com/download/desktop

* ROS2 Humble https://docs.ros.org/en/humble/Tutorials.html

### Installation (PC)

1. Direct to the source folder of your ROS2 workspace
   ```sh
   cd ros2_ws/src
   ```
3. Clone the repo
   ```sh
   git clone https://github.com/github_username/repo_name.git
   ```
4. Install NPM packages
   ```sh
   npm install
   ```
5. Enter your API in `config.js`
   ```js
   const API_KEY = 'ENTER YOUR API';
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>
