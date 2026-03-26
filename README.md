# Opencv-Robotic-Arm
# 🤖 Hand Gesture Controlled Robotic Arm

## 📌 Overview

This project is a Python + Arduino-based robotic arm system that uses
real-time hand tracking to control a physical robotic arm. By simply
moving your hand in front of a webcam, the robotic arm mimics your
motion, creating a seamless human-machine interaction system.

The system uses computer vision to detect hand landmarks and translates
them into servo motor movements for the shoulder, elbow, and gripper.

------------------------------------------------------------------------

## 🚀 Features

-   Real-time hand tracking using webcam\
-   Accurate mapping of hand movement to robotic joints\
-   Controls 3 degrees of freedom:
    -   Shoulder rotation\
    -   Elbow movement\
    -   Gripper open/close\
-   Smooth motion using angle mapping\
-   Serial communication between Python and Arduino\
-   Gesture-based control (pinch for gripper)

------------------------------------------------------------------------

## 🛠️ Tech Stack

-   Python (OpenCV, MediaPipe, PySerial)
-   Arduino UNO
-   Servo Motors (3x)
-   Breadboard & Jumper Wires

------------------------------------------------------------------------

## ⚙️ Hardware Setup

### Components

-   Arduino UNO\
-   3x Servo Motors\
-   Breadboard\
-   USB Cable

------------------------------------------------------------------------

## 🧠 How It Works

### Hand Detection

Uses MediaPipe to detect hand landmarks.

### Motion Mapping

-   Wrist X → Shoulder angle\
-   Wrist Y → Elbow angle

### Gripper Control

Distance between thumb and index finger controls the gripper.

### Data Transmission

Example: S90E120G60

------------------------------------------------------------------------

## ▶️ How to Run

### Install Dependencies

pip install opencv-python mediapipe pyserial

### Connect Arduino

Update COM port in Python: arduino = serial.Serial('COM5', 9600)

### Run Program

python main.py

------------------------------------------------------------------------

## 📊 Applications

-   Assistive robotics\
-   Industrial automation\
-   Gesture-based control systems

------------------------------------------------------------------------

## 🔮 Future Improvements

-   AI-based gesture recognition\
-   Wireless control\
-   Add more joints

------------------------------------------------------------------------

## 🧑‍💻 Author

Arnav Upadhyay. I spent 6hrs in buliding this project

