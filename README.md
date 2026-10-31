# Vision-Based Human Tracking & Avoidance for Quadruped Robots

**Autonomous Robotics | Computer Vision | ROS 2 | Feedback Control**

## Overview

This project develops vision-based autonomous navigation behaviors for the **Puppertrot quadruped robot**. Using a monocular camera, real-time human detection, and closed-loop PID control, the robot responds to a detected person by adjusting its forward motion and heading. Two behaviors were demonstrated: approaching a person and stopping at a target distance, and continuously following a person while maintaining alignment.

## Problem Statement

Enable a quadruped robot to detect and respond to a human using monocular visual feedback, without relying on a dedicated depth sensor for the described control strategy. The central challenge is translating image-space observations into stable linear and angular motion commands.

## Engineering Approach

- **ROS 2 integration:** Developed a ROS 2-based pipeline connecting camera observations, human detection, feedback control, and robot motion commands.
- **Visual feedback:** Used the detected person's bounding box to estimate relative target proximity through its image area and horizontal alignment through its position in the image.
- **Distance regulation:** Applied PID control to the bounding-box-area error to regulate the robot's forward/backward motion relative to a target apparent size.
- **Heading regulation:** Applied PID control to the horizontal image-center error to steer the robot toward the detected person.
- **Behavior design:** Implemented a seek-and-stop behavior and a continuous human-following behavior.

Because bounding-box area is a visual proxy for proximity, the controller regulates *apparent target size*, not an independently measured metric distance.

## Demonstrated Behaviors

### 1. Seek and Stop

The robot detects a person, approaches while correcting its heading, and stops when the visual target-size condition is reached.

### 2. Continuous Human Following

The robot continuously adjusts linear and angular commands from camera feedback to follow a moving person while regulating relative proximity and image alignment.

## Results

Both autonomous behaviors were successfully demonstrated on the quadruped platform. The project illustrates how monocular object detection and coupled feedback controllers can support responsive human-aware mobile robot navigation.

## Tools & Technologies

- **ROS 2** — robotics software integration
- **Python** — control and perception logic
- **Computer vision and object detection** — human localization in camera frames
- **PID control** — linear and angular motion regulation
- **Monocular camera** — visual sensing
- **Puppertrot quadruped robot** — robotic platform

## Technical Scope

The documented outcomes concern human seeking and following. Although the repository title includes *avoidance*, no separate experimentally verified avoidance behavior is described in the supplied project summary.
