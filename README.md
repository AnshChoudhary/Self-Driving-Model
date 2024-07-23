# Self-Driving-Model

![Self Driving GIF](https://github.com/AnshChoudhary/Self-Driving-Model/blob/main/self-driving-curve-trim.gif)

# Lane Detection and Vehicle Steering using UNet and Bird's Eye View Transformation

This project demonstrates a lane detection and vehicle steering application using a UNet deep learning model. The application captures the game screen, applies a bird's eye view perspective transformation, and utilizes lane detection to steer the vehicle in a simulated environment.

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Usage](#usage)
- [How it Works](#how-it-works)
  - [Screen Capture](#screen-capture)
  - [Bird's Eye View Transformation](#birds-eye-view-transformation)
  - [Lane Detection](#lane-detection)
  - [Steering Mechanism](#steering-mechanism)
  - [Debugging and Visualization](#debugging-and-visualization)
- [Dependencies](#dependencies)
- [Demo](#demo)

## Introduction

The project aims to perform lane detection on a driving simulator by capturing the screen and applying a UNet model for semantic segmentation. By transforming the captured screen into a bird's eye view, we improve lane detection accuracy, which helps in robust vehicle steering.

## Features

- **Bird's Eye View Transformation**: Converts the captured frame into a bird's eye view for better lane detection.
- **Lane Detection**: Uses a UNet model for detecting lanes in the game.
- **Vehicle Steering**: Automatically steers the vehicle based on detected lane information.
- **Debugging Information**: Displays real-time lane detection and steering actions on the screen.
- **Performance Logging**: Logs key press counts and steering history for debugging and analysis.

## Usage
1. Run the Application:
```bash
python simulator.py
```

2. Switch to Game Window:
After running the script, switch to the game window within 3 seconds.

3. Stop the Application:
Press q in the application window to stop the program.

## How it Works

### Screen Capture
The application uses the mss library to capture the game screen. The captured frame is then processed for lane detection. The screen dimensions are set to 1440x900, which is defined in the capture_screen() function.

### Bird's Eye View Transformation
The perspective transformation is applied to get a bird's eye view, making lane detection easier and more accurate. This is achieved using OpenCV's cv2.getPerspectiveTransform() and cv2.warpPerspective() functions. The source points (src_points) and destination points (dst_points) define the region of interest for transformation.

### Lane Detection
A UNet model is used to detect lanes in the transformed image. The model processes the frame to output a mask indicating lane positions, which is then used for steering decisions.

### Steering Mechanism
The application calculates the center of the detected lane and compares it to the center of the screen. Based on this difference, the vehicle is steered using pyautogui to simulate key presses. The steering logic smoothens the transition using a weighted average of the current and previous steering values, making it more realistic.

### PID Controller

The basic idea behind a PID controller is to read a sensor, then compute the desired actuator output by calculating proportional, integral, and derivative responses and summing those three components to compute the output.

![image](https://user-images.githubusercontent.com/59261333/73613652-94495680-4600-11ea-9e6d-124fb5bfc188.png)

#### P - Proportional part
- The proportional part is easiest to understand: The output of the proportional part is the product of gain and measured error e. Hence, larger proportional gain or error makes for greater output from the proportional part. Setting the proportional gain too high causes a controller to repeatedly overshoot the setpoint, leading to oscillation.
- e = r – y
e: error,    r: user input reference,    y: actual measured output
- Proportional part = Kp * e
Kp: input gain for P – Regulator (Proportional Regulator)

```
# Proportional part
Proportional = kp * e_current
```

#### I - Integral part
- Think of the integral as a basket in which the loop stores all measured errors in E. Remember that error can be positive or negative, so sometimes error fills the basket (when positive error is added to positive error or negative error is added to negative) and sometimes it empties the basket — as when positive error is added to negative, or vice versa.
- collect all of these errors over time (take integral over time).
- Integral part = Ki * E  
  where E = E + e(current) * ∆t,         the loop stores all measured errors in E

```
# integral part
self.vars.E = self.vars.E + e_current * delta_t 
integral = ki * self.vars.E
```
#### D - Derivative part
- the derivative is looking at the rate of change of the error. The more error changes, the larger the derivative factor becomes.
- Kd have to be less than 1.
- Derivative part = Kd * ((e(current) - e(previous)) / ∆t)

```
# derivative part
if delta_t == 0:
  derivate = 0
else:
  derivate = kd * ((e_current - self.vars.e_previous)/delta_t)
```
#### u - System input signal (PID controller output signal)
```
u = Proportional + integral + derivate    # u : input signal

if u >= 0:
  throttle_output = u
  brake_output    = 0
elif u < 0:
  throttle_output = 0
  brake_output    = -u
```

### Debugging and Visualization
Real-time debugging information is displayed, including the steering angle, direction, and lane overlay on the screen. Additionally, a history of steering values and key press counts is logged for analysis.

## Dependencies
The project requires the following libraries:

Python 3.8+
- OpenCV
- PyTorch
- NumPy
- PIL (Pillow)
- PyAutoGUI
- MSS
- TorchVision

You can install them with:
```bash
pip install opencv-python-headless torch torchvision numpy Pillow pyautogui mss
```

## Demo
![Self Driving GIF](https://github.com/AnshChoudhary/Self-Driving-Model/blob/main/self-driving-curve-trim.gif)
