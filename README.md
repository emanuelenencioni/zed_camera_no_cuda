# zed_camera_no_cuda

A ROS2 package for interfacing with ZED cameras without requiring CUDA.  This version utilizes OpenCV for image processing instead of CUDA.

## Overview

This package provides a ROS2 interface for Stereolabs ZED cameras, enabling access to video streams. Other sensors are in TODOs. It is designed to function without CUDA support, leveraging OpenCV for computationally intensive tasks.  This makes it suitable for systems where CUDA is unavailable or undesirable.

## Features

*   **Image Acquisition:** Publishes left and right camera images as ROS2 Image messages.
*   **Camera Info:** Provides camera calibration information for each camera.
*   **No CUDA Dependency:** Uses OpenCV for image processing, eliminating the need for CUDA-enabled hardware.

## Dependencies

*   ROS 2 (Tested on Humble)
*   OpenCV (version 4.6 or higher recommended)
*   rclcpp
*   sensor_msgs
*   image_transport
*   cv_bridge
*   tf2_ros
*   diagnostic_updater
*   ament_cmake


