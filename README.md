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


### Published Topics

| Topic                | Message Type                    | Description                              |
| -------------------- | ------------------------------- | ---------------------------------------- |
| `/left/image_raw`    | `sensor_msgs/msg/Image`         | Raw image from the left camera sensor.   |
| `/left/camera_info`  | `sensor_msgs/msg/CameraInfo`    | Calibration data for the left camera.    |
| `/right/image_raw`   | `sensor_msgs/msg/Image`         | Raw image from the right camera sensor.  |
| `/right/camera_info` | `sensor_msgs/msg/CameraInfo`    | Calibration data for the right camera.   |

### Parameters

| Parameter        | Type     | Default Value                                | Description                                                                        |
| ---------------- | -------- | -------------------------------------------- | ---------------------------------------------------------------------------------- |
| `conf_file_path` | `string` | *(path to file in `config` dir)* | Full path to the camera calibration `.conf` file.                                  |
| `video_device`   | `string` | `/dev/video2`                                | The V4L2 device file for the camera.                                               |
| `frame_width`    | `int`    | `2560`                                       | Total width of the side-by-side image to request. (HD: 2560, FHD: 3840).            |
| `frame_height`   | `int`    | `720`                                        | Height of the image to request. (HD: 720, FHD: 1080).                               |
| `fps`            | `int`    | `30`                                         | Frame rate to request from the camera.                                             |

### Performance Note: High CPU Usage


