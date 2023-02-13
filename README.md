# VL53L5CX ROS 2 Driver for Linux

This repository contains ROS 2 package for STM [VL53L5CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html) ToF ranging sensor.

Tested on Raspberry Pi 4 Model B.

## Features

- Basic ranging (distance measurement)
- Multiple sensors with the same configuration
- Configuration through `ros2 param`

## Installation

### Building

Assume the workspace:

```plaintext
your_ws/
  src/
    vl53l5cx_ros2/
```

Make sure the dependencies are installed.

```shell
rosdep install --from-paths src
```

Build and source the workspace.

```shell
colcon build
. install/setup.bash
```

### I2C and GPIO Permissions

This package requires I2C and GPIO (if you use RST, LPn or INT pin) access.
Make sure the user has privileges for `/dev/i2c-1` and `/dev/gpiochip*`.

These permissions can be easily handled by using a Docker container.
See [Docker section](#docker).

## Usage

### Single Sensor

1. Connect I2C wires (SCL, SDA, 3V3 and GND).
2. Run the node.

    ``` shell
    ros2 run vl53l5cx ranging_node
    ```

    Then, the node starts initialization.

3. Start ranging by calling `/vl53l5cx/start_ranging` service using `rqt_service_caller` or following command:

    ```shell
    ros2 service call /vl53l5cx/start_ranging std_srvs/srv/Empty
    ```

4. Stop ranging by calling `/vl53l5cx/stop_ranging` service or just pressing Ctrl+C.

### Multiple Sensors (Launch)

1. Connect I2C wires and RST/LPn for each sensor.
2. In your own config file, specify an arbitrary I2C address and the RST/LPn pin number (according to your connection) for each sensor.

    **Note**
    Use `gpiod` to check the GPIO pin availability.

    ```shell
    sudo apt install gpiod
    gpioinfo
    ```

3. Launch. See `launch/ranging.launch.py` for details.

    ```shell
    # Launch node using config/two_sensors.yaml
    ros2 launch vl53l5cx ranging.launch.py config_file:='two_sensors.yaml'
    ```

4. Start/stop ranging as described in the previous section.

## Published Topics

| Name                       | Type                           | Description             |
| -------------------------- | ------------------------------ | ----------------------- |
| `~/x[ADDRESS]/image`       | `sensor_msgs::msg::Image`      | Distance (metric float) |
| `~/x[ADDRESS]/camera_info` | `sensor_msgs::msg::CameraInfo` | Camera info             |

## Services

| Name              | Type                      | Description   |
| ----------------- | ------------------------- | ------------- |
| `~/start_ranging` | `sensor_srvs::srv::Empty` | Start ranging |
| `~/stop_ranging`  | `sensor_srvs::srv::Empty` | Stop ranging  |

## Parameters

The parameters can be configured dynamically.
If a parameter is changed while the node is running, the change is applied when ranging is restarted.

See `src/vl53l5cx_params.yaml` for the parameter definitions and descriptions.

## Docker

You can use your own Docker image or one built from `docker/Dockerfile` in this repository.
Here is an example using `ros:humble` image. In `your_ws/`:

```shell
docker run -v $(pwd):/workspaces/your_ws --device /dev/i2c-1 --device /dev/gpiochip0 --net host -it ros:humble
```

The options `--device /dev/i2c-1 --device /dev/gpiochip0` give the user I2C and GPIO privileges.
