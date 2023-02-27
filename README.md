# VL53L5CX ROS 2 Driver for Linux

This repository contains ROS 2 package for STM [VL53L5CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html) and [VL53L7CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l7cx.html) ToF ranging sensor.

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

The ranging node is a `LifecycleNode`.
Each transition performs the following operations:

| Transition | Operation                            |
| ---------- | ------------------------------------ |
| configure  | Initialize devices (takes some time) |
| activate   | Start ranging                        |
| deactivate | Stop ranging                         |

Here are example steps.

1. Connect I2C wires (SCL, SDA, 3V3 and GND) and RST/LPn for each sensor (if multiple sensors are connected).
2. Create your own config file in `config/`. Then specify arbitrary I2C addresses and the GPIO chip and pins (according to your connection). See `config/` for examples.
3. Start the node by using `ranging.launch.py`. `two_sensors.yaml` is used as config file here.

    ``` shell
    ros2 launch vl53l5cx ranging.launch.py config_file:=two_sensors.yaml
    ```

    The launch file invokes the *configure* transition.

4. Start ranging by triggering the *activate* transition.

    ```shell
    ros2 lifecycle set /vl53l5cx activate
    ```

    Or, set `activate_on_configure:=true` in launch argument to start ranging immediately after initialization.

    ``` shell
    ros2 launch vl53l5cx ranging.launch.py config_file:=two_sensors.yaml activate_on_configure:=true
    ```

5. Stop ranging by triggering the *deactivate* transition.

    ```shell
    ros2 lifecycle set /vl53l5cx deactivate
    ```

## Published Topics

| Name                       | Type                           | Description             |
| -------------------------- | ------------------------------ | ----------------------- |
| `~/x[ADDRESS]/image`       | `sensor_msgs::msg::Image`      | Distance (metric float) |
| `~/x[ADDRESS]/camera_info` | `sensor_msgs::msg::CameraInfo` | Camera info             |

## Parameters

The parameters can be configured dynamically.
If a parameter is changed in the *active* state, the change will be applied when ranging is restarted.

See `src/vl53l5cx_params.yaml` for the parameter definitions and descriptions.

## Docker

You can use your own Docker image or one built from `docker/Dockerfile` in this repository.
Here is an example using `ros:humble` image. In `your_ws/`:

```shell
docker run -v $(pwd):/workspaces/your_ws --device /dev/i2c-1 --device /dev/gpiochip0 --net host -it ros:humble
```

The options `--device /dev/i2c-1 --device /dev/gpiochip0` give the user I2C and GPIO privileges.
