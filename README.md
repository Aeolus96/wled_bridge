# WLED Control Node

This is a ROS (Robot Operating System) node for controlling a WLED device using it's JSON API, allowing you to update ticker text and manage other features via ROS messages.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [ROS Messages](#ros-messages)
- [Contributing](#contributing)
- [License](#license)

## Prerequisites

- ROS (Noetic preferred) installed on your system. [ROS Installation Instructions](http://wiki.ros.org/ROS/Installation)
- WLED release [v0.14.0-b1](https://github.com/Aircoookie/WLED/releases/tag/v0.14.0-b1) or later
- I'm using 32x8 WS2812B panels from BTF-LIGHTING with WLED Controller from Domestic Automation LLC for testing
- Follow instruction at [WLED](https://kno.wled.ge/) for Hardware Configuration and check the hardware_setup folder for example configurations that work on the test hardware mentioned above.

## Installation

1. Clone the repository to your ROS workspace:

    ```bash
    git clone https://github.com/Aeolus96/wled_bridge.git
    ```

2. Build the package:

    ```bash
    cd <your-ros-workspace>
    catkin_make
    ```

## Usage

Launch the WLED control node:

- Source ROS
- Update the WLED Device IP address (x.x.x.x) in the launch file or via command line argument

```bash
roslaunch wled_control_node wled_control.launch

# OR Use with IPv4

roslaunch wled_control_node wled_control.launch wled_device_address:=<Device-IP>
```

## ROS Messages

### `/wled_bridge/ticker_text` (std_msgs/String)

This topic is used to update the ticker text (scrolling right to left) on the WLED device.

Example:

```bash
rostopic pub /wled_bridge/ticker_text std_msgs/String "Hello, WLED!"
```

### `/wled_bridge/image` (sensor_msgs/Image)

This topic takes ROS Image messages, uses OpenCV to resize to them to fit the LED Matrix size defined in Dynamic Recofigure. Default is `32x8`.
> NOTE: Support for other LED segment types or WLED effects is limited due to the scope of this project

## Contributing

Feel free to contribute to this project. Fork the repository, make your changes, and submit a pull request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
