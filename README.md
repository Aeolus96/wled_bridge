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
- WLED release v0.14.0-b1 or later

## Installation

1. Clone the repository to your ROS workspace:

    ```bash
    git clone https://github.com/Aeolus96/wled_control_node.git
    ```

2. Build the package:

    ```bash
    cd <your-ros-workspace>
    catkin_make
    ```

## Usage

1. Launch the WLED control node:

    - Source ROS
    - Update the WLED Device IP address (x.x.x.x) in the launch file or via command line argument

    ```bash
    roslaunch wled_control_node wled_control.launch
    ```

    OR

    ```bash
    roslaunch wled_control_node wled_control.launch wled_device_address:=<Device-IP>
    ```

2. Send ROS messages to control the WLED device:

    - Update ticker text:

        ```bash
        rostopic pub /wled_bridge/ticker_text std_msgs/String "your_text_here"
        ```

    - Other ROS messages for controlling lights and features.

## ROS Messages

### `/wled_bridge/ticker_text` (std_msgs/String)

This topic is used to update the ticker text on the WLED device.

Example:

```bash
rostopic pub /wled_bridge/ticker_text std_msgs/String "Hello, WLED!"
```

More features will be added as the project develops.

## Contributing

Feel free to contribute to this project. Fork the repository, make your changes, and submit a pull request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
