# Base-Station-Flask

This is a ROS2-Flask application that serves as a base station for monitoring and controlling a robot. It provides an api for admins to manage tokens, and a web socket connection for real-time communication with the robot; (one way communication from the robot to the base station). The application is built using Flask and ROS2, and it uses a MariaDB database to store tokens.


This will run on all operating systems that support Python and ROS2, including Windows, macOS, and Linux. Follow the [ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html) to set up ROS2 on your system. Also follow the [Setup Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) to configure your ROS2 environment.

> [!IMPORTANT]
> Make sure to follow the installation guide and setup carefully to ensure a successful setup.


## Description
The Base Station is a critical component of the Loon-E ASV’s software system, responsible for receiving
data from the ASV and providing it to users on a web client application. It acts as an intermediary between
the ASV and the users, allowing for remote access to the ASV’s data and operations.
The base station computer stays on the same network and relays ASV data to the web client, but it does not control the ASV or perform navigation. In other words, The base station acts as a server that receives ASV data and serves it to the web client, while the access point, switch, and router provide the network connectivity. It is designed to meet the specifications required for participation in maritime robotics competitions such as RoboBoat and Njord.

### Compatibility

The Base Station can be a Windows, Linux, or macOS computer running ROS2 Humble, which is the same version of ROS2 used on the ASV’s computer.

> [!WARNING]
> The base station must be on the same network, DDS, Domain ID, and ROS2 version as the ASV’s
> computer to receive data from the ASV.

Using CycloneDDS as the data distribution service, the base station will subscribe to all topics published by
the ASV, allowing it to receive and process all relevant data from the ASV’s operations. The base station
will then relay this data to the web client application for visualization and monitoring by users.

## Tech Stack

- Flask: A micro web framework for Python.
- ROS2: A set of software libraries and tools for building robot applications.
- [ZED SDK](https://www.stereolabs.com/en-ca/developers/release): A software development kit for working with ZED cameras.
- CUDA
- MariaDB: An open-source relational database management system.

> [!NOTE]
> The flask application will connect to another website server. that other website server hosts the web client application, which is responsible for visualizing the data received from the ASV. The flask application will relay the data to the web client application using web sockets, allowing for real-time updates and visualization of the ASV’s data.


## File Structure

It runs both ROS2, a flask application, and a MariaDB database. The ROS2 nodes are responsible for receiving data from the ASV and relaying it to the web client, while the Flask application serves as a web socket for the data and as an API for token generation. The MariaDB database is used to store tokens for authentication and authorization purposes.

```bash
Base-Station-Flask/
├── LICENSE
├── README.md
├── docker-compose.yml
├── flask_app/
│   ├── app.py
│   ├── requirements.txt
│   └── templates/
│       └── index.html
├── ros2_nodes/
│   ├── src/
│   │   ├── zed-ros-interfaces/
│   │   ├── zed-bounding-box/
│   │   └── base-station-receiver/
│   ├── build/
│   ├── install/
│   └── log/
└── sql/
    └── init.sql
```

## Installation

Following environment variables are required to run the application:

| Variable Name | Value |
|----------------|-------|
| `ROS_DOMAIN_ID` | 0 |
| `ROS_DISTRO` | humble |
| `ROS_VERSION` | 2 |
| `ROS_PYTHON_VERSION` | 3 |
| `ROS_LOCALHOST_ONLY` | 0 |
| `CYCLONEDDS_URI` | `file:///path/to/cyclonedds.xml` |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |





