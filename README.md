# Readme


---


Tentative files needed for us:
---

# Vision Module (Shivam)

### `camera_publisher.py`

Captures frames from Raspberry Pi camera and publishes them as `/camera/image_raw`.

### `aruco_detector.py`

Subscribes to camera images, detects ArUco markers, estimates pose, and publishes `/aruco/pose`.

### `marker_visualizer.py` (optional)

Shows live annotated camera feed with detected markers for debugging.

---

# Obstacle Detection Module

### `lidar_obstacle_detector.py`

Subscribes to `/scan`, checks distances, and publishes `/obstacle_status`.

---

# Motion & Navigation Module

### `motion_controller.py`

Subscribes to marker pose + obstacle status and publishes movement commands `/cmd_vel`.

### `simple_behaviors.py`

Provides reusable functions like stop(), turn_left(), turn_right(), move_forward().

---

# Integration & Coordination

### `navigation_state_machine.py`

Implements logic such as: detect marker → move to marker → stop at marker → continue path.

---

# ROS Setup Files

### `CMakeLists.txt`

Defines how your Python ROS package is built/installed.

### `package.xml`

Declares ROS dependencies (e.g., rospy, sensor_msgs, cv_bridge, aruco, OpenCV).

---

# Launch Files

### `vision.launch`

Starts camera node + aruco detection node.

### `navigation.launch`

Starts motion controller + obstacle detector.

### `full_system.launch`

Runs everything together in one command.

---

# Optional if we want

### `record_data.sh`

shell script recording rosbag of camera + lidar during testing.

### `play_sim.launch`

Launch for simulation mode (Gazebo or RViz only).

---

# I think these are minimum required files to complete the project

 `camera_publisher.py`
 `aruco_detector.py`
 `lidar_obstacle_detector.py`
 `motion_controller.py`
 `full_system.launch`
 `package.xml`
 `CMakeLists.txt`

---



How to Run based on GPT:

chmod +x camera_publisher.py
ros2 run <package_name> camera_publisher


chmod +x aruco_detector.py
ros2 run <package_name> aruco_detector



setup.py must include:

entry_points={
    'console_scripts': [
        'camera_publisher = <package_name>.camera_publisher:main',
        'aruco_detector = <package_name>.aruco_detector:main'
    ],
},

so commands become:

ros2 run <package_name> camera_publisher
ros2 run <package_name> aruco_detector




---

save full_system.launch:

<your_package_name>/launch/full_system.launch

run:

roslaunch <your_package_name> full_system.launch



setup.py:
