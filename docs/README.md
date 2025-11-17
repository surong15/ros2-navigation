# Isaac Sim Auto Navigation Extension (with ROS Integration)

This extension provides **Auto Navigation** functionality and integrates **ROS2 / rosbridge_websocket** with **camera image publishing**.  
Users can input target coordinates through the UI or ROS Topics to make the robot automatically move to specified positions.

---

## Features

- **Auto Navigation**: Input target coordinates in the UI, and the robot will navigate to the target point  
- **ROS Integration**: Receive navigation coordinates and stop commands from ROS Topics via rosbridge  
- **Camera Publishing**: Support publishing Isaac Sim camera images and camera info to ROS2 Topics  
- **UI Controls**: Provides a simple control interface for inputting coordinates, starting/stopping navigation, and controlling camera publishing  
- **Status Reporting**: Publishes `/ROBOTNAME/navigation_status` and `/ROBOTNAME/robot_pose`  

---

## Installation and Usage

1. **Place Extension**
   - Place `extension.py` in your Isaac Sim extension project directory, for example:  
     ```
     <your_isaac_sim_extensions>/isaac.autonav.rosbridge/
     ```
   - Ensure you have the corresponding `extension.toml`

2. **Enable Extension**
   - In Isaac Sim, open: `Window > Extension Manager`  
   - Search for `Auto Navigation with ROS Bridge` → Enable  

3. **Start Simulation**
   - Load the scene and ensure the robot and camera exist at the specified paths  
   - Click **Play**  

4. **Navigation Control**
   - Open the `Auto Navigation with ROS Bridge` window  
   - Manually input coordinates and press **Go to Target**, or send targets via ROS topics  

5. **ROS Verification**
   - Ensure rosbridge_server is running:  
     ```bash
     ros2 launch rosbridge_server rosbridge_websocket_launch.xml
     ```
   - Check in ROS environment:  
     ```bash
     ros2 topic echo /ROBOTNAME/navigation_status
     ros2 topic echo /ROBOTNAME/robot_pose
     ```

---

## 📡 ROS Topics

### Subscribed (From ROS → Isaac Sim)
- `/ROBOTNAME/navigation_goal` (`geometry_msgs/PoseStamped`): Navigation target coordinates  
- `/ROBOTNAME/navigation_coordinates` (`std_msgs/String`): String coordinate input (e.g., `"2.0, 3.0, 0.0"`)  
- `/ROBOTNAME/navigation_stop` (`std_msgs/Empty`): Stop navigation  
- `/ROBOTNAME/tf` (`tf2_msgs/TFMessage`): TF for updating robot position and orientation  

### Published (From Isaac Sim → ROS)
- `/ROBOTNAME/navigation_status` (`std_msgs/String`): Navigation status report  
- `/ROBOTNAME/robot_pose` (`geometry_msgs/PoseStamped`): Robot's current position  
- `/isaac/camera/persp` (`sensor_msgs/Image`): Camera RGB image  
- `/isaac/camera/persp_camera_info` (`sensor_msgs/CameraInfo`): Camera intrinsics  

---

## Code Sections Users Need to Modify

Different users need to adjust the following code based on their scene:

### 1. Robot Path (TODO)
```python
self.robot_prim_path = "/World/Demo_8F/_R05D00002_only_bottom_sim_"
```
- Modify to your robot model's **USD Prim path**

---

### 2. Camera Setup
```python
self.camera_prim_path = "/World/Camera_persp"
self.camera_topic_name = "/isaac/camera/persp"
self.camera_frame_id = "camera_persp"
```
- `camera_prim_path`: Camera prim path in Isaac Sim  
- `camera_topic_name`: ROS Topic name  
- `camera_frame_id`: Frame ID for ROS image  

---

### 3. rosbridge Connection
```python
self.ros_bridge_client = RosBridgeWebSocketClient(self, host="localhost", port=9090)
```
- If rosbridge_server is on a remote machine, modify the host and port, for example:  
  ```python
  host="192.168.1.10", port=9090
  ```

---

### 4. Publishing Frequency and Resolution
In `publish_camera_rgb` and `publish_camera_info`, you can adjust:  
```python
freq=30, width=640, height=480
```

---

## Troubleshooting Guide

- **Robot doesn't move**  
  - Verify that `robot_prim_path` matches the path in your scene  
- **Camera image not publishing**  
  - Verify that `camera_prim_path` is correct  
  - Check if `omni.replicator.core` and `omni.isaac.ros2_bridge` are enabled  
- **ROS connection fails**  
  - Verify that `rosbridge_server` is running on the corresponding IP/Port  
- **Coordinate input not responding**  
  - Please input floating-point numbers and press Enter, then click **Go to Target**  

---

## Usage Workflow Summary

1. Start Isaac Sim and load the scene  
2. Modify the robot path and camera settings in the code (if needed)  
3. Enable the `Auto Navigation with ROS Bridge` Extension  
4. Start rosbridge_server (`localhost:9090` or specified IP)  
5. Press **Play**  
6. Send navigation coordinates using the UI or ROS Topics  
7. Verify that navigation and camera data are published correctly  

## Command Publishing Example

1. Service
```bash
source install/setup.bash
```
```bash
ros2 service call /ROBOTNAME/set_goal_pose msgs_interface/srv/SetGoalPose "
{
task_mode: 0,
task_times: 1,
to_point: {
id: 'warehouse_entrance',
x: 5.0,
y: -7.7,
theta: 0.0,
velocity_level: 1.0
}
}
"
```
