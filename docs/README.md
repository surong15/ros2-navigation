# Isaac Sim Auto Navigation Extension (with ROS Integration)

這個 Extension 提供 **自動導航 (Auto Navigation)** 功能，並整合 **ROS2 / rosbridge_websocket** 與 **相機影像發布**。  
使用者可透過 UI 或 ROS Topic 輸入目標座標，讓機器人自動移動到指定位置。

---

## 📦 功能

- **自動導航**：在 UI 輸入目標座標，機器人會前往目標點  
- **ROS 整合**：透過 rosbridge 接收 ROS Topic 中的導航座標與停止指令  
- **相機發布**：支援將 Isaac Sim 的相機畫面與相機資訊發布到 ROS2 Topic  
- **UI 操作**：提供簡易的控制介面，可輸入座標、開始/停止導航、控制相機發布  
- **狀態回報**：發布 `/baymax/navigation_status` 與 `/baymax/robot_pose`  

---

## 🚀 安裝與使用

1. **放置 Extension**
   - 將 `extension.py` 放到 Isaac Sim extension 專案目錄，例如：  
     ```
     <your_isaac_sim_extensions>/isaac.autonav.rosbridge/
     ```
   - 確保有對應的 `extension.toml`

2. **啟用 Extension**
   - 在 Isaac Sim 開啟：`Window > Extension Manager`  
   - 搜尋 `Auto Navigation with ROS Bridge` → 啟用  

3. **啟動模擬**
   - 載入場景，並確保機器人與相機存在於指定路徑  
   - 點擊 **▶️ Play**  

4. **導航控制**
   - 開啟 `Auto Navigation with ROS Bridge` 視窗  
   - 手動輸入座標後按下 **Go to Target**，或透過 ROS topic 發送目標  

5. **ROS 驗證**
   - 確保 rosbridge_server 正在執行：  
     ```bash
     ros2 launch rosbridge_server rosbridge_websocket_launch.xml
     ```
   - 在 ROS 環境檢查：  
     ```bash
     ros2 topic echo /baymax/navigation_status
     ros2 topic echo /baymax/robot_pose
     ```

---

## 📡 ROS Topics

### 訂閱 (從 ROS → Isaac Sim)
- `/baymax/navigation_goal` (`geometry_msgs/PoseStamped`)：導航目標座標  
- `/baymax/navigation_coordinates` (`std_msgs/String`)：字串座標輸入 (例如 `"2.0, 3.0, 0.0"`)  
- `/baymax/navigation_stop` (`std_msgs/Empty`)：停止導航  
- `/baymax/tf` (`tf2_msgs/TFMessage`)：TF，用於更新機器人位置與方向  

### 發布 (從 Isaac Sim → ROS)
- `/baymax/navigation_status` (`std_msgs/String`)：導航狀態回報  
- `/baymax/robot_pose` (`geometry_msgs/PoseStamped`)：機器人當前位置  
- `/isaac/camera/persp` (`sensor_msgs/Image`)：相機 RGB 影像  
- `/isaac/camera/persp_camera_info` (`sensor_msgs/CameraInfo`)：相機內參  

---

## ⚙️ 使用者需要修改的程式碼

不同使用者需依據場景調整以下程式碼：

### 1. 機器人路徑 (TODO)
```python
self.robot_prim_path = "/World/Demo_8F/_R05D00002_only_bottom_sim_"
```
- 修改成你機器人模型的 **USD Prim 路徑**

---

### 2. 相機設置
```python
self.camera_prim_path = "/World/Camera_persp"
self.camera_topic_name = "/isaac/camera/persp"
self.camera_frame_id = "camera_persp"
```
- `camera_prim_path`：相機 prim 在 Isaac Sim 的路徑  
- `camera_topic_name`：ROS Topic 名稱  
- `camera_frame_id`：ROS 影像的 Frame ID  

---

### 3. rosbridge 連線
```python
self.ros_bridge_client = RosBridgeWebSocketClient(self, host="localhost", port=9090)
```
- 若 rosbridge_server 在遠端，請修改 host 與 port，例如：  
  ```python
  host="192.168.1.10", port=9090
  ```

---

### 4. 發布頻率與解析度
在 `publish_camera_rgb` 與 `publish_camera_info` 可調整：  
```python
freq=30, width=640, height=480
```

---

## 🛠️ 除錯指南

- **機器人不動**  
  - 確認 `robot_prim_path` 與場景中的路徑一致  
- **相機影像無法發布**  
  - 確認 `camera_prim_path` 是否正確  
  - 檢查 `omni.replicator.core` 與 `omni.isaac.ros2_bridge` 是否啟用  
- **ROS 無法連線**  
  - 確認 `rosbridge_server` 是否在對應 IP/Port 運行  
- **座標輸入沒反應**  
  - 請輸入浮點數並按下 Enter，再點擊 **Go to Target**  

---

## 📖 使用流程簡要

1. 啟動 Isaac Sim 並載入場景  
2. 修改程式碼中的機器人路徑與相機設置（如有需要）  
3. 啟用 `Auto Navigation with ROS Bridge` Extension  
4. 啟動 rosbridge_server (`localhost:9090` 或指定 IP)  
5. 按 **▶️ Play**  
6. 使用 UI 或 ROS Topic 發送導航座標  
7. 驗證導航與相機資料是否正確發布  

## 指令發布範例

1. service
```python
source install/setup.bash
```
```python
ros2 service call /baymax/set_goal_pose msgs_interface/srv/SetGoalPose "
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
