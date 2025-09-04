# 僅可使用ROBOTNAME
# 可用座標連續導航，無法躲避障礙物
# 使用 WebSocket 與 rosbridge_server 通信，避免庫衝突
# 可以遠端暫停，不會當機
# 改成ros service版本
# 透過websocket會無法正確中斷進行中的導航任務，在電腦自己送沒問題

"""
Auto Navigation Extension for Isaac Sim with ROS Integration
- 允許用戶在UI輸入目標座標，機器人自動導航至該點
- 支援透過 rosbridge_websocket 接收 ROS 座標輸入
"""
import omni.ext
import omni.ui as ui
import omni.usd
from pxr import UsdGeom, Gf, UsdPhysics
import math
import threading
import json
import time

# Isaac Sim Camera 相關 imports
try:
    import omni.syntheticdata as sd
    import omni.replicator.core as rep
    import omni.graph.core as og
    from omni.isaac.core.utils import extensions
    CAMERA_PUBLISHING_AVAILABLE = True
    print("[AutoNav] Camera publishing imports successful")
except ImportError as e:
    print(f"[AutoNav] Camera publishing imports failed: {e}")
    CAMERA_PUBLISHING_AVAILABLE = False

# 檢查 WebSocket 是否可用
try:
    import websocket
    import ssl
    WEBSOCKET_AVAILABLE = True
    print("[AutoNav] websocket-client imported successfully")
except ImportError:
    print("[AutoNav] websocket-client not available, attempting to install...")
    try:
        import subprocess
        import sys
        import importlib
        
        # 嘗試安裝 websocket-client
        subprocess.check_call([sys.executable, "-m", "pip", "install", "websocket-client"])
        print("[AutoNav] websocket-client installed successfully")
        
        # 強制重新載入模組路徑
        importlib.invalidate_caches()
        
        # 嘗試添加用戶安裝路徑到 sys.path
        import site
        site.main()
        
        # 重新嘗試導入
        import websocket
        import ssl
        WEBSOCKET_AVAILABLE = True
        print("[AutoNav] websocket-client imported successfully after installation")
    except ImportError as import_error:
        print(f"[AutoNav] Still cannot import websocket after installation: {import_error}")
        # 嘗試直接添加路徑
        try:
            import os
            user_site = os.path.expanduser("~/.local/lib/python3.10/site-packages")
            if user_site not in sys.path:
                sys.path.insert(0, user_site)
                print(f"[AutoNav] Added {user_site} to sys.path")
            
            # 再次嘗試導入
            import websocket
            import ssl
            WEBSOCKET_AVAILABLE = True
            print("[AutoNav] websocket-client imported successfully after path fix")
        except Exception as final_error:
            print(f"[AutoNav] Final attempt failed: {final_error}")
            print(f"[AutoNav] sys.path: {sys.path[:3]}...")  # 只顯示前3個路徑
            WEBSOCKET_AVAILABLE = False
    except Exception as e:
        print(f"[AutoNav] Failed to install or import websocket-client: {e}")
        WEBSOCKET_AVAILABLE = False

if WEBSOCKET_AVAILABLE:
    class RosBridgeWebSocketClient:
        """透過 WebSocket 與 rosbridge_server 通信的客戶端 - Service 版本"""
        
        def __init__(self, navigation_extension, host="localhost", port=9090):
            self.nav_ext = navigation_extension
            self.host = host
            self.port = port
            self.ws_url = f"ws://{host}:{port}"
            self.ws = None
            self.connected = False
            self.running = False
            
            # Service 相關屬性
            self.current_service_call_id = None  # 儲存當前服務呼叫的 ID
            self.current_service_name = None     # 儲存當前服務名稱
            self.service_in_progress = False     # 標記是否有服務正在處理中
            
            print(f"[AutoNav] 初始化 ROS Bridge WebSocket 客戶端 (Service模式): {self.ws_url}")
        
        def connect(self):
            """連接到 rosbridge websocket server"""
            try:
                print(f"[AutoNav] 嘗試連接到 rosbridge: {self.ws_url}")
                
                # 創建 WebSocket 連接
                self.ws = websocket.WebSocketApp(
                    self.ws_url,
                    on_open=self._on_open,
                    on_message=self._on_message,
                    on_error=self._on_error,
                    on_close=self._on_close
                )
                
                # 在背景執行緒中運行
                self.running = True
                self.ws_thread = threading.Thread(target=self._run_websocket, daemon=True)
                self.ws_thread.start()
                
                print(f"[AutoNav] WebSocket 客戶端已啟動，等待連接...")
                
            except Exception as e:
                print(f"[AutoNav] WebSocket 連接失敗: {e}")
                self.nav_ext.update_ros_bridge_status(connected=False)
        
        def _run_websocket(self):
            """在背景執行緒中運行 WebSocket"""
            try:
                print("[AutoNav] WebSocket 執行緒已啟動")
                self.ws.run_forever(ping_interval=30, ping_timeout=10)
            except Exception as e:
                print(f"[AutoNav] WebSocket 執行錯誤: {e}")
                self.connected = False
                if hasattr(self.nav_ext, 'update_ros_bridge_status'):
                    self.nav_ext.update_ros_bridge_status(connected=False)
        
        def _on_open(self, ws):
            """WebSocket 連接建立"""
            self.connected = True
            print("[AutoNav] ✅ ROS Bridge WebSocket 已連接")
            
            # 更新 UI 狀態
            self.nav_ext.update_ros_bridge_status(connected=True)
            
            # 註冊 services 而不是訂閱 topics
            self._advertise_services()
            
            # 仍然需要發布狀態的 topics
            self._advertise_publishing_topics()
            
            # 仍然需要訂閱 TF topic 來獲取機器人位置
            self._subscribe_to_tf_topic()
            
            # 訂閱 navigation stop topic
            self._subscribe_to_stop_topic()
            
            # 開始狀態發布定時器
            self._start_status_timer()
        
        def _on_close(self, ws, close_status_code, close_msg):
            """WebSocket 連接關閉"""
            self.connected = False
            print(f"[AutoNav] ❌ ROS Bridge WebSocket 連接關閉: {close_status_code}, {close_msg}")
            if hasattr(self.nav_ext, 'update_ros_bridge_status'):
                self.nav_ext.update_ros_bridge_status(connected=False)
        
        def _on_error(self, ws, error):
            """WebSocket 錯誤"""
            self.connected = False
            print(f"[AutoNav] ❌ ROS Bridge WebSocket 錯誤: {error}")
            if hasattr(self.nav_ext, 'update_ros_bridge_status'):
                self.nav_ext.update_ros_bridge_status(connected=False)
        
        def _on_message(self, ws, message):
            """接收 WebSocket 訊息"""
            try:
                data = json.loads(message)
                op = data.get("op")
                
                # 只針對重要操作顯示訊息，忽略常規的 publish
                if op == "call_service":
                    # 處理服務呼叫
                    service = data.get("service")
                    args = data.get("args", {})
                    call_id = data.get("id")
                    
                    print(f"[AutoNav] 🔔 收到服務呼叫:")
                    print(f"[AutoNav]   服務: {service}")
                    print(f"[AutoNav]   ID: {call_id}")
                    print(f"[AutoNav]   參數: {json.dumps(args, indent=2)}")
                    
                    if service == "/ROBOTNAME/set_goal_pose":
                        self._handle_set_goal_pose_service(args, call_id, service)
                        
                elif op == "publish":
                    # 處理 topic 發布 (主要是 TF) - 不顯示訊息
                    topic = data.get("topic")
                    msg = data.get("msg", {})
                    
                    if topic == "/ROBOTNAME/tf":
                        self._parse_tf_message(msg)
                    elif topic == "/ROBOTNAME/navigation_stop":
                        # std_msgs/Empty 格式 - 停止導航
                        # ros2 topic pub --once /ROBOTNAME/navigation_stop std_msgs/Empty "{}"
                        current_time = time.time()
                        print(f"[AutoNav] 🛑 收到停止導航指令 via topic (時間: {current_time:.3f})")
                        print(f"[AutoNav] 🛑 Stop 訊息內容: {json.dumps(msg)}")
                        print(f"[AutoNav] 🛑 訊息來源: ros2 topic pub 或類似命令")
                        
                        # 檢查是否是重複的停止訊息（在很短時間內）
                        time_since_last_stop = current_time - getattr(self.nav_ext, '_last_stop_message_time', 0)
                        if time_since_last_stop < 0.5:  # 0.5秒內的重複訊息
                            print(f"[AutoNav] 🛑 忽略重複停止訊息 (間隔: {time_since_last_stop:.3f}s)")
                            return
                        
                        # 🔧 移除停止請求檢查 - 允許停止覆蓋任何狀態
                        # 停止訊息應該總是被執行，不管當前狀態如何
                        
                        # 記錄這次停止訊息的時間
                        self.nav_ext._last_stop_message_time = current_time
                        
                        # std_msgs/Empty 可能是空字典 {} 或空物件
                        # 不管訊息內容如何，只要收到這個 topic 就執行停止
                        print(f"[AutoNav] 🛑 執行停止指令（無論訊息內容為何）")
                        
                        # 立即設置停止 flag - 會在下一個物理步驟中被處理
                        self.nav_ext._ros_stop_requested = True
                        
                        # 如果導航正在進行，也立即調用停止方法確保即時性
                        if hasattr(self.nav_ext, 'nav_active') and self.nav_ext.nav_active:
                            print("[AutoNav] 🛑 導航正在進行中，立即設置停止標誌")
                            # 立即設置導航為 False，提供多重保護
                            self.nav_ext.nav_active = False
                            # 不需要設置 _force_stop，_ros_stop_requested 就足夠了
                            # 物理線程會在下一步中立即處理
                            print("[AutoNav] 🛑 已設置 nav_active=False，物理線程將立即處理")
                        else:
                            print("[AutoNav] 🛑 導航未啟動，但仍設置停止標誌以防萬一")
                    else:
                        # 顯示其他未處理的 topic，幫助調試
                        if topic not in ["/ROBOTNAME/navigation_status", "/ROBOTNAME/robot_pose"]:
                            print(f"[AutoNav] 📩 收到未處理的 topic: {topic}")
                
                # 其他重要操作才顯示
                elif op in ["service_response", "advertise_service"]:
                    print(f"[AutoNav] 📋 操作回應: {op}")
                    if "result" in data:
                        print(f"[AutoNav] 📋 結果: {data.get('result')}")
                        
            except Exception as e:
                print(f"[AutoNav] ❌ 處理 WebSocket 訊息失敗: {e}")
                print(f"[AutoNav] 原始訊息: {message}")
        
        def _handle_set_goal_pose_service(self, args, call_id, service_name):
            """處理 SetGoalPose 服務呼叫 - 強制中斷舊導航並立即切換目標"""
            try:
                print(f"[AutoNav] 🎯 處理服務呼叫:")
                print(f"[AutoNav]   ID: {call_id}")
                print(f"[AutoNav]   服務: {service_name}")
                print(f"[AutoNav]   參數: {json.dumps(args, indent=2)}")

                # 強制中斷舊導航與 service
                if self.service_in_progress:
                    print(f"[AutoNav] 🔄 強制中斷舊 service 任務")
                    if self.current_service_call_id and self.current_service_name:
                        self._send_service_response(
                            self.current_service_call_id,
                            self.current_service_name,
                            success=False,
                            message="Navigation task cancelled by new service request"
                        )
                    self.nav_ext.nav_active = False
                    self.nav_ext._stop_all_movement()
                    self.current_service_call_id = None
                    self.current_service_name = None
                    self.service_in_progress = False

                # 解析參數
                task_mode = args.get("task_mode", 0)
                task_times = args.get("task_times", 1)
                from_point = args.get("from_point", {})
                to_point = args.get("to_point", {})
                target_x = to_point.get("x", 0.0)
                target_y = to_point.get("y", 0.0)
                target_theta = to_point.get("theta", 0.0)
                velocity_level = to_point.get("velocity_level", 1.0)
                point_id = to_point.get("id", "")
                print(f"[AutoNav] 目標: ID={point_id}, x={target_x}, y={target_y}")

                if not isinstance(target_x, (int, float)) or not isinstance(target_y, (int, float)):
                    print(f"[AutoNav] ❌ 無效的座標參數")
                    self._send_service_response(
                        call_id, service_name,
                        success=False,
                        message="Invalid coordinate parameters"
                    )
                    return

                current_pos = self.nav_ext.get_current_position()
                if not current_pos:
                    print(f"[AutoNav] ⚠️ 機器人位置不可獲取，仍嘗試執行導航")

                # 儲存新 service 狀態
                self.current_service_call_id = call_id
                self.current_service_name = service_name
                self.service_in_progress = True
                print(f"[AutoNav] 💾 儲存服務狀態: ID={self.current_service_call_id}")

                # 立即切換目標並開始導航
                self.nav_ext.set_target_from_service(target_x, target_y, 0.0, point_id)
                print(f"[AutoNav] ✅ 服務請求已接受，開始導航到 ({target_x}, {target_y})")
            except Exception as e:
                print(f"[AutoNav] ❌ 處理服務呼叫失敗: {e}")
                import traceback
                traceback.print_exc()
                self._send_service_response(
                    call_id, service_name,
                    success=False,
                    message=f"Service processing error: {str(e)}"
                )
        
        def _is_robot_ready(self):
            """檢查機器人是否準備好接受新的導航任務"""
            try:
                # 檢查導航系統是否空閒
                if hasattr(self.nav_ext, 'nav_active') and self.nav_ext.nav_active:
                    return False
                
                # 檢查機器人位置是否可獲取
                current_pos = self.nav_ext.get_current_position()
                if not current_pos:
                    return False
                
                return True
            except:
                return False
        
        def _send_service_response(self, call_id, service_name, success, message):
            """發送服務回應 - 修正版本"""
            try:
                # 嘗試 rosbridge 標準格式
                response_msg = {
                    "op": "service_response",
                    "service": service_name,
                    "values": {
                        "success": success,
                        "message": message
                    },
                    "id": call_id,
                    "result": True  # 添加 result 字段
                }
                
                print(f"[AutoNav] 🔍 發送服務回應: ID={call_id}, Success={success}")
                print(f"[AutoNav] 📤 回應內容: {json.dumps(response_msg, indent=2)}")
                
                if self.connected and self.ws:
                    message_str = json.dumps(response_msg)
                    self.ws.send(message_str)
                    print(f"[AutoNav] ✅ WebSocket 訊息已發送: {len(message_str)} 字節")
                    
                    # 等待一小段時間確保訊息發送
                    time.sleep(0.1)
                else:
                    print(f"[AutoNav] ❌ WebSocket 未連接")
                
                # 清除當前服務狀態
                self.current_service_call_id = None
                self.current_service_name = None
                self.service_in_progress = False
                
            except Exception as e:
                print(f"[AutoNav] ❌ 發送服務回應失敗: {e}")
                import traceback
                traceback.print_exc()
                # 即使發送失敗也要清除狀態
                self.current_service_call_id = None
                self.current_service_name = None
                self.service_in_progress = False
        
        def send_navigation_success_response(self, message="Navigation completed successfully"):
            """導航成功時呼叫此方法發送回應"""
            if self.current_service_call_id and self.current_service_name:
                self._send_service_response(
                    self.current_service_call_id,
                    self.current_service_name,
                    success=True,
                    message=message
                )
        
        def send_navigation_failure_response(self, message="Navigation failed"):
            """導航失敗時呼叫此方法發送回應"""
            if self.current_service_call_id and self.current_service_name:
                self._send_service_response(
                    self.current_service_call_id,
                    self.current_service_name,
                    success=False,
                    message=message
                )
        
        def _parse_tf_message(self, tf_msg):
            """解析 TF 訊息，更新機器人位置 (減少日誌輸出)"""
            try:
                transforms = tf_msg.get("transforms", [])
                for transform in transforms:
                    header = transform.get("header", {})
                    child_frame_id = transform.get("child_frame_id", "")
                    
                    # 尋找機器人的框架
                    if ("base_link" in child_frame_id or 
                        "ROBOTNAME" in child_frame_id or 
                        "robot" in child_frame_id or
                        "tn__" in child_frame_id or
                        "R05D00002" in child_frame_id):
                        
                        translation = transform.get("transform", {}).get("translation", {})
                        rotation = transform.get("transform", {}).get("rotation", {})
                        
                        # 更新機器人位置
                        self.nav_ext.robot_position = [
                            translation.get("x", 0.0),
                            translation.get("y", 0.0),
                            translation.get("z", 0.0)
                        ]
                        
                        # 更新機器人方向
                        self.nav_ext.robot_orientation = [
                            rotation.get("x", 0.0),
                            rotation.get("y", 0.0),
                            rotation.get("z", 0.0),
                            rotation.get("w", 1.0)
                        ]
                        
                        self.nav_ext.tf_position_updated = True
                        self.nav_ext.tf_last_update_time = time.time()
                        
                        # 只在位置有顯著變化時才輸出 (可選)
                        # if not hasattr(self, '_last_logged_pos') or \
                        #    abs(self.nav_ext.robot_position[0] - self._last_logged_pos[0]) > 0.1 or \
                        #    abs(self.nav_ext.robot_position[1] - self._last_logged_pos[1]) > 0.1:
                        #     print(f"[AutoNav] TF 位置更新: ({self.nav_ext.robot_position[0]:.2f}, {self.nav_ext.robot_position[1]:.2f})")
                        #     self._last_logged_pos = self.nav_ext.robot_position.copy()
                        
                        break
                        
            except Exception as e:
                print(f"[AutoNav] 解析 TF 訊息失敗: {e}")
        
        def _advertise_services(self):
            """註冊 ROS services"""
            services = [
                {
                    "op": "advertise_service",
                    "type": "msgs_interface/srv/SetGoalPose",  # 修正：添加 /srv/
                    "service": "/ROBOTNAME/set_goal_pose"
                }
            ]
            
            for service_config in services:
                if self.connected and self.ws:
                    self.ws.send(json.dumps(service_config))
                    print(f"[AutoNav] 📢 註冊服務: {service_config['service']}")
                    print(f"[AutoNav] 📢 服務類型: {service_config['type']}")
                    time.sleep(0.1)
                
        def _subscribe_to_tf_topic(self):
            """仍然需要訂閱 TF topic 來獲取機器人位置"""
            tf_subscription = {
                "op": "subscribe",
                "topic": "/ROBOTNAME/tf",
                "type": "tf2_msgs/TFMessage"
            }
            
            if self.connected and self.ws:
                self.ws.send(json.dumps(tf_subscription))
                print(f"[AutoNav] 訂閱 TF topic: /ROBOTNAME/tf")
        
        def _subscribe_to_stop_topic(self):
            """訂閱 navigation stop topic - 只訂閱，不廣告（因為我們是接收方）"""
            stop_subscription = {
                "op": "subscribe",
                "topic": "/ROBOTNAME/navigation_stop",
                "type": "std_msgs/Empty"
            }
            
            if self.connected and self.ws:
                self.ws.send(json.dumps(stop_subscription))
                print(f"[AutoNav] 🛑 訂閱 Stop topic: /ROBOTNAME/navigation_stop (std_msgs/Empty)")
                time.sleep(0.1)  # 小延遲確保訂閱生效
        
        def _advertise_publishing_topics(self):
            """廣告我們要發布的 topics (保持原有邏輯)"""
            topics = [
                {
                    "op": "advertise",
                    "topic": "/ROBOTNAME/navigation_status",
                    "type": "std_msgs/String"
                },
                {
                    "op": "advertise",
                    "topic": "/ROBOTNAME/robot_pose", 
                    "type": "geometry_msgs/PoseStamped"
                }
            ]
            
            for topic_config in topics:
                if self.connected and self.ws:
                    self.ws.send(json.dumps(topic_config))
                    print(f"[AutoNav] 廣告發布 topic: {topic_config['topic']}")
        
        def _start_status_timer(self):
            """開始狀態發布定時器 (保持原有邏輯)"""
            def publish_status():
                while self.running and self.connected:
                    try:
                        # 發布導航狀態
                        status_msg = {
                            "op": "publish",
                            "topic": "/ROBOTNAME/navigation_status",
                            "msg": {"data": str(self.nav_ext.status)}
                        }
                        if self.connected and self.ws:
                            self.ws.send(json.dumps(status_msg))
                        
                        # 發布機器人位置
                        current_pos = self.nav_ext.get_current_position()
                        if current_pos:
                            current_time = time.time()
                            if (hasattr(self.nav_ext, 'tf_position_updated') and 
                                self.nav_ext.tf_position_updated and 
                                (current_time - self.nav_ext.tf_last_update_time) < 5.0):
                                orientation = {
                                    "x": self.nav_ext.robot_orientation[0],
                                    "y": self.nav_ext.robot_orientation[1], 
                                    "z": self.nav_ext.robot_orientation[2],
                                    "w": self.nav_ext.robot_orientation[3]
                                }
                            else:
                                orientation = {"x": 0, "y": 0, "z": 0, "w": 1}
                            
                            pose_msg = {
                                "op": "publish",
                                "topic": "/ROBOTNAME/robot_pose",
                                "msg": {
                                    "header": {
                                        "stamp": {"sec": int(time.time()), "nanosec": 0},
                                        "frame_id": "world"
                                    },
                                    "pose": {
                                        "position": {
                                            "x": current_pos[0],
                                            "y": current_pos[1], 
                                            "z": current_pos[2]
                                        },
                                        "orientation": orientation
                                    }
                                }
                            }
                            if self.connected and self.ws:
                                self.ws.send(json.dumps(pose_msg))
                        
                        time.sleep(0.5)  # 2Hz 發布頻率
                        
                    except Exception as e:
                        print(f"[AutoNav] 發布狀態失敗: {e}")
                        time.sleep(1.0)
            
            self.status_thread = threading.Thread(target=publish_status, daemon=True)
            self.status_thread.start()
        
        def disconnect(self):
            """斷開 WebSocket 連接"""
            self.running = False
            self.connected = False
            
            # 如果有進行中的服務，發送失敗回應
            if self.service_in_progress:
                self.send_navigation_failure_response("Connection lost during navigation")
            
            if self.ws:
                self.ws.close()
            print("[AutoNav] ROS Bridge WebSocket 已斷開")


# 相機影像發布相關函數
def publish_camera_rgb(camera_prim_path, topic_name, frame_id, freq=10, width=640, height=480):
    """發布相機 RGB 影像到指定 ROS topic"""
    if not CAMERA_PUBLISHING_AVAILABLE:
        print("[AutoNav][Error] Camera publishing not available - missing Isaac Sim modules")
        return None
        
    try:
        print(f"[AutoNav] 設置相機 RGB 發布器:")
        print(f"[AutoNav]   Camera Prim: {camera_prim_path}")
        print(f"[AutoNav]   Topic: {topic_name}")
        print(f"[AutoNav]   Frame ID: {frame_id}")
        print(f"[AutoNav]   Frequency: {freq} Hz")
        print(f"[AutoNav]   Resolution: {width}x{height}")

        # 啟用 ROS2 Bridge extension (如果尚未啟用)
        extensions.enable_extension("omni.isaac.ros2_bridge")

        # 檢查相機是否存在
        stage = omni.usd.get_context().get_stage()
        camera_prim = stage.GetPrimAtPath(camera_prim_path)
        if not camera_prim or not camera_prim.IsValid():
            print(f"[AutoNav][Error] Camera prim not found: {camera_prim_path}")
            return None

        # 用 Replicator 建立 render product - 使用傳入的解析度
        render_product = rep.create.render_product(camera_prim_path, resolution=(width, height))
        if not render_product:
            print(f"[AutoNav][Error] Replicator create.render_product 失敗: {camera_prim_path}")
            return None

        # 設置發布參數
        step_size = int(60/freq)
        queue_size = 1
        node_namespace = ""

        rv = sd.SyntheticData.convert_sensor_type_to_rendervar("Rgb")
        writer = rep.writers.get(rv + "ROS2PublishImage")

        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name
        )

        # 附加到 render product
        writer.attach([render_product])

        # 設置執行頻率控制
        gate_path = sd.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", render_product.path
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

        print(f"[AutoNav] ✅ 相機 RGB 發布器設置完成")
        return writer

    except Exception as e:
        print(f"[AutoNav][Error] 設置相機 RGB 發布器失敗: {e}")
        import traceback
        traceback.print_exc()
        return None


def publish_camera_info(camera_prim_path, topic_name, frame_id, freq=10, width=640, height=480):
    """發布相機內參信息到 ROS topic"""
    if not CAMERA_PUBLISHING_AVAILABLE:
        print("[AutoNav][Error] Camera info publishing not available - missing Isaac Sim modules")
        return None
        
    try:
        print(f"[AutoNav] 設置相機 Info 發布器:")
        print(f"[AutoNav]   Camera Prim: {camera_prim_path}")
        print(f"[AutoNav]   Topic: {topic_name}")
        print(f"[AutoNav]   Frame ID: {frame_id}")
        print(f"[AutoNav]   Frequency: {freq} Hz")
        print(f"[AutoNav]   Resolution: {width}x{height}")

        # 啟用 ROS2 Bridge extension (如果尚未啟用)
        extensions.enable_extension("omni.isaac.ros2_bridge")

        # 檢查相機是否存在
        stage = omni.usd.get_context().get_stage()
        camera_prim = stage.GetPrimAtPath(camera_prim_path)
        if not camera_prim or not camera_prim.IsValid():
            print(f"[AutoNav][Error] Camera prim not found: {camera_prim_path}")
            return None

        # 用 Replicator 建立 render product - 使用傳入的解析度
        render_product = rep.create.render_product(camera_prim_path, resolution=(width, height))
        if not render_product:
            print(f"[AutoNav][Error] Replicator create.render_product 失敗: {camera_prim_path}")
            return None

        # 使用傳入的解析度計算內參矩陣
        k = [width, 0, width/2, 0, height, height/2, 0, 0, 1]
        r = [1,0,0,0,1,0,0,0,1]
        p = [width, 0, width/2, 0, 0, height, height/2, 0, 0, 0, 1, 0]
        distortion_model = "plumb_bob"
        d = [0,0,0,0,0]

        step_size = int(60/freq)
        queue_size = 1
        node_namespace = ""

        writer = rep.writers.get("ROS2PublishCameraInfo")
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name,
            width=width,
            height=height,
            projectionType=distortion_model,
            k=[k],
            r=[r],
            p=[p],
            physicalDistortionModel=distortion_model,
            physicalDistortionCoefficients=d,
        )
        writer.attach([render_product])

        gate_path = sd.SyntheticData._get_node_path(
            "PostProcessDispatchIsaacSimulationGate", render_product.path
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

        print(f"[AutoNav] ✅ 相機 Info 發布器設置完成: {topic_name}")
        return writer

    except Exception as e:
        print(f"[AutoNav][Error] 設置相機 Info 發布器失敗: {e}")
        import traceback
        traceback.print_exc()
        return None


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.nav_active = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.status = "Idle"
        
        # 🔧 重要：初始化停止相關標誌
        self._ros_stop_requested = False
        self._force_stop = False
        self._last_stop_message_time = 0.0  # 追蹤最後停止訊息的時間
        
        # TODO
        self.robot_prim_path = "/World/Demo_8F/_R05D00002_only_bottom_sim_"  # USD parent prim
        self.articulation_prim_path = None  # 真正物理位置的 prim
        self._timeline_sub = None
        self._physx_sub = None
        
        # WebSocket ROS Bridge 整合
        self.ros_bridge_client = None
        self.last_ros_coordinates = "None"
        self.current_point_id = ""  # 新增：儲存當前目標點 ID
        
        # TF 座標資訊 (優先使用 ROS TF 資料)
        self.robot_position = [0.0, 0.0, 0.0]  # x, y, z
        self.robot_orientation = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w (四元數)
        self.tf_position_updated = False
        self.tf_last_update_time = 0.0
        
        # 相機發布相關屬性
        # TODO
        self.camera_publishers = {}  # 儲存相機發布器
        self.camera_prim_path = "/World/Camera_persp"  # 俯視相機路徑
        self.camera_topic_name = "/isaac/camera/persp"  # ROS topic 名稱
        self.camera_frame_id = "camera_persp"  # ROS frame ID
        self.camera_publishing_active = False  # 相機發布狀態
        
        # 初始化 ROS Bridge WebSocket (如果可用)
        if WEBSOCKET_AVAILABLE:
            self._init_ros_bridge()
        
        # UI 建立
        self._window = ui.Window("Auto Navigation with ROS Bridge", width=500, height=350)
        self._build_ui()
        
        # 在 UI 建立後嘗試連接 ROS Bridge
        if WEBSOCKET_AVAILABLE:
            self._try_connect_ros_bridge()
            # 定期更新連接狀態
            self._setup_status_timer()
        
        # 訂閱 timeline event
        import omni.timeline
        timeline = omni.timeline.get_timeline_interface()
        event_stream = timeline.get_timeline_event_stream()
        self._timeline_sub = event_stream.create_subscription_to_pop(self._on_timeline_event)
        
        # 如果 Timeline 已經在 Play 狀態，立即訂閱 _on_physics_step
        if timeline.is_playing():
            import omni.physx
            if not self._physx_sub:
                self._physx_sub = omni.physx.acquire_physx_interface().subscribe_physics_step_events(self._on_physics_step)
        # --- 自動啟動 Replicator 及相機發布器 ---
        try:
            extensions.enable_extension("omni.replicator.core")
            print("[AutoNav] Replicator extension enabled.")
        except Exception as e:
            print(f"[AutoNav][Error] Replicator extension enable failed: {e}")
        self.setup_camera_publisher()

    def _init_ros_bridge(self):
        """初始化 ROS Bridge WebSocket 連接"""
        try:
            self.ros_bridge_client = RosBridgeWebSocketClient(self)
            print("[AutoNav] ROS Bridge WebSocket 整合已啟動")
            print("[AutoNav] 可通過以下 topics 接收座標:")
            print("[AutoNav]   - /ROBOTNAME/navigation_goal (geometry_msgs/PoseStamped)")
            print("[AutoNav]   - /ROBOTNAME/navigation_coordinates (std_msgs/String)")
            print("[AutoNav]   - /ROBOTNAME/navigation_stop (std_msgs/Empty)")
            print("[AutoNav]     使用命令: ros2 topic pub --once /ROBOTNAME/navigation_stop std_msgs/Empty \"{}\"")
            print("[AutoNav]   - /ROBOTNAME/tf (tf2_msgs/TFMessage) [機器人位置]")
            print("[AutoNav] 確保 rosbridge_server 在 localhost:9090 運行")
            
            # 在 UI 建立後再嘗試連接
            
        except Exception as e:
            print(f"[AutoNav] ROS Bridge 初始化失敗: {e}")
            self.ros_bridge_client = None

    def _setup_status_timer(self):
        """設置狀態更新定時器"""
        def update_status():
            while hasattr(self, '_window') and self._window:
                try:
                    if self.ros_bridge_client:
                        self.update_ros_bridge_status(connected=self.ros_bridge_client.connected)
                    # ROS 停止請求的處理已移到 _on_physics_step 中優先處理
                    # 這裡不再處理停止請求，避免重複執行
                    # 如果物理步驟沒有運行，停止請求會在 WebSocket 處理中立即生效
                    time.sleep(0.1)  # 每0.1秒更新一次，減少延遲
                except Exception as e:
                    print(f"[AutoNav] 狀態更新異常: {e}")
                    break
        
        self._status_thread = threading.Thread(target=update_status, daemon=True)
        self._status_thread.start()

    def _try_connect_ros_bridge(self):
        """嘗試連接到 ROS Bridge"""
        if self.ros_bridge_client:
            self.ros_bridge_client.connect()
        else:
            print("[AutoNav] ROS Bridge 客戶端未初始化")

    def on_shutdown(self):
        if self._timeline_sub:
            self._timeline_sub = None
        if self._physx_sub:
            self._physx_sub = None
        
        # 關閉相機發布器
        if hasattr(self, 'camera_publishing_active') and self.camera_publishing_active:
            self.stop_camera_publisher()
        
        # 關閉 ROS Bridge
        if self.ros_bridge_client:
            self.ros_bridge_client.disconnect()
            
        self._window = None

    def update_ros_bridge_status(self, connected=False):
        """更新 ROS Bridge 連接狀態顯示"""
        if hasattr(self, 'ros_status_label'):
            if connected:
                bridge_status = "🟢 ROS Bridge Connected"
                status_color = 0xFF00AA00
            elif WEBSOCKET_AVAILABLE and self.ros_bridge_client:
                bridge_status = "🟡 ROS Bridge Connecting..."
                status_color = 0xFFAAAA00
            elif WEBSOCKET_AVAILABLE:
                bridge_status = "🔴 WebSocket Available but Not Connected"
                status_color = 0xFFAA5500
            else:
                bridge_status = "🔴 WebSocket Not Available"
                status_color = 0xFFAA0000
            
            self.ros_status_label.text = f"ROS Bridge Status: {bridge_status}"
            self.ros_status_label.style = {"color": status_color}

    def set_target_from_service(self, x, y, z, point_id=""):
        """從 ROS Service 接收目標座標"""
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.current_point_id = point_id  # 儲存點的 ID
        
        # 更新 UI 顯示
        if hasattr(self, 'x_field'):
            self.x_field.model.set_value(x)
            self.y_field.model.set_value(y)
            self.z_field.model.set_value(z)
        
        # 更新最後接收的座標顯示
        self.last_ros_coordinates = f"({x:.2f}, {y:.2f}, {z:.2f}) [ID: {point_id}]"
        if hasattr(self, 'last_ros_coords_label'):
            self.last_ros_coords_label.text = f"Last Service Request: {self.last_ros_coordinates}"
        
        print(f"[AutoNav] Service 目標設定: x={x}, y={y}, z={z}, ID={point_id}")
        
        # 開始導航
        self._start_navigation()

    def set_target_from_ros(self, x, y, z):
        """從 ROS 接收目標座標"""
        self.target_x = x
        self.target_y = y
        self.target_z = z
        
        # 更新 UI 顯示
        if hasattr(self, 'x_field'):
            self.x_field.model.set_value(x)
            self.y_field.model.set_value(y)
            self.z_field.model.set_value(z)
        
        # 更新最後接收的座標顯示
        self.last_ros_coordinates = f"({x:.2f}, {y:.2f}, {z:.2f})"
        if hasattr(self, 'last_ros_coords_label'):
            self.last_ros_coords_label.text = f"Last ROS Coordinates: {self.last_ros_coordinates}"
        
        print(f"[AutoNav] ROS 目標設定: x={x}, y={y}, z={z}")
        
        # 自動開始導航
        self._start_navigation()
    
    def stop_navigation_from_ros(self):
        """從 ROS 接收停止指令，強制中斷所有導航與 service"""
        print("[AutoNav] 🛑 執行 ROS 停止導航命令")
        self.nav_active = False
        self._stop_all_movement()
        self._ros_stop_requested = False
        self._force_stop = False
        self.status = "Navigation stopped by ROS command."
        if hasattr(self, 'status_label'):
            self.status_label.text = f"Status: {self.status}"
        # 強制中斷 service 任務
        if (self.ros_bridge_client and 
            hasattr(self.ros_bridge_client, 'service_in_progress') and 
            self.ros_bridge_client.service_in_progress):
            print("[AutoNav] 🛑 強制中斷 service 任務（被停止指令覆蓋）")
            self.ros_bridge_client.send_navigation_failure_response("Navigation stopped by ROS stop command")
            self.ros_bridge_client.current_service_call_id = None
            self.ros_bridge_client.current_service_name = None
            self.ros_bridge_client.service_in_progress = False
        print("[AutoNav] 🛑 ROS 停止指令處理完成")
    
    def get_current_position(self):
        """獲取機器人當前位置（優先使用 TF 資料）"""
        # 優先使用 TF 資料 (如果在 5 秒內有更新)
        current_time = time.time()
        if (hasattr(self, 'tf_position_updated') and 
            self.tf_position_updated and 
            (current_time - self.tf_last_update_time) < 5.0):
            return self.robot_position
        
        # 回退到 USD prim 座標
        try:
            stage = omni.usd.get_context().get_stage()
            if self.articulation_prim_path:
                prim = stage.GetPrimAtPath(self.articulation_prim_path)
                if prim and prim.IsValid():
                    xform = UsdGeom.Xformable(prim)
                    local_to_world = xform.ComputeLocalToWorldTransform(0)
                    pos = local_to_world.ExtractTranslation()
                    return [float(pos[0]), float(pos[1]), float(pos[2])]
        except:
            pass
        return None

    def setup_camera_publisher(self):
        """設置相機影像發布器"""
        if not CAMERA_PUBLISHING_AVAILABLE:
            print("[AutoNav][Error] 相機發布功能不可用 - 缺少 Isaac Sim 模組")
            return False
            
        try:
            print(f"[AutoNav] 正在設置相機發布器...")
            
            # 檢查相機是否存在
            stage = omni.usd.get_context().get_stage()
            camera_prim = stage.GetPrimAtPath(self.camera_prim_path)
            if not camera_prim or not camera_prim.IsValid():
                print(f"[AutoNav][Error] 相機 prim 不存在: {self.camera_prim_path}")
                return False
            
            # 設置 RGB 影像發布器
            rgb_publisher = publish_camera_rgb(
                camera_prim_path=self.camera_prim_path,
                topic_name=self.camera_topic_name,
                frame_id=self.camera_frame_id,
                freq=30
            )
            
            if rgb_publisher:
                self.camera_publishers['rgb'] = rgb_publisher
                # 同時設置相機內參發布器
                info_topic = self.camera_topic_name + "_camera_info"
                info_publisher = publish_camera_info(
                    camera_prim_path=self.camera_prim_path,
                    topic_name=info_topic,
                    frame_id=self.camera_frame_id,
                    freq=30
                )
                if info_publisher:
                    self.camera_publishers['info'] = info_publisher
                self.camera_publishing_active = True
                print(f"[AutoNav] ✅ 相機發布器設置成功!")
                print(f"[AutoNav]   RGB Topic: {self.camera_topic_name}")
                print(f"[AutoNav]   Info Topic: {info_topic}")
                print(f"[AutoNav]   Frame ID: {self.camera_frame_id}")
                # Replicator 4.x/5.x 會自動啟動 pipeline，不需手動呼叫 rep.run()
                return True
            else:
                print("[AutoNav][Error] RGB 發布器設置失敗")
                return False
                
        except Exception as e:
            print(f"[AutoNav][Error] 設置相機發布器時發生異常: {e}")
            return False

    def stop_camera_publisher(self):
        """停止相機發布器"""
        try:
            if self.camera_publishers:
                print("[AutoNav] 正在停止相機發布器...")
                # 清理發布器 (Isaac Sim 會自動處理資源清理)
                self.camera_publishers.clear()
                self.camera_publishing_active = False
                print("[AutoNav] ✅ 相機發布器已停止")
        except Exception as e:
            print(f"[AutoNav][Error] 停止相機發布器時發生異常: {e}")

    def toggle_camera_publishing(self):
        """切換相機發布狀態"""
        if self.camera_publishing_active:
            self.stop_camera_publisher()
        else:
            self.setup_camera_publisher()
        
        # 更新 UI 狀態
        if hasattr(self, 'camera_btn'):
            if self.camera_publishing_active:
                self.camera_btn.text = "Stop Camera Publishing"
                self.camera_btn.style = {"color": 0xFFFF0000}
            else:
                self.camera_btn.text = "Start Camera Publishing"
                self.camera_btn.style = {"color": 0xFF00AA00}

    def _build_ui(self):
        # 自動尋找 articulation root
        stage = omni.usd.get_context().get_stage()
        import re
        parent_prim = stage.GetPrimAtPath(self.robot_prim_path)
        self.articulation_prim_path = None
        if parent_prim and parent_prim.IsValid():
            for child in parent_prim.GetChildren():
                if re.match(r"tn__.*", child.GetName()):
                    self.articulation_prim_path = child.GetPath().pathString
                    break
        if not self.articulation_prim_path:
            self.articulation_prim_path = self.robot_prim_path
            
        # 取得 articulation root 的初始位置
        prim = stage.GetPrimAtPath(self.articulation_prim_path)
        default_x, default_y, default_z = 0.0, 0.0, 0.0
        if prim and prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            local_to_world = xform.ComputeLocalToWorldTransform(0)
            pos = local_to_world.ExtractTranslation()
            default_x, default_y, default_z = pos[0], pos[1], pos[2]
            
        with self._window.frame:
            with ui.VStack():
                ui.Label(f"Current Robot Path: {self.robot_prim_path}")
                self.articulation_label = ui.Label(f"Articulation Root: {self.articulation_prim_path}", style={"color": 0xFF00AA00})
                
                # ROS Bridge 狀態顯示
                if WEBSOCKET_AVAILABLE and self.ros_bridge_client:
                    bridge_status = "🟢 ROS Bridge Connecting..."
                    status_color = 0xFF00AAFF
                elif WEBSOCKET_AVAILABLE:
                    bridge_status = "🔴 WebSocket Available but Not Connected"
                    status_color = 0xFFAA5500
                else:
                    bridge_status = "🔴 WebSocket Not Available"
                    status_color = 0xFFAA0000
                self.ros_status_label = ui.Label(f"ROS Bridge Status: {bridge_status}", style={"color": status_color})
                
                ui.Spacer(height=5)
                ui.Separator()
                ui.Spacer(height=5)
                
                ui.Label("Manual Coordinate Input:")
                ui.Label("(After typing, press Enter before clicking Go to Target)", style={"color": 0xFFAA0000})
                
                with ui.HStack():
                    self.x_field = ui.FloatField(model=ui.SimpleFloatModel(default_x))
                    self.y_field = ui.FloatField(model=ui.SimpleFloatModel(default_y))
                    self.z_field = ui.FloatField(model=ui.SimpleFloatModel(default_z))
                
                with ui.HStack():
                    self.go_btn = ui.Button("Go to Target", clicked_fn=self._on_go_clicked)
                    self.stop_btn = ui.Button("Stop Navigation", clicked_fn=self._on_stop_clicked)
                
                ui.Spacer(height=10)
                ui.Label("ROS Coordinate Reception (via rosbridge_websocket):")
                if WEBSOCKET_AVAILABLE:
                    ui.Label("Navigation: /ROBOTNAME/navigation_goal, /ROBOTNAME/navigation_coordinates", style={"color": 0xFF888888})
                    ui.Label("Position Source: /ROBOTNAME/tf (TF Transform)", style={"color": 0xFF888888})
                    ui.Label("Stop: /ROBOTNAME/navigation_stop", style={"color": 0xFF888888})
                    ui.Label("  命令: ros2 topic pub --once /ROBOTNAME/navigation_stop std_msgs/Empty \"{}\"", style={"color": 0xFF666666})
                    ui.Label("Ensure rosbridge_server is running on localhost:9090", style={"color": 0xFF888888})
                    
                    # ROS Bridge 重新連接按鈕
                    ui.Button("Reconnect to ROS Bridge", clicked_fn=self._on_reconnect_clicked)
                else:
                    ui.Label("websocket-client not available - pip install websocket-client", style={"color": 0xFFAA0000})
                
                # 顯示最後接收的 ROS 座標
                self.last_ros_coords_label = ui.Label(f"Last ROS Coordinates: {self.last_ros_coordinates}", style={"color": 0xFF00FFAA})
                ui.Label("ROS Service Reception (via rosbridge_websocket):")
                if WEBSOCKET_AVAILABLE:
                    ui.Label("Service: /ROBOTNAME/set_goal_pose (msgs_interface/SetGoalPose)", style={"color": 0xFF888888})
                    ui.Label("Position Source: /ROBOTNAME/tf (TF Transform)", style={"color": 0xFF888888})
                    ui.Label("Response: Waits for navigation completion", style={"color": 0xFF888888})
                    ui.Label("Ensure rosbridge_server is running on localhost:9090", style={"color": 0xFF888888})

                # 相機發布控制區域
                ui.Spacer(height=10)
                ui.Separator()
                ui.Spacer(height=5)
                ui.Label("Camera Publishing Control:")
                
                if CAMERA_PUBLISHING_AVAILABLE:
                    ui.Label(f"Camera Prim: {self.camera_prim_path}", style={"color": 0xFF888888})
                    ui.Label(f"Topic: {self.camera_topic_name}", style={"color": 0xFF888888})
                    ui.Label(f"Frame ID: {self.camera_frame_id}", style={"color": 0xFF888888})
                    
                    # 相機發布控制按鈕
                    with ui.HStack():
                        self.camera_btn = ui.Button(
                            "Start Camera Publishing", 
                            clicked_fn=self.toggle_camera_publishing,
                            style={"color": 0xFF00AA00}
                        )
                        
                    # 相機設置調整
                    ui.Spacer(height=5)
                    ui.Label("Camera Settings:")
                    with ui.HStack():
                        ui.Label("Prim Path:", width=80)
                        self.camera_path_field = ui.StringField(model=ui.SimpleStringModel(self.camera_prim_path))
                        
                    with ui.HStack():
                        ui.Label("Topic Name:", width=80)
                        self.camera_topic_field = ui.StringField(model=ui.SimpleStringModel(self.camera_topic_name))
                        
                    with ui.HStack():
                        ui.Label("Frame ID:", width=80)
                        self.camera_frame_field = ui.StringField(model=ui.SimpleStringModel(self.camera_frame_id))
                        
                    ui.Button("Update Camera Settings", clicked_fn=self._update_camera_settings)
                    
                else:
                    ui.Label("Camera publishing not available - missing Isaac Sim modules", style={"color": 0xFFAA0000})
                
                # 快速測試按鈕
                ui.Spacer(height=10)
                ui.Label("Quick Test Points:")
                with ui.HStack():
                    ui.Button("Warehouse Entrance", clicked_fn=lambda: self._set_quick_target(2.0, 3.0, 0.0))
                    ui.Button("Loading Area", clicked_fn=lambda: self._set_quick_target(3.0, -5.5, 0.0))
                    ui.Button("Origin", clicked_fn=lambda: self._set_quick_target(0.0, 0.0, 0.0))
                
                self.status_label = ui.Label(f"Status: {self.status}")

    def _on_reconnect_clicked(self):
        """重新連接到 ROS Bridge"""
        print("[AutoNav] 用戶請求重新連接 ROS Bridge")
        if WEBSOCKET_AVAILABLE:
            if self.ros_bridge_client:
                # 先斷開現有連接
                self.ros_bridge_client.disconnect()
            
            # 重新初始化並連接
            self._init_ros_bridge()
            self._try_connect_ros_bridge()
        else:
            print("[AutoNav] WebSocket 不可用，無法重新連接")

    def _set_quick_target(self, x, y, z):
        """設定快速目標點"""
        self.x_field.model.set_value(x)
        self.y_field.model.set_value(y) 
        self.z_field.model.set_value(z)
        self._on_go_clicked()

    def _update_camera_settings(self):
        """更新相機設置"""
        if hasattr(self, 'camera_path_field'):
            new_path = self.camera_path_field.model.get_value_as_string()
            new_topic = self.camera_topic_field.model.get_value_as_string()
            new_frame = self.camera_frame_field.model.get_value_as_string()
            
            # 檢查是否有變更
            if (new_path != self.camera_prim_path or 
                new_topic != self.camera_topic_name or 
                new_frame != self.camera_frame_id):
                
                print(f"[AutoNav] 更新相機設置:")
                print(f"[AutoNav]   路徑: {self.camera_prim_path} -> {new_path}")
                print(f"[AutoNav]   Topic: {self.camera_topic_name} -> {new_topic}")
                print(f"[AutoNav]   Frame: {self.camera_frame_id} -> {new_frame}")
                
                # 如果相機發布器正在運行，先停止
                was_active = self.camera_publishing_active
                if was_active:
                    self.stop_camera_publisher()
                
                # 更新設置
                self.camera_prim_path = new_path
                self.camera_topic_name = new_topic
                self.camera_frame_id = new_frame
                
                # 如果之前是啟動狀態，重新啟動
                if was_active:
                    self.setup_camera_publisher()
                
                print("[AutoNav] ✅ 相機設置已更新")

    def _start_navigation(self):
        """開始導航（修正版）- 僅在沒有停止請求時啟動"""
        print(f"[AutoNav] Target set: x={self.target_x}, y={self.target_y}, z={self.target_z}")
        if getattr(self, '_ros_stop_requested', False):
            print("[AutoNav] 🛑 檢測到停止請求，拒絕啟動新導航")
            self.status = "Navigation cancelled due to stop request"
            if hasattr(self, 'status_label'):
                self.status_label.text = f"Status: {self.status}"
            if (self.ros_bridge_client and 
                hasattr(self.ros_bridge_client, 'service_in_progress') and 
                self.ros_bridge_client.service_in_progress):
                self.ros_bridge_client.send_navigation_failure_response("Navigation cancelled due to stop request")
            return
        self._ros_stop_requested = False
        self._force_stop = False
        self._last_stop_message_time = 0.0
        print("[AutoNav] 🧹 已清理所有停止標誌，開始新的導航任務")
        self.status = "Navigating..."
        if hasattr(self, 'status_label'):
            self.status_label.text = f"Status: {self.status}"
            
        # 每次都重新尋找 articulation root
        stage = omni.usd.get_context().get_stage()
        import re
        parent_prim = stage.GetPrimAtPath(self.robot_prim_path)
        self.articulation_prim_path = None
        if parent_prim and parent_prim.IsValid():
            for child in parent_prim.GetChildren():
                if re.match(r"tn__.*", child.GetName()):
                    self.articulation_prim_path = child.GetPath().pathString
                    break
        if not self.articulation_prim_path:
            self.articulation_prim_path = self.robot_prim_path
            
        # 更新 UI 顯示 articulation root
        if hasattr(self, "articulation_label"):
            self.articulation_label.text = f"Articulation Root: {self.articulation_prim_path}"
            
        # 只有目標和目前位置距離大於 0.02 才啟動導航
        prim = stage.GetPrimAtPath(self.articulation_prim_path)
        if prim and prim.IsValid():
            xform = UsdGeom.Xformable(prim)
            local_to_world = xform.ComputeLocalToWorldTransform(0)
            pos = local_to_world.ExtractTranslation()
            robot_x, robot_y = pos[0], pos[1]
            dist = math.hypot(self.target_x - robot_x, self.target_y - robot_y)
            if dist > 0.02:
                self.nav_active = True
            else:
                self.status = "Already at target!"
                if hasattr(self, 'status_label'):
                    self.status_label.text = f"Status: {self.status}"
                self.nav_active = False
        else:
            self.status = f"Articulation prim not found!"
            if hasattr(self, 'status_label'):
                self.status_label.text = f"Status: {self.status}"
            self.nav_active = False

    def _on_stop_clicked(self):
        self.nav_active = False  # 關閉導航
        self._stop_all_movement()
        self.status = "Navigation stopped by user."
        if hasattr(self, 'status_label'):
            self.status_label.text = f"Status: {self.status}"
        
        # 如果有進行中的 service，發送失敗回應
        if self.ros_bridge_client and self.ros_bridge_client.service_in_progress:
            self.ros_bridge_client.send_navigation_failure_response("Navigation stopped by user")

    def _on_go_clicked(self):
        self.target_x = self.x_field.model.get_value_as_float()
        self.target_y = self.y_field.model.get_value_as_float()
        self.target_z = self.z_field.model.get_value_as_float()
        self._start_navigation()

    def _on_timeline_event(self, event):
        import omni.timeline
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            import omni.physx
            if not self._physx_sub:
                self._physx_sub = omni.physx.acquire_physx_interface().subscribe_physics_step_events(self._on_physics_step)
            # 強制重啟 camera publisher，確保 Replicator pipeline啟動
            self.setup_camera_publisher()
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self.nav_active = False
            self._physx_sub = None

    def _on_physics_step(self, step):
        # 優先檢查 ROS 停止請求 - 這樣停止指令會立即生效
        if getattr(self, "_ros_stop_requested", False):
            print("[AutoNav] 🛑 物理步驟中檢測到 ROS 停止請求，立即執行停止")
            self.stop_navigation_from_ros()
            # 標誌已在 stop_navigation_from_ros 中清除
            return  # 立即停止，不執行任何導航邏輯
        
        # 額外檢查：如果 nav_active 已被外部設為 False（如 WebSocket 處理）
        if not self.nav_active:
            # 確保徹底停止，但不重複執行停止邏輯
            if getattr(self, "_force_stop", False):
                print("[AutoNav] 🛑 檢測到 _force_stop 標誌，執行物理停止")
                self._stop_all_movement()
                self._force_stop = False  # 立即清除標誌
            return  # 完全不做物理操作，避免 race condition
        
        # 額外檢查：如果狀態已經是 "Target Reached" ，強制停止
        if hasattr(self, 'status') and "Target Reached" in str(self.status):
            self.nav_active = False
            self._change_wheel_velocity(0, 0)  # 確保停止
            return
            
        try:
            # 優先使用 TF 資料
            current_time = time.time()
            use_tf_data = (hasattr(self, 'tf_position_updated') and 
                          self.tf_position_updated and 
                          (current_time - self.tf_last_update_time) < 5.0)
            
            if use_tf_data:
                # 使用 TF 資料
                robot_x, robot_y = self.robot_position[0], self.robot_position[1]
                
                # 從四元數計算 yaw 角度
                qx, qy, qz, qw = self.robot_orientation
                siny_cosp = 2 * (qw * qz + qx * qy)
                cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                
                # print(f"[AutoNav] 使用 TF 座標: ({robot_x:.3f}, {robot_y:.3f})")
                
            else:
                # 回退到 USD prim 座標
                stage = omni.usd.get_context().get_stage()
                prim = stage.GetPrimAtPath(self.articulation_prim_path)
                if not prim or not prim.IsValid():
                    print(f"[AutoNav][Error] Articulation prim not found: {self.articulation_prim_path}")
                    self.status = f"Articulation prim not found!"
                    if hasattr(self, 'status_label'):
                        self.status_label.text = f"Status: {self.status}"
                    self.nav_active = False
                    return
                    
                xform = UsdGeom.Xformable(prim)
                local_to_world = xform.ComputeLocalToWorldTransform(0)
                pos = local_to_world.ExtractTranslation()
                robot_x, robot_y = pos[0], pos[1]
                
                rot = local_to_world.ExtractRotation()
                q = rot.GetQuaternion()
                siny_cosp = 2 * (q.GetReal() * q.GetImaginary()[2] + q.GetImaginary()[0] * q.GetImaginary()[1])
                cosy_cosp = 1 - 2 * (q.GetImaginary()[2]**2 + q.GetImaginary()[0]**2)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                
                print(f"[AutoNav] 使用 USD 座標: ({robot_x:.3f}, {robot_y:.3f})")
            
            dx = self.target_x - robot_x
            dy = self.target_y - robot_y
            dist = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            yaw_diff = target_yaw - yaw
            
            # 在計算過程中再次檢查停止請求，防止執行導航指令
            if getattr(self, "_ros_stop_requested", False) or getattr(self, "_force_stop", False) or not self.nav_active:
                print("[AutoNav] 🛑 在導航計算中檢測到停止請求，中止導航邏輯")
                if getattr(self, "_ros_stop_requested", False):
                    self.stop_navigation_from_ros()
                    self._ros_stop_requested = False
                elif getattr(self, "_force_stop", False):
                    self._stop_all_movement()
                    self._force_stop = False
                return
            
            while yaw_diff > math.pi:
                yaw_diff -= 2*math.pi
            while yaw_diff < -math.pi:
                yaw_diff += 2*math.pi
                
            angle_thres = 0.1
            dist_thres = 0.015  # 稍微收緊精度，但不要太嚴格
            max_speed = 300
            turn_speed = 150
            
            # 到達目標 - 優先檢查距離
            if dist < dist_thres:
                # 參考 ROBOTNAME_move 的停止邏輯
                self._stop_all_movement()
                self.status = "Target Reached!"
                if hasattr(self, 'status_label'):
                    self.status_label.text = f"Status: {self.status}"

                # 發送 service 成功回應
                if self.ros_bridge_client and self.ros_bridge_client.service_in_progress:
                    point_id = getattr(self, 'current_point_id', '')
                    message = f"Navigation completed successfully to point {point_id}" if point_id else "Navigation completed successfully"
                    self.ros_bridge_client.send_navigation_success_response(message)

                data_source = "TF" if use_tf_data else "USD"
                print(f"[AutoNav] 目標到達! 使用 {data_source} 資料")
                print(f"[AutoNav] 最終位置: ({robot_x:.3f}, {robot_y:.3f}), 目標: ({self.target_x:.3f}, {self.target_y:.3f})")
                print(f"[AutoNav] 誤差: dx={dx:.3f}, dy={dy:.3f}, dist={dist:.3f}")
                return
            
            # 接近目標時減少角度控制靈敏度，避免過度轉向
            if dist < 0.05:  # 非常接近目標時
                angle_thres = 0.3  # 放寬角度閾值
                turn_speed = 80   # 降低轉向速度
            elif dist < 0.2:  # 接近目標時
                angle_thres = 0.2  # 稍微放寬角度閾值
                turn_speed = 100  # 稍微降低轉向速度
            
            # 如果非常接近目標且速度很慢，直接停止避免微調
            if dist < 0.03 and abs(yaw_diff) < 0.3:  # 距離很近且角度還行
                print(f"[AutoNav] 非常接近目標且角度合理，提前停止避免微調")
                self._stop_all_movement()
                self.status = "Target Reached!"
                if hasattr(self, 'status_label'):
                    self.status_label.text = f"Status: {self.status}"

                # 發送 service 成功回應
                if self.ros_bridge_client and self.ros_bridge_client.service_in_progress:
                    point_id = getattr(self, 'current_point_id', '')
                    message = f"Navigation completed successfully to point {point_id}" if point_id else "Navigation completed successfully"
                    self.ros_bridge_client.send_navigation_success_response(message)
                
                data_source = "TF" if use_tf_data else "USD"
                print(f"[AutoNav] 目標到達! 使用 {data_source} 資料")
                print(f"[AutoNav] 最終位置: ({robot_x:.3f}, {robot_y:.3f}), 目標: ({self.target_x:.3f}, {self.target_y:.3f})")
                print(f"[AutoNav] 誤差: dx={dx:.3f}, dy={dy:.3f}, dist={dist:.3f}")
                return
                
            # 導航邏輯
            if abs(yaw_diff) > angle_thres:
                # 在執行轉向前最後檢查停止請求
                if getattr(self, "_ros_stop_requested", False) or getattr(self, "_force_stop", False) or not self.nav_active:
                    print("[AutoNav] 🛑 在執行轉向前檢測到停止請求，中止操作")
                    return
                    
                if yaw_diff > 0:
                    self._change_wheel_velocity(-turn_speed, turn_speed)  # left turn
                else:
                    self._change_wheel_velocity(turn_speed, -turn_speed)  # right turn
            else:
                # 在執行前進前最後檢查停止請求
                if getattr(self, "_ros_stop_requested", False) or getattr(self, "_force_stop", False) or not self.nav_active:
                    print("[AutoNav] 🛑 在執行前進前檢測到停止請求，中止操作")
                    return
                    
                # 根據距離調整速度，離目標越近速度越慢
                if dist < 0.1:  # 非常接近時大幅減速
                    speed_factor = min(0.3, dist / 0.1)  # 最大30%速度
                    adjusted_speed = max(30, int(max_speed * speed_factor))  # 最小速度 30
                elif dist < 0.3:  # 接近時減速
                    speed_factor = min(0.6, dist / 0.3)  # 最大60%速度
                    adjusted_speed = max(50, int(max_speed * speed_factor))  # 最小速度 50
                else:
                    speed_factor = min(1.0, dist / 0.5)  # 距離小於 0.5 時開始減速
                    adjusted_speed = max(80, int(max_speed * speed_factor))  # 最小速度 80
                
                self._change_wheel_velocity(adjusted_speed, adjusted_speed)
                
        except Exception as e:
            self.status = f"Navigation Error: {e}"
            if hasattr(self, 'status_label'):
                self.status_label.text = f"Status: {self.status}"
            self._stop_all_movement()
            print(f"[AutoNav] Exception: {e}")

    def _change_wheel_velocity(self, L_velocity, R_velocity):
        """設置輪子速度，參考 ROBOTNAME_move 的邏輯"""
        # 在實際設置輪子速度前進行最後檢查（不重複執行停止邏輯）
        if getattr(self, "_ros_stop_requested", False) or getattr(self, "_force_stop", False) or not self.nav_active:
            # 只設置速度為 0，不執行完整的停止邏輯（避免重複）
            L_velocity = 0
            R_velocity = 0
            
        stage = omni.usd.get_context().get_stage()
        try:
            wheel_L_01_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{self.robot_prim_path}/joints/wheel_L_01"), "angular")
            wheel_R_01_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{self.robot_prim_path}/joints/wheel_R_01"), "angular")
            if not wheel_L_01_drive or not wheel_R_01_drive:
                print(f"[AutoNav][Error] Wheel joint not found!")
                return
            wheel_L_01_drive.GetTargetVelocityAttr().Set(L_velocity)
            wheel_R_01_drive.GetTargetVelocityAttr().Set(R_velocity)
        except Exception as e:
            print(f"[AutoNav][Error] Failed to set wheel velocity: {e}")
    
    def _stop_all_movement(self):
        """完全停止所有移動，參考 ROBOTNAME_move 的邏輯"""
        try:
            # 強制設置輪子速度為 0，並重複設置以確保生效
            for _ in range(3):
                self._change_wheel_velocity(0, 0)
            # 額外的停止邏輯：明確設置 drive target velocity 為 0 並設高阻尼
            stage = omni.usd.get_context().get_stage()
            wheel_L_01_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{self.robot_prim_path}/joints/wheel_L_01"), "angular")
            wheel_R_01_drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{self.robot_prim_path}/joints/wheel_R_01"), "angular")
            if wheel_L_01_drive and wheel_R_01_drive:
                wheel_L_01_drive.GetTargetVelocityAttr().Set(0.0)
                wheel_R_01_drive.GetTargetVelocityAttr().Set(0.0)
                wheel_L_01_drive.GetDampingAttr().Set(100.0)
                wheel_R_01_drive.GetDampingAttr().Set(100.0)
            print("[AutoNav] 所有移動已停止 - 增強版")
        except Exception as e:
            print(f"[AutoNav][Error] 停止移動失敗: {e}")
