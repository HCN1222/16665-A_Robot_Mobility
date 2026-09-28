# CMU Robot Mobility Lab 4：完整實作教學

**需求重述：**完整閱讀指定專案的所有子資料夾與檔案，依作業要求解釋實作順序、各項功能的正確解法及驗證方法。

**理解確認：**目標是讓你能按照教學完成 Pure Pursuit、RRT 局部避障與實體車整合，並知道每一步為什麼這樣寫、怎麼判斷是否正確。

閱讀日期：2026-09-25。專案：<https://github.com/cmu-robot-mobility/robot-mobility-lab4>。

本教學對應 commit `93f2718a4f48cd422a325625ac02f594066f4157`。已遞迴讀取全部 47 個版本控制內檔案：41 個文字／空白檔及 6 個圖像檔（含 PGM 地圖）；附錄提供逐檔清單。後續 master 若更新，請以新版本要求為準。

這是一份依原始碼整理的解題教學；本文新增的函式與參數是建議實作，不是助教官方解答。環境沒有 ROS 2，因此沒有宣稱已完成 colcon、模擬器或實體車驗證。文末列出真正需要在你的環境完成的測試。

## 1. 最終要完成什麼

| 部分 | 輸入 | 你要做的事 | 輸出與驗收 |
|---|---|---|---|
| Part A | 有順序的 waypoint CSV、車輛 pose | Pure Pursuit：選目標、座標轉換、曲率轉轉角 | 車輛完成一圈；RViz 顯示所有點和當前目標 |
| Part B | 同一份 CSV、pose、LaserScan | 建局部 occupancy grid；有障礙才 RRT；用 Pure Pursuit 追新路徑 | 模擬與實車避開至少一個障礙；顯示樹、路徑、目標與 grid |
| Part C | Part B 的 RRT | RRT* 的選 parent、rewire、cost 更新 | 選做加分 +5 |
| 已提供 | AIMS 地圖、particle filter | 啟動並確認定位；無須重新實作 MCL | `/pf/viz/inferred_pose` |

README 的基本配分是 A 40 分、B 40 分，共 80 分，另有 C +5；不自行推測其餘 20 分。C++、Python 二選一，不必兩套都完成。

**建議先用 Python 完成 A+B，實際量測規劃時間後再決定是否需要 C++。** 下文以 Python 的程式位置為主，數學和資料結構同樣適用 C++。

## 2. 資料夾的用途與修改範圍

| 位置 | 內容／狀態 | 實作時的處理 |
|---|---|---|
| `README.md`、`SUBMISSION.md` | 作業、評分、影片欄位 | 讀完要求；最後填影片 |
| `pure_pursuit/scripts/pure_pursuit_node.py` | 幾乎空白的控制器骨架 | Part A 主程式 |
| `pure_pursuit/scripts/waypoint_logger.py` | 可記錄模擬 odometry；CSV 有 `x,y` 標題 | 新增實體車 pose 適配 |
| `pure_pursuit/src/pure_pursuit_node.cpp` | C++ 替代骨架 | 選 Python 時不必實作；仍注意 CMake 會建它 |
| `pure_pursuit/waypoints/` | 只有 `.gitkeep` | 放你錄的模擬與硬體 CSV |
| `motion_planning/scripts/rrt_node.py` | RRT 骨架，還有啟動錯誤 | Part B 主程式 |
| `motion_planning/include/rrt/rrt.h` | C++ 節點／函式宣告 | C++ 路線才修改 |
| `motion_planning/src/rrt.cpp`、`rrt_node.cpp` | C++ 演算法骨架與入口 | C++ 路線才修改 |
| 兩個 package 的 CMake、package.xml | 真正使用的 ament_cmake 建置 | 補依賴、資料安裝設定 |
| 兩個 package 的 setup.py / setup.cfg | 殘留 Python 建置設定 | 不要誤以為目前是純 ament_python |
| `particle_filter/particle_filter/` | MCL、RangeLibc 呼叫、座標工具 | 了解介面與 frame，不重寫演算法 |
| `particle_filter/config/localize.yaml` | AIMS、3000 粒子、rmgpu | 用提供的實車設定 |
| `particle_filter/launch/localize_launch.py` | PF + map server + lifecycle manager | 實體車定位入口 |
| `particle_filter/maps/` | AIMS、Levine 各一組 YAML + PGM | AIMS 用於實車；Levine 用於模擬 |
| `particle_filter/rviz/pf.rviz` | 舊式 RViz 設定 | 建議在 RViz2 重建顯示並另存 |
| `particle_filter/test/` | copyright、flake8、PEP257 | 不是控制器或規劃演算法測試 |
| `imgs/` | RRT 偽碼、RRT/RRT* 比較、grid 示意 | 理解概念；不必做多層 costmap |

目前沒有已錄好的 waypoint CSV、沒有 Pure Pursuit/RRT 的 launch 檔，也沒有註解所說的 `rrt_params.yaml`。需要自行建立，或先用 `--ros-args -p ...` 傳參數。

## 3. 正確的實作順序

| 順序 | 任務 | 完成才進下一步的標準 |
|---|---|---|
| 0 | 修建置／啟動骨架、確認 ROS topic | 節點可啟動；能收到 pose |
| 1 | 記錄 Levine waypoints | CSV 有標題，路徑方向一致，繞完整一圈 |
| 2 | 寫 CSV loader 和 RViz markers | 點正確疊在道路上，未穿牆 |
| 3 | 寫座標轉換、Pure Pursuit | 左目標左轉，右目標右轉，直線直走 |
| 4 | 低速調 L 與速度 | 模擬完整一圈，保存 A1 影片 |
| 5 | 啟動 PF、錄 AIMS 路徑 | scan 對齊地圖、定位穩定、map frame CSV |
| 6 | Part A 上實車 | 完成 A3，包含實車與 RViz |
| 7 | 建 local occupancy grid | 障礙位置、膨脹半徑、frame 正確 |
| 8 | 單獨驗證 collision checking | 穿牆線段被拒絕、超界被拒絕 |
| 9 | 寫 RRT 基本函式與主迴圈 | 固定車位也能畫出有效繞障路線 |
| 10 | 整合 RRT 路徑與 Pure Pursuit | 模擬先直線，再繞障，保存 B2 |
| 11 | Part B 上實車 | AIMS 至少一個障礙，保存 B3 |
| 12 | 有時間才 RRT* | rewire 後 parent、cost、子孫一致 |
| 13 | 打包、影片與重建檢查 | 兩個 package、全部 CSV、SUBMISSION.md |

先驗證 grid 再寫 RRT 是關鍵。錯的座標或 collision checker，無法靠增加取樣數修好。

## 4. 開始前必須修正／理解的骨架問題

### 4.1 Python RRT 的 `Node` 名稱衝突

目前先 `from rclpy.node import Node`，後面又 `class Node(object)`；所以 `class RRT(Node)` 繼承的是樹節點，不是 ROS 節點。呼叫 `self.create_subscription` 就會出錯。此外也沒有呼叫 ROS 的 `super().__init__`。

正確結構是把樹節點改名 `TreeNode`，讓 `RRT` 繼承 `ROSNode`，並初始化 node name。以下是結構修正的 unified diff；原檔其他 TODO 仍需依後文填完。

```diff
--- a/motion_planning/scripts/rrt_node.py
+++ b/motion_planning/scripts/rrt_node.py
@@ -1,3 +1,4 @@
+#!/usr/bin/env python3
 """
 This file contains the class definition for tree nodes and RRT
 Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
@@ -7,7 +8,7 @@
 import math
 
 import rclpy
-from rclpy.node import Node
+from rclpy.node import Node as ROSNode
 from sensor_msgs.msg import LaserScan
 from geometry_msgs.msg import PoseStamped
 from geometry_msgs.msg import PointStamped
@@ -21,7 +22,7 @@
 
 # class def for tree nodes
 # It's up to you if you want to use this
-class Node(object):
+class TreeNode(object):
     def __init__(self):
         self.x = None
         self.y = None
@@ -30,8 +31,9 @@
         self.is_root = False
 
 # class def for RRT
-class RRT(Node):
+class RRT(ROSNode):
     def __init__(self):
+        super().__init__('rrt_node')
         # topics, not saved as attributes
         # TODO: grab topics from param file, you'll need to change the yaml file
         pose_topic = "ego_racecar/odom"
```

這是上述修正概念的完整可啟動最小範例，並非完成避障的提交程式；正式 RRT 應保留原檔函式並按第 10–12 節實作。

```python
#!/usr/bin/env python3
from dataclasses import dataclass
import rclpy
from rclpy.node import Node as ROSNode
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

@dataclass
class TreeNode:
    x: float
    y: float
    parent: int = -1
    cost: float = 0.0
    is_root: bool = False

class RRT(ROSNode):
    def __init__(self):
        super().__init__('rrt_node')
        self.latest_pose = None
        self.latest_scan = None
        self.pose_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

    def pose_callback(self, msg):
        self.latest_pose = msg

    def scan_callback(self, msg):
        self.latest_scan = msg

def main(args=None):
    rclpy.init(args=args)
    node = RRT()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.2 CMake 與 setup.py 不能混著猜

兩個主要 package 的 `package.xml` 都宣告 `ament_cmake`，CMake 用 `install(PROGRAMS ...)` 安裝 Python。因此 Python 執行名稱是：

```bash
ros2 run pure_pursuit pure_pursuit_node.py
ros2 run pure_pursuit waypoint_logger.py
ros2 run motion_planning rrt_node.py
```

沒有 `.py` 的 `pure_pursuit_node`、`rrt_node` 是 CMake 產出的 C++ 執行檔。原本 setup.py 的 entry point 指向不存在的 Python 模組，不是當前應使用的入口。

RRT Python 原檔沒有 shebang，應加 `#!/usr/bin/env python3`。兩個 package 都在 CMake 中使用 `nav_msgs`，但 package.xml 未宣告，應補 `<depend>nav_msgs</depend>`。使用 NumPy 的程式應宣告 `<exec_depend>python3-numpy</exec_depend>`；若採用 SciPy inflation，再加相應依賴。C++ 若直接使用 tf2 headers，建議明確宣告／link tf2，不只依賴 tf2_ros 的傳遞依賴。

CSV 要能從 installed share 找到，可在 `ament_package()` 前增加 `install(DIRECTORY waypoints DESTINATION share/${PROJECT_NAME})`。開發時也可直接傳 CSV 絕對路徑；不依賴執行時的工作目錄。

### 4.3 建置命令

以下假設工作區叫 `~/sim_ws`，已有先前作業使用的 ROS 2 與模擬器；路徑請對應你的環境。沒有新增一個「本 repo 已提供的 simulator launch」：本 repo 並不含模擬器。

```bash
mkdir -p ~/sim_ws/src
cd ~/sim_ws/src
git clone https://github.com/cmu-robot-mobility/robot-mobility-lab4.git
cd ~/sim_ws
rosdep install --from-paths src/robot-mobility-lab4/pure_pursuit src/robot-mobility-lab4/motion_planning --ignore-src -r -y
colcon build --symlink-install --packages-select pure_pursuit motion_planning
source install/setup.bash
ros2 pkg executables pure_pursuit
ros2 pkg executables motion_planning
```

先 source 你已安裝 ROS 發行版的 `setup.bash` 再執行。Python 路線仍會編譯 CMake 裡的 C++ target；如果某 ROS 發行版對 Pure Pursuit C++ 的 `PoseStamped::ConstPtr` 報錯，可改成 ROS 2 慣用 `ConstSharedPtr`，或在確定只採 Python 時移除該未使用 target 的建立／安裝。不要只因 Python 語法成功便宣稱 package 可編譯。

## 5. 先統一 topic、frame 和車輛參考點

### 5.1 模擬與實車切換

| 用途 | 模擬 | 實車 |
|---|---|---|
| 地圖 | Levine／前一作業的 levine_obs | AIMS |
| pose topic | `/ego_racecar/odom` | `/pf/viz/inferred_pose` |
| message type | `nav_msgs/Odometry` | `geometry_msgs/PoseStamped` |
| 取得 Pose | `msg.pose.pose` | `msg.pose` |
| 全域路線 | `levine.csv` | `aims.csv` |
| 掃描 | 確認實際 `/scan` | 確認實際 `/scan` |
| drive | 確認模擬器訂閱的 drive topic | 確認車上 mux 的控制輸入 |

建議參數：`pose_source`、`pose_topic`、`drive_topic`、`waypoint_file`、`lookahead`、`wheelbase`、`max_steer`、`speed`、`closed_loop`。這些是你要新增的介面，不是 skeleton 已經支援的參數。

對兩種 message 各寫一個 adapter，把抽出的 Pose 傳入同一個 `control_step(pose, stamp)`。ROS remapping 只能改 topic 名稱，不能把 Odometry 轉成 PoseStamped。

### 5.2 `map` 不等於 `odom`，位置也不等於參考點

CSV、車輛 pose、global goal 必須在相同的全域 frame。即使模擬 pose topic 名稱有 `odom`，也要檢查 `header.frame_id` 和模擬器定義；若其內容實際不是 map 座標，透過 TF 做真正轉換，不能只改 frame 字串。

本 repo 的 PF 把估計 pose 同時用在 `/pf/viz/inferred_pose` 和 `map -> laser` 的 TF，**沒有在發布 pose 時補償 LiDAR 到後軸的偏移**。Pure Pursuit 理論上的控制原點是後軸中心，因此接車前應確認現有 TF、車輛幾何與定位參考點。

以齊次轉換記號表示：已知 `T_map_laser` 和靜態 `T_rear_laser`，則：

\[
T_{map,rear}=T_{map,laser}(T_{rear,laser})^{-1}.
\]

不要把這個乘法順序反過來。建議錄 waypoint 和追蹤時都使用轉換後的後軸 pose。若按課堂近似直接使用 PF pose，應明確承認忽略偏移，不能把偏移誤差全部靠調 L 補掉。

另外 PF 寫的是 `/map`、`/laser` 等帶前導斜線的 frame 字串。ROS2/TF 整合若報 frame 錯誤，需統一成實際 TF 使用的名稱。對同一個已確認 frame 的前導斜線清理是命名相容修正，不是 `odom -> map` 的座標轉換。

### 5.3 先觀察資料

```bash
ros2 topic list
ros2 topic info /ego_racecar/odom -v
ros2 topic echo /ego_racecar/odom --once
ros2 topic info /scan -v
ros2 topic hz /scan
ros2 topic info /drive -v
```

`/drive` 是常見例子，repo 沒有替你保證車上一定使用它。以真正接到 actuator/mux 的 topic 為準。同一時間只允許一個自主控制器控制該入口；Part B 不要讓 A 與 B 同時各發一份轉向命令。

`LaserScan` 訂閱可使用 `qos_profile_sensor_data`。若 publisher 是 BEST_EFFORT，RELIABLE subscriber 可能收不到。對 pose 和 map 也檢查實際 QoS；一次發布的靜態 map／markers 要考慮 transient-local，或定期重發。

## 6. Waypoints：來源、記錄與清理

### 6.1 你需要自己建立路線

repo 不含任何 CSV。作業允許你手動駕駛錄軌跡，或在地圖選關鍵點再插值。**最直接是低速手動開一圈，記錄行駛路徑。** 不必先寫全域路徑規劃器。

模擬器啟動後：

```bash
ros2 run pure_pursuit waypoint_logger.py --ros-args \
  -p output_file:=$HOME/sim_ws/src/robot-mobility-lab4/pure_pursuit/waypoints/levine.csv \
  -p min_distance:=0.1
```

`min_distance=0.1` 表示距離上個保存點至少 10 cm 才新增，不是每 0.1 秒記錄。原 logger 每次新增會重寫整個 CSV，小型課堂路線可以用。

CSV 第一列是 `x,y`。使用 `np.loadtxt(..., delimiter=',', skiprows=1, usecols=(0,1), ndmin=2)` 或 CSV DictReader；不跳過標題會讀取失敗。載入後檢查 NaN、Inf、點數過少、重複點和不合理的大跳躍。

原 logger 預設路徑由 `__file__` 往上找 package.xml，在 installed 目錄可能找不到而退回目前工作目錄。使用上述明確路徑可避免「錄了但找不到 CSV」。重新以同一檔名執行 logger，第一個點就會覆寫舊 CSV，記錄新路線請用新名字。

### 6.2 實體車的記錄步驟

1. 啟動課堂提供的硬體 bringup，確認 `/scan`、`/odom` 在發布。
2. `ros2 launch particle_filter localize_launch.py`。
3. RViz2 Fixed Frame 設 `map`，加入 `/map`、scan、PF particles。
4. 用 **2D Pose Estimate** 指出大致位置與朝向，慢速移動確認 scan 對齊牆面。
5. 將 logger 改為 PoseStamped 訂閱 `/pf/viz/inferred_pose`，位置欄位改成 `msg.pose.position`；若採後軸座標，先做第 5 節的參考點轉換。
6. 手動開一圈，輸出 `aims.csv`。**不要用 `/odom` 的累積里程路徑當作 map 路線。**
7. 在 RViz 顯示 CSV，檢查每段都位於走道內且遠離櫃子。

PF 的 README 說不需修改演算法。GPU RangeLibc 是啟動依賴，不是 Part A/B 解題內容；車上可能已安裝，先測試 `python3 -c 'import range_libc'`。未安裝時依 PF README 在 Jetson 建置 CUDA wrapper。模擬階段用 ground-truth pose，不必為 Pure Pursuit 先安裝 GPU PF。

PF 的 pose 發布位於 `visualize()`，因此 `viz=0` 會讓指定的 `/pf/viz/inferred_pose` 不發布；保留 `viz=1`。它也只在有訂閱者時發 pose。定位更新由 odometry callback 觸發，所以 scan 有資料但 odom 沒資料也不會正常更新。

### 6.3 路線品質與閉環

- 刪除重複的相鄰點；避免停車抖動產生一堆點。
- 維持錄製的順序，不能按 x 或 y 排序。
- `closed_loop=True` 只用在真正閉合的路線；最後到第一點那一段也必須可行。
- 不要只因資料格式是循環索引，就把不相連的起終點強行接起來。
- 平滑後重新檢查障礙；樣條曲線可能在轉角往內切，穿過櫃子。
- 模擬 Levine 與實車 AIMS 是不同地圖，兩份 CSV 不能互換。

若從圖像像素手工選點，不能把 pixel 當公尺。AIMS resolution 是 0.05 m，origin 是 `[-31.1,-17.5,0]`；圖像 v 軸朝下，要先轉成 map grid 的朝上 y，再乘解析度及套用 origin。用 cell center 時需考慮半格偏移。通常直接在 RViz 讀取 map 座標比人工換算可靠。

## 7. Part A：Pure Pursuit 每一步的正確解法

### 7.1 選目標：按路線進度往前找

不要單純對全部 waypoints 找「離車距離最接近 L」的點。轉角、平行走道或繞圈處可能挑到後方／另一段路。

建議流程：

1. 啟動時找到最近且方向合理的路線段，把車投影到該段。
2. 保存目前進度（segment index + segment 上比例，或累積弧長 s）。
3. 下一步只在上次位置附近的前向窗口搜尋，允許少量回看修正定位噪音。
4. 從投影位置沿路線前進，找 lookahead 目標。
5. 閉環路線允許跨尾端；RRT 的短路徑是開放路線，不循環。
6. 找不到前方合法目標時停車／重新取得路線，不能把後方點硬塞進控制器。

兩個合理選法：

- **圓交點法：**從目前路線段往前，找以車為中心、半徑 L 的圓與 polyline 的第一個有效前向交點。此時實際目標距離就是 L。
- **弧長法：**沿路線再走 L 公尺後插值得目標。這個 L 是沿路長度，不一定等於車到目標的直線距離，因此曲率分母應使用實際距離平方。

圓與一段 `a + t(b-a)` 的交點可解：令 `d=b-a`、`f=a-car`，

\[
A=d\cdot d,\quad B=2f\cdot d,\quad C=f\cdot f-L^2,
\]
\[
t=\frac{-B\pm\sqrt{B^2-4AC}}{2A}.
\]

只收 `0≤t≤1`、位於當前路線進度之後、車身座標 x>0 的候選，按路線順序選第一個。忽略零長度段，負判別式表示沒有交点。車離路線太遠時可能無交點，需重新接近／停車，不要假設永遠存在。

### 7.2 Quaternion 轉 yaw

四元數為 `(qx,qy,qz,qw)`，先確認有效並正規化，然後：

\[
\psi=\operatorname{atan2}\left(2(q_wq_z+q_xq_y),1-2(q_y^2+q_z^2)\right).
\]

`orientation.z` 本身不是 yaw。

### 7.3 全域目標轉車身座標

車在 `(xc,yc)`、朝向 ψ，目標 `(xg,yg)`：

\[
\Delta x=x_g-x_c,\qquad \Delta y=y_g-y_c,
\]
\[
x_b=\cos\psi\Delta x+\sin\psi\Delta y,
\]
\[
y_b=-\sin\psi\Delta x+\cos\psi\Delta y.
\]

這是 `R(-ψ)`，不是 `R(ψ)`。採 x 朝前、y 朝左；左方目標 yb>0。

### 7.4 有符號曲率轉 steering angle

\[
d^2=x_b^2+y_b^2,\qquad \kappa=\frac{2y_b}{d^2},
\]
\[
\delta=\arctan(\ell\kappa),\qquad
\delta_{cmd}=\operatorname{clip}(\delta,-\delta_{max},\delta_{max}).
\]

其中 ℓ 是 **wheelbase 軸距**，不是 lookahead。README 的 `2|y|/L²` 寫的是曲率大小；實作必須保留左右符號。也不能把 κ 直接當 rad 的 steering angle；曲率單位是 1/m。

當目標確實在 lookahead 圓上，`d²=L²`；否則使用實際 `d²`。距離接近 0 時不要除；無效目標令速度為 0。

```python
import math

def pure_pursuit_angle(goal_body, wheelbase, max_steer):
    x, y = map(float, goal_body)
    if not all(math.isfinite(v) for v in (x, y, wheelbase, max_steer)):
        raise ValueError('non-finite input')
    if wheelbase <= 0.0 or max_steer <= 0.0:
        raise ValueError('invalid vehicle geometry')
    d2 = x*x + y*y
    if x <= 0.0 or d2 < 1e-8:
        return None  # caller must command stop
    curvature = 2.0*y/d2
    steering = math.atan(wheelbase*curvature)
    return max(-max_steer, min(max_steer, steering))
```

### 7.5 發布控制與速度

建立 `AckermannDriveStamped`，時間戳使用 node clock，填入 `drive.steering_angle` 與 `drive.speed`。header 若填 frame，必須是你實際採用的車身控制 frame；填 header 不會替你轉换數值。

先用固定低速調 L，再加入依曲率降速：

\[
v_{curve}=\sqrt{a_{lat,max}/\max(|\kappa|,\epsilon)}.
\]

再取直線速度上限與這個曲率速度的較小值，必要時加速度限幅。遇到無路徑或資料失效仍直接停止，不要用 `min_speed` 把停止命令抬高。

另設 timer 檢查 pose/scan 新鮮度。若只在 pose callback 裡停止，pose 中斷時 callback 根本不會進來。需搭配控制器 watchdog 與硬體命令逾時；單執行緒規劃太久也會拖住 timer，所以 RRT 必須有時間上限。

### 7.6 RViz 必備

| 顯示物件 | 建議 message | 重點 |
|---|---|---|
| 全部 waypoints | Marker POINTS 或 LINE_STRIP | map frame；LINE_STRIP 依順序 |
| 正在追的目標 | Marker SPHERE | 換顏色、稍大；每次 callback 更新 |
| 車的 pose | Pose / TF | 與 waypoint 同一張地圖 |

Marker 設 `pose.orientation.w=1`、合理 scale、`color.a=1`，保持 namespace/id 穩定以覆寫上一筆。全部點若只發一次，用 transient-local publisher 並讓 RViz QoS 匹配，或以低頻定期重發。不是只在程式啟動时 publish 一次就假設晚開的 RViz 看得到。

## 8. Part A 的測試與調參

先做三個數學測試：

1. 車 `(0,0,0)`，目標 `(2,0)`：转角 0。
2. 同車姿，目標 `(2,1)`：轉角正；目標 `(2,-1)`：轉角負且大小相同。
3. 車 `(1,2,π/2)`，世界目標 `(1,4)`：車身目標 `(2,0)`，應直走。

再在模擬先直線、再單一轉角、最後完整一圈。觀察目標 marker 是否在前方且沿路順序移動。

| 症狀 | 優先檢查 |
|---|---|
| 全部轉向都向同側 | 是否錯用 abs(y) |
| 車转過 90° 後失控 | 旋轉矩陣正負、yaw 抽取 |
| 直線左右搖 | L 太小、點不均匀、pose 噪聲、速度過高 |
| 彎道切角撞牆 | L 太大、速度太高、waypoints 過近內牆 |
| 快到一圈終點突然迴轉 | closed-loop 接縫、進度索引 |
| 控制輸出正常但車不動 | drive topic／mux／不同控制器搶控制 |

可嘗試的低速起點：L 約 0.6–1.0 m、速度約 0.5 m/s、記錄間隔 0.1 m。這些不是作業指定值，也不保證適合你的車或走道。軸距、最大轉角必須來自實際車輛設定／量測，不把常見的 0.33 m 或 0.4189 rad 當 repo 已確認常數。

## 9. Part B：先建立 occupancy grid

### 9.1 建議架構

採 2D workspace RRT，樹節點只有 `(x,y)`；global waypoints 留在 map frame，每次規劃轉到同一個局部座標。這符合 README 允許的做法。

Grid 可例如取車前 5 m、左右各 2 m，後方保留一小段以覆蓋車身，解析度 0.05 m。實际大小需依走道與車尺寸調整。RRT sampling 範圍可限前方，但 occupancy 的範圍仍要能表達車身和感測器原點。

**一個規劃週期使用同一份 pose/scan/grid snapshot。** 若 scan 屬於 t0 車身 frame，不能把它直接當 t1 車身 frame。可在 scan 時刻保存 pose、把障礙和規劃结果轉到 map 保存，再於控制時轉回當前車身；或使用帶正確時間戳的 TF／同步機制。重用舊路徑時尤其不能把舊 local 點原封不動當現在 local 點。

### 9.2 LaserScan 轉障礙點

第 i 條 ray：

\[
\theta_i=angle_{min}+i\,angle_{increment},
\]
\[
p_i^{laser}=(r_i\cos\theta_i,r_i\sin\theta_i).
\]

再套用實際 `T_body_laser` 轉到 grid frame。不要假設雷射在後軸原點。

有效 finite 回波要符合 range 範圍；NaN、過小值不當有效障礙。`+inf` 或最大距離可能表示無回波，也可能有驅動特定語義，要先確認；一般不把它們全部標在遠方一圈當障礙。確認為「無障礙到最大距離」的 ray 才能清除到感測上限。

### 9.3 free、occupied、unknown

推荐內部保留三種狀態：已知 free、occupied、unknown。每次新 snapshot 從 unknown 開始，對有效 ray：

1. 射線原點到回波之前的格子標 free。
2. 真正回波終點標 occupied。
3. 被擋住的後方仍 unknown。
4. 合併所有 rays 時 **occupied 優先於 free**；避免後處理的 ray 把障礙擦掉。
5. 若融合靜態 map，明確定義衝突規則；已知牆壁不能隨意被一條 ray 清掉。

如果全部初始化 free 然後只點亮回波，你其實是假設「沒有觀察到的地方都能走」，會穿過遮蔽區。保守做法是 unknown 不可通行；若此做法令 sparse rays 間到處 unknown，需增加合理的可視區 rasterization／融合靜態圖，而不是偷偷把全部 unknown 設成 free。

每次重新建立 local grid 比不清除地累積障礙簡單。若用持久地圖則需要座標轉換與清除／衰減，不然會留下過期障礙。

### 9.4 座標轉格子

grid origin `(xmin,ymin)`，解析度 r：

\[
c=\lfloor(x-x_{min})/r\rfloor,
\quad q=\lfloor(y-y_{min})/r\rfloor.
\]

以 `grid[row=q, col=c]` 存取。展平成 `OccupancyGrid.data` 時，`index=row*width+col`。檢查 `0≤row<height` 和 `0≤col<width`；超界視為不可走。使用 floor，不要對負數用 int 截斷。

ROS 顯示採 `-1` unknown、`0` free、`100` occupied，讓 RViz 清楚呈現。不要直接 publish 內部二元的 1 並以為會顯示為 100% occupied。`origin.orientation.w=1`；frame 和 stamp 需對應該 snapshot。

### 9.5 Inflation：不能把車當沒有大小的點

2D 點路徑要避開完整車身，需要擴張 obstacles。保守的圓形 footprint 半徑取「從規劃參考點到車身最遠角落的距離 + margin」。參考點在後軸時，通常不是單純車寬一半，也不一定是車中心的半對角線。

若將 occupied/unknown 都視為 blocked，可以對 blocked mask 做圓形 dilation，半徑格數至少 `ceil(R/r)`，再加格子離散誤差餘裕。Grid 外邊界也視為 blocked，避免車中心在界內但車身越界。若分開保存 unknown，collision checker 同樣應拒絕 footprint 會踏入 unknown 的位置。

保守圓形模型可能使窄走道全部封死。這時應改更準確的矩形 footprint／帶 heading 的 collision checking，或合理擴大已觀測區域，不能只把半徑調得比車還小。

### 9.6 先獨立驗證 grid

車不動時看 RViz：前方障礙在前，左側障礙在左；移動障礙後下一張圖更新；障礙厚度包含車寬／足跡；轉車後座標依正確 frame 更新。這幾項沒過，不開始動車測 RRT。

## 10. RRT 每個函式要怎麼寫

定義樹節點欄位：`x,y,parent,cost,is_root`。`parent` 使用 tree list 的整數索引；root 的 parent=-1。一般 RRT 的 cost 可不參與決策，但保留它方便之後做 RRT*。

### 10.1 `sample()`

在 bounded ROI 內均勻採樣，只接受 inflated grid 中可通行的點。以約 5–15% 的機率直接取 goal（goal bias），但 goal 本身也必須 free。

**拒絕採樣也要限制次數。** 全部 blocked 時不能在 `while True` 卡死；達上限回 None 讓規劃器結束／重試。

### 10.2 `nearest(tree, sampled_point)`

\[
i^*=\arg\min_i[(x_i-x_s)^2+(y_i-y_s)^2].
\]

回傳索引，不是節點物件。小型局部樹先線性搜尋即可；不需先優化 KD-tree。

### 10.3 `steer(nearest_node, sampled_point)`

這裡的 steer 是「擴展樹」而不是車輪轉角。

\[
v=q_s-q_n,\ d=\|v\|,
\quad q_{new}=q_n+\min(\eta,d)\frac{v}{d}.
\]

η 為單次最大 extension；d≈0 回 None。若樣本距離小於 η，new 就等於樣本，不可越過樣本。新增樹節點時另設定 parent=nearest index。

### 10.4 `check_collision(a,b)`

**原骨架的回傳語意是 True=有碰撞，False=無碰撞。** 圖中的 `ObstacleFree` 則相反，不能直接照圖把 if 的意思寫反。

檢查的是整段邊，不只是終點。嚴格的格網檢查使用 supercover traversal，把線段接觸／穿過的全部 cells 都列入，包括跨格角落時的相鄰格。一般只抽樣有限點，即使間距小於半格，也可能漏掉擦角；可作保守膨脹下的近似，但不要說那是完整幾何保證。

對教學小型 grid，最容易驗證的正確基準版本是「線段對每個 blocked cell 的矩形做相交檢查」。速度慢一些，但無需自己先寫 DDA 的 corner case。附錄給完整 Python 參考核心。之後可換成等價的 supercover DDA 提速。

### 10.5 `is_goal(new, gx, gy)`

\[
\|q_{new}-q_g\|\le\epsilon_g.
\]

只有距離近不代表成功連到 goal，還必須檢查 `new -> goal`。中間可能有一堵牆。推薦通過後新增真正 goal 節點，parent 指向 new，回溯到精確 goal。

### 10.6 `find_path(tree, latest_added_node)`

從 goal index 開始，不斷把該節點加入清單，再走到 parent；到 -1 停止，最後 reverse。結果順序應為 root→goal，而不是 goal→root。

不建議用 `(x,y)` 重新搜尋節點索引，可能有重複點。可將介面改成接收 `goal_index`；原 skeleton 明確允許自訂實作。除錯時加上 parent 範圍與 cycle 檢查。

### 10.7 一次 RRT 主迴圈

```text
固定這次的 pose、scan、inflated_grid
選定前方且在 grid 內的 free goal
如果資料過期、root/goal 無效：發布停止
如果正常路線與即將執行的控制軌跡均安全：追原 waypoints
否則：
    tree = [root]
    在時間預算與迭代數上限內：
        sampled = sample()
        找不到 free sample -> 結束此次規劃
        i = nearest(tree, sampled)
        new = steer(tree[i], sampled)
        new 無效 -> continue
        check_collision(tree[i], new) -> continue
        new.parent = i
        加入 tree
        如果 is_goal(new) 且 new -> goal 無碰撞：
            加入 goal
            path = find_path(tree, goal_index)
            完成
    如果成功：檢查／整理 path，交給 Pure Pursuit
    如果失敗：停止，等待下次感測和重新規劃
```

給定初始 tree、snapshot 與 goal，每次 planning tick 重新建局部樹，符合這次作業設定。不要不轉座標就保留上一個車身 frame 的 tree。

## 11. Global goal、RRT path、Pure Pursuit target 是三件事

| 名稱 | 作用 | 典型距離關係 |
|---|---|---|
| Global route | Part A 錄的完整路線 | 整圈 |
| RRT planning goal | 希望繞障後接回的路線前方點 | 比控制目標遠 |
| Pure Pursuit tracking target | 當下用來算轉角的路徑前方點 | lookahead 較近 |

例如規劃看前方 2.0 m、控制看前方 0.7 m，可作初始概念，但以可視區、轉角與車速為準。不要強制所有路況只能用這兩個數字。

規劃 goal 要沿原路線挑，在 grid 內、前方且不在膨脹障礙內。若落在障礙中，沿路線找其他有效候選或縮短 horizon；不能把 occupied cell 改成 free。若沒有可達目標就停止，不能一直 sample 等奇蹟。

README 說直達 goal 沒擋住就可直接 Pure Pursuit。工程上還要注意：**車到遠處 goal 的直線**與**沿彎曲 waypoints 的路段**、以及**Pure Pursuit 真正走的圓弧**不是同一條線。避免直線沒擋卻在追蹤過程撞牆，至少檢查要追的路線段與短期控制 rollout。

## 12. 如何讓車真的走得動 RRT 路徑

### 12.1 補點、縮短和轉彎

RRT 常產生不等距折線。先可做 collision-checked shortcut：若 path[i] 能安全直達 path[j]，才刪除中間點。再以固定間距（例如 0.05–0.1 m）沿每條線段線性補點；每段長度 d，用 `ceil(d/spacing)` 個小間隔，保留終點，避免重複點。

補點只增加取樣密度，**不會把尖角變成車可行的圓弧**。任意 spline 平滑又可能穿障礙，因此平滑後重新 collision checking。

### 12.2 不要讓 Pure Pursuit 跳過避障轉折

對 RRT 的 local path 從 root 往 goal 按順序選目標；不可回去對 global CSV 選目標，否則車會直接朝障礙後面的原 waypoint 開去。

L 太長會跳過繞障彎折而切入障礙。可縮小避障時 lookahead、降低速度，並驗證即將執行的運動。不能只因 RRT 的折線避障就宣稱車一定不撞。

對候選轉角 δ，用 bicycle model 做短期 rollout：

\[
\dot x=v\cos\psi,\quad\dot y=v\sin\psi,
\quad\dot\psi=(v/\ell)\tan\delta.
\]

沿 rollout 檢查膨脹 grid（或完整 footprint）；若碰撞，嘗試更短的合法目標／更低速方案並重新檢查，仍不安全就停。檢查 horizon 至少包含反應與煞停距離：

\[
d_{stop}\approx v\tau+\frac{v^2}{2a_{brake}}+margin.
\]

τ 包含感測、計算、通訊與控制延遲；a_brake 必須依實際能力估計。若格子內可觀测空間連此範圍都不足，應降低速度。

### 12.3 2D RRT 的能力邊界

2D RRT 允許任意方向的直線邊，不保證車的非完整約束。最低轉彎半徑約：

\[
R_{min}=\ell/\tan(\delta_{max}).
\]

你不能讓車原地轉彎去追一個 90° 折角。低速、較平滑路徑與 rollout 可以改善基本版本；更嚴格的版本要用 `(x,y,yaw)` 狀態與 bicycle motion primitives、Dubins 等適合前進車的 steering。這類改進可作選做 variant，但須在 SUBMISSION.md 說明理由，不能自行保證加分。

### 12.4 重規劃失敗與舊路徑

初版最容易寫對：規劃失敗就速度 0。若保留上一條路徑，必须先轉到 map 保存、截去已走段、用最新 grid 重新驗證，再檢查有效期與短期 rollout。禁止舊 local path 隨車一起錯誤漂移。

設計有限迭代數與 elapsed-time budget；例如 500 次與數十毫秒只是起點，要量測平均及高分位延遲。不是固定跑 500 次就保證可即時。

## 13. RRT 可視化與驗收

| 資料 | 建議顯示 | 注意 |
|---|---|---|
| 全域 route | LINE_STRIP | 地圖座標 |
| planning goal | SPHERE | 與 tracking target 用不同顏色 |
| 樹的全部 parent-child edges | LINE_LIST | 每條邊放兩個 points |
| 最後路徑 | `nav_msgs/Path` 或 LINE_STRIP | root→goal，stamp/frame 一致 |
| PP tracking target | SPHERE | 在目前要追的 path 上 |
| local grid | OccupancyGrid | origin/resolution/frame 正確 |

每次重規劃更新同一 Marker id，或刪掉上一輪不用的 markers，避免歷史樹堆滿 RViz。規劃失敗也清除舊成功路徑顯示，或明確標成過期，不讓影片看起来像還有有效路徑。

測試順序：

1. 無障礙直線：車沿原路線直走，不必隨機繞遠。
2. 固定車位的單一障礙：RRT 畫出繞行路徑，所有 edges 無碰撞。
3. 低速單一障礙：車真的繞過，回到原路线。
4. 窄通道：若車身不夠過，必須拒絕。
5. 完全堵住：有限時間退出並停車。
6. goal 在障礙中／grid 外：重新選有效 goal 或停車。
7. 中斷 pose 或 scan：停止發送有效前進控制，watchdog 生效。
8. 再移植到 AIMS 的實車 topic、CSV 與 TF。

影片的障礙必須真的被模擬器／LiDAR 看見。只在 RViz 畫一個紅色圓球不是障礙，除非你也將它加入感測／collision representation；作業推薦前次的 `levine_obs`，本 repo 沒有提供那份檔案。

## 14. Part C：正確的 RRT* 不只是挑短一點的 path

基本成本採路徑長度：

\[
line\_cost(a,b)=\|a-b\|,\qquad
cost(n)=cost(parent(n))+line\_cost(parent(n),n).
\]

新增 new 之後：

1. `near(tree,new)` 找半徑內鄰居。
2. **Choose parent：**比較所有能無碰撞連到 new 的候選，選 `cost(parent)+distance(parent,new)` 最小者；最近點保持為可行 fallback。
3. 把 new 加進樹並設定 cost。
4. **Rewire：**對每個能從 new 無碰撞連接的鄰居，若走 new 比原路徑便宜，改 parent。
5. 遞迴／遍歷更新該鄰居的所有子孫 cost，或每次沿 parent 重新計算真實 cost。
6. 不建立 cycle、不改 root 成其他節點的孩子。
7. 找到第一條路徑後，在剩餘預算內持續改善，維護當前最好 goal 解。

2D 常用理論形式：

\[
r_n=\min\left(\eta,\gamma(\log n/n)^{1/2}\right).
\]

γ 和 free-space 面積及理論条件有關；固定半徑的課堂實作可運作，但不能不加條件聲稱沿用所有漸近最優性保證。正式 RRT* 的保證來自指定论文的假設；局部有限樣本、每週期重建不代表每次求得全域最優。

Skeleton 的 `cost/line_cost/near` 都只是佔位；只填這三個而沒有改主迴圈的 ChooseParent/Rewire，並不構成 RRT*。

## 15. 實車定位與原始碼內的其他細節

這些是檢查原始碼得到的整合注意事項，不是要你重寫提供的 PF：

- `config/localize.yaml` 選 AIMS、`range_method=rmgpu`、`rangelib_variant=2`、`viz=1`。
- map server service 是 `/map_server/map`，PF 啟動會等它；卡住先查 lifecycle/map server。
- launch 的 `map_name` 是從預設 config 先讀出的；只用 `localize_config:=別的yaml` 不一定改到 map server 的 map。此作業直接用預設 AIMS 即可。
- `pf.rviz` 裡有 `rviz/...` 的 ROS1 風格 plugin class，不能保證 RViz2 直接載入；而 setup.py 也沒有安裝 `rviz/`。可直接在 RViz2 新增顯示後另存設定。
- PF 的全域初始化程式存在，但實車仍按 README 用 2D Pose Estimate 啟動更可靠。不是數學上「未點選就絕不可能定位」。
- yaw 的線性平均／角度跳變等是提供 PF 的數值限制；若接近 ±π 出現不連續，先記錄訊息確認，不要誤判成你 PP 的公式問題。

## 16. 建議你把程式分成哪些責任

可在兩個 package 中使用模組化函式，但不要留下提交時找不到的外部私人模組。

| 功能 | 函式或責任 | 測試方式 |
|---|---|---|
| 路線讀取 | CSV loader + clean + closed-loop | 標題、NaN、重複點、接縫 |
| 姿態適配 | Odometry/PoseStamped → 統一 pose | 兩種 message 解析 |
| 全域進度 | nearest segment + continuity | 平行走道、圈尾 |
| PP 幾何 | transform + target + steering | 左右鏡像、90° 車姿 |
| 感測建圖 | scan → free/occupied/unknown | 雷射外參、無效值 |
| 車身碰撞 | inflate + segment_collision | 擦角、超界、窄縫 |
| RRT | sample/nearest/steer/path | 固定 seed、堵路失敗 |
| 控制整合 | route/RRT selection + rollout | 不切角、失敗停車 |
| ROS I/O | pub/sub、QoS、timestamps | topic info、RViz |
| 失效處理 | data age + command timeout | 停資料、超時規劃 |

共享 PP 函式時，若 motion_planning import pure_pursuit 模組，就在 motion_planning/package.xml 宣告 pure_pursuit 依賴，並用 `colcon build --packages-up-to motion_planning` 包含依賴；別用依賴工作目錄的 sys.path hack。

## 17. 繳交前檢查

作業明確要求一個 ZIP，上傳 Canvas；四個 YouTube 連結寫在 SUBMISSION.md。影片可以 unlisted。程式個人繳交；兩支硬體影片按團隊。

| 影片 | 必須看得見的內容 |
|---|---|
| A1 | 模擬 Pure Pursuit 完整一圈；所有 waypoints + 當前目標 |
| A3 | AIMS 實體車追蹤；PF；實車畫面搭配 RViz 的 map/markers |
| B2 | 模擬 RRT，至少一個真實障礙；實際繞障；規劃路徑與每步 goal |
| B3 | AIMS 實體車，至少一個障礙並成功避開 |

評分會扣的項目包括：缺視覺化、模擬只跑直線沒避障、實車片沒有避障證據、追蹤未完成一圈、RRT 撞障礙。A3 影片沒有搭配 RViz 也有扣分。

ZIP 的 top level 是 `pure_pursuit/`、`motion_planning/`、`SUBMISSION.md`；waypoints/ 內含 sim 與 hardware 所有用過的 CSV。不需要把 build/install/log 加進去。README 沒要求把提供的 PF 加進該 ZIP。

```bash
cd ~/sim_ws/src/robot-mobility-lab4
# 將下面兩個字串換成你的 Andrew ID 與組別。
lab4_andrew_id='YOUR_ANDREW_ID'
lab4_team_number='YOUR_TEAM_NUMBER'
zip -r "lab4_${lab4_andrew_id}_${lab4_team_number}.zip" \
  pure_pursuit motion_planning SUBMISSION.md \
  -x '*/__pycache__/*' '*.pyc' '*/build/*' '*/install/*' '*/log/*'
```

在乾淨工作區重建解壓後的兩個 package，確認 CSV 路徑不依賴你電腦的特定絕對路徑。把使用方式、參數與 RRT* 改進說明寫入 SUBMISSION.md，保留四個必要影片欄位。README 有一处誤拼 `SUBMSSION.md`，實際檔名是 `SUBMISSION.md`。

## 18. 參考來源與驗證邊界

- [本次檢查的固定版本](https://github.com/cmu-robot-mobility/robot-mobility-lab4/tree/93f2718a4f48cd422a325625ac02f594066f4157)：作業要求、所有程式與設定的直接依據。
- [作業 README](https://github.com/cmu-robot-mobility/robot-mobility-lab4/blob/93f2718a4f48cd422a325625ac02f594066f4157/README.md)。
- [Python RRT 原始骨架](https://github.com/cmu-robot-mobility/robot-mobility-lab4/blob/93f2718a4f48cd422a325625ac02f594066f4157/motion_planning/scripts/rrt_node.py)。
- [PF 原始碼](https://github.com/cmu-robot-mobility/robot-mobility-lab4/blob/93f2718a4f48cd422a325625ac02f594066f4157/particle_filter/particle_filter/particle_filter.py)。
- [Karaman & Frazzoli, Sampling-based Algorithms for Optimal Motion Planning](https://arxiv.org/abs/1105.1186)：作業指定 RRT/RRT* 論文。

演算法公式、邊界情況與建議架構是依上述程式及幾何推導整理。參數表是調試起點；車輛軸距、外參、控制 topic、實際 ROS 發行版和硬體性能仍必須在現場確認。沒有捏造跑圈成功、規劃頻率或硬體避障結果。


## 附錄 A：可獨立測試的幾何與 RRT 參考核心

下面是完整新檔 `lab4_core.py`，使用 Python 3 + NumPy，不依賴 ROS。它涵蓋座標轉換、PP 轉角、線段碰撞、RRT 搜尋／回溯及線性補點。不是直接可上車的完整節點：CSV 進度、感測建圖、footprint inflation、ROS I/O、時間同步、控制 rollout 必須依正文整合。

collision checker 採線段對 blocked cells 的矩形相交，偏重正確與易驗證，複雜度隨 blocked cells 成長。即時版可換 supercover DDA；提供的 time_budget 不是硬即時保證，單次運算不會被中途搶占。

程式接收的 `Grid.blocked` 必須已完成車身膨脹、unknown 禁行及邊界餘裕。若你拿未膨脹的 LiDAR 點直接傳入，就只是在替點機器人規劃。

```python
"""ROS-independent reference for Lab 4 geometry and basic RRT.
Grid must ALREADY include vehicle-footprint inflation; unknowns are blocked.
This module does not implement ROS adapters, scan mapping, or a controller.
"""
from dataclasses import dataclass
import math
import time
import numpy as np


def world_to_body(point, pose):
    x, y, yaw = pose
    dx, dy = np.asarray(point, dtype=float) - np.array([x, y])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c*dx+s*dy, -s*dx+c*dy])


def steering_to(point_body, wheelbase, max_steer):
    x, y = map(float, point_body)
    if not all(math.isfinite(v) for v in (x, y, wheelbase, max_steer)):
        raise ValueError('non-finite control input')
    if wheelbase <= 0.0 or max_steer <= 0.0:
        raise ValueError('invalid geometry')
    d2 = x*x + y*y
    if x <= 0.0 or d2 < 1e-8:
        return None
    return float(np.clip(math.atan(wheelbase*2*y/d2), -max_steer, max_steer))


def segment_hits_boxes(a, b, lower, upper):
    """Closed segment vs closed axis-aligned rectangles (slab method).
    Touching a blocked cell boundary counts as collision.
    """
    if len(lower) == 0:
        return False
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    delta = b-a
    t_enter = np.zeros(len(lower))
    t_leave = np.ones(len(lower))
    possible = np.ones(len(lower), dtype=bool)
    for axis in range(2):
        if abs(delta[axis]) < 1e-12:
            possible &= ((a[axis] >= lower[:, axis]-1e-12) &
                         (a[axis] <= upper[:, axis]+1e-12))
        else:
            t1 = (lower[:, axis]-a[axis])/delta[axis]
            t2 = (upper[:, axis]-a[axis])/delta[axis]
            t_enter = np.maximum(t_enter, np.minimum(t1, t2))
            t_leave = np.minimum(t_leave, np.maximum(t1, t2))
    return bool(np.any(possible & (t_enter <= t_leave+1e-12)))


class Grid:
    """Axis-aligned grid in one frozen local frame.
    blocked[row,col] is True for inflated obstacles AND forbidden unknowns.
    The upstream mapper must also reserve footprint clearance at boundaries.
    """
    def __init__(self, blocked, resolution, origin):
        self.blocked = np.asarray(blocked, dtype=bool).copy()
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        if (self.blocked.ndim != 2 or 0 in self.blocked.shape or
                not math.isfinite(self.resolution) or self.resolution <= 0 or
                self.origin.shape != (2,) or not np.all(np.isfinite(self.origin))):
            raise ValueError('invalid grid')
        self.height, self.width = self.blocked.shape
        self.limit = self.origin + self.resolution*np.array([self.width, self.height])
        rows, cols = np.nonzero(self.blocked)
        self.lower = self.origin + self.resolution*np.column_stack((cols, rows))
        self.upper = self.lower + self.resolution

    def inside(self, point):
        p = np.asarray(point, dtype=float)
        return bool(np.all(np.isfinite(p)) and
                    np.all(p >= self.origin) and np.all(p < self.limit))

    def check_collision(self, a, b):
        if not self.inside(a) or not self.inside(b):
            return True
        # The rectangular domain is convex: in-domain endpoints imply
        # the whole straight segment stays inside the domain.
        return segment_hits_boxes(a, b, self.lower, self.upper)

    def free(self, point):
        return not self.check_collision(point, point)


@dataclass
class TreeNode:
    x: float
    y: float
    parent: int = -1
    cost: float = 0.0

    @property
    def point(self):
        return np.array([self.x, self.y])


def nearest(tree, sampled):
    return min(range(len(tree)),
               key=lambda i: float(np.sum((tree[i].point-sampled)**2)))


def steer(node, sampled, step):
    delta = np.asarray(sampled, dtype=float)-node.point
    distance = float(np.linalg.norm(delta))
    if distance < 1e-10:
        return None
    p = node.point + min(step, distance)*delta/distance
    return TreeNode(float(p[0]), float(p[1]))


def find_path(tree, goal_index):
    path, seen = [], set()
    i = goal_index
    while i != -1:
        if i in seen or not 0 <= i < len(tree):
            raise ValueError('invalid parent chain')
        seen.add(i)
        path.append(tree[i].point)
        i = tree[i].parent
    return np.asarray(path[::-1])


def rrt_plan(grid, start, goal, *, step=0.35, goal_tolerance=0.3,
             goal_bias=0.1, max_iterations=1000, time_budget=0.05,
             seed=None):
    """Return (path_or_None, tree). No vehicle dynamic feasibility guarantee.
    time_budget bounds the sampling loop, not a hard real-time deadline.
    Collision computations already in progress are not preempted.
    """
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if (step <= 0 or goal_tolerance < 0 or not 0 <= goal_bias < 1 or
            max_iterations <= 0 or time_budget <= 0):
        raise ValueError('invalid planning parameters')
    deadline = time.monotonic()+time_budget
    if not grid.free(start) or not grid.free(goal):
        return None, []
    tree = [TreeNode(float(start[0]), float(start[1]))]
    distance = float(np.linalg.norm(goal-start))
    if distance < 1e-10:
        return np.array([start]), tree
    if not grid.check_collision(start, goal):
        tree.append(TreeNode(float(goal[0]), float(goal[1]), 0, distance))
        return find_path(tree, 1), tree
    rng = np.random.default_rng(seed)
    for _ in range(max_iterations):
        if time.monotonic() >= deadline:
            break
        sampled = None
        for _ in range(32):
            if time.monotonic() >= deadline:
                break
            candidate = goal if rng.random() < goal_bias else rng.uniform(grid.origin, grid.limit)
            if grid.free(candidate):
                sampled = candidate
                break
        if sampled is None:
            continue
        parent = nearest(tree, sampled)
        new = steer(tree[parent], sampled, step)
        if new is None or grid.check_collision(tree[parent].point, new.point):
            continue
        new.parent = parent
        new.cost = tree[parent].cost + float(np.linalg.norm(new.point-tree[parent].point))
        tree.append(new)
        gap = float(np.linalg.norm(new.point-goal))
        if gap <= goal_tolerance and not grid.check_collision(new.point, goal):
            if gap < 1e-10:
                return find_path(tree, len(tree)-1), tree
            tree.append(TreeNode(float(goal[0]), float(goal[1]), len(tree)-1, new.cost+gap))
            return find_path(tree, len(tree)-1), tree
    return None, tree


def densify(path, spacing=0.1):
    """Linear interpolation of an OPEN path, not smoothing."""
    if spacing <= 0:
        raise ValueError('invalid spacing')
    if len(path) == 0:
        return np.empty((0, 2))
    out = [np.asarray(path[0], dtype=float)]
    for a, b in zip(path[:-1], path[1:]):
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        d = float(np.linalg.norm(b-a))
        if d < 1e-10:
            continue
        steps = math.ceil(d/spacing)
        out.extend(a+(b-a)*i/steps for i in range(1, steps+1))
    return np.asarray(out)
```

將上段存成 `lab4_core.py`，下面存成同目錄的 `check_core.py`，執行 `python3 check_core.py`。

```python
import math
import numpy as np
from lab4_core import Grid, TreeNode, world_to_body, steering_to, steer, rrt_plan, densify

assert np.allclose(world_to_body((1, 4), (1, 2, math.pi/2)), (2, 0))
assert steering_to((2, 0), 0.33, 0.4) == 0.0
assert steering_to((2, 1), 0.33, 0.4) > 0
assert math.isclose(steering_to((2, 1), 0.33, 0.4), -steering_to((2, -1), 0.33, 0.4))
assert steering_to((-1, 0), 0.33, 0.4) is None
assert np.allclose(steer(TreeNode(0, 0), (0.1, 0), 0.35).point, (0.1, 0))
assert steer(TreeNode(0, 0), (0, 0), 0.35) is None
mask = np.zeros((50, 60), dtype=bool)
mask[15:35, 27:33] = True
# Artificial already-inflated map; point robot clearance used by this unit test.
grid = Grid(mask, 0.1, (0, -2.5))
assert grid.check_collision((0.5, 0), (5.5, 0))
assert not grid.check_collision((0.5, 2), (5.5, 2))
assert grid.check_collision((-0.01, 0), (1, 0))
assert grid.check_collision((2.6, -1.1), (2.7, -1.0))  # blocked corner touch
assert grid.check_collision((2.7, -1.0), (2.7, -1.0))
path, tree = rrt_plan(grid, (0.5, 0), (5.5, 0), seed=7, max_iterations=4000, time_budget=5)
assert path is not None
assert np.allclose(path[0], (0.5, 0)) and np.allclose(path[-1], (5.5, 0))
assert all(not grid.check_collision(a, b) for a, b in zip(path[:-1], path[1:]))
dense = densify(path)
assert np.all(np.linalg.norm(np.diff(dense, axis=0), axis=1) <= 0.10000001)
wall = mask.copy()
wall[:, 27:33] = True
failure, _ = rrt_plan(Grid(wall, 0.1, (0, -2.5)), (0.5, 0), (5.5, 0), seed=7,
                      max_iterations=300, time_budget=2)
assert failure is None
invalid, _ = rrt_plan(grid, (0.5, 0), (3.0, 0))
assert invalid is None
print('PASS: transforms, steering signs, steer bounds, corner/out-of-bounds collision,')
print('RRT obstacle detour, path edge validation, interpolation, blocked goal and wall failure.')
print(f'Detour: {len(tree)} tree nodes, {len(path)} path vertices, {len(dense)} interpolated points.')
```

```bash
python3 check_core.py
```

本次已實際執行上述測試並通過：90° 座標轉換、左右轉符號、擴展不超過 sample、零長度擴展、穿障礙／擦角／越界、單一障礙繞行、每條路徑邊碰撞檢查、補點距離、完全阻擋及 goal 無效。固定 seed=7 的繞障案例取得 57 個 tree nodes、21 個路徑頂點及 79 個補點後位置。這些是合成 2D 格網測試結果，不代表 ROS2 或實車已驗證。

## 附錄 B：完整逐檔閱讀清單

以下所有路徑都相對於本次 repo 根目錄。文字檔已全文讀取；空檔已確認；PNG/PGM 已視覺檢視，PGM 的尺寸與對應 YAML 也已核對。Git 的內部版本資料不屬於作業內容，不計入。

| 檔案 | 檢視方式 |
|---|---|
| `.gitignore` | 全文閱讀 |
| `LICENSE` | 全文閱讀 |
| `README.md` | 全文閱讀 |
| `SUBMISSION.md` | 全文閱讀 |
| `imgs/grid.png` | 圖像／地圖視覺檢視 |
| `imgs/rrt.png` | 圖像／地圖視覺檢視 |
| `imgs/rrt_algo.png` | 圖像／地圖視覺檢視 |
| `motion_planning/CMakeLists.txt` | 全文閱讀 |
| `motion_planning/LICENSE` | 全文閱讀 |
| `motion_planning/include/rrt/rrt.h` | 全文閱讀 |
| `motion_planning/motion_planning/__init__.py` | 空檔確認 |
| `motion_planning/package.xml` | 全文閱讀 |
| `motion_planning/scripts/rrt_node.py` | 全文閱讀 |
| `motion_planning/setup.cfg` | 全文閱讀 |
| `motion_planning/setup.py` | 全文閱讀 |
| `motion_planning/src/rrt.cpp` | 全文閱讀 |
| `motion_planning/src/rrt_node.cpp` | 全文閱讀 |
| `particle_filter/.gitignore` | 全文閱讀 |
| `particle_filter/README.md` | 全文閱讀 |
| `particle_filter/config/localize.yaml` | 全文閱讀 |
| `particle_filter/imgs/aims_map.png` | 圖像／地圖視覺檢視 |
| `particle_filter/launch/localize_launch.py` | 全文閱讀 |
| `particle_filter/maps/aims.pgm` | 圖像／地圖視覺檢視 |
| `particle_filter/maps/aims.yaml` | 全文閱讀 |
| `particle_filter/maps/levine.pgm` | 圖像／地圖視覺檢視 |
| `particle_filter/maps/levine.yaml` | 全文閱讀 |
| `particle_filter/package.xml` | 全文閱讀 |
| `particle_filter/particle_filter/__init__.py` | 空檔確認 |
| `particle_filter/particle_filter/particle_filter.py` | 全文閱讀 |
| `particle_filter/particle_filter/utils.py` | 全文閱讀 |
| `particle_filter/resource/particle_filter` | 空檔確認 |
| `particle_filter/rviz/pf.rviz` | 全文閱讀 |
| `particle_filter/setup.cfg` | 全文閱讀 |
| `particle_filter/setup.py` | 全文閱讀 |
| `particle_filter/test/test_copyright.py` | 全文閱讀 |
| `particle_filter/test/test_flake8.py` | 全文閱讀 |
| `particle_filter/test/test_pep257.py` | 全文閱讀 |
| `pure_pursuit/CMakeLists.txt` | 全文閱讀 |
| `pure_pursuit/LICENSE` | 全文閱讀 |
| `pure_pursuit/package.xml` | 全文閱讀 |
| `pure_pursuit/pure_pursuit/__init__.py` | 空檔確認 |
| `pure_pursuit/scripts/pure_pursuit_node.py` | 全文閱讀 |
| `pure_pursuit/scripts/waypoint_logger.py` | 全文閱讀 |
| `pure_pursuit/setup.cfg` | 全文閱讀 |
| `pure_pursuit/setup.py` | 全文閱讀 |
| `pure_pursuit/src/pure_pursuit_node.cpp` | 全文閱讀 |
| `pure_pursuit/waypoints/.gitkeep` | 空檔確認 |
