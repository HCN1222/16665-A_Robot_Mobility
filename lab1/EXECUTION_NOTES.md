# Lab 1 執行筆記

工作區位置：`lab1_ws/`（在 repo 根目錄，不是 `lab1/` 底下）
Package：`lab1_pkg`（Python 版本：`talker.py` + `relay.py`）

---

## 1. 環境準備

每次開一個新的終端機，第一件事一定要 source ROS 2：

```bash
source /opt/ros/humble/setup.bash
```

確認 ROS 2 有裝好：

```bash
ros2 topic list
# 應該只看到 /parameter_events /rosout
```

如果是用 Docker（無法原生裝 Ubuntu 才需要）：

```bash
docker run -it -v <repo路徑>/lab1_ws/src/:/lab1_ws/src/ --name robot_mobility_lab1 ros:humble
```
容器裡建議裝 `tmux` 開多個終端機（`apt update && apt install tmux`）。

---

## 2. 安裝依賴（rosdep）

`package.xml` 已宣告 `ackermann_msgs`、`rclcpp`、`rclpy` 等依賴，用 rosdep 安裝：

```bash
cd ~/lab1_ws   # 或你的 lab1_ws 路徑
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

---

## 3. Build（colcon build）— 最容易踩雷的地方

### ⚠️ 最重要的規則：一定要在 `lab1_ws` 這個「工作區根目錄」下執行 `colcon build`，絕對不要在 `lab1_ws/src` 裡面執行！

之前就是因為在 `src/` 裡誤跑過一次 `colcon build`，導致 `src/` 底下多長出一份不完整、壞掉的 `build/`、`install/`、`log/`（已刪除修正）。

正確流程：

```bash
cd /home/hcn1222/projects/16665-A_Robot_Mobility/lab1_ws   # 確認 pwd 是這一層
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
```

只想重新 build 單一 package（比較快）：

```bash
colcon build --packages-select lab1_pkg
```

用 `--symlink-install` 的話，改 Python 檔案 / launch file / yaml 參數檔不用重新 build 就會生效（因為 install 目錄裡放的是 symlink，不是複製檔）：

```bash
colcon build --symlink-install
```

**每次開新終端機都要重新 source 兩個東西（且順序不能反）：**
```bash
source /opt/ros/humble/setup.bash      # 系統的 ROS 2
source install/local_setup.bash        # 這個 workspace 自己 build 出來的 overlay
```

---

## 4. 執行 Node

### 4.1 分別手動執行（測試用）

開兩個終端機（或用 tmux 分頁），各自 source 好環境後：

```bash
# 終端機 1：talker，用參數指定 v, d
ros2 run lab1_pkg talker.py --ros-args -p v:=1.0 -p d:=0.5

# 終端機 2：relay
ros2 run lab1_pkg relay.py
```

### 4.2 用 launch file 一次啟動兩個 node

```bash
ros2 launch lab1_pkg lab1_launch.py
```

`lab1_launch.py` 裡已經有帶入 `config/talker_params.yaml`（`v: 1.0`, `d: 0.5`）給 `talker` node。

**注意**：`get_package_share_directory('lab1_pkg')` 找的是 **install 之後** 的 share 目錄，不是原始碼裡的 `src/lab1_pkg/config/`。所以：
- 如果只改了 `config/talker_params.yaml` 或 `launch/lab1_launch.py`，**沒有用 `--symlink-install`** 的話，必須重新 `colcon build` 才會反映到 `install/` 裡，`ros2 launch` 抓到的才是新版本。
- 這也是 SUBMISSION.md Q3 在問的東西：直接在原始碼目錄下用 `ros2 launch` 執行 vs. package 裝好之後再執行，行為會不一樣，建議實際測一次再寫答案。

---

## 5. 測試 ROS 2 指令（README 第7節要求）

在兩個 node 都跑起來的狀態下，另開一個終端機（記得 source）：

```bash
ros2 topic list
ros2 topic info /drive
ros2 topic echo /drive
ros2 node list
ros2 node info /talker
ros2 node info /relay
```

應該會看到 `/drive`、`/drive_relay` 兩個 topic，以及 `/talker`、`/relay` 兩個 node。

---

## 6. 常見注意事項 / Debug checklist

- **`ros2 run` 找不到 executable**：通常是忘記 `source install/local_setup.bash`，或是改完程式後忘記重新 `colcon build`。
- **改了 Python 檔案沒生效**：看是不是用 `--symlink-install` build 的；不是的話要重新 build。
- **launch file 抓不到新的 yaml 參數**：同上，是抓 `install/` 裡的版本，不是 `src/` 原始碼。
- **`colcon build` 位置錯誤**：一定要在 `lab1_ws`（workspace 根目錄）執行，不要在 `lab1_ws/src` 或 `lab1_ws/src/lab1_pkg` 裡執行，不然會產生多餘、位置錯誤的 `build/`、`install/`、`log/`。
- **queue_size（`qos_profile`）**：目前 code 裡 publisher/subscriber 都用 `qos_profile=10`，代表訊息佇列深度為 10；佇列滿了之後，新訊息會怎麼處理跟 QoS policy 有關（可搭配 SUBMISSION.md Q2 一起研究、寫進答案）。
- **package 資料夾要乾淨**：`lab1_pkg` 底下不該有多餘的 `build/install/log`，也不要有巢狀重複的 `src/` 資料夾——繳交前打包的 [lab1_submission/](lab1_submission/) 只放原始碼（`CMakeLists.txt`、`package.xml`、`config/`、`launch/`、`scripts/`），不含建置產物。

---

## 7. 繳交前檢查清單

- [ ] `lab1_pkg` 內 `package.xml` 有宣告 `ackermann_msgs`
- [ ] `talker.py` / `talker.cpp`：讀 `v`、`d` 參數，publish `AckermannDriveStamped` 到 `/drive`
- [ ] `relay.py` / `relay.cpp`：訂閱 `/drive`，speed/steering_angle 各乘 3，publish 到 `/drive_relay`
- [ ] `lab1_launch.py`：能一次啟動 talker + relay
- [ ] 用第 5 節的指令實際測試過，確認 topic / node 都正常
- [ ] `SUBMISSION.md` 三題問答填完（結合第 3、6 節的測試心得寫）
- [ ] 把 `lab1_pkg` 原始碼 + 填好的 `SUBMISSION.md` 一起放進 [lab1_submission/](lab1_submission/)，壓縮成 zip 上傳 Canvas
