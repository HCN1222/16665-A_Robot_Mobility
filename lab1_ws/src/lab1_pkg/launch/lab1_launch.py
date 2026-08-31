# ROS 2 launch 的核心類別
# LaunchDescription 可以理解成「這次 launch 要執行哪些動作」的清單
from launch import LaunchDescription

# ROS 2 launch 中用來啟動 ROS Node 的類別
from launch_ros.actions import Node

# 用來尋找某個 ROS 2 package 安裝後的 share directory
from ament_index_python.packages import get_package_share_directory

# Python 標準函式庫
# 這裡主要拿 os.path.join() 組合檔案路徑
import os


# ============================================================
# ROS 2 launch file 的主要入口
# 當你執行 ros2 launch 時，ROS 2 會呼叫這個 function
# ============================================================
def generate_launch_description():

    # 建立一個空的 LaunchDescription
    # 之後會把所有要啟動的 Node 加進這裡
    ld = LaunchDescription()


    # ========================================================
    # 找到 talker_params.yaml 設定檔
    # ========================================================

    # get_package_share_directory('lab1_pkg')
    # 會找到 lab1_pkg package 的 share directory
    #
    # 假設結果是：
    # /home/user/lab1_ws/install/lab1_pkg/share/lab1_pkg
    #
    # 再接上：
    # config/talker_params.yaml
    #
    # 最後 config 就會是一個完整檔案路徑
    config = os.path.join(
        get_package_share_directory('lab1_pkg'),
        'config',
        'talker_params.yaml'
    )


    # ========================================================
    # 1. Talker Node
    # ========================================================

    talker_node = Node(

        # 要去哪個 ROS package 找 executable
        package='lab1_pkg',

        # 要執行該 package 裡的哪個 executable
        #
        # 大致相當於：
        #
        # ros2 run lab1_pkg talker.py
        executable='talker.py',

        # ROS graph 裡面這個 Node 的名稱
        #
        # ros2 node list
        #
        # 可能會看到：
        # /talker
        name='talker',

        # 傳給 talker 的 ROS parameter
        #
        # 這裡直接把 talker_params.yaml 的檔案路徑傳進去
        # ROS 2 會自動讀取這個 yaml 檔案
        #
        # 前提：yaml 裡的最上層 key 要跟這個 Node 的
        # name（也就是上面的 'talker'）完全一致，
        # 否則 ROS 2 不會套用這些參數
        #
        # 對應到：
        #
        # ros2 run lab1_pkg talker.py --ros-args --params-file talker_params.yaml
        parameters=[config]
    )


    # ========================================================
    # 2. Relay Node
    # ========================================================

    relay_node = Node(

        # relay 所屬的 ROS package
        package='lab1_pkg',

        # 要執行 relay.py 這個 executable
        #
        # 大致相當於：
        # ros2 run lab1_pkg relay.py
        executable='relay.py',

        # ROS Node 名稱
        #
        # ros2 node list
        #
        # 可能會看到：
        # /relay
        name='relay',
    )


    # ========================================================
    # Finalize
    # 把前面定義好的 Node 全部加入 LaunchDescription
    # ========================================================

    # 啟動 talker
    ld.add_action(talker_node)

    # 啟動 relay
    ld.add_action(relay_node)


    # ========================================================
    # 把完整的 LaunchDescription 回傳給 ROS 2 launch system
    # ========================================================

    return ld
