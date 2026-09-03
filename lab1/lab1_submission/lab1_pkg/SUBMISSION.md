# Lab 1: Intro to ROS 2

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: ```source /opt/ros/humble/setup.bash``` and ```source install/local_setup.bash```. Functionally what is the difference between the two?

Answer: These two commands set environment variables, allowing ROS2 to find packages and dependencies needed. ```source /opt/ros/humble/setup.bash``` sets the environment of ROS2 Humble as underlay, while ```source install/local_setup.bash``` only add current workspace as overlay. 

### Q2: What does the ```queue_size``` argument control when creating a subscriber or a publisher? How does different ```queue_size``` affect how messages are handled?

Answer: `queue_size` determines the number of messages that can be buffered during communication. In `ROS2`, the `ROS1` queue_size argument is replaced by the history and depth settings in the QoS profile. Its effect depends on whether it is applied to a publisher or subscriber and on the QoS reliability policy (`BEST_EFFORT` or `RELIABLE`), as summarized below.

||BEST_EFFORT| RELIABLE|
| -- | -- |--|
| **Publisher**  | limits the number of recent messages stored in the publisher history. When full, older messages will be replaced by newer ones and lost messages are not retransmitted. | limits the number of recent messages stored in the publisher history and available for reliable delivery/retransmission. A larger depth can better tolerate temporary delays but uses more memory.|
| **Subscriber** | limits the number of received messages waiting to be processed. If the queue is full, older messages are dropped and lost messages are not recovered.|limits the number of received messages waiting to be processed. Reliable delivery attempts to recover transmission losses. While a larger depth can reduce drops caused by slow processing, it might increase latency.|


### Q3: Do you have to call ```colcon build``` again after you've changed a launch file in your package? (Hint: consider two cases: calling ```ros2 launch``` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

Answer: It depends. If we run the launch file through file paths (`ros2 launch XXX/launch_file.py`), ROS2 will directly read the original file. Therefore, `colcon build` is not required.

In contrast, if we execute through installed packages (`ros2 launch <package_name> <launch_file>`), ROS2 will use the copy under `install/`. In this case, `colcon build` is needed. However, if we use `--symlink-install` when building the package, since the launch file is linked to the original file through symbolic link, no rebuild is needed.
