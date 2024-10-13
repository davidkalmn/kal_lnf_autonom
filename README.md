# Array Sorter #

Ez a ROS2 projekt egy egyszerű tömb rendezőt valósít meg.
Ez a package két node-ból áll. Az első node, az ```/array_publisher```, véletlenszerű számokat generál (a számok határértékei, illetve a generálásuk sebessége változtatható az ```/array_publisher```-ben) és ezeket egy ```std_msgs/msg/Float64MultiArray``` típusú, ```unsorted_array```-nak nevezett topicban hirdeti. 
A második node, az ```/array_sorter```, fogadja az érkező adatokat, majd rendezi őket. Az így rendezett tömböt egy másik ```std_msgs/msg/Float64MultiArray``` típusú topicban hirdeti, amelyet ```sorted_array```-nak nevezünk.
Megvalósítás ```ROS 2 Humble``` alatt.

## Packages and build ##

### Clone the packages ###
It is assumed that the workspace is ```~/ros2_ws/```.
```
cd ~/ros2_ws/src
```
```
git clone https://github.com/davidkalmn/kal_lnf_autonom
```

### Build ROS2 packages ###
```
cd ~/ros2_ws
```
```
colcon build
```
Dont forget to source before ROS commands (use this command in every terminal):
```
source ~/ros2_ws/install/setup.bash
```

### Run ROS2 packages ###
Run these commands on separate terminals:
```
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash
ros2 run kal_array_sorter array_publisher
```
```
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash
ros2 run kal_array_sorter array_sorter_node
```

### Check the randomly generated arrays ###
Unsorted:
```
ros2 topic echo /unsorted_array
```
Sorted:
```
ros2 topic echo /sorted_array
```

## Results ##
Unsorted array on the left, and the sorted array on the right:
![image](https://github.com/user-attachments/assets/d329f44a-bfbf-4cf3-8251-7fff43c6e324)




