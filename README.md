# nav2_safeplan
nav2_safeplan is a ros2 package integrated with nav2 to run global planner from safeplan benchmark based on config
 
To run the code:

1. Clone the repositiary

2. Build the code

```bash
colcon build
```
3. Run the turtlebot code, and run nav2 using

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py params_file:=$HOME/nav2_safeplan/src/nav2_straightline_planner/nav2_params.yaml

```

4. Create a symlink for config/algos.yaml in safeplan_ros2

```bash
cd install/safeplan_ros2/share/safeplan_ros2/config
rm algos.yaml
ln -s ../../../../../src/safeplan_ros2/config/algos.yaml algos.yaml

```
5. Run the safeplan launch
```bash
ros2 launch safeplan_ros2 safe_planner_launch.py

```

You can close safeplan_ros2 node , update yaml to change planner and run again.