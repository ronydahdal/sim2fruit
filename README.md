# Companion Code to Paper: Sim2Fruit (IEEE SEIDS)
## Sim2Fruit: Simulated and End-to-End Pipeline for Robotic Arm + RL
### UNDER CONSTRUCTION CURRENTLY
```bash
git clone https://github.com/ronydahdal/sim2fruit.git
cd sim2fruit_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

```bash
ros2 launch sim2fruit_description mycobot_gazebo_minimal.launch.py
```
This spawns:
- MyCobot robot (frozen)
- A Pheno4D-derived plant model (static)

```bash
ros2 run sim2fruit_perception pointnet_node
```

```bash
ros2 run sim2fruit_perception dummy_policy_node
```

To convert PCD into a mesh for Gazebo (for plant models):
```bash
python3 pcd_conversion.py
```

