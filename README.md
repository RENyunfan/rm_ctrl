# rm_ctrl

# Dependency

## fmt

```
mkdir build
cd build/
cmake -DBUILD_SHARED_LIBS=TRUE ..
make
sudo make install
```

# Usage

Set some parameter at [./rm_ctrl/config.param.yaml](./rm_ctrl/config/param.yaml)

```yaml
topics:
  odom_topic: "/lidar_slam/odom"
  ctrl_topic: "/cmd_vel"
  cmd_topic: "/planning/pos_cmd"

position_gain:
  x: 1.0
  y: 1.0
velocity_gain:
  x: 1.0
  y: 1.0
attitude_gain:
  yaw: 1.0
  yaw_dot: 1.0
```

Then run with

```
roslaunch rm_ctrl run_rm_ctrl.launch
```

