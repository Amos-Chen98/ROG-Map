# ROG-Map for DRAGON Navigation

Sliding local occupancy mapping for `motion_primitive_planner`, provided by `rog_map` and `rog_map_msgs`.

```bash
cd /path/to/motion_planning_ws
catkin build rog_map_msgs rog_map motion_primitive_planner
source devel/setup.bash
roslaunch rog_map local_map.launch
```

The mapper consumes point clouds and timestamped TF, and publishes local occupancy snapshots. Configure its topics and frames through the launch arguments.

See the [node interfaces and snapshot contract](rog_map/README.md) and [license](LICENSE).
