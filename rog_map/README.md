# ROG local mapping node

`rog_map_node` maintains only a sliding local probability grid and its incrementally inflated occupancy. It consumes timestamped scans and TF, and publishes complete local snapshots for planning. It does not build an OctoMap, retain a global map, load a PCD, or write per-scan logs.

```bash
source /path/to/motion_planning_ws/devel/setup.bash
roslaunch rog_map local_map.launch
```

The window size and resolution are configurable. ROG rounds the dimensions upward to its odd-cell layout. The window follows the physical LiDAR origin at each scan timestamp and stays aligned with the world axes. Cells leaving the window are forgotten; returning to an old location does not restore their history. There are no fixed navigation bounds or virtual ground/ceiling in this node.

The `pcl_topic` argument selects the input point cloud. Both the cloud transform and the transform for `sensor_origin_frame_id` must be available at the cloud timestamp. XYZ float32/float64 clouds, including organized clouds and clouds without intensity, are supported; nonfinite points are removed. An unavailable historical transform causes the scan to be rejected, rather than paired with the latest odometry. A single pending scan bounds backlog; overwritten scans are counted in diagnostics.

Raycasting is bounded by the local window and the configured ray range. Empty valid scans keep the accumulated map while updating its window. Hit/miss probabilities, clamping bounds and occupancy thresholds are configured in `config/local_map.yaml`. Unknown cells are traversable for the current planner. These ROG fusion rules are not probability-identical to OctomapServer. Do not set the occupied threshold to 0.5: unobserved cells start at log-odds zero.

## Interfaces

The interfaces below use node-private names; the launch file selects the namespace and remaps the snapshot topic through `map_topic`.

| Interface | Type | Purpose |
| --- | --- | --- |
| `~local_map` | `rog_map_msgs/LocalMap` | Complete local raw/inflated occupancy snapshot, one per processed scan, non-latched |
| `~reset` | `std_srvs/Empty` | Clear history and inflation, start a new epoch, publish an empty snapshot if already located |
| `~occupied_cells` | `visualization_msgs/MarkerArray` | Local occupied voxel volumes and window edges, published when subscribed according to the configured rate |
| `~diagnostics` | `diagnostic_msgs/DiagnosticArray` | Input/fusion rates, overwritten/rejected scans, bytes, stage timings and map age, 1 Hz |

The `local_map_size`, `voxel_width`, `dilate_radius`, `max_range`, `world_frame_id`, `sensor_origin_frame_id`, `pcl_topic` and `map_topic` launch arguments configure the integration. Detailed settings are in `config/local_map.yaml`. Inflation uses the same resolution as the raw grid and a cube of radius `ceil(dilate_radius / voxel_width)`, matching repeated 26-neighbor route dilation. The collision consumer must use only the raw bitmap.

The private parameter `occupied_cells_publish_rate` controls visualization timing: `-1` publishes with every snapshot, including resets; a positive value selects a ROS-time timer frequency in Hz. Visualization is only published when subscribed.

A reset uses the last observation timestamp and a new epoch. Before any scan has established the window location, reset publishes nothing. A backwards scan clock also clears history and starts a new epoch, allowing bag loops without fusing different timelines. Epochs use wall-clock nanoseconds and increase within the node lifetime; run one mapper publisher per topic.

## Snapshot contract

`rog_map_msgs` is a separate message-only package. Consumers do not need to link the ROG mapping library. `LocalMap` contains the observation header, epoch/version, resolution, minimum world corner, three voxel counts, inflation steps, and two packed bitmaps. Bounds are half-open: `origin <= p < origin + size * resolution`. The center of cell `(x,y,z)` is `origin + resolution * (x+0.5,y+0.5,z+0.5)`.

Both bitmaps use linear index `x + nx*(y + ny*z)`, least-significant bit first, with `ceil(nx*ny*nz/8)` bytes and zero padding bits. Every raw occupied bit must also be present in the inflated bitmap. The bitmaps describe the same completed update and same window. Dropping a snapshot is permitted because later snapshots are self-contained. Window metadata must travel with the bitmaps; an obstacle bounding box cannot represent map coverage.

Export reads ring storage directly, without PCL conversion or visualization queries. Bitmap size depends on the configured window dimensions and resolution, independent of flight history. No planner work, ROS serialization of trees, or global-map traversal occurs in the mapper process.

## Validation and measurement

```bash
catkin build rog_map_msgs rog_map motion_primitive_planner
catkin build rog_map motion_primitive_planner --no-deps --catkin-make-args run_tests
catkin_test_results build/rog_map/test_results
catkin_test_results build/motion_primitive_planner/test_results
rosrun rog_map measure_local_map.py --duration 90 --output /tmp/local_map_metrics.json
```

The measurement script observes filtered-cloud reception, snapshot reception and optional planner scene-ready notifications, and samples process CPU/RSS and diagnostics. It reports warmup-excluded rates and latency percentiles. Scene notification latency includes notification transport and therefore provides an upper bound on scene availability relative to the observer's cloud reception. Use an isolated ROS master for recorded robot data. For an old mapper comparison, pass `--old-octomap --map /dragon/octomap/full --node /dragon/octomap_server`; replay exactly the same prefiltered clouds and TF.

The acceptance target on the onboard computer is at least 9 Hz fusion and scene updates for 10 Hz valid scans, without growing backlog, and p95 filtered-cloud-to-scene latency below 150 ms. Desktop results do not establish onboard performance. This change does not add trajectory revalidation against every arriving map: planning continues to validate each candidate batch against one immutable scene.
