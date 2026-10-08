# agri_lidar_adapter

This package owns the hardware-independent navigation contract for the
MID-360.  The simulator and the real sensor both publish the calibrated
`mid360_sensor_frame` point cloud.  `mid360_cloud_filter` transforms it into
the horizontal `mid360_scan_frame`, applies range/height/self filters, and
publishes `/mid360/navigation_points`.  The launch file then uses the Jazzy
`pointcloud_to_laserscan` component to publish `/scan`.

The scan frame has the same origin as the optical centre.  Its fixed +15°
pitch correction cancels the measured -15° sensor mount pitch; the point cloud
header is never relabelled to fake this transform.

Start it independently when a `/mid360/points` stream and robot TF are already
running:

```bash
ros2 launch agri_lidar_adapter lidar_to_scan.launch.py
```

The default slice is `-0.40..0.40 m` in `mid360_scan_frame`, range
`0.10..40 m`, and a 1° angular increment.  Tune these in
`config/lidar_to_scan.yaml` after checking the ground, chassis, plant stems,
and row-end geometry.  `self_filter_enabled` and `voxel_size` are deliberately
off in the baseline so that the first scan can be audited without hidden
decimation.

The standard converter is lazy and only consumes the cloud while `/scan` has a
subscriber.  RViz, SLAM Toolbox, Nav2, or `ros2 topic echo /scan` therefore
must be started before judging the point-cloud filter's output rate.
