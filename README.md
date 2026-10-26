# NatNet ROS Bridge

ROS 1 bridge for OptiTrack Motive, based on the NatNet SDK 4.5.1 Python sample client. Tested with Motive 3.1.4 streaming NatNet 4.1.

## Requirements

- Python 3, ROS 1 and catkin.
- ROS packages: `rospy`, `geometry_msgs`, `sensor_msgs`.
- Network connectivity to Motive, with NatNet streaming and the required rigid-body/marker streams enabled.

## Package structure

```text
natnet_bridge/
├── package.xml
├── CMakeLists.txt
├── setup.py
├── scripts/
│   └── natnet_node.py
├── src/natnet_bridge/
│   ├── __init__.py
│   ├── NatNetClient.py
│   ├── MoCapData.py
│   └── DataDescriptions.py
├── launch/
│   └── natnet.launch
└── config/
    └── rigid_bodies.yaml
```

`natnet_node.py` handles ROS configuration and publishing; the Python modules under `src/` decode NatNet packets. The launch file configures client/server addresses, multicast mode, frame ID and the body-name configuration file. `rigid_bodies.yaml` maps Motive Streaming IDs to readable ROS topic names.

## Node behavior and topics

Publishers are created dynamically as bodies and named marker sets arrive. Unmapped bodies use `id_<StreamingID>` as their topic name.

| Topic | Message type |
| --- | --- |
| `/optitrack/rigid_bodies/<name>/pose` | `geometry_msgs/PoseStamped` |
| `/optitrack/marker_sets/name_<hex-encoded-name>/markers` | `sensor_msgs/PointCloud` |
| `/optitrack/unlabeled_markers` | `sensor_msgs/PointCloud` |

Only valid rigid-body poses are published. Marker-set clouds may include model-filled positions; point indices are not persistent marker IDs. The aggregate marker set `all` is skipped.

Coordinates pass through unchanged, timestamps use local receipt time, and no TF is broadcast.

The three SDK-derived modules retain their original copyright notices and remain subject to the applicable NaturalPoint/OptiTrack SDK license terms.
