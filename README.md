# NatNet ROS Bridge

ROS 1 bridge for OptiTrack Motive, based on the NatNet SDK 4.5.1 Python sample client. Tested with Motive 3.1.4 streaming NatNet 4.1.

The bridge publishes rigid-body poses and marker clouds. An optional `optitrack_tf.py` node connects one selected rigid body to a calibrated TF tree. **TF publishing is disabled by default**, allowing another project to enable it with its own frame names and calibration.

[Repository](https://github.com/EliasNotEliars/NatNet-Bridge/tree/main)

## Requirements

- Python 3, Git, ROS 1, catkin and `catkin_tools` (`catkin build`).
- ROS packages: `rospy`, `geometry_msgs` and `sensor_msgs`.
- For TF integration: `tf2_ros` and `visualization_msgs`. The current `optitrack_tf.py` also imports `jsk_rviz_plugins`, so it must be installed when running that node.
- Network connectivity to the Motive computer, with NatNet streaming and the required rigid-body/marker streams enabled.

Source your ROS installation before building or running the package. For the lab's Debian ROS installation:

```bash
source /opt/ros/one/setup.bash
```

On another installation, use its corresponding ROS setup script.

## Installation and quick start

### 1. Create a workspace and clone the package

```bash
mkdir -p ~/natnet_ws/src
cd ~/natnet_ws/src
git clone https://github.com/EliasNotEliars/NatNet-Bridge.git natnet_bridge
```

Alternatively, clone directly into an empty package directory using `.`:

```bash
mkdir -p ~/natnet_ws/src/natnet_bridge
cd ~/natnet_ws/src/natnet_bridge
git clone https://github.com/EliasNotEliars/NatNet-Bridge.git .
```

Use one of these alternatives. The repository root contains `package.xml`; its `src/natnet_bridge/` directory contains the Python modules, not another catkin workspace.

### 2. Build and source

```bash
cd ~/natnet_ws
catkin build
source devel/setup.bash
```

Source the ROS installation and this workspace's `devel/setup.bash` in each new terminal before using the package.

### 3. Configure and launch

In Motive, enable streaming and check the server address, client interface and multicast setting. The launch defaults match the lab network:

| Argument | Default |
| --- | --- |
| `client_address` | `10.0.0.1` |
| `server_address` | `10.0.0.2` |
| `use_multicast` | `true` |
| `frame_id` | `optitrack_world` |
| `publish_tf` | `false` |
| `rigid_body_id` | `2` |

Edit `config/rigid_bodies.yaml` so that Motive Streaming IDs map to the desired body names. Launch with:

```bash
roslaunch natnet_bridge natnet.launch
```

Override network settings or the mapping file as needed:

```bash
roslaunch natnet_bridge natnet.launch \
  client_address:=10.0.0.1 \
  server_address:=10.0.0.2 \
  bodies_config:=/absolute/path/to/rigid_bodies.yaml
```

### 4. Check the output

In another sourced terminal:

```bash
rostopic list
rostopic echo /optitrack/rigid_bodies/tello/pose
```

Replace `tello` with the name assigned in `rigid_bodies.yaml`. Names come from that mapping; the `bodies_config` launch argument selects the file. Unmapped bodies use `id_<StreamingID>`.

## Package structure

| Path | Purpose |
| --- | --- |
| `package.xml`, `CMakeLists.txt`, `setup.py` | Catkin package and Python setup |
| `scripts/natnet_node.py` | NatNet reception, ROS configuration and publishing |
| `scripts/optitrack_tf.py` | Optional TF integration and tracking-status marker |
| `src/natnet_bridge/__init__.py` | Python package initialization |
| `src/natnet_bridge/NatNetClient.py` | SDK-derived NatNet client |
| `src/natnet_bridge/MoCapData.py` | SDK-derived motion-capture data structures |
| `src/natnet_bridge/DataDescriptions.py` | SDK-derived data descriptions |
| `launch/natnet.launch` | Network, body mapping and optional TF configuration |
| `config/rigid_bodies.yaml` | Motive Streaming ID-to-name mapping |
| `config/optitrack_tf.yaml` | TF frame names and geometric calibration |

Workspace-generated directories such as `build/`, `devel/` and `logs/` are build outputs, not package source files.

## Node behavior and topics

Publishers are created dynamically as bodies and named marker sets arrive.

| Topic | Message type |
| --- | --- |
| `/optitrack/rigid_bodies/<name>/pose` | `geometry_msgs/PoseStamped` |
| `/optitrack/marker_sets/name_<hex-encoded-name>/markers` | `sensor_msgs/PointCloud` |
| `/optitrack/unlabeled_markers` | `sensor_msgs/PointCloud` |
| `/optitrack_tf/tracking_status` (when TF is enabled with the default node name) | `visualization_msgs/Marker` |

Only valid rigid-body poses are published. Marker-set clouds may include model-filled positions; point indices are not persistent marker IDs. The aggregate marker set `all` is skipped.

**The NatNet pose and point-cloud topics retain the received coordinates**, with the configured frame ID and local receipt timestamps. Enabling TF does not rewrite those messages; it provides transforms for consumers that need calibrated coordinates.

## Optional TF integration

The TF node subscribes to the pose topic for one selected Streaming ID, reusing the same body-name mapping as `natnet_node.py`. Its default frame chain is:

```text
world → optitrack_world → tello_mocap → tello_base
```

- `world → optitrack_world`: static room alignment.
- `optitrack_world → tello_mocap`: measured rigid-body pose, updated from incoming messages.
- `tello_mocap → tello_base`: static mounting calibration.

A project's `robot_state_publisher` can extend the tree from `tello_base` to its URDF links; this bridge does not launch it.

Before enabling TF, review `config/optitrack_tf.yaml` for your setup. The `world_to_optitrack` and `mocap_to_base` entries each require `translation: [x, y, z]` in metres and `rotation_xyzw: [x, y, z, w]` as a quaternion. These describe the child frame's origin and orientation in its parent. Frame names and calibration must match the consuming project. Avoid another node publishing the same TF child frames.

Enable TF directly with:

```bash
roslaunch natnet_bridge natnet.launch publish_tf:=true rigid_body_id:=2
```

The node rejects invalid poses, broadcasts accepted measurements using their message timestamps, and publishes a world-anchored RViz text marker when tracking becomes unavailable. Add a Marker display subscribed to its tracking-status topic to see the warning.

## Include in another project

Include the bridge in your project's launch file and explicitly enable TF:

```xml
<include file="$(find natnet_bridge)/launch/natnet.launch">
    <arg name="publish_tf" value="true"/>
</include>
```

This uses Streaming ID `2` and the bridge's default configuration files. To select a different body and use your project's calibration:

```xml
<include file="$(find natnet_bridge)/launch/natnet.launch">
    <arg name="publish_tf" value="true"/>
    <arg name="rigid_body_id" value="1"/>
    <arg name="bodies_config" value="$(find your_package)/config/rigid_bodies.yaml"/>
    <arg name="tf_config" value="$(find your_package)/config/optitrack_tf.yaml"/>
</include>
```

Replace `your_package` with your package name. Build the bridge in the same catkin workspace, or source the workspace containing it before building and launching your project.

## SDK license

The three SDK-derived modules retain their original copyright notices and remain subject to the applicable NaturalPoint/OptiTrack SDK license terms.
