# Magdalene Camera Wrapper (ros2)

ROS2 packages for Intel RealSense D555 camera integration over Ethernet PoE, supporting multiple cameras identified by serial number.

## Architecture

The D555 cameras connect via **Ethernet PoE**. This repository contains two packages that work together:

- **`magdalene_camera_sdk`** — C++ library that wraps `librealsense2`. Handles the direct connection to each camera over the network, configures streams (resolution, FPS), and exposes frames as `cv::Mat`. Has no ROS2 dependency.
- **`magdalene_camera_ros`** — ROS2 node that uses the SDK as its backend. Reads parameters, instantiates one `magdalene_camera` object per camera, and publishes frames as `sensor_msgs/msg/Image`.

```
librealsense2
     ↓
magdalene_camera_sdk    ← camera connection, no ROS2
     ↓
magdalene_camera_ros    ← ROS2 bridge: SDK → topics
     ↓
/camera_55/color/image_raw
/camera_55/infra2/image_raw
/camera_56/color/image_raw
/camera_56/infra2/image_raw
```

`colcon` builds `magdalene_camera_sdk` first (declared dependency), then `magdalene_camera_ros`.

### Why SDK over DDS subscribe/republish

An alternative architecture would be to subscribe to topics published by a remote `realsense2_camera` node and republish them locally. This was tested and discarded for three reasons:

1. **No FPS control** — `realsense2_camera` published at 30 fps by default. Without SSH access to the remote machine, FPS cannot be changed from this node. The SDK sets FPS directly via `color_fps` / `infra2_fps` parameters (default 15 fps), halving the bandwidth.

2. **Packet loss on Infrared streams** — DDS uses BEST_EFFORT QoS for sensor data over the network. At 30 fps with two cameras the link became saturated, causing dropped packets and `sequence size exceeds remaining buffer` deserialization errors on the Infrared streams. librealsense2 uses its own reliable transport protocol to receive frames directly from the camera, eliminating these drops.

3. **Double serialization** — The DDS approach serialized frames twice: once from the remote camera node to this machine, then again from our bridge to local subscribers. The SDK approach serializes only once (locally), reducing CPU and latency.

## Package structure

```
magdalene_camera/
├── magdalene_camera_sdk/            # Hardware abstraction library (ament_cmake)
│   ├── include/
│   │   └── magdalene_camera.hpp    # magdalene_camera class declaration
│   └── src/
│       ├── magdalene_camera_wrapper.cpp  # librealsense2 wrapper implementation
│       └── main.cpp               # Standalone OpenCV demo (no ROS2)
│
└── magdalene_camera_ros/            # ROS2 node
    ├── include/magdalene_camera_ros/
    │   └── magdalene_camera_node.hpp  # MagdaleneCameraNode declaration
    ├── src/
    │   ├── magdalene_camera_node.cpp  # MagdaleneCameraNode implementation
    │   └── main.cpp                   # main()
    ├── config/
    │   └── cameras.yaml               # All camera parameters — edit here to configure
    └── launch/
        └── multi_camera.launch.py     # Reads cameras.yaml, launches one node per camera
```

## Prerequisites

| Dependency | Version | Notes |
|---|---|---|
| ROS2 | Humble | |
| librealsense2 | **2.58.2** | installed to `/usr/local` |
| OpenCV | 4.5.4 | |
| `ros-humble-cv-bridge` | | |
| `ros-humble-image-transport` | | |
| `ros-humble-rmw-cyclonedds-cpp` | | required — see Known Issues |

### Camera firmware

Check firmware version via `realsense-viewer` → select device → Device tab → Firmware.

## Environment setup

Add to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is **required**. `librealsense2 2.58` bundles its own FastDDS internally, which conflicts with ROS2's default `rmw_fastrtps_cpp`. CycloneDDS avoids this conflict entirely.

## Build

```bash
cd ~/magdalene_ws
colcon build --symlink-install
source install/setup.bash
```

To rebuild only these packages:

```bash
colcon build --symlink-install --packages-select magdalene_camera_sdk magdalene_camera_ros
```

## Camera configuration

All parameters live in `config/cameras.yaml`. Common parameters are declared once under `/**` and apply to all cameras. Each camera section only needs the values that are unique to it (`serial` and `frame_id`):

```yaml
/**:
  ros__parameters:
    projector:     false
    color_width:   640
    color_height:  360
    color_fps:     15
    infra2_width:  640
    infra2_height: 360
    infra2_fps:    15

/camera_55/camera_55:
  ros__parameters:
    serial:   "419222301921"   # 192.168.11.55
    frame_id: "camera_55"

/camera_56/camera_56:
  ros__parameters:
    serial:   "353522302286"   # 192.168.11.56
    frame_id: "camera_56"
```

To add a camera, add a new `/name/name` section — the launch file auto-discovers all entries and starts one node per camera.

## Running

```bash
ros2 launch magdalene_camera_ros multi_camera.launch.py
```

Verify topics are publishing:

```bash
ros2 topic list
ros2 topic hz /camera_55/color/image_raw
```

## SDK standalone test

`main.cpp` opens one camera and displays color and infrared streams with OpenCV — no ROS2 required. Accepts the serial number as an optional argument:

```bash
# specific camera
~/magdalene_ws/install/magdalene_camera_sdk/lib/magdalene_camera_sdk/magdalene_camera_demo 419222301921

# first available camera
~/magdalene_ws/install/magdalene_camera_sdk/lib/magdalene_camera_sdk/magdalene_camera_demo
```

Controls: `q` / `Esc` — quit · `p` — toggle IR projector

## Node parameters

These are set in `config/cameras.yaml`. The table below lists all available parameters with their default values (used as fallback if not specified in the YAML).

| Parameter       | Default   | Description                                    |
|-----------------|-----------|------------------------------------------------|
| `serial`        | `""`      | Device serial number (empty = first available) |
| `frame_id`      | node name | TF frame id stamped on published images        |
| `projector`     | `false`   | Enable IR projector                            |
| `color_width`   | `640`     | Color stream width in pixels                   |
| `color_height`  | `360`     | Color stream height in pixels                  |
| `color_fps`     | `15`      | Color stream frame rate                        |
| `infra2_width`  | `640`     | Infrared stream width in pixels                |
| `infra2_height` | `360`     | Infrared stream height in pixels               |
| `infra2_fps`    | `15`      | Infrared stream frame rate                     |

## Known issues

### `std::bad_array_new_length` on node startup

`librealsense2 2.58` ships with a bundled copy of FastDDS for its network streaming feature. ROS2 Humble's default middleware (`rmw_fastrtps_cpp`) shares the same `eprosima::fastdds::*` symbol namespace, causing a conflict at runtime that crashes the node before it initializes.

**Fix:** use CycloneDDS — already configured via `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in `~/.bashrc`.

Long-term fix: rebuild `librealsense2` without the network server to remove the FastDDS dependency:
```bash
cmake -DBUILD_RS_SERVER=false ...
```
