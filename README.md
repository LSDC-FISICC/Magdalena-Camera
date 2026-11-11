# Magdalene Camera

ROS 2 wrapper and SDK abstraction for Intel RealSense D555 PoE cameras. Designed for multi-camera agricultural perception in the Magdalena sugarcane fertirrigation project at LSDC-FISICC, Universidad Galileo.

## Architecture

The D555 cameras connect via Power-over-Ethernet. This repository contains two packages that work together:

- **`magdalene_camera_sdk`**: C++17 library wrapping `librealsense2`. Pure hardware abstraction with no ROS 2 dependencies; can be linked into non-ROS applications. Handles network connection, profile negotiation, and exposes frames as `cv::Mat`.
- **`magdalene_camera_ros`**: ROS 2 node using the SDK as backend. Reads parameters per camera, instantiates one `magdalene_camera` object per device, publishes as `sensor_msgs/msg/Image` with the default `image_transport` QoS (reliable, keep last 10).

```
librealsense2
     ↓
magdalene_camera_sdk    (camera connection, no ROS 2)
     ↓
magdalene_camera_ros    (ROS 2 bridge: SDK to topics)
     ↓
/camera_55/color/image_raw
/camera_56/color/image_raw
…                             (one per camera, camera_55 … camera_59)
/camera_55/infra2/image_raw   (only with infra2_enabled: true)
```

`colcon` builds `magdalene_camera_sdk` first (declared dependency), then `magdalene_camera_ros`.

### Why SDK over DDS subscribe/republish

An alternative architecture would be to subscribe to topics published by a remote `realsense2_camera` node and republish them locally. This was tested and discarded for three reasons:

1. **No FPS and resolution control.** The D555 firmware publishes via DDS at 896×504 @ 30 fps by default. The `realsense2_camera` wrapper only subscribes to these streams and cannot renegotiate the profile with the camera firmware. The SDK requests the profile directly through `color_profile` / `infra2_profile` (new default: 640×360 @ 15 fps), reducing wire bandwidth approximately 4×.

2. **Packet loss on streams.** At the firmware default of 896×504 @ 30 fps, each color frame fragments into multiple UDP datagrams at MTU 9000. With multiple cameras the link saturates, fragments drop, and Fast-DDS reports `sequence size exceeds remaining buffer` errors on partial reassembly. The SDK profile (640×360 @ 15 fps) cuts per-camera bandwidth approximately 4×, eliminating saturation; bypassing the wrapper's republish queue also removes it as a backpressure point.

3. **Double serialization.** In the wrapper architecture, frames are deserialized from the camera's DDS stream and re-serialized as `sensor_msgs/Image` before downstream subscribers see them. The SDK reads librealsense's internal buffers directly, eliminating the wrapper's intermediate (de)serialization round and reducing CPU usage and latency.

## Package structure

```
magdalene_camera/
├── magdalene_camera_sdk/             # Hardware abstraction library (ament_cmake)
│   ├── include/
│   │   └── magdalene_camera.hpp      # magdalene_camera class declaration
│   └── src/
│       ├── magdalene_camera_wrapper.cpp  # librealsense2 wrapper implementation
│       └── main.cpp                  # Standalone OpenCV demo (no ROS 2)
│
└── magdalene_camera_ros/             # ROS 2 node
    ├── include/magdalene_camera_ros/
    │   └── magdalene_camera_node.hpp # MagdaleneCameraNode declaration
    ├── src/
    │   ├── magdalene_camera_node.cpp # MagdaleneCameraNode implementation
    │   └── main.cpp                  # main()
    ├── config/
    │   └── cameras.yaml              # All camera parameters: edit here to configure
    └── launch/
        └── multi_camera.launch.py    # Reads cameras.yaml, launches one node per camera
```

## Hardware requirements

| Component | Requirement | Notes |
|---|---|---|
| PoE switch | IEEE 802.3at (PoE+), 30 W per port | D555 negotiates 30 W class. Total budget must cover all cameras. |
| Switch features | Gigabit, jumbo frames (9000 B) | Unmanaged consumer switches often silently drop jumbo frames. |
| Switch mode | Extend Mode **OFF** | If enabled, port speed drops to 10 Mbps. |
| Host NIC | Gigabit minimum, MTU 9000 capable | D555 firmware currently requires MTU 9000. |
| Host NIC (4+ cameras) | Multi-queue or 2.5 GbE | Single-queue 1 GbE saturates softirq processing at ~700 Mbps. |
| Cable | Cat6 or above with RJ45 | Cat5e works but Cat6 reduces retransmits at MTU 9000. |

### PoE budget by camera count

| Cameras | PoE budget needed | Example switch |
|---|---|---|
| 1–2 | 60 W | TP-Link LS108GP (62 W) |
| 3–4 | 120 W | TP-Link TL-SG1210MPE (123 W) |
| 5–8 | 150–240 W | TP-Link TL-SG1218MP (250 W) |

Common consumer 8-port unmanaged switches (TP-Link LS108GP at 62 W, Netgear GS308PP at 83 W) are insufficient for 5 cameras.

### Per-camera bandwidth at default SDK profile (640×360 @ 15 fps)

| Stream | Format | Size/frame | Rate |
|---|---|---|---|
| Color | RGB8 | 0.69 MB | 10.4 MB/s |
| Infrared 2 | Y8 | 0.23 MB | 3.5 MB/s |
| **Total per camera** | | | **~13.9 MB/s** |

By default only the color stream is enabled (`infra2_enabled: false`): about 10.4 MB/s per camera and ~52 MB/s = 420 Mbps for 5 cameras. With infrared enabled: ~70 MB/s = 560 Mbps. Both fit within Gigabit headroom on multi-queue NICs.

## Software prerequisites

| Dependency | Version | Notes |
|---|---|---|
| OS | Ubuntu 22.04 LTS | required for ROS 2 Humble |
| ROS 2 | Humble | |
| CMake | >= 3.16 | |
| librealsense2 | **2.58.2** | installed to `/usr/local`, built with `-DBUILD_WITH_DDS=ON` |
| OpenCV | 4.5.4 | |
| `ros-humble-cv-bridge` | | |
| `ros-humble-image-transport` | | |
| `ros-humble-rmw-cyclonedds-cpp` | | required, see [Known issues](#known-issues) |
| D555 firmware | 7.58.38448.10612 tested | needs librealsense ≥ 2.58.2; with 2.57 the cameras are listed but cannot be opened. Check with `rs-enumerate-devices --eth -s` |

Refer to the [librealsense release notes](https://github.com/realsenseai/librealsense/wiki/Release-Notes) for the latest firmware compatible with SDK 2.58.

## Network setup

Each D555 requires a unique static IP on the same subnet as the host. IPs are assigned once with `rs-dds-config` while the camera is connected via USB-C, then persist in firmware.

### Assigning camera IPs

```bash
# Camera 1
rs-dds-config --serial-number 419222301921 \
    --ip 192.168.11.55 \
    --mask 255.255.255.0 \
    --gateway 192.168.11.1 \
    --priority eth-first

# Camera 2
rs-dds-config --serial-number 353522302286 \
    --ip 192.168.11.56 \
    --mask 255.255.255.0 \
    --gateway 192.168.11.1 \
    --priority eth-first

# Camera 3
rs-dds-config --serial-number 409122301544 \
    --ip 192.168.11.57 \
    --mask 255.255.255.0 \
    --gateway 192.168.11.1 \
    --priority eth-first

# Camera 4
rs-dds-config --serial-number 408222300679 \
    --ip 192.168.11.58 \
    --mask 255.255.255.0 \
    --gateway 192.168.11.1 \
    --priority eth-first

# Camera 5
rs-dds-config --serial-number 353522301659 \
    --ip 192.168.11.59 \
    --mask 255.255.255.0 \
    --gateway 192.168.11.1 \
    --priority eth-first
```

`--priority eth-first` makes the camera publish via DDS on power-up without requiring USB.

### Host NIC configuration

```bash
# Static IP on the camera subnet, MTU 9000 (jumbo frames mandatory)
sudo ip addr add 192.168.11.1/24 dev eno2
sudo ip link set eno2 mtu 9000
sudo ip link set eno2 up
```

Make this persistent via `netplan` or `NetworkManager` for production.

### Verify discovery

```bash
rs-enumerate-devices --eth
```

Both cameras should appear with their serial numbers and IPs. If empty, see [Troubleshooting](#troubleshooting).

## Environment setup

Add to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is **required**. `librealsense2 2.58` bundles its own Fast-DDS internally, which conflicts with ROS 2's default `rmw_fastrtps_cpp`. CycloneDDS avoids this conflict entirely.

## Build

```bash
cd ~/magdalene_ws
colcon build --symlink-install
source install/setup.bash
```

To rebuild only these packages:

```bash
colcon build --symlink-install \
    --packages-select magdalene_camera_sdk magdalene_camera_ros
```

## Camera configuration

All parameters live in `config/cameras.yaml`. Common parameters are declared once under `/**` and apply to all cameras. Each camera section only needs the values that are unique to it (`serial` and `frame_id`):

```yaml
/**:
  ros__parameters:
    projector:      false
    infra2_enabled: false
    color_width:   640
    color_height:  360
    color_fps:     15
    color_format:  "RGB8"
    infra2_width:  640
    infra2_height: 360
    infra2_fps:    15
    infra2_format: "Y8"

/camera_55/camera_55:
  ros__parameters:
    serial:   "419222301921"   # 192.168.11.55
    frame_id: "camera_55"

/camera_56/camera_56:
  ros__parameters:
    serial:   "353522302286"   # 192.168.11.56
    frame_id: "camera_56"

/camera_57/camera_57:
  ros__parameters:
    serial:   "409122301544"   # 192.168.11.57
    frame_id: "camera_57"

/camera_58/camera_58:
  ros__parameters:
    serial:   "408222300679"   # 192.168.11.58
    frame_id: "camera_58"

/camera_59/camera_59:
  ros__parameters:
    serial:   "353522301659"   # 192.168.11.59
    frame_id: "camera_59"
```

To add a camera, append a new `/name/name` section. The launch file auto-discovers all entries and starts one node per camera.

### Valid profile combinations

Color and IR2 must use one of the profiles supported by the D555 firmware (datasheet v1.1):

| Resolution | Supported FPS |
|---|---|
| 1280×720 | 5, 15, 30 |
| 896×504 | 5, 15, 30, 60 |
| 640×360 | 5, 15, 30, 60 (default) |
| 448×252 | 5, 15, 30, 60 |

Requesting an unsupported combination raises an exception at pipeline start.

## Running

```bash
ros2 launch magdalene_camera_ros multi_camera.launch.py
```

Verify all cameras are publishing:

```bash
ros2 topic list | grep image_raw
ros2 topic hz /camera_55/color/image_raw   # should report ~15 fps
ros2 topic bw /camera_55/color/image_raw   # should report ~10.4 MB/s
```

`ros2 topic hz` may under-report for image topics due to CPU overhead in the CLI subscriber. `ros2 topic bw` is more accurate; divide by the per-frame size to get the true rate.

## SDK standalone test

`main.cpp` opens one camera and displays the color stream with OpenCV (and the infrared stream with `--infra2`). No ROS 2 required. Accepts the serial number as an optional argument:

```bash
source ~/magdalene_ws/install/setup.bash

# specific camera
ros2 run magdalene_camera_sdk magdalene_camera_demo 419222301921

# first available camera
ros2 run magdalene_camera_sdk magdalene_camera_demo

# color and infrared
ros2 run magdalene_camera_sdk magdalene_camera_demo 419222301921 --infra2
```

Controls:
- `q` or `Esc`: quit
- `p`: toggle IR projector (with `--infra2`)

## Node parameters

These are set in `config/cameras.yaml`. The table below lists all available parameters with their default values (used as fallback if not specified in the YAML).

| Parameter | Default | Description |
|---|---|---|
| `serial` | `""` | Device serial number (empty = first available) |
| `frame_id` | node name | TF frame id stamped on published images |
| `projector` | `false` | Enable IR projector |
| `infra2_enabled` | `false` | Stream and publish the right infrared image; when `false` only color is captured and the `infra2/image_raw` topic is not created |
| `color_width` | `640` | Color stream width in pixels |
| `color_height` | `360` | Color stream height in pixels |
| `color_fps` | `15` | Color stream frame rate |
| `color_format` | `RGB8` | Color format: `RGB8`, `BGR8`, `YUYV` |
| `infra2_width` | `640` | Infrared stream width in pixels |
| `infra2_height` | `360` | Infrared stream height in pixels |
| `infra2_fps` | `15` | Infrared stream frame rate |
| `infra2_format` | `Y8` | Infrared format (only `Y8` supported) |

## Jetson Orin Nano over PoE

Tested on a Jetson Orin Nano Developer Kit (8 GB, L4T 36.5 / JetPack 6.2, 15 W, `jetson_clocks`) with five D555 on a TP-Link PoE switch: all five color streams at 15.0 Hz, no gap longer than one frame (70 ms), 282 Mb/s on the 1 GbE port, no packet or UDP buffer drops, 11 % CPU. Measured with [`tools/sysmon.py`](tools/README.md).

**This holds for about 20 minutes, not indefinitely** (see [*Two cameras fail together after ~20 min*](#two-cameras-fail-together-after-20-min)). Five PoE cameras through a gigabit switch into the Jetson's single 1 GbE port is not reliable for continuous operation; the field system uses USB 3.2.

### One-time setup

1. **librealsense 2.58.2 with DDS**, built on the Jetson into `/usr/local` (about an hour). The ROS 2 package of librealsense (`/opt/ros/humble/lib/.../librealsense2.so.2.57`) has no DDS support and cannot see PoE cameras; it stays installed for USB cameras.
   ```bash
   sudo apt install -y git cmake build-essential pkg-config libssl-dev libusb-1.0-0-dev libudev-dev
   git clone --depth 1 --branch v2.58.2 https://github.com/realsenseai/librealsense.git ~/librealsense-2.58.2
   cd ~/librealsense-2.58.2 && mkdir -p build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
     -DBUILD_WITH_DDS=ON -DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_CUDA=OFF \
     -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=ON -DBUILD_GRAPHICAL_EXAMPLES=OFF \
     -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_UNIT_TESTS=OFF
   make -j4 && sudo make install && sudo ldconfig
   ```
   `BUILD_EXAMPLES=ON` without `BUILD_GRAPHICAL_EXAMPLES=ON` fails in 2.58 (`OpenGL::GL` not found).
2. **CycloneDDS**: `sudo apt install ros-humble-rmw-cyclonedds-cpp`.
3. **Persistent settings**: `sudo bash tools/jetson_poe_setup.sh` sets the UDP buffers (`/etc/sysctl.d/60-dds.conf`), runs `jetson_clocks` at every boot, creates the NetworkManager profile `poe-camaras` (`192.168.11.1/24`, MTU 9000, connected at boot) and adds ROS 2 Humble + CycloneDDS + this workspace to `~/.bashrc`. `bash tools/jetson_poe_setup.sh --check` reports the current state. The profile takes the wired port; for internet on the same port, `sudo nmcli con up <dhcp-profile>`, and back with `sudo nmcli con up poe-camaras`.
4. **Build this workspace against `/usr/local`**:
   ```bash
   colcon build --symlink-install --cmake-args -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
   ```
   The node then links `librealsense2.so.2.58` from `/usr/local/lib`, and the binaries carry an RPATH to that directory. This matters since `ros-humble-librealsense2` 2.58.4 (no DDS) uses the same soname: the ROS environment puts it first in `LD_LIBRARY_PATH`, and without the RPATH the node would load it and find no PoE camera. Check with the ROS environment sourced:
   ```bash
   ldd ~/magdalene_ws/install/magdalene_camera_ros/lib/magdalene_camera_ros/magdalene_camera_node | grep realsense   # /usr/local/lib/...
   ```
   The librealsense tools are not covered: run them as `env -u LD_LIBRARY_PATH /usr/local/bin/rs-enumerate-devices --eth`.
5. **PoE switch**: turn off the port *recovery* (auto power-cycle) and *extend* modes of TP-Link switches; with recovery on, two of five cameras stopped for 10–35 s at a time.

### What each item fixed

| Symptom | Cause | Fix |
|---|---|---|
| `stream 'ObjectDetection' is of unknown type`; `No device connected` | firmware 7.58 needs librealsense ≥ 2.58.2 | librealsense 2.58.2 |
| `rs-enumerate-devices` lists nothing, `--eth` lists the cameras | DDS discovery is off by default in 2.58 | the SDK enables it in its context |
| `No device connected` right after start | Ethernet devices appear a few seconds after the context is created | the SDK retries for 15 s |
| Some cameras deliver no frames; `sysmon` reports UDP receive-buffer overflows | default `net.core.rmem_max` (212 kB) | `60-dds.conf` |
| Cameras stop for 10–35 s at a time | switch PoE recovery mode | recovery off |
| CPU at 730 MHz after a reboot | `jetson_clocks` is not persistent | `jetson-clocks.service` |
| After an `apt upgrade`, every node fails with `No device connected` and `rs-enumerate-devices --eth` finds nothing, while the cameras answer ping | `ros-humble-librealsense2` 2.58.4 (no DDS) has the same soname as the DDS build in `/usr/local` and comes first in `LD_LIBRARY_PATH` | rebuild this workspace (RPATH to `/usr/local/lib`); run the tools with `env -u LD_LIBRARY_PATH` |

### Two cameras fail together after ~20 min

In 30- and 60-minute runs on the TP-Link switch, after 20–25 min two cameras slowed down over 2–3 minutes and then delivered almost no frames, while still answering ping. Restarting the node of **either** camera of the pair brought the **other** one back to 15 Hz within one second, before the restarted camera streamed again; the restarted camera then also ran at 15 Hz. Later another pair failed the same way.

| Run (60 min, `sysmon`) | |
|---|---|
| 22–25 min | cameras .57 and .59 fall to ~0 Hz; .55, .56, .58 stay at 15 Hz |
| 25.3 min | node of .59 restarted: .57 back to 15 Hz in 1 s, .59 after 6 s |
| 41–44 min | cameras .55 and .57 fall; node of .55 restarted: .57 back in 1 s |
| whole run | 0 UDP buffer overflows, 0 IP reassembly failures, 0 NIC/driver drops, Tj 60 °C, ping never lost |

During each failure the Jetson port still received 274–278 Mb/s instead of 282: the cameras kept sending, and the ~5–8 Mb/s missing fit a few percent of the two cameras' packets being lost before the Jetson. One 640×360 frame is ~460 KB, about 50 jumbo packets, so losing ~5 % of a camera's packets loses nearly every frame.

What this points to: the two failing streams interfere with each other on a path they share, and the Jetson counts no loss, which leaves the switch port towards the Jetson. Each camera sends a frame as a ~3.7 ms burst at 1 Gb/s; the cameras are not synchronized, so their bursts drift relative to each other. When two bursts start to overlap, the switch must buffer up to half a frame for the single 1 GbE egress port, and a small unmanaged switch drops the excess. Restarting either camera shifts its phase and ends the overlap until the drift brings two streams together again. A hot switch would not recover in one second, and a fault in one node process could not stop a camera served by another process.

On a Cisco Catalyst 3850, two of five cameras also lost most frames; its default egress buffers are small (check `show interfaces <port> | include output drops`; `qos queue-softmax-multiplier 1200` enlarges them).

Options, from the most to the least robust:
- **Multi-gigabit host link**: a 2.5 or 10 GbE NIC on the Jetson (e.g. an M.2 Key M card; check that L4T has its driver) and a switch with a multi-gigabit uplink, as the D555 documentation recommends for several cameras. The egress then drains faster than two 1 Gb/s bursts arrive.
- **A switch with deep egress buffers**, or two Jetson ports with at most 2–3 cameras each.
- **Restarting the stream automatically** when a camera's rate falls (not implemented): it hides the symptom at the cost of a 1–6 s gap for that camera.
- **USB 3.2**, as in the field system.

To check a run, `sudo python3 tools/sysmon.py record` counts the traffic of each camera address: a failing camera whose traffic drops by a few percent while the Jetson counts no loss fits this explanation.

## Troubleshooting

To find whether a problem comes from memory, the GPU, the network or USB, record a run with [`tools/sysmon.py`](tools/README.md): it logs every drop counter between the cable and the ROS topic, memory and GPU use, and the real rate of each camera topic, and names the likely cause.

### Cameras not discovered (`rs-enumerate-devices --eth` returns empty)

1. Verify host NIC MTU is **9000**:
   ```bash
   ip link show eno2 | head -1
   ```
   D555 firmware currently requires MTU 9000 for DDS discovery. A future firmware will support MTU 1500.

2. Verify Layer 3 reachability:
   ```bash
   ping 192.168.11.55
   ping 192.168.11.56
   ping 192.168.11.57
   ping 192.168.11.58
   ping 192.168.11.59
   ```

3. Verify the switch is **not** in Extend Mode. Port LEDs should indicate Gigabit, not 10/100 Mbps.

4. Verify PoE budget: `total camera count × 30 W` must not exceed switch budget. Underpowered cameras may boot but fail DDS handshake.

5. Verify librealsense2 was built with `-DBUILD_WITH_DDS=ON`:
   ```bash
   ldd $(which rs-enumerate-devices) | grep fastdds
   ```
   If empty, rebuild librealsense2 with DDS support.

### Low FPS or `sequence size exceeds remaining buffer` errors

The host NIC is dropping UDP fragments. Diagnose:

```bash
ip -s link show eno2 | grep -A1 RX
# Watch the "missed" or "dropped" counter. If it grows during streaming,
# NIC ring buffer is overflowing.
```

Mitigations in order of effort:

1. Confirm the actual published profile matches the configured one:
   ```bash
   ros2 topic bw /camera_55/color/image_raw
   # Expected for 640×360 RGB8 @ 15 fps: ~10.4 MB/s
   ```

2. Increase kernel UDP buffers:
   ```bash
   sudo tee /etc/sysctl.d/60-dds.conf > /dev/null <<'EOF'
   net.core.rmem_max=2147483647
   net.core.rmem_default=134217728
   net.core.wmem_max=2147483647
   net.core.wmem_default=134217728
   net.ipv4.ipfrag_time=3
   net.ipv4.ipfrag_high_thresh=134217728
   EOF
   sudo sysctl --system
   ```

3. For NICs with multi-queue support, expand RX queues:
   ```bash
   sudo ethtool -l eno2          # check max queues
   sudo ethtool -L eno2 combined 4
   ```

4. Increase NIC ring buffer if supported:
   ```bash
   sudo ethtool -g eno2          # check max ring size
   sudo ethtool -G eno2 rx 4096
   ```

5. For single-queue Realtek NICs (common on consumer laptops) bandwidth ceiling is ~700 Mbps before softirq saturation. Reduce per-camera profile, add a second NIC, or upgrade to multi-queue hardware.

### `ros2 topic hz` reports half the expected rate

The CLI tool deserializes every message in Python and falls behind for large image payloads. Use `ros2 topic bw` instead and compute the rate from message size, or [`tools/sysmon.py topics`](tools/README.md), a subscriber that counts messages without deserializing them. Subscribe reliable, like the publisher: a best-effort subscriber of large raw images can lose frames inside DDS even when nothing else is wrong.

### `std::bad_array_new_length` on node startup

See [Known issues](#known-issues).

### Camera publishes but image stream freezes after a few seconds

On TP-Link switches, first turn off the port recovery (auto power-cycle) mode: it restarts cameras that it judges unresponsive. Otherwise, the likely cause is PoE undervoltage. Check:

```bash
sudo ethtool eno2 | grep -E "Speed|Duplex"
# Should report "Speed: 1000Mb/s" and "Duplex: Full"
```

If Speed drops to 100 Mb/s during streaming, the camera is renegotiating link due to insufficient power. Verify switch PoE budget or move the camera to a different port.

## Known issues

### `std::bad_array_new_length` on node startup

`librealsense2 2.58` ships with a bundled copy of Fast-DDS for its network streaming feature. ROS 2 Humble's default middleware (`rmw_fastrtps_cpp`) shares the same `eprosima::fastdds::*` symbol namespace, causing a conflict at runtime that crashes the node before it initializes.

**Fix:** use CycloneDDS, already configured via `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in `~/.bashrc`.

Long-term fix: rebuild `librealsense2` without the network server to remove the Fast-DDS dependency:
```bash
cmake -DBUILD_RS_SERVER=false ...
```

### Bandwidth ceiling on single-queue NICs

Consumer NICs based on Realtek r8169 (common in laptops including ASUS TUF) are single-queue and saturate softirq processing at roughly 700 Mbps. For deployments of 4+ cameras consider:

- Intel I225/I226 onboard NIC (2.5 GbE, multi-queue, well-supported by `igc` driver)
- USB-C 2.5 GbE dongle based on Realtek RTL8156 as secondary NIC
- Intel I350-T2 PCIe NIC (industrial-grade, 8 queues per port, `igb` driver)

### `realsense2_camera` wrapper coexistence

If the standard `realsense2_camera` wrapper runs in parallel with this node and subscribes to the camera's raw native topics (~30 fps), aggregate wire bandwidth from the camera doubles relative to running only the SDK node. Either disable the wrapper entirely or set `enable_color: false` and `enable_infra2: false` in its launch.

### MTU 1500 hosts

D555 firmware as of 2025 only supports MTU 9000. Hosts behind switches that do not pass jumbo frames will fail discovery. Intel has indicated a future firmware will add MTU 1500 support at the cost of reduced throughput. Track progress on the [librealsense issue tracker](https://github.com/realsenseai/librealsense/issues).

## License

Apache License 2.0. See `LICENSE` file for details.

## Citation

If you use this software in academic work, please cite:

```bibtex
@software{magdalene_camera,
  author  = {Fajardo, Julio and LSDC-FISICC},
  title   = {Magdalene Camera: ROS 2 SDK wrapper for Intel RealSense D555 PoE},
  year    = {2026},
  url     = {https://github.com/LSDC-FISICC/magdalene_camera},
  institution = {Laboratorio de Sistemas Dinámicos y Control,
                 Universidad Galileo, Guatemala}
}
```

## Contact

Laboratorio de Sistemas Dinámicos y Control (LSDC-FISICC)
Universidad Galileo, Guatemala
[lsdc@galileo.edu](mailto:lsdc@galileo.edu)