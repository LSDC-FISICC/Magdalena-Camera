# sysmon: finding what limits the camera pipeline

> On a Jetson, run `sudo bash tools/jetson_poe_setup.sh` once before measuring (UDP buffers, `jetson_clocks` at boot, camera network profile, ROS 2 environment); see the main README, *Jetson Orin Nano over PoE*.

`sysmon.py` tells whether a capture problem (dropped frames, low or irregular rate, freezes, crashes) comes from **memory**, the **GPU**, the **network** or **USB**. It runs unchanged on the Jetson Orin Nano and on x86 laptops with an NVIDIA GPU, so the same run can be compared across machines.

It needs only Python 3. `matplotlib` is optional (for the plot) and `ethtool` is optional (for the NIC driver counters).

> **Jetson note:** the Orin Nano has no separate VRAM. CPU and GPU share the same 8 GB, so on the Jetson "VRAM" and "RAM" are one resource: the RAM column.

## What it records

`record` logs one row per second:

| Group | Values | Source |
|---|---|---|
| Memory | RAM used/available, swap, `/dev/shm` | `/proc/meminfo` |
| GPU | memory used/total, load, temperature, power | `nvidia-smi` (laptop), `tegrastats` (Jetson) |
| Network, per interface (loopback included) | rx/tx Mb/s, dropped packets in kernel and driver, link speed, link up/down events | `/sys/class/net`, `ethtool -S` |
| Kernel network stack | UDP receive/send buffer overflows, UDP errors, failed IP reassemblies | `/proc/net/snmp` |
| CPU | total, busiest core, softirq (network interrupt) time of the busiest core | `/proc/stat` |
| Processes | memory (RSS) of the camera, inference and Modbus processes, plus the 3 largest; GPU memory per process | `/proc`, `nvidia-smi` / nvmap (Jetson, root only) |
| Kernel log | new kernel messages: USB resets, link down, OOM kills, GPU errors | `dmesg -W` or `journalctl -k` (root) |

`topics` subscribes to the camera topics and logs, per topic and second, the messages received, the longest gap between messages and the mean age of the frames (now − `header.stamp`).

`snapshot` saves the configuration that usually explains why one machine works and another does not: RAM, GPU, JetPack and power mode, `sysctl` network buffers, MTU, link speed, NIC driver and ring sizes, `usbfs_memory_mb`, USB tree, ROS 2 middleware variables, and whether `librealsense` links CUDA.

## Install

```bash
cd ~/magdalene_ws/src/Magdalena-Camera     # or wherever the repository is
git fetch origin && git checkout main && git pull   # claude/sysmon until the PR is merged
python3 tools/sysmon.py --help

sudo apt install ethtool python3-matplotlib    # optional, recommended
```

Use the same `RMW_IMPLEMENTATION` as the pipeline (`rmw_cyclonedds_cpp`, see the main README) in the terminal that runs `topics`.

## Quick start

```bash
# Once per machine: configuration snapshot, then compare two machines
python3 tools/sysmon.py snapshot -o tuf3070.json
python3 tools/sysmon.py compare tuf3070.json orin.json

# With the pipeline running: two terminals, 2-3 minutes, then Ctrl+C in both
sudo python3 tools/sysmon.py record -o runs/orin_poe_5cam      # terminal 1 (no ROS needed)
python3 tools/sysmon.py topics -o runs/orin_poe_5cam           # terminal 2 (ROS sourced, no sudo)

# Summary, likely causes and plot.png
python3 tools/sysmon.py report runs/orin_poe_5cam
```

`record` and `topics` write into the same folder. `sudo` for `record` is optional: without it the kernel log and the per-process GPU memory on the Jetson are skipped, and the script says so. Do not run `topics` with `sudo`, because `sudo` drops the ROS environment.

Both commands accept `--duration <seconds>` instead of Ctrl+C.

## Test plan

The goal is to find the camera count, link and machine at which the pipeline starts to fail, and which counter rises when it does.

### 1. Snapshots

On each machine (TUF Dash RTX 3070, RTX 2060 laptop, Orin Nano):

```bash
python3 tools/sysmon.py snapshot -o $(hostname).json
```

Copy the JSON files to one machine and compare each pair:

```bash
python3 tools/sysmon.py compare tuf3070.json rtx2060.json
python3 tools/sysmon.py compare tuf3070.json orin.json
```

Look first at `sysctl.net.core.rmem_max`, `interfaces.<nic>.mtu`, `interfaces.<nic>.speed_mbps`, `interfaces.<nic>.driver`, `usbfs_memory_mb`, `ros_env.RMW_IMPLEMENTATION`, `librealsense.links_cuda` and `ram_total_mb` / `gpu`. A difference in any of them can explain why the pipeline works on one machine and not on another.

On the Jetson, fix the power mode before testing, and keep it the same for every run:

```bash
sudo nvpmodel -q          # note the mode; MAXN for the tests
sudo jetson_clocks
```

### 2. Runs

Run each row for about 3 minutes, on each machine. Name the folder `runs/<machine>_<link>_<cameras>[_inference]`.

| Run | Cameras | Link | Inference node |
|---|---|---|---|
| A | 1 | PoE | off |
| B | 3 | PoE | off |
| C | 5 | PoE | off |
| D | 5 | PoE | on |
| E | 5 | USB | off |
| F | 5 | USB | on |

All five PoE cameras:

```bash
ros2 launch magdalene_camera_ros multi_camera.launch.py
```

One camera (repeat with other numbers for three cameras):

```bash
CFG=$(ros2 pkg prefix magdalene_camera_ros)/share/magdalene_camera_ros/config/cameras.yaml
ros2 run magdalene_camera_ros magdalene_camera_node --ros-args \
    -r __ns:=/camera_55 -r __node:=camera_55 --params-file $CFG
```

The namespace and node name must match a `/camera_XX/camera_XX` section of `cameras.yaml`, or the node gets no serial and opens any camera.

The USB cameras of the field pipeline (Magdalena-CV-Pipeline) are monitored the same way. If their topics are not called `.../color/image_raw`, pass a regular expression:

```bash
python3 tools/sysmon.py topics -o runs/orin_usb_5cam --topics 'camera[0-9]+.*color/image_raw$'
```

### 3. What to send back

For each run, the folder (`snapshot.json`, `sys.csv`, `procs.csv`, `kernel.csv`, `topics.csv`, `summary.txt`, `plot.png`) plus the snapshot JSON of each machine:

```bash
zip -r sysmon_runs.zip runs/ *.json
```

## Reading the report

`report` prints the summary, writes it to `summary.txt` and draws `plot.png` (memory, network, drops, CPU/GPU, topic rates over time). Its last section, **possible causes**, lists what the counters point to. The table below is the same logic.

| What rises | Where the problem is | What to try |
|---|---|---|
| RAM near the total, swap growing, or "out of memory" in the kernel messages | Memory (on the Jetson this is also the GPU memory) | Fewer processes or streams; check `procs.csv` for a process that grows |
| GPU memory near the total (laptop) | VRAM | Fewer engines or a smaller batch; check GPU MB per process |
| UDP receive-buffer overflows | Kernel socket buffers too small for 0.69 MB frames | `sysctl` settings in the main README, *Low FPS* section |
| IP reassembly failures | Fragments of large UDP packets lost | Same `sysctl` settings (`ipfrag_high_thresh`), MTU 9000 end to end |
| Drops on the camera interface (driver count in parentheses) | NIC ring buffer overflowing | `ethtool -G <nic> rx <max>`; multi-queue NIC |
| softirq on one core above ~50 % | Network interrupts saturate one core (single-queue NICs, e.g. Realtek r8169) | Fewer streams per NIC, second NIC, multi-queue NIC |
| Link speed below 1000 Mb/s, or link up/down events | Cable, switch port, Extend Mode or PoE budget | See *Camera publishes but image stream freezes* in the main README |
| Camera interface above 70 % of the link speed | Link saturated | Turn off unused streams (infra2 is ~25 % of the traffic); lower resolution |
| High Mb/s on `lo` | Raw images copied between ROS processes over loopback | Composable nodes with intra-process communication |
| USB reset/disconnect messages | USB cameras: cable, hub power, bandwidth | Powered hub, one camera per controller, `usbfs_memory_mb=1000` |
| Topic below 13.5 Hz with none of the above | CPU-bound stage or middleware | Check the busiest core and the process list |

The expected wire bandwidth at the default profile is in the main README: about 111 Mb/s per camera with color and infra2, about 560 Mb/s for five.

To check a fix, change one setting, repeat the same run and compare the two `summary.txt` files.

## Notes

- `topics` subscribes **reliable** by default, like the pipeline's own subscribers. With large raw images a best-effort subscriber loses frames inside DDS even when nothing else is wrong: in a test with two simulated 640×360 cameras at 15 Hz, best effort received 4-10 Hz and reliable 15 Hz. Use `--best-effort` only to reproduce a best-effort consumer.
- `topics` adds one more subscriber of every monitored stream. The default monitors only `color/image_raw` (and any `Trigger` topic). Add infra2 with `--topics 'image_raw$'` if needed.
- Frame age is meaningful only when the publisher stamps with the clock of the machine running `topics`, which is the case for this node.
- Process memory is matched by command-line substrings (`--procs`, default `magdalene,camera,inference,modbus,realsense,yolo,trtexec,component_container`).
- On a laptop, `nvidia-smi` per-process memory covers compute processes only (CUDA, TensorRT), not the desktop.
