#!/usr/bin/env python3
"""
sysmon.py
=========
System monitor for the Magdalena camera pipeline. Runs unchanged on a Jetson
(reads tegrastats) and on an x86 laptop with an NVIDIA GPU (reads nvidia-smi),
with the Python standard library only (matplotlib optional, for the plot).

It answers "is it memory, the GPU, the network or USB?" by logging, once per
second: RAM/swap and /dev/shm, GPU memory and load, per-interface traffic and
every drop counter between the cable and the application (NIC ring, driver,
kernel UDP socket buffers, IP reassembly), per-core CPU and softirq load, the
memory of the pipeline processes, kernel messages (USB resets, link down, OOM)
and, optionally, the real rate of the ROS 2 camera topics.

On a Jetson the GPU has no memory of its own: CPU and GPU share the same RAM,
so "VRAM" there is the RAM column.

Usage
-----
  python3 sysmon.py snapshot -o laptop3070.json      # configuration, once
  python3 sysmon.py compare laptop3070.json orin.json
  sudo python3 sysmon.py record -o run1               # terminal 1, until Ctrl+C
  python3 sysmon.py topics -o run1                    # terminal 2, ROS sourced
  python3 sysmon.py report run1                       # summary + plot

sudo is optional: without it the kernel log and the per-process GPU memory on
the Jetson are skipped.

Author: Julio Fajardo, PhD
"""

import argparse
import csv
import importlib
import json
import os
import platform
import re
import shutil
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

PROC_PATTERNS = "magdalene,camera,inference,modbus,realsense,yolo,trtexec,component_container"
TOPIC_REGEX = r"color/image_raw$|[Tt]rigger"
SYSCTLS = ["net.core.rmem_max", "net.core.rmem_default", "net.core.wmem_max", "net.core.wmem_default",
           "net.core.netdev_max_backlog", "net.ipv4.ipfrag_high_thresh", "net.ipv4.ipfrag_time",
           "net.ipv4.udp_mem", "vm.swappiness"]
ROS_ENV = ["ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY",
           "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE", "CYCLONEDDS_URI"]


# ---------------------------------------------------------------- helpers ---

def run(cmd, timeout=5):
    """Runs a command and returns its stdout, or None if it is missing or fails."""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout if r.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def read(path, default=None):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return default


def is_jetson():
    return Path("/etc/nv_tegra_release").exists() or shutil.which("tegrastats") is not None


def interfaces():
    """All network interfaces that exist, loopback included (DDS between local processes uses it)."""
    return sorted(p.name for p in Path("/sys/class/net").iterdir())


def meminfo():
    m = {}
    for line in read("/proc/meminfo", "").splitlines():
        k, v = line.split(":", 1)
        m[k] = int(v.split()[0]) / 1024  # MB
    return m


def cpu_times():
    """Per-CPU (total, idle, softirq) jiffies from /proc/stat."""
    out = {}
    for line in read("/proc/stat", "").splitlines():
        if line.startswith("cpu"):
            f = line.split()
            v = [int(x) for x in f[1:9]]
            out[f[0]] = (sum(v), v[3] + v[4], v[6])
    return out


def snmp():
    """Kernel IP/UDP counters from /proc/net/snmp (header and value lines come in pairs)."""
    out = {}
    lines = read("/proc/net/snmp", "").splitlines()
    for h, v in zip(lines[::2], lines[1::2]):
        p, keys = h.split(":", 1)
        _, vals = v.split(":", 1)
        for k, x in zip(keys.split(), vals.split()):
            out[f"{p}.{k}"] = int(x)
    return out


NIC_DROP_FILES = ["rx_dropped", "rx_missed_errors", "rx_fifo_errors", "rx_over_errors", "rx_errors"]


def nic_counters(iface):
    s = Path("/sys/class/net") / iface / "statistics"
    c = {k: int(read(s / k, "0")) for k in ["rx_bytes", "tx_bytes"] + NIC_DROP_FILES}
    c["carrier_changes"] = int(read(Path("/sys/class/net") / iface / "carrier_changes", "0") or 0)
    return c


def ethtool_drops(iface):
    """Sum of the driver's own receive drop counters (ethtool -S), or None if unavailable."""
    out = run(["ethtool", "-S", iface], timeout=2)
    if out is None:
        return None
    total = 0
    for line in out.splitlines():
        if ":" not in line:
            continue
        k, v = (s.strip() for s in line.split(":", 1))
        if k.startswith("tx") or not re.search(r"drop|miss|discard|fifo|no_?buf|overrun|err", k):
            continue
        try:
            total += int(v)
        except ValueError:
            pass
    return total


def link_speed(iface):
    try:
        return int(read(Path("/sys/class/net") / iface / "speed", "-1"))
    except ValueError:
        return -1


def shm_used_mb():
    try:
        st = os.statvfs("/dev/shm")
        return (st.f_blocks - st.f_bfree) * st.f_frsize / 2**20, st.f_blocks * st.f_frsize / 2**20
    except OSError:
        return None, None


def proc_name(pid):
    """Readable process name: executable, python script, ROS node name if given."""
    argv = (read(f"/proc/{pid}/cmdline", "") or "").split("\0")
    argv = [a for a in argv if a]
    if not argv:
        return read(f"/proc/{pid}/comm", "?")
    name = os.path.basename(argv[0])
    if name.startswith("python") and len(argv) > 1:
        name += " " + os.path.basename(argv[1])
    node = next((a.split(":=", 1)[1] for a in argv if a.startswith("__node:=")), None)
    return f"{name} [{node}]" if node else name


def processes(patterns, top=3):
    """RSS of the processes whose command line matches a pattern, plus the largest few."""
    rows = []
    for p in Path("/proc").iterdir():
        if not p.name.isdigit():
            continue
        try:
            status = (p / "status").read_text()
            cmd = (p / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
        except OSError:
            continue
        m = re.search(r"VmRSS:\s+(\d+)", status)
        if not m or not cmd:
            continue
        rows.append((int(p.name), int(m.group(1)) / 1024, cmd))
    me = os.getpid()
    keep = {pid for pid, _, cmd in rows if pid != me and any(s in cmd for s in patterns)}
    keep |= {pid for pid, _, _ in sorted(rows, key=lambda r: -r[1])[:top]}
    return [(pid, rss) for pid, rss, _ in rows if pid in keep]


# -------------------------------------------------------------- GPU sources ---

class NvidiaSmi:
    """Discrete GPU (laptop/desktop): global and per-process memory from nvidia-smi."""
    available = staticmethod(lambda: shutil.which("nvidia-smi") is not None)

    def sample(self):
        out = run(["nvidia-smi", "--query-gpu=memory.used,memory.total,utilization.gpu,temperature.gpu,power.draw",
                   "--format=csv,noheader,nounits"])
        g = {}
        if out:
            f = [x.strip() for x in out.splitlines()[0].split(",")]
            for k, v in zip(["gpu_mem_used_mb", "gpu_mem_total_mb", "gpu_util_pct", "gpu_temp_c", "power_w"], f):
                try:
                    g[k] = float(v)
                except ValueError:
                    pass
        return g

    def per_process(self):
        out = run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"]) or ""
        res = {}
        for line in out.splitlines():
            try:
                pid, mb = (int(x.strip()) for x in line.split(","))
                res[pid] = res.get(pid, 0) + mb
            except ValueError:
                pass
        return res

    def close(self):
        pass


TEGRA_RE = {
    "gpu_util_pct": re.compile(r"GR3D_FREQ (\d+)%"),
    "emc_util_pct": re.compile(r"EMC_FREQ (\d+)%"),
    "gpu_temp_c": re.compile(r"gpu@([\d.]+)C"),
    "tj_temp_c": re.compile(r"tj@([\d.]+)C"),
    "power_w": re.compile(r"VDD_IN (\d+)mW"),
}


def parse_tegrastats(line):
    g = {}
    for k, rx in TEGRA_RE.items():
        m = rx.search(line)
        if m:
            g[k] = float(m.group(1)) / (1000 if k == "power_w" else 1)
    return g


class Tegrastats:
    """Jetson: GPU load, temperatures and board power from tegrastats; memory is the shared RAM."""
    available = staticmethod(is_jetson)

    def __init__(self):
        self.last = {}
        self.proc = subprocess.Popen(["tegrastats", "--interval", "1000"], stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, text=True)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            self.last = parse_tegrastats(line)

    def sample(self):
        return dict(self.last)

    def per_process(self):
        """GPU (nvmap) memory per process; needs root and debugfs, otherwise empty."""
        for path in ("/sys/kernel/debug/nvmap/iovmm/clients", "/sys/kernel/debug/nvmap/generic-0/clients"):
            txt = read(path)
            if not txt:
                continue
            res = {}
            for line in txt.splitlines():
                f = line.split()
                if len(f) >= 4 and f[2].isdigit() and f[-1].endswith("K"):
                    res[int(f[2])] = res.get(int(f[2]), 0) + int(f[-1][:-1]) / 1024
            return res
        return {}

    def close(self):
        self.proc.terminate()


class NoGpu:
    available = staticmethod(lambda: True)
    sample = per_process = lambda self: {}
    close = lambda self: None


def gpu_source():
    for cls in (Tegrastats, NvidiaSmi):
        if cls.available():
            try:
                return cls()
            except OSError:
                pass
    return NoGpu()


# ----------------------------------------------------------- kernel log ---

class KernelLog:
    """Follows new kernel messages (USB resets, link up/down, OOM kills, nvgpu errors)."""

    def __init__(self, path):
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "line"])
        self.lock = threading.Lock()
        self.proc = None
        for cmd in (["dmesg", "-W"], ["journalctl", "-k", "-f", "-n", "0", "-o", "cat"]):
            if not shutil.which(cmd[0]):
                continue
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            time.sleep(0.5)
            if p.poll() is None:
                self.proc, self.source = p, cmd[0]
                threading.Thread(target=self._read, daemon=True).start()
                return
        self.source = None

    def _read(self):
        for line in self.proc.stdout:
            with self.lock:
                self.w.writerow([f"{time.time():.3f}", line.rstrip()])
                self.f.flush()

    def close(self):
        if self.proc:
            self.proc.terminate()
        self.f.close()


# ------------------------------------------------------------- commands ---

def cmd_snapshot(args):
    """Configuration that differs between machines and explains most capture problems."""
    s = {"host": platform.node(), "kernel": platform.release(), "machine": platform.machine(),
         "jetson": is_jetson(), "jetpack": read("/etc/nv_tegra_release"),
         "nvpmodel": (run(["nvpmodel", "-q"]) or "").strip() or None,
         "cpus": os.cpu_count(), "ram_total_mb": round(meminfo().get("MemTotal", 0)),
         "swap_total_mb": round(meminfo().get("SwapTotal", 0))}
    shm = shm_used_mb()
    s["dev_shm_size_mb"] = round(shm[1]) if shm[1] else None
    gpu = run(["nvidia-smi", "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader"])
    s["gpu"] = gpu.strip() if gpu else ("integrated (shared RAM)" if s["jetson"] else None)
    s["sysctl"] = {k: read("/proc/sys/" + k.replace(".", "/")) for k in SYSCTLS}
    s["usbfs_memory_mb"] = read("/sys/module/usbcore/parameters/usbfs_memory_mb")
    s["usb_autosuspend"] = read("/sys/module/usbcore/parameters/autosuspend")
    s["lsusb_tree"] = run(["lsusb", "-t"])
    s["ros_env"] = {k: os.environ.get(k) for k in ROS_ENV}
    lib = next((line.split("=>")[-1].strip() for line in (run(["ldconfig", "-p"]) or "").splitlines()
                if "librealsense2.so" in line), None)
    s["librealsense"] = {"path": lib, "real": os.path.realpath(lib) if lib else None,
                         "links_cuda": bool(lib and re.search(r"libcuda|libcudart", run(["ldd", lib]) or ""))}
    s["interfaces"] = {}
    for i in interfaces():
        n = Path("/sys/class/net") / i
        drv = n / "device" / "driver"
        s["interfaces"][i] = {"speed_mbps": link_speed(i), "mtu": read(n / "mtu"), "state": read(n / "operstate"),
                              "driver": os.path.basename(os.path.realpath(drv)) if drv.exists() else None,
                              "ring": (run(["ethtool", "-g", i], timeout=2) or "").strip() or None}
    text = json.dumps(s, indent=2)
    if args.output:
        Path(args.output).write_text(text)
        print(f"written {args.output}")
    else:
        print(text)


def flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(flatten(v, f"{prefix}{k}."))
        else:
            out[f"{prefix}{k}"] = v
    return out


def cmd_compare(args):
    a, b = (flatten(json.loads(Path(p).read_text())) for p in (args.a, args.b))
    print(f"{'setting':42s} {Path(args.a).stem:>24s}   {Path(args.b).stem}")
    for k in sorted(set(a) | set(b)):
        if a.get(k) != b.get(k) and k not in ("host",):
            va, vb = (str(x.get(k)).replace("\n", " | ")[:60] for x in (a, b))
            print(f"{k:42s} {va:>24s}   {vb}")


def give_back(paths):
    """Under sudo, hand folders created by `record` (and the files in the output folder) back to
    the invoking user, so that `topics` and `report`, which run without sudo, can write there."""
    uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
    if os.geteuid() != 0 or not uid:
        return
    for path in paths:
        for p in [path] + (list(path.rglob("*")) if path.is_dir() else []):
            try:
                os.chown(p, int(uid), int(gid or uid))
            except OSError:
                pass


def cmd_record(args):
    out = Path(args.output).expanduser().absolute()
    created = [d for d in [out, *out.parents] if not d.exists()]  # folders this run creates
    out.mkdir(parents=True, exist_ok=True)
    cmd_snapshot(argparse.Namespace(output=str(out / "snapshot.json")))
    give_back(created + [out])
    ifs = args.iface.split(",") if args.iface else interfaces()
    patterns = [p for p in args.procs.split(",") if p]
    gpu, klog = gpu_source(), KernelLog(out / "kernel.csv")
    if klog.source is None:
        print("kernel log not readable (run with sudo to record USB resets, link changes and OOM kills)")
    if isinstance(gpu, Tegrastats) and not gpu.per_process():
        print("per-process GPU memory needs sudo on the Jetson; total shared RAM is still recorded")

    cols = ["t", "ram_used_mb", "ram_avail_mb", "ram_total_mb", "swap_used_mb", "shm_used_mb", "cpu_pct",
            "cpu_max_core_pct", "softirq_max_core_pct", "gpu_util_pct", "emc_util_pct", "gpu_mem_used_mb",
            "gpu_mem_total_mb", "gpu_temp_c", "tj_temp_c", "power_w", "udp_rcvbuf_errors", "udp_in_errors",
            "udp_sndbuf_errors", "ip_reasm_fails", "ip_in_discards"]
    for i in ifs:
        cols += [f"{i}_rx_mbps", f"{i}_tx_mbps", f"{i}_drops", f"{i}_hw_drops", f"{i}_carrier_changes",
                 f"{i}_speed"]
    fs, fp = open(out / "sys.csv", "w", newline=""), open(out / "procs.csv", "w", newline="")
    ws, wp = csv.DictWriter(fs, cols), csv.writer(fp)
    ws.writeheader()
    wp.writerow(["t", "pid", "name", "rss_mb", "gpu_mb"])

    prev_cpu, prev_snmp, prev_t = cpu_times(), snmp(), time.time()
    prev_nic = {i: nic_counters(i) for i in ifs}
    prev_hw = {i: ethtool_drops(i) for i in ifs}
    names, t_end = {}, (time.time() + args.duration) if args.duration else None
    print(f"recording to {out}/ every {args.interval}s on {', '.join(ifs)}; Ctrl+C to stop")
    try:
        while t_end is None or time.time() < t_end:
            time.sleep(max(0.0, prev_t + args.interval - time.time()))
            now = time.time()
            dt = now - prev_t
            m = meminfo()
            row = {"t": f"{now:.3f}", "ram_total_mb": round(m["MemTotal"]),
                   "ram_avail_mb": round(m["MemAvailable"]), "ram_used_mb": round(m["MemTotal"] - m["MemAvailable"]),
                   "swap_used_mb": round(m["SwapTotal"] - m["SwapFree"])}
            shm = shm_used_mb()[0]
            row["shm_used_mb"] = round(shm, 1) if shm is not None else ""
            cur = cpu_times()
            busy, soft = {}, {}
            for c, (tot, idle, si) in cur.items():
                d = tot - prev_cpu.get(c, (0, 0, 0))[0] or 1
                busy[c] = 100 * (1 - (idle - prev_cpu.get(c, (0, 0, 0))[1]) / d)
                soft[c] = 100 * (si - prev_cpu.get(c, (0, 0, 0))[2]) / d
            cores = [c for c in cur if c != "cpu"]
            row["cpu_pct"] = round(busy.get("cpu", 0), 1)
            row["cpu_max_core_pct"] = round(max((busy[c] for c in cores), default=0), 1)
            row["softirq_max_core_pct"] = round(max((soft[c] for c in cores), default=0), 1)
            prev_cpu = cur
            row.update({k: round(v, 1) for k, v in gpu.sample().items() if k in cols})
            sn = snmp()
            for col, key in (("udp_rcvbuf_errors", "Udp.RcvbufErrors"), ("udp_in_errors", "Udp.InErrors"),
                             ("udp_sndbuf_errors", "Udp.SndbufErrors"), ("ip_reasm_fails", "Ip.ReasmFails"),
                             ("ip_in_discards", "Ip.InDiscards")):
                row[col] = sn.get(key, 0) - prev_snmp.get(key, 0)
            prev_snmp = sn
            for i in ifs:
                c, p = nic_counters(i), prev_nic[i]
                row[f"{i}_rx_mbps"] = round(8 * (c["rx_bytes"] - p["rx_bytes"]) / dt / 1e6, 2)
                row[f"{i}_tx_mbps"] = round(8 * (c["tx_bytes"] - p["tx_bytes"]) / dt / 1e6, 2)
                row[f"{i}_drops"] = sum(c[k] - p[k] for k in NIC_DROP_FILES)
                row[f"{i}_carrier_changes"] = c["carrier_changes"] - p["carrier_changes"]
                row[f"{i}_speed"] = link_speed(i)
                hw = ethtool_drops(i) if prev_hw[i] is not None else None
                row[f"{i}_hw_drops"] = hw - prev_hw[i] if hw is not None else ""
                prev_nic[i], prev_hw[i] = c, hw
            ws.writerow(row)
            fs.flush()
            gmem = gpu.per_process()
            for pid, rss in processes(patterns):
                names.setdefault(pid, proc_name(pid))
                wp.writerow([f"{now:.3f}", pid, names[pid], round(rss, 1), gmem.get(pid, "")])
            fp.flush()
            prev_t = now
            if not args.quiet:
                rx = " ".join(f"{i}:{row[f'{i}_rx_mbps']:.0f}" for i in ifs if row[f"{i}_rx_mbps"] >= 1)
                g = f" gpu {row.get('gpu_mem_used_mb', '-')}/{row.get('gpu_mem_total_mb', '-')}MB" \
                    if "gpu_mem_used_mb" in row else ""
                drops = sum(row[f"{i}_drops"] for i in ifs) + row["udp_rcvbuf_errors"] + row["ip_reasm_fails"]
                print(f"\rram {row['ram_used_mb']}/{row['ram_total_mb']}MB swap {row['swap_used_mb']}MB{g} "
                      f"cpu {row['cpu_pct']}% rx[Mb/s] {rx or '-'} drops {drops}   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        gpu.close()
        klog.close()
        fs.close()
        fp.close()
        give_back([out])
        print(f"done; run: python3 {sys.argv[0]} report {out}")


def cmd_topics(args):
    """Actual rate, longest gap and age of the ROS 2 topics (run with the ROS environment sourced)."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
    except ImportError:
        sys.exit("rclpy not found: source /opt/ros/<distro>/setup.bash (and the workspace) first")
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("magdalena_sysmon")
    t0 = time.time()
    while time.time() - t0 < args.discovery:
        rclpy.spin_once(node, timeout_sec=0.1)
    topics = sorted((n, t[0]) for n, t in node.get_topic_names_and_types() if re.search(args.topics, n))
    if not topics:
        sys.exit(f"no topic matches {args.topics!r}; is the pipeline running with the same ROS_DOMAIN_ID?")
    stats = {}
    for name, typ in topics:
        pkg, _, msg = typ.split("/")
        try:
            cls = getattr(importlib.import_module(f"{pkg}.msg"), msg)
        except (ImportError, AttributeError):
            print(f"  {name}: type {typ} not importable (source the workspace that defines it); skipped")
            continue
        fields = list(cls.get_fields_and_field_types().items())
        stamped = bool(fields) and fields[0][1] == "std_msgs/Header"
        stats[name] = {"n": 0, "last": None, "gap": 0.0, "age": []}

        def cb(raw, name=name, stamped=stamped):
            now = time.time()
            s = stats[name]
            s["n"] += 1
            if s["last"] is not None:
                s["gap"] = max(s["gap"], now - s["last"])
            s["last"] = now
            if stamped and len(raw) >= 12:  # CDR: 4-byte encapsulation, then header.stamp
                sec, nsec = struct.unpack_from("<iI" if raw[1] & 1 else ">iI", raw, 4)
                s["age"].append(now - (sec + nsec * 1e-9))

        node.create_subscription(cls, name, cb, qos_profile_sensor_data if args.best_effort else 10, raw=True)
        print(f"  {name} ({typ})")
    if not stats:
        sys.exit("none of the matching topics could be subscribed")
    f = open(out / "topics.csv", "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "topic", "hz", "max_gap_ms", "mean_age_ms"])
    t_end = (time.time() + args.duration) if args.duration else None
    nxt = time.time() + 1
    print(f"recording {len(topics)} topics to {out}/topics.csv; Ctrl+C to stop")
    try:
        while t_end is None or time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.05, nxt - time.time())))
            if time.time() < nxt:
                continue
            now = time.time()
            line = []
            for name, s in stats.items():
                gap = max(s["gap"], now - s["last"]) if s["last"] else now - t0
                age = 1000 * sum(s["age"]) / len(s["age"]) if s["age"] else ""
                w.writerow([f"{now:.3f}", name, s["n"], round(1000 * gap), round(age, 1) if age != "" else ""])
                line.append(f"{name.split('/')[1] if name.count('/') > 1 else name}:{s['n']}")
                s.update(n=0, gap=0.0, age=[])
            f.flush()
            if not args.quiet:
                print("\r" + " ".join(line) + "   ", end="", flush=True)
            nxt += 1
    except KeyboardInterrupt:
        pass
    finally:
        print()
        f.close()
        node.destroy_node()
        rclpy.try_shutdown()


# --------------------------------------------------------------- report ---

def load_csv(path):
    if not Path(path).exists():
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def num(rows, col):
    return [float(r[col]) for r in rows if r.get(col) not in (None, "")]


def cmd_report(args):
    d = Path(args.dir)
    snap = json.loads(read(d / "snapshot.json", "{}"))
    rows, procs = load_csv(d / "sys.csv"), load_csv(d / "procs.csv")
    topics, kernel = load_csv(d / "topics.csv"), load_csv(d / "kernel.csv")
    if not rows and not topics:
        sys.exit(f"nothing recorded in {d}")
    lines, hints = [], []
    say = lines.append
    say(f"== {snap.get('host', '?')}  ({'Jetson, shared CPU/GPU memory' if snap.get('jetson') else snap.get('gpu')})")
    if snap.get("nvpmodel"):
        say("power mode: " + snap["nvpmodel"].replace("\n", " | "))
    if rows:
        dur = float(rows[-1]["t"]) - float(rows[0]["t"])
        say(f"duration {dur:.0f} s, {len(rows)} samples")
        tot = float(rows[0]["ram_total_mb"])
        used, swap = num(rows, "ram_used_mb"), num(rows, "swap_used_mb")
        say(f"RAM used: start {used[0]:.0f}, max {max(used):.0f}, end {used[-1]:.0f} of {tot:.0f} MB; "
            f"swap max {max(swap):.0f} MB")
        if max(used) > 0.9 * tot:
            hints.append(f"RAM nearly full ({max(used):.0f}/{tot:.0f} MB)"
                         + (" - on the Jetson this is also the GPU memory" if snap.get("jetson") else ""))
        if max(swap) - swap[0] > 100:
            hints.append(f"swap grew by {max(swap) - swap[0]:.0f} MB: memory pressure, expect stalls")
        if len(used) > 30 and used[-1] - used[0] > 0.1 * tot:
            hints.append(f"RAM grew {used[-1] - used[0]:.0f} MB during the run: look for a leak in procs.csv")
        shm = num(rows, "shm_used_mb")
        if shm and snap.get("dev_shm_size_mb"):
            say(f"/dev/shm used max {max(shm):.0f} of {snap['dev_shm_size_mb']} MB")
            if max(shm) > 0.9 * snap["dev_shm_size_mb"]:
                hints.append("/dev/shm full: DDS shared-memory transport falls back or fails (Docker default is 64 MB)")
        g = num(rows, "gpu_mem_used_mb")
        if g:
            gt = num(rows, "gpu_mem_total_mb")[0]
            say(f"GPU memory used max {max(g):.0f} of {gt:.0f} MB")
            if max(g) > 0.9 * gt:
                hints.append(f"GPU memory nearly full ({max(g):.0f}/{gt:.0f} MB): VRAM is the limit")
        for col, label in (("gpu_util_pct", "GPU load"), ("cpu_pct", "CPU total"), ("cpu_max_core_pct", "busiest core"),
                           ("softirq_max_core_pct", "softirq on busiest core"), ("power_w", "board/GPU power W"),
                           ("tj_temp_c", "Tj C"), ("gpu_temp_c", "GPU C")):
            v = num(rows, col)
            if v:
                say(f"{label}: mean {sum(v) / len(v):.1f}, max {max(v):.1f}")
        si = num(rows, "softirq_max_core_pct")
        if si and max(si) > 50:
            hints.append(f"one core spent up to {max(si):.0f}% in softirq: network interrupts saturate a core "
                         "(spread them: irqbalance / RPS)")
        if num(rows, "cpu_max_core_pct") and max(num(rows, "cpu_max_core_pct")) > 97:
            hints.append("a core reached 100%: a single-threaded stage (capture, conversion, DDS) may be the limit")
        for col, label, hint in (
                ("udp_rcvbuf_errors", "UDP receive-buffer overflows",
                 "UDP socket buffers overflow: raise net.core.rmem_max/rmem_default (e.g. 2147483647 / 26214400)"),
                ("ip_reasm_fails", "IP reassembly failures",
                 "fragments of large UDP packets are lost: raise net.ipv4.ipfrag_high_thresh, use jumbo frames (MTU 9000 "
                 "on NIC and switch) or a DDS profile with smaller datagrams"),
                ("udp_in_errors", "UDP input errors", None), ("ip_in_discards", "IP input discards", None)):
            v = sum(num(rows, col))
            say(f"{label}: {v:.0f}")
            if v > 0 and hint:
                hints.append(f"{v:.0f} {label.lower()}: {hint}")
        say("interfaces (Mb/s rx mean/max, link speed, drops, link flaps):")
        for i, meta in snap.get("interfaces", {}).items():
            rx = num(rows, f"{i}_rx_mbps")
            if not rx:
                continue
            drops, hw = sum(num(rows, f"{i}_drops")), sum(num(rows, f"{i}_hw_drops"))
            flaps = sum(num(rows, f"{i}_carrier_changes"))
            speed = max(num(rows, f"{i}_speed") or [-1])
            if max(rx) < 1 and not drops and not flaps:
                continue
            say(f"  {i:10s} {sum(rx) / len(rx):7.1f} / {max(rx):7.1f}   {speed:>6.0f} Mb/s  mtu {meta.get('mtu')}  "
                f"drops {drops:.0f} (driver {hw:.0f})  flaps {flaps:.0f}")
            if i != "lo" and speed > 0:
                if max(rx) > 0.7 * speed:
                    hints.append(f"{i} reached {max(rx):.0f} of {speed:.0f} Mb/s: the link is saturated")
                if speed < 1000 and max(rx) > 50:
                    hints.append(f"{i} negotiated only {speed:.0f} Mb/s (cable, port or USB adapter)")
            if drops + hw:
                hints.append(f"{i} dropped {drops + hw:.0f} packets in the NIC/driver: enlarge the ring "
                             f"(ethtool -G {i} rx <max>) and net.core.netdev_max_backlog")
            if flaps:
                hints.append(f"{i} link went down/up {flaps:.0f} times (cable, PoE budget, switch)")
            if i == "lo" and max(rx) > 200:
                hints.append(f"{max(rx):.0f} Mb/s over loopback: raw images copied between ROS processes via DDS "
                             "(composable nodes/intra-process or shared-memory transport avoid it)")
    if procs:
        say("largest processes (RSS max MB, growth MB, GPU MB):")
        by = {}
        for r in procs:
            by.setdefault((r["pid"], r["name"]), []).append(r)
        summ = []
        for (pid, name), rs in by.items():
            rss = [float(r["rss_mb"]) for r in rs]
            gm = [float(r["gpu_mb"]) for r in rs if r["gpu_mb"]]
            summ.append((max(rss), rss[-1] - rss[0], max(gm) if gm else None, name, pid))
        for mx, grow, gm, name, pid in sorted(summ, reverse=True)[:12]:
            say(f"  {mx:8.0f} {grow:+8.0f} {'' if gm is None else f'{gm:8.0f}':>8s}  {name} ({pid})")
            if grow > 200:
                hints.append(f"{name} grew {grow:.0f} MB during the run")
        cams = [s for s in summ if "camera" in s[3]]
        if cams:
            say(f"  camera processes: {len(cams)}, total RSS {sum(s[0] for s in cams):.0f} MB")
    if kernel:
        cats = {"USB": r"usb|xhci|uvc", "network link": r"link (is )?(up|down)|nic link|carrier|eth|enp|eno|enP",
                "out of memory": r"out of memory|oom|killed process", "GPU": r"nvgpu|nvmap|nvrm|xid"}
        say(f"kernel messages: {len(kernel)}")
        for cat, rx in cats.items():
            hits = [r["line"] for r in kernel if re.search(rx, r["line"], re.I)]
            if hits:
                say(f"  {cat}: {len(hits)}  e.g. {hits[-1][:110]}")
                if cat == "out of memory":
                    hints.append("the kernel killed a process for lack of memory (OOM)")
                if cat == "USB" and any(re.search(r"reset|disconnect|error|fail", h, re.I) for h in hits):
                    hints.append("USB resets/disconnects in the kernel log (cable, hub power, bandwidth)")
    if topics:
        say(f"topics (Hz mean/min, % of seconds below {0.9 * args.fps:.1f} Hz, longest gap ms, mean age ms):")
        by = {}
        for r in topics:
            by.setdefault(r["topic"], []).append(r)
        for name, rs in sorted(by.items()):
            hz, gap = [float(r["hz"]) for r in rs], [float(r["max_gap_ms"]) for r in rs]
            age = [float(r["mean_age_ms"]) for r in rs if r["mean_age_ms"]]
            low = 100 * sum(h < 0.9 * args.fps for h in hz) / len(hz)
            say(f"  {name:40s} {sum(hz) / len(hz):5.1f} / {min(hz):4.0f}  {low:5.1f}%  {max(gap):7.0f}  "
                f"{(sum(age) / len(age)) if age else float('nan'):7.1f}")
            if "image" in name and low > 5:
                hints.append(f"{name} below {0.9 * args.fps:.0f} Hz in {low:.0f}% of seconds, gaps up to {max(gap):.0f} ms")
    sysc = snap.get("sysctl", {})
    try:
        if int(sysc.get("net.core.rmem_max") or 0) < 8 * 2**20:
            hints.append(f"net.core.rmem_max is {sysc.get('net.core.rmem_max')} bytes; one 640x360 BGR frame is "
                         "691 kB, so UDP-based cameras/DDS need tens of MB")
    except ValueError:
        pass
    usbfs = snap.get("usbfs_memory_mb")
    if usbfs not in (None, "0") and int(usbfs) < 256 and "librealsense" in json.dumps(snap):
        hints.append(f"usbfs_memory_mb is {usbfs}: several USB cameras need more (RealSense suggests 1000)")
    if (snap.get("librealsense") or {}).get("links_cuda"):
        hints.append("librealsense is built with CUDA: every camera process creates its own CUDA context "
                     "(hundreds of MB each, from shared RAM on the Jetson)")
    say("")
    say("possible causes:" if hints else "no obvious bottleneck in the recorded counters")
    for h in dict.fromkeys(hints):
        say("  - " + h)
    text = "\n".join(lines)
    print(text)
    (d / "summary.txt").write_text(text + "\n")
    if rows and not args.no_plot:
        plot(d, rows, topics, snap)


def plot(d, rows, topics, snap):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib not installed: no plot)")
        return
    t0 = float(rows[0]["t"])
    t = [float(r["t"]) - t0 for r in rows]
    n = 5 if topics else 4
    fig, ax = plt.subplots(n, 1, figsize=(10, 2.4 * n + 0.6), sharex=True)
    ax[0].plot(t, num(rows, "ram_used_mb"), label="RAM used")
    ax[0].plot(t, num(rows, "swap_used_mb"), label="swap used")
    if num(rows, "gpu_mem_used_mb"):
        ax[0].plot(t, num(rows, "gpu_mem_used_mb"), label="GPU memory used")
        ax[0].axhline(num(rows, "gpu_mem_total_mb")[0], color="C2", ls=":", lw=0.8, label="GPU memory total")
    ax[0].axhline(float(rows[0]["ram_total_mb"]), color="0.6", ls="--", lw=0.8, label="RAM total")
    ax[0].set_ylabel("MB")
    for i in snap.get("interfaces", {}):
        rx = num(rows, f"{i}_rx_mbps")
        if rx and max(rx) >= 1:
            ax[1].plot(t, rx, label=f"{i} rx")
    ax[1].set_ylabel("Mb/s")
    drops = [sum(float(r[c]) for c in r if (c.endswith("_drops") or c in ("udp_rcvbuf_errors", "ip_reasm_fails"))
                 and r[c] not in ("", None)) for r in rows]
    ax[2].bar(t, drops, width=1.0, color="#c0392b")
    ax[2].set_ylim(bottom=0, top=max(1, max(drops) * 1.1))
    ax[2].set_ylabel("drops / s")
    ax[3].plot(t, num(rows, "cpu_pct"), label="CPU total")
    ax[3].plot(t, num(rows, "cpu_max_core_pct"), label="busiest core")
    ax[3].plot(t, num(rows, "softirq_max_core_pct"), label="softirq, busiest core")
    if num(rows, "gpu_util_pct"):
        ax[3].plot(t, num(rows, "gpu_util_pct"), label="GPU load")
    ax[3].set_ylabel("%")
    if topics:
        for name in sorted({r["topic"] for r in topics}):
            rs = [r for r in topics if r["topic"] == name]
            ax[4].plot([float(r["t"]) - t0 for r in rs], [float(r["hz"]) for r in rs], lw=1, label=name)
        ax[4].set_ylabel("messages / s")
    ax[-1].set_xlabel("s")
    for a in ax:
        a.grid(alpha=0.3)
        if a.get_legend_handles_labels()[0]:
            a.legend(fontsize=8, loc="upper left")
    fig.suptitle(snap.get("host", ""))
    fig.tight_layout()
    fig.savefig(d / "plot.png", dpi=110)
    print(f"plot: {d / 'plot.png'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("snapshot", help="configuration of this machine (JSON)")
    p.add_argument("-o", "--output")
    p = sub.add_parser("compare", help="differences between two snapshots")
    p.add_argument("a")
    p.add_argument("b")
    p = sub.add_parser("record", help="log memory, GPU, network, CPU and kernel messages")
    p.add_argument("-o", "--output", required=True, help="output folder")
    p.add_argument("--iface", help="comma-separated interfaces (default: all, loopback included)")
    p.add_argument("--procs", default=PROC_PATTERNS, help="command-line substrings of processes to track")
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--duration", type=float, help="seconds (default: until Ctrl+C)")
    p.add_argument("-q", "--quiet", action="store_true")
    p = sub.add_parser("topics", help="rate, gaps and age of ROS 2 topics")
    p.add_argument("-o", "--output", required=True, help="output folder (same as record)")
    p.add_argument("--topics", default=TOPIC_REGEX, help=f"regex of topic names (default {TOPIC_REGEX!r})")
    p.add_argument("--discovery", type=float, default=3.0, help="seconds to wait for topic discovery")
    p.add_argument("--best-effort", action="store_true",
                   help="subscribe best effort (SensorDataQoS) instead of reliable; large raw images then lose "
                        "frames in DDS, so use it only to reproduce a best-effort consumer")
    p.add_argument("--duration", type=float)
    p.add_argument("-q", "--quiet", action="store_true")
    p = sub.add_parser("report", help="summary and plot of a recorded folder")
    p.add_argument("dir")
    p.add_argument("--fps", type=float, default=15.0, help="expected camera rate")
    p.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    {"snapshot": cmd_snapshot, "compare": cmd_compare, "record": cmd_record,
     "topics": cmd_topics, "report": cmd_report}[args.cmd](args)


if __name__ == "__main__":
    main()
