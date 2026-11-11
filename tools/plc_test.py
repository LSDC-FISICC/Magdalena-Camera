#!/usr/bin/env python3
"""
plc_test.py: network and Modbus TCP checks of the LOGO! PLC from the Jetson.

  python3 plc_test.py read  -o RUN [--rate 10] [--duration 300]
      Reads the valve coils (FC1, read only: nothing moves) at a fixed rate and logs
      the round-trip time of every request. Safe at any time.

  python3 plc_test.py write -o RUN --hydraulics-off [--on-ms 1000] [--cycles 3]
      Switches each valve coil on and off in turn, reads it back and logs the write
      round-trip and the read-back. The VALVES MOVE: run it only with the pump and the
      hydraulics de-energised and nobody at the outputs. All coils are written off at
      the end, also on Ctrl+C or an error.

  python3 plc_test.py watchdog -o RUN --hydraulics-off [--heartbeat-coil 10] [--trials 4]
      Tests the PLC watchdog: switches the valve coils on, toggles the heartbeat coil every
      --period-ms for --hold-s while reading the PLC outputs (Q1.. at Modbus address --q-addr,
      8192 = Q1 on a LOGO! 8), then stops the heartbeat, alternately leaving it at 1 and at 0,
      and measures the time from the last heartbeat edge until every output that was on turns
      off. The VALVES MOVE, as for write. Valve and heartbeat coils are written off at the end.

All modes write <RUN>/plc_<mode>.csv and print a summary (also in <RUN>/plc_<mode>.txt):
requests, failures, round-trip percentiles, requests above 50/200 ms, and the TCP
retransmissions of the connection. Stop the modbus_controller node first, so this is
the only client of the PLC.

Needs pyModbusTCP (pip3 install --user pyModbusTCP), as the pipeline does.
"""
import argparse
import csv
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

from pyModbusTCP.client import ModbusClient


def attr(obj, name):
    """pyModbusTCP 0.1 exposes these as methods, 0.2+ as properties."""
    v = getattr(obj, name, None)
    return v() if callable(v) else v


def coil_range(spec):
    a, _, b = spec.partition("-")
    return list(range(int(a), int(b or a) + 1))


def retrans(host, port):
    """Retransmitted segments of the TCP connection to host:port (ss -ti), or None."""
    try:
        out = subprocess.run(["ss", "-tin", "dst", f"{host}:{port}"], capture_output=True, text=True,
                             timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    m = re.search(r"retrans:\d+/(\d+)", out)
    return int(m.group(1)) if m else (0 if host in out else None)


def summary(rows, label, host, port, extra=""):
    rtt = sorted(r[3] for r in rows if r[2])
    fails = [r for r in rows if not r[2]]
    lines = [f"== {label}: {len(rows)} requests to {host}:{port}, {len(fails)} failed"]
    if rtt:
        pct = lambda q: rtt[min(len(rtt) - 1, int(q * len(rtt)))]
        lines.append(f"round trip ms: min {rtt[0]:.2f}  p50 {pct(.5):.2f}  p95 {pct(.95):.2f}  "
                     f"p99 {pct(.99):.2f}  max {rtt[-1]:.2f}")
        lines.append(f"above 50 ms: {sum(x > 50 for x in rtt)}   above 200 ms: {sum(x > 200 for x in rtt)}")
    if fails:
        lines.append("first failures: " + "; ".join(f"{r[1]} {r[4]}" for r in fails[:5]))
    rt = retrans(host, port)
    lines.append(f"TCP retransmissions on this connection: {'n/a' if rt is None else rt}")
    if extra:
        lines.append(extra)
    return "\n".join(lines)


def watchdog_test(args, c, coils, req):
    """Stop the heartbeat after --hold-s and time the release of the PLC outputs."""
    hb, period = args.heartbeat_coil, args.period_ms / 1000
    lines, times = [], []
    for trial in range(args.trials):
        last = trial % 2 == 0                  # heartbeat left at 1, then at 0, alternately
        req("valves=1", c.write_multiple_coils, coils[0], [True] * len(coils))
        val, q_on, t_end = False, None, time.perf_counter() + args.hold_s
        while time.perf_counter() < t_end:
            val = not val
            req(f"hb={int(val)}", c.write_single_coil, hb, val)
            q, _ = req("readQ", c.read_coils, args.q_addr, args.q_count)
            q_on = q or q_on
            time.sleep(period)
        if val == last:                        # end on an edge to the value under test
            req(f"hb={int(not last)}", c.write_single_coil, hb, not last)
            time.sleep(period)
        req(f"hb={int(last)} (last)", c.write_single_coil, hb, last)
        t0 = time.perf_counter()
        on = [i for i, x in enumerate(q_on or []) if x]
        if not on:
            lines.append(f"trial {trial + 1} (heartbeat stopped at {int(last)}): no output on while the heartbeat "
                         f"ran (read {q_on}); check the program and --q-addr")
        else:
            t_off = None
            while time.perf_counter() - t0 < args.max_wait_s:
                q, _ = req("readQ", c.read_coils, args.q_addr, args.q_count)
                if q and not any(q[i] for i in on):
                    t_off = 1000 * (time.perf_counter() - t0)
                    break
                time.sleep(0.01)
            names = ",".join(f"Q{i + 1}" for i in on)
            if t_off is None:
                lines.append(f"trial {trial + 1} (heartbeat stopped at {int(last)}): {names} still on after "
                             f"{args.max_wait_s:.0f} s - the watchdog did not release them")
            else:
                times.append(t_off)
                lines.append(f"trial {trial + 1} (heartbeat stopped at {int(last)}): {names} off after {t_off:.0f} ms")
        print("\n  " + lines[-1])
        req("valves=0", c.write_multiple_coils, coils[0], [False] * len(coils))
        req("hb=0", c.write_single_coil, hb, False)
        time.sleep(2)
    if times:
        lines.append(f"release time ms: min {min(times):.0f}  mean {sum(times) / len(times):.0f}  max {max(times):.0f} "
                     f"({len(times)} of {args.trials} trials released)")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["read", "write", "watchdog"])
    ap.add_argument("-o", "--output", required=True, help="output folder")
    ap.add_argument("--host", default="192.168.11.60")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--unit-id", type=int, default=1)
    ap.add_argument("--coils", default="0-4", help="valve coils (config.yaml camera_map), e.g. 0-4")
    ap.add_argument("--timeout", type=float, default=1.0, help="seconds per request")
    ap.add_argument("--rate", type=float, default=10.0, help="read: requests per second")
    ap.add_argument("--duration", type=float, default=300.0, help="read: seconds")
    ap.add_argument("--on-ms", type=int, default=1000, help="write: time each coil stays on")
    ap.add_argument("--cycles", type=int, default=3, help="write: passes over all coils")
    ap.add_argument("--heartbeat-coil", type=int, default=10, help="watchdog: heartbeat coil (config.yaml)")
    ap.add_argument("--period-ms", type=int, default=200, help="watchdog: heartbeat toggle period")
    ap.add_argument("--hold-s", type=float, default=5.0, help="watchdog: heartbeat time before stopping it")
    ap.add_argument("--q-addr", type=int, default=8192, help="watchdog: Modbus address of output Q1 (LOGO! 8: 8192)")
    ap.add_argument("--q-count", type=int, default=8, help="watchdog: outputs to read")
    ap.add_argument("--trials", type=int, default=4, help="watchdog: stops of the heartbeat (alternating 1/0)")
    ap.add_argument("--max-wait-s", type=float, default=5.0, help="watchdog: give up waiting for the release")
    ap.add_argument("--hydraulics-off", action="store_true",
                    help="write: confirm that the pump and hydraulics are de-energised")
    args = ap.parse_args()
    if args.mode in ("write", "watchdog") and not args.hydraulics_off:
        sys.exit(f"{args.mode} moves the valves: de-energise the pump/hydraulics, then add --hydraulics-off")

    out = Path(args.output).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    coils = coil_range(args.coils)
    c = ModbusClient(host=args.host, port=args.port, unit_id=args.unit_id, timeout=args.timeout,
                     auto_open=True)
    rows = []  # t, op, ok, rtt_ms, detail
    f = open(out / f"plc_{args.mode}.csv", "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "op", "ok", "rtt_ms", "detail"])

    def req(op, fn, *a):
        t0 = time.perf_counter()
        res = fn(*a)
        dt = 1000 * (time.perf_counter() - t0)
        ok = res is not None and res is not False
        detail = ""
        if not ok:
            err, exc = attr(c, "last_error_as_txt"), attr(c, "last_except_as_txt")
            detail = err if exc in (None, "", "no exception") else f"{err}: {exc}"
        if ok and isinstance(res, list):
            detail = "".join("1" if x else "0" for x in res)
        row = (time.time(), op, ok, round(dt, 3), detail)
        rows.append(row)
        w.writerow([f"{row[0]:.3f}", op, int(ok), row[3], detail])
        return res, dt

    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    extra = ""
    print(f"{args.mode}: {args.host}:{args.port} unit {args.unit_id}, coils {coils[0]}-{coils[-1]}; Ctrl+C to stop")
    try:
        if args.mode == "read":
            period, t_end = 1.0 / args.rate, time.time() + args.duration
            nxt = time.perf_counter()
            last_print = 0
            while time.time() < t_end:
                req("read", c.read_coils, coils[0], len(coils))
                nxt += period
                time.sleep(max(0.0, nxt - time.perf_counter()))
                if time.time() - last_print > 1:
                    last_print = time.time()
                    rtt = [r[3] for r in rows[-int(args.rate):] if r[2]]
                    bad = sum(not r[2] for r in rows)
                    print(f"\r{len(rows)} requests, last s: max {max(rtt) if rtt else float('nan'):.1f} ms, "
                          f"failed so far {bad}   ", end="", flush=True)
                f.flush()
        elif args.mode == "write":
            state, _ = req("read", c.read_coils, coils[0], len(coils))
            print(f"coils before the test: {state}")
            mism = 0
            for cyc in range(args.cycles):
                for k in coils:
                    for val in (True, False):
                        req(f"write{k}={int(val)}", c.write_single_coil, k, val)
                        back, _ = req(f"readback{k}", c.read_coils, k, 1)
                        if not back or bool(back[0]) != val:
                            mism += 1
                            print(f"  coil {k}: wrote {int(val)}, read back {back}")
                        print(f"\rcycle {cyc + 1}/{args.cycles}  coil {k} = {int(val)}   ", end="", flush=True)
                        time.sleep(args.on_ms / 1000)
            extra = f"read-back mismatches: {mism}"
        else:
            extra = watchdog_test(args, c, coils, req)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if args.mode in ("write", "watchdog"):
            ok = c.write_multiple_coils(coils[0], [False] * len(coils))
            if args.mode == "watchdog":
                ok = c.write_single_coil(args.heartbeat_coil, False) and ok
            print(f"all coils written off: {'ok' if ok else 'FAILED - check the PLC'}")
        text = summary(rows, args.mode, args.host, args.port, extra)
        c.close()
        f.close()
        print(text)
        (out / f"plc_{args.mode}.txt").write_text(text + "\n")


if __name__ == "__main__":
    main()
