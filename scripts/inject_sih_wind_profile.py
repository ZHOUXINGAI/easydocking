#!/usr/bin/env python3

import argparse
import csv
import math
import random
import sys
import time
from pathlib import Path
from typing import List, Tuple

from pymavlink import mavutil


def parse_endpoints(raw: str) -> List[str]:
    endpoints = [item.strip() for item in raw.split(",") if item.strip()]
    if not endpoints:
        raise ValueError("at least one MAVLink endpoint is required")
    return endpoints


def parse_step_events(raw: str) -> List[Tuple[float, float, float]]:
    if not raw.strip():
        return []
    events: List[Tuple[float, float, float]] = []
    for item in raw.split(";"):
        token = item.strip()
        if not token:
            continue
        parts = token.split(":")
        if len(parts) != 2:
            raise ValueError(f"invalid step event '{token}'")
        t_sec = float(parts[0].strip())
        vec = [v.strip() for v in parts[1].split(",")]
        if len(vec) != 2:
            raise ValueError(f"invalid step vector '{parts[1]}'")
        events.append((t_sec, float(vec[0]), float(vec[1])))
    events.sort(key=lambda item: item[0])
    return events


def wait_heartbeat(master: mavutil.mavfile, timeout_sec: float) -> None:
    hb = master.wait_heartbeat(timeout=timeout_sec)
    if hb is None:
        raise RuntimeError(f"heartbeat timeout on {master.address}")


def set_param(master: mavutil.mavfile, name: str, value: float, param_type: int) -> None:
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("utf-8"),
        float(value),
        param_type,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Inject SIH wind profile over MAVLink.")
    parser.add_argument("--endpoints", default="127.0.0.1:14541,127.0.0.1:14542")
    parser.add_argument("--duration-sec", type=float, default=95.0)
    parser.add_argument("--update-period-sec", type=float, default=0.5)
    parser.add_argument("--base-n", type=float, default=0.0)
    parser.add_argument("--base-e", type=float, default=0.0)
    parser.add_argument("--gust-enable", action="store_true")
    parser.add_argument("--gust-amp", type=float, default=0.0)
    parser.add_argument("--gust-change-sec", type=float, default=1.5)
    parser.add_argument("--step-events", default="")
    parser.add_argument("--seed", type=int, default=1234)
    parser.add_argument("--log-file", default="")
    args = parser.parse_args()

    try:
        endpoint_tokens = parse_endpoints(args.endpoints)
        step_events = parse_step_events(args.step_events)
    except ValueError as exc:
        print(f"[wind] invalid args: {exc}", file=sys.stderr)
        return 2

    rng = random.Random(args.seed)
    masters: List[mavutil.mavfile] = []
    for token in endpoint_tokens:
        conn = mavutil.mavlink_connection(f"udpin:{token}")
        masters.append(conn)

    try:
        for master in masters:
            wait_heartbeat(master, timeout_sec=45.0)
            set_param(master, "COM_WIND_WARN", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            set_param(master, "COM_WIND_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            set_param(master, "COM_WIND_MAX_ACT", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print(f"[wind] connected endpoints={endpoint_tokens}")
    except Exception as exc:
        print(f"[wind] connection failed: {exc}", file=sys.stderr)
        return 3

    log_writer = None
    log_fp = None
    if args.log_file:
        log_path = Path(args.log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_fp = log_path.open("w", newline="", encoding="utf-8")
        log_writer = csv.writer(log_fp)
        log_writer.writerow(["t_sec", "wind_n_mps", "wind_e_mps"])

    start = time.monotonic()
    next_gust_change = 0.0
    gust_n = 0.0
    gust_e = 0.0
    step_n = 0.0
    step_e = 0.0
    next_step_index = 0

    try:
        while True:
            elapsed = time.monotonic() - start
            if elapsed > args.duration_sec:
                break

            while next_step_index < len(step_events) and elapsed >= step_events[next_step_index][0]:
                _, dn, de = step_events[next_step_index]
                step_n += dn
                step_e += de
                next_step_index += 1

            if args.gust_enable and args.gust_amp > 0.0 and elapsed >= next_gust_change:
                angle = rng.uniform(0.0, 2.0 * math.pi)
                mag = rng.uniform(0.0, args.gust_amp)
                gust_n = mag * math.cos(angle)
                gust_e = mag * math.sin(angle)
                next_gust_change = elapsed + max(args.gust_change_sec, 0.05)

            wind_n = args.base_n + step_n + gust_n
            wind_e = args.base_e + step_e + gust_e

            for master in masters:
                set_param(master, "SIH_WIND_N", wind_n, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                set_param(master, "SIH_WIND_E", wind_e, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

            if log_writer is not None:
                log_writer.writerow([f"{elapsed:.3f}", f"{wind_n:.4f}", f"{wind_e:.4f}"])

            time.sleep(max(args.update_period_sec, 0.05))
    finally:
        for master in masters:
            try:
                set_param(master, "SIH_WIND_N", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                set_param(master, "SIH_WIND_E", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            except Exception:
                pass
        if log_fp is not None:
            log_fp.close()

    print("[wind] profile completed and reset to zero")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
