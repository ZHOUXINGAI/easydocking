#!/usr/bin/env python3
"""Diagnostic: show what the docking planner commands for both aircraft."""
import csv, io, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PHASE_COLORS = {
    'IDLE': '#aaaaaa', 'APPROACH': '#3399ff', 'TRACKING': '#ff6600',
    'DOCKING': '#cc0000', 'COMPLETED': '#00cc44', 'FAILED': '#000000',
}

def load(csv_path):
    txt = Path(csv_path).read_text(encoding='utf-8', errors='ignore').replace('\x00','')
    return list(csv.DictReader(io.StringIO(txt)))

def phase_segments(t, phase):
    segs = []
    cur_start = 0
    cur_p = phase[0]
    for i in range(1, len(phase)):
        if phase[i] != cur_p:
            segs.append((cur_start, i, cur_p))
            cur_start = i
            cur_p = phase[i]
    segs.append((cur_start, len(phase)-1, cur_p))
    return segs

def main():
    if len(sys.argv) < 2:
        print("usage: plot_docking_planner.py <result_dir>")
        raise SystemExit(1)
    out = Path(sys.argv[1])
    rows = load(out / "docking_log.csv")
    if not rows:
        raise SystemExit("no data")

    t = np.array([float(r['t']) for r in rows])
    phase = np.array([r['phase'] for r in rows])
    car_x = np.array([float(r['carrier_x']) for r in rows])
    car_y = np.array([float(r['carrier_y']) for r in rows])
    car_z = np.array([float(r['carrier_z']) for r in rows])
    car_vx = np.array([float(r['carrier_vx']) for r in rows])
    car_vy = np.array([float(r['carrier_vy']) for r in rows])
    car_sp_x = np.array([float(r['carrier_sp_x']) for r in rows])
    car_sp_y = np.array([float(r['carrier_sp_y']) for r in rows])
    car_sp_z = np.array([float(r['carrier_sp_z']) for r in rows])
    car_cmd_vx = np.array([float(r['carrier_cmd_vx']) for r in rows])
    car_cmd_vy = np.array([float(r['carrier_cmd_vy']) for r in rows])
    mini_x = np.array([float(r['mini_x']) for r in rows])
    mini_y = np.array([float(r['mini_y']) for r in rows])
    mini_z = np.array([float(r['mini_z']) for r in rows])
    mini_vx = np.array([float(r['mini_vx']) for r in rows])
    mini_vy = np.array([float(r['mini_vy']) for r in rows])
    rel_dist = np.array([float(r['relative_distance']) for r in rows])
    along_err = np.array([float(r['controller_terminal_along_error']) for r in rows])
    lat_err = np.array([float(r['controller_terminal_lateral_error']) for r in rows])
    vert_err = np.array([float(r['controller_terminal_vertical_error']) for r in rows])
    target_along_spd = np.array([float(r['controller_target_along_speed']) for r in rows])

    segs = phase_segments(t, phase)
    car_speed = np.sqrt(car_vx**2 + car_vy**2)
    cmd_speed = np.sqrt(car_cmd_vx**2 + car_cmd_vy**2)
    mini_speed = np.sqrt(mini_vx**2 + mini_vy**2)

    fig, axes = plt.subplots(2, 3, figsize=(28, 16))
    (ax_traj, ax_zoom, ax_dist), (ax_speed, ax_along, ax_phase) = axes

    # ═══ Panel 1: Full XY trajectory ═══
    for si, ei, ph in segs:
        c = PHASE_COLORS.get(ph, '#888888')
        ax_traj.plot(car_x[si:ei+1], car_y[si:ei+1], color=c, linewidth=1.0, alpha=0.85)
    ax_traj.plot(mini_x, mini_y, '--', color='#22aa22', linewidth=0.7, alpha=0.6, label='mini')
    # Orbit reference circle
    orbit_c = (10.0, -6.0)
    orbit = plt.Circle(orbit_c, 80.0, fill=False, color='gray', linestyle=':', linewidth=0.8, alpha=0.4)
    ax_traj.add_patch(orbit)
    # Carrier setpoint dots (sparse)
    step = max(1, len(t)//120)
    ax_traj.scatter(car_sp_x[::step], car_sp_y[::step], c='magenta', s=4, alpha=0.5, marker='.', label='carrier SP')
    # Start/end markers
    ax_traj.scatter(*orbit_c, c='gray', s=120, marker='+', linewidths=2)
    ax_traj.scatter(car_x[0], car_y[0], c='blue', s=60, marker='o', zorder=5)
    ax_traj.scatter(car_x[-1], car_y[-1], c='blue', s=60, marker='s', zorder=5)
    ax_traj.scatter(mini_x[0], mini_y[0], c='darkgreen', s=60, marker='o', zorder=5)
    # Label phases
    for si, ei, ph in segs:
        if ph in ('APPROACH', 'TRACKING', 'DOCKING'):
            mi = (si + ei)//2
            ax_traj.annotate(ph, (car_x[mi], car_y[mi]), fontsize=7, color=PHASE_COLORS.get(ph,'#888'),
                           ha='center', va='bottom', fontweight='bold')
    ax_traj.set_xlabel('X (m)'); ax_traj.set_ylabel('Y (m)')
    ax_traj.set_title('Full trajectory (magenta dots = carrier setpoint)')
    ax_traj.legend(fontsize=7, loc='upper right')
    ax_traj.set_aspect('equal'); ax_traj.grid(True, alpha=0.25)

    # ═══ Panel 2: Zoom on DOCKING region ═══
    dock_idx = [i for i,p in enumerate(phase) if p == 'DOCKING']
    if dock_idx:
        pad = 40
        z0, z1 = max(0, dock_idx[0]-pad), min(len(t)-1, dock_idx[-1]+pad)
        for si, ei, ph in segs:
            if ei < z0 or si > z1: continue
            s0, e0 = max(si, z0), min(ei, z1)
            c = PHASE_COLORS.get(ph, '#888888')
            ax_zoom.plot(car_x[s0:e0+1], car_y[s0:e0+1], color=c, linewidth=1.2, alpha=0.9)
        ax_zoom.plot(mini_x[z0:z1+1], mini_y[z0:z1+1], '--', color='#22aa22', linewidth=0.8, alpha=0.6)
        # Setpoints
        ax_zoom.scatter(car_sp_x[z0:z1+1:3], car_sp_y[z0:z1+1:3], c='magenta', s=6, alpha=0.5, marker='.')
        # Command arrows
        arr_step = max(1, (z1-z0)//25)
        for i in range(z0, z1, arr_step):
            spd = cmd_speed[i]
            if spd > 0.3:
                dx = car_cmd_vx[i]/spd * 3.0
                dy = car_cmd_vy[i]/spd * 3.0
                ax_zoom.arrow(car_x[i], car_y[i], dx, dy, head_width=1.0, head_length=1.5,
                             fc='red', ec='red', alpha=0.4, width=0.2)
        # Connect carrier-mini with lines at key points
        for frac in [0.0, 0.3, 0.6, 0.9]:
            i = z0 + int((z1-z0)*frac)
            ax_zoom.plot([car_x[i], mini_x[i]], [car_y[i], mini_y[i]], '-', color='#ff9900', linewidth=0.5, alpha=0.5)
            ax_zoom.annotate(f'{rel_dist[i]:.1f}m', ((car_x[i]+mini_x[i])/2, (car_y[i]+mini_y[i])/2),
                           fontsize=6, color='#ff6600', ha='center')
        # Phase labels
        for si, ei, ph in segs:
            if ph in ('TRACKING','DOCKING') and ei >= z0 and si <= z1:
                mi = max(si, z0) + (min(ei, z1) - max(si, z0))//2
                ax_zoom.annotate(ph, (car_x[mi], car_y[mi]), fontsize=8, color=PHASE_COLORS.get(ph,'#888'),
                               ha='center', va='bottom', fontweight='bold')
        ax_zoom.set_xlabel('X (m)'); ax_zoom.set_ylabel('Y (m)')
        ax_zoom.set_title('Zoom: DOCKING region (red arrows=carrier cmd vel, orange=relative distance)')
        ax_zoom.set_aspect('equal'); ax_zoom.grid(True, alpha=0.25)

    # ═══ Panel 3: Relative distance ═══
    for si, ei, ph in segs:
        c = PHASE_COLORS.get(ph, '#888888')
        ax_dist.axvspan(t[si], t[ei], color=c, alpha=0.1)
    ax_dist.plot(t, rel_dist, color='#333333', linewidth=1.0)
    ax_dist.axhline(y=3.0, color='orange', linestyle='--', linewidth=0.7, alpha=0.5, label='3m')
    ax_dist.axhline(y=1.2, color='red', linestyle='--', linewidth=0.7, alpha=0.5, label='1.2m')
    ax_dist.set_xlabel('Time (s)'); ax_dist.set_ylabel('Distance (m)')
    ax_dist.set_title('Relative distance (dashed=thresholds)')
    ax_dist.legend(fontsize=7); ax_dist.grid(True, alpha=0.25)

    # ═══ Panel 4: Speed profiles ═══
    for si, ei, ph in segs:
        c = PHASE_COLORS.get(ph, '#888888')
        ax_speed.axvspan(t[si], t[ei], color=c, alpha=0.07)
    ax_speed.plot(t, car_speed, color='#3366cc', linewidth=1.0, label='carrier actual')
    ax_speed.plot(t, cmd_speed, color='#cc3333', linewidth=0.8, linestyle='--', label='carrier cmd')
    ax_speed.plot(t, mini_speed, color='#22aa22', linewidth=0.7, alpha=0.7, label='mini')
    ax_speed.set_xlabel('Time (s)'); ax_speed.set_ylabel('Horizontal speed (m/s)')
    ax_speed.set_title('Speed: carrier actual vs commanded vs mini')
    ax_speed.legend(fontsize=7); ax_speed.grid(True, alpha=0.25)

    # ═══ Panel 5: Along-track & lateral errors ═══
    for si, ei, ph in segs:
        c = PHASE_COLORS.get(ph, '#888888')
        ax_along.axvspan(t[si], t[ei], color=c, alpha=0.07)
    ax_along.plot(t, along_err, color='#3366cc', linewidth=1.0, label='along error (>0=carrier behind)')
    ax_along.plot(t, lat_err, color='#cc6600', linewidth=0.8, label='lateral error')
    ax_along.axhline(y=0, color='#333333', linewidth=0.5)
    ax_along.set_xlabel('Time (s)'); ax_along.set_ylabel('Error (m)')
    ax_along.set_title('Along-track error (carrier vs mini in velocity frame)')
    ax_along.legend(fontsize=7); ax_along.grid(True, alpha=0.25)

    # ═══ Panel 6: Phase timeline + altitude ═══
    phase_map = {'IDLE':0,'TAKEOFF':1,'APPROACH':2,'TRACKING':3,'DOCKING':4,'COMPLETED':5,'FAILED':6}
    phase_num = np.array([phase_map.get(p, -1) for p in phase])
    for si, ei, ph in segs:
        c = PHASE_COLORS.get(ph, '#888888')
        ax_phase.fill_between([t[si], t[ei]], phase_num[si]-0.35, phase_num[si]+0.35, color=c, alpha=0.25)
    ax_phase.step(t, phase_num, where='post', color='#333333', linewidth=1.5)
    ax_phase.set_yticks(list(phase_map.values())); ax_phase.set_yticklabels(list(phase_map.keys()), fontsize=8)
    ax_phase.set_xlabel('Time (s)'); ax_phase.set_title('Phase timeline')
    ax_phase.grid(True, alpha=0.3, axis='y')
    # Altitude overlay
    ax_z = ax_phase.twinx()
    ax_z.plot(t, car_z, color='#3366cc', linewidth=0.7, alpha=0.4, label='carrier Z')
    ax_z.plot(t, mini_z, color='#22aa22', linewidth=0.7, alpha=0.4, label='mini Z')
    ax_z.set_ylabel('Altitude (m)', fontsize=8, color='#666666')
    ax_z.legend(fontsize=7, loc='lower right')

    fig.tight_layout()
    out_path = out / "docking_planner_diagnostic.png"
    fig.savefig(out_path, dpi=150)
    print(f"Saved: {out_path}")
    plt.close(fig)

if __name__ == "__main__":
    main()
