"""54 assumed mounts/boxes. Default: analytic artifacts only; --render uses CGL."""
import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from sim.models import make_wbr_3d_simple as gen
from tools.ground_plane.synth import FY
from tools.simcam import scenario as sc
from .geometry import vertical_fov, visibility

PITCHES = (0, 5, 10, 15, 20, 25)
HEIGHTS = (0.15, 0.21, 0.30)
BOXES = (0.046, 0.110, 0.132)
OUT = Path("tools/mount_study/results")


@dataclass(frozen=True)
class Config:
    height_m: float
    pitch_deg: float
    box_m: float

    @property
    def key(self):
        return f"H{self.height_m:.3f}_P{self.pitch_deg:02g}_B{self.box_m:.3f}"

    @property
    def l0(self):
        return min(.20, max(.06, self.height_m-gen.R_WHEEL-gen.HIP_BELOW_TOP))

    @property
    def camera_body_z(self):
        body_world_z = gen.R_WHEEL + self.l0 + gen.HIP_BELOW_TOP-gen.BODY_HALF[2]
        return self.height_m-body_world_z

    @property
    def expected(self):
        return "STOP_CANDIDATE" if self.box_m > .12 else "JUMP_CANDIDATE"


def configurations():
    return [Config(h, p, b) for h in HEIGHTS for p in PITCHES for b in BOXES]


def make_scenario(c):
    # Start at 2 m CAMERA-to-front-face gap. End at positive 0.025 m gap.
    return sc.validate_scenario(dict(
        name=c.key, description="ASSUMED mount; kinematic approach, no contact",
        status="ASSUMED", robot=dict(start_x=-2-sc.CAMERA_POS[0], speed_m_s=.25,
        duration_s=7.9, l0=c.l0, jump_at_s=None), obstacles=[dict(
        type="box", x_face=0., depth=.20, width=.40, height=c.box_m)]))


def build_xml(c):
    root = ET.fromstring(sc.build_xml(make_scenario(c)))
    camera = root.find(f".//camera[@name='{sc.CAMERA_NAME}']")
    camera.set("pos", f"{sc.CAMERA_POS[0]} 0 {c.camera_body_z:.12g}")
    angle = math.radians(c.pitch_deg)
    camera.set("xyaxes", f"0 -1 0 {math.sin(angle):.12g} 0 {math.cos(angle):.12g}")
    return ET.tostring(root, encoding="unicode")


def aggregate(rows, c):
    """Summarize actual observations, never stale EMA as a fresh detection.

    Plane truth test: fresh fit, height within 2 cm, pitch within 2 deg,
    roll within 2 deg of the known level simulation ground.
    """
    def endpoints(selected, prefix):
        result = {f"{prefix}_{k}": None for k in
                  ("first_tick", "last_tick", "far_gap_m", "blind_gap_m")}
        if selected:
            result.update({f"{prefix}_first_tick": selected[0]["t"],
                           f"{prefix}_last_tick": selected[-1]["t"],
                           f"{prefix}_far_gap_m": max(r["gap_m"] for r in selected),
                           f"{prefix}_blind_gap_m": min(r["gap_m"] for r in selected)})
        return result
    valid = [r for r in rows if r["precontact"]]
    advisory = [r for r in valid if r["raw_advisory"] == c.expected]
    observed = [r for r in advisory if r["fresh_correct"]]
    fresh = [r for r in valid if r["fresh_correct"]]
    return dict(**endpoints(advisory, "advisory"), **endpoints(observed, "supported"),
                **endpoints(fresh, "detection"), ticks=len(valid),
                plane_valid_fraction=sum(r["plane_valid"] for r in valid)/len(valid) if valid else None,
                plane_truth_fraction=sum(r["plane_truth"] for r in valid)/len(valid) if valid else None,
                plane_reused_ticks=sum(r["plane_reused"] for r in valid),
                supported_ticks=len(observed),
                supported_runs=sum(1 for i,r in enumerate(valid) if r in observed and
                                   (i == 0 or valid[i-1] not in observed)),
                start_censored=bool(observed and observed[0] is valid[0]),
                end_censored=bool(observed and observed[-1] is valid[-1]))


def render_timeline(c, path):
    """Only entry point importing MuJoCo or constructing a graphics context."""
    import mujoco
    from tools.simcam import run as sr
    from tools.simcam.depth import clipping_distances, optical_depth
    model = mujoco.MjModel.from_xml_string(build_xml(c))
    data = mujoco.MjData(model)
    near, far = clipping_distances(model.vis.map.znear, model.vis.map.zfar, model.stat.extent)
    camera_id = model.camera(sc.CAMERA_NAME).id
    K, loop = sc.intrinsics(), sr.PerceptionLoop()
    scenario = make_scenario(c)
    rows = []
    path.parent.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(model, height=sc.HEIGHT, width=sc.WIDTH) as renderer:
        with path.open("w", newline="") as stream:
            writer = None
            for t in sr.tick_times(scenario["robot"]["duration_s"]):
                pose = sr.pose_at(t, scenario["robot"])
                robot_x = sr.place_robot(model, data, scenario["robot"], t, pose)
                sr.update_camera(renderer, data, K, near)
                renderer.enable_depth_rendering()
                depth = optical_depth(renderer.render(), near, far)
                frame, decision, gated, reset = loop.update(t, depth, pose, .25)
                H, pitch = sr.camera_measurement(data, camera_id)
                row = sr.timeline_row(scenario, t, robot_x, H, pitch, pose,
                                      frame, decision, gated, reset, 0.)
                gap = -float(data.cam_xpos[camera_id, 0])
                truth = (frame.plane_valid and not frame.plane_reused and
                         abs(frame.plane_height_m-H) <= .02 and
                         abs(math.degrees(frame.plane_pitch_rad)-pitch) <= 2 and
                         abs(math.degrees(frame.plane_roll_rad)) <= 2)
                ob = frame.nearest
                fresh = bool(truth and ob is not None and abs(ob.height_m-c.box_m) <= .015
                             and gap-.02 <= ob.distance_m <= gap+.20+.02)
                row.update(gap_m=gap, plane_truth=truth,
                           plane_height_error_m=frame.plane_height_m-H,
                           plane_pitch_error_deg=math.degrees(frame.plane_pitch_rad)-pitch,
                           cluster_distance_m=ob.distance_m if ob else None,
                           cluster_height_m=ob.height_m if ob else None,
                           fresh_correct=fresh,
                           precontact=robot_x+max(gen.BODY_HALF[0]+.020,
                               gen.R_WHEEL) < 0)
                if writer is None:
                    writer = csv.DictWriter(stream, fieldnames=list(row))
                    writer.writeheader()
                writer.writerow(row)
                stream.flush()
                rows.append(row)
    return rows


def plots(metrics, out):
    import matplotlib.pyplot as plt
    for name, simulated in (("blind_vs_pitch", False), ("analytic_vs_simulated", True)):
        fig, axes = plt.subplots(1, 3, figsize=(13, 4), sharey=True)
        for ax, H in zip(axes, HEIGHTS):
            for b in BOXES:
                group = [r for r in metrics if r["height_m"] == H and r["box_m"] == b]
                ax.plot([r["pitch_deg"] for r in group], [r["top_near_m"] for r in group],
                        "--", label=f"{b:.3f} m front edge (calculated)")
                if simulated:
                    ax.plot([r["pitch_deg"] for r in group],
                            [r.get("supported_blind_gap_m") if r.get("supported_blind_gap_m")
                             is not None else math.nan for r in group], "o-", label=f"{b:.3f} m supported")
            ax.set_title(f"Camera height {H:.2f} m")
            ax.set_xlabel("Pitch down (deg)")
            ax.grid(True)
        axes[0].set_ylabel("Camera-to-front-face gap (m)")
        axes[-1].legend(fontsize=7)
        fig.suptitle("CALCULATED front-edge visibility; simulated marks only when rendered")
        fig.tight_layout()
        fig.savefig(out/f"{name}.png", dpi=160)
        plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--render", action="store_true")
    p.add_argument("--out", type=Path, default=OUT)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    metrics = []
    for c in configurations():
        row = dict(**asdict(c), l0_m=c.l0, camera_body_z_m=c.camera_body_z,
                   expected=c.expected, **visibility(c.height_m, c.box_m, c.pitch_deg, vertical_fov(FY)),
                   **aggregate([], c), simulation_status="PENDING_RENDER")
        if a.render:
            row.update(aggregate(render_timeline(c, a.out/"timelines"/f"{c.key}.csv"), c),
                       simulation_status="MEASURED_IN_SIMULATION")
        # JSON cannot represent infinity: null + explicit upper-ray interpretation.
        row["top_far_m"] = None if math.isinf(row["top_far_m"]) else row["top_far_m"]
        metrics.append(row)
        print(f"{c.key}: {row['simulation_status']}", flush=True)
    with (a.out/"metrics.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(metrics[0]))
        writer.writeheader()
        writer.writerows(metrics)
    (a.out/"summary.json").write_text(json.dumps(dict(
        geometry_status="CALCULATED", simulation_status=metrics[0]["simulation_status"],
        vfov_deg=vertical_fov(FY), speed_m_s=.25, tick_hz=10, gap_step_m=.025,
        top_far_null_means="unbounded geometric top-edge range, not detector range",
        metrics=metrics), indent=2, allow_nan=False)+"\n")
    plots(metrics, a.out)
    write_table(metrics, a.out)


def write_table(metrics, out):
    lines = ["# Mount sweep table", "", "Distances are camera-to-front-face gaps in metres. "
             "Pending values are not measurements; near is CALCULATED front top-edge visibility.", "",
             "| H | Pitch | Box | Near | Supported blind | Supported far | Plane valid / truth | Status |",
             "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    pending = metrics[0]["simulation_status"] == "PENDING_RENDER"
    def fmt(value):
        return ("PENDING" if pending else "NONE") if value is None else f"{value:.3f}"
    for r in metrics:
        lines.append(f"| {r['height_m']:.2f} | {r['pitch_deg']} | {r['box_m']:.3f} | "
                     f"{r['top_near_m']:.3f} | {fmt(r.get('supported_blind_gap_m'))} | "
                     f"{fmt(r.get('supported_far_gap_m'))} | {fmt(r.get('plane_valid_fraction'))} / "
                     f"{fmt(r.get('plane_truth_fraction'))} | {r['simulation_status']} |")
    if metrics[0]["simulation_status"] != "PENDING_RENDER":
        lines += ["", "Blank support metrics in CSV/JSON (NONE in this table) mean no "
                  "supported correct advisory, not zero blind distance."]
    (out/"TABLE.md").write_text("\n".join(lines)+"\n")


if __name__ == "__main__":
    main()
