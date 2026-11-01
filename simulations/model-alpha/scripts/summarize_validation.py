"""Summarize actual Webots reports and optionally plot recorded telemetry."""

from pathlib import Path
import csv
import hashlib
import json
import math
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
RESULTS = MODEL / "results"
SCENARIOS = ["balance", "tilt_positive", "tilt_negative", "push", "steering", "drive"]
NAMES = {
    "balance": "Steady hold",
    "tilt_positive": "Start tilted +3 degrees",
    "tilt_negative": "Start tilted -3 degrees",
    "push": "1 N push for 0.15 s",
    "steering": "Left/right turns",
    "drive": "Slow forward/backward travel",
}


def result_folder(scenario):
    folder = RESULTS / "dynamics"
    return folder if scenario == "balance" else folder / scenario


def telemetry(scenario):
    with (result_folder(scenario) / "telemetry.csv").open(newline="", encoding="utf-8") as stream:
        return [{key: (value if key == "pose" else float(value)) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def main():
    if len(json.loads((MODEL/'parameters.json').read_text()).get('finger_angles_rad', [0, 1, 2]))==2:
        from summarize_two_finger import main as current_summary
        return current_summary()
    params = json.loads((MODEL / "parameters.json").read_text())
    source_paths = [WORKSPACE / "simulations" / "common" / name for name in
                    ["robot_controller.py", "generate_worlds.py", "control_math.py", "mass_properties.py"]] + [MODEL / "parameters.json"]
    expected_hashes = {str(path.relative_to(WORKSPACE)): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths}
    rows = []
    for scenario in SCENARIOS:
        report = json.loads((result_folder(scenario) / "report.json").read_text())
        if report["scenario"] != scenario or report["model"] != "Alpha":
            raise ValueError(f"Wrong model/scenario report: {scenario}")
        if report["balance_gains"] != params["balance_gains"]:
            raise ValueError(f"Stale gains in report: {scenario}")
        if report.get("source_sha256") != expected_hashes:
            raise ValueError(f"Stale source/configuration in report: {scenario}")
        rows.append({
            "scenario": scenario, "description": NAMES[scenario],
            "passed": report["dynamics_validation_passed"], "duration_s": report["time_s"],
            "peak_pitch_deg": math.degrees(report["max_abs_pitch_rad"]),
            "final_drift_mm": report["final_drift_m"] * 1000,
            "max_displacement_m": report["max_displacement_m"],
            "peak_heading_deg": math.degrees(report["max_abs_yaw_rad"]),
            "final_heading_deg": math.degrees(report["final_yaw_rad"]),
            "peak_wheel_torque_nm": report["max_applied_wheel_torque_nm"],
            "peak_wheel_speed_rad_s": report["max_wheel_joint_speed_rad_s"],
            "criteria": report["dynamics_checks"],
        })
    source_paths = [WORKSPACE / "simulations" / "common" / name for name in
                    ["robot_controller.py", "generate_worlds.py", "control_math.py", "mass_properties.py"]]
    source_paths.append(MODEL / "parameters.json")
    summary = {
        "recorded_date": "2026-10-23", "timezone": "Asia/Calcutta", "model": "Alpha",
        "all_base_cases_passed": all(row["passed"] for row in rows), "cases": rows,
        "source_sha256": {str(path.relative_to(WORKSPACE)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in source_paths},
        "limitations": ["Idealized simulated sensors and actuators; no hardware validation.",
                        "These six cases cover a folded arm only. See ARM_CONTROL_RESULTS.md and GRASP_RESULTS.md for separate arm and prepared-sphere experiments.",
                        "Self-collision enabled for nonadjacent solids. Only the recorded paths were checked; joint pockets and full workspace remain unvalidated.",
                        "Mass distribution is a provisional design assumption, not a measured battery/electronics layout.",
                        "No broad friction, backlash, delay, electrical or payload uncertainty sweep."],
    }
    (RESULTS / "base-validation-summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    text = ["# Alpha base-control results", "", "Recorded: 23 October 2026 (Asia/Calcutta).",
            "", "Actual Webots R2025a runs, with the arm held folded. Each run lasted 30 simulated seconds.", "",
            "| Test | Result | Peak pitch (deg) | Final drift (mm) | Peak wheel torque (N m) |",
            "| --- | --- | ---: | ---: | ---: |"]
    for row in rows:
        text.append(f"| {row['description']} | {'PASS' if row['passed'] else 'FAIL'} | {row['peak_pitch_deg']:.3f} | {row['final_drift_mm']:.2f} | {row['peak_wheel_torque_nm']:.4f} |")
    text += ["", "## Model changes", "",
             f"The lumped inner-body mass is {params['body_mass']:.2f} kg with its assumed COM {abs(params['body_com_x'])*1000:.1f} mm behind the axle center. This centers the folded assembly; the modeled total mass is 1.223 kg.",
             "Wheel feedback uses bounded torque commands and encoder odometry corrected for chassis pitch. The wheel torque limit remains 0.45 N m per wheel, and the speed limit is 14 rad/s. These are assumed limits, not selected motors.",
             "", "## Acceptance checks", "",
             "Complete 30 s without falling; peak pitch below 20 degrees; final pitch below 1 degree; final planar drift below 5 cm; wheel speeds within the assumed limit (0.5 rad/s numerical allowance). The steering case must exceed 1 rad heading change and return within 10 degrees. The drive case must travel over 5 cm.",
             "", "## Limits", ""]
    text.extend("- " + limitation for limitation in summary["limitations"])
    text += ["", "The first failed controller and the mass-layout-only trial remain in the model archive. All reports and CSV files are retained beside this summary. Source/configuration hashes are in base-validation-summary.json.", ""]
    (RESULTS / "BASE_CONTROL_RESULTS.md").write_text("\n".join(text), encoding="utf-8")

    plot_packages = WORKSPACE / ".local-tools" / "plotting-packages"
    if plot_packages.exists():
        sys.path.insert(0, str(plot_packages))
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Reports written; matplotlib unavailable, so no plot generated.")
    else:
        fig, axes = plt.subplots(2, 2, figsize=(12, 7.6))
        for scenario, color in [("tilt_positive", "#2563a6"), ("tilt_negative", "#c25b22"), ("push", "#23856e")]:
            data = telemetry(scenario)
            axes[0, 0].plot([r["time_s"] for r in data], [math.degrees(r["pitch_rad"]) for r in data],
                            label=NAMES[scenario], color=color, linewidth=1.4)
        axes[0, 0].set(title="Recovery with arm folded", ylabel="Body pitch (degrees)")
        axes[0, 0].legend(fontsize=8)
        steering = telemetry("steering")
        axes[0, 1].plot([r["time_s"] for r in steering], [math.degrees(r["yaw_rad"]) for r in steering], color="#2563a6")
        axes[0, 1].set(title="Wheel steering while balancing", ylabel="Heading (degrees)")
        drive = telemetry("drive")
        times = [r["time_s"] for r in drive]
        axes[1, 0].plot(times, [r["x_m"] for r in drive], label="Simulated axle position", color="#2563a6")
        reference = [0.04 * max(0, min(t-6, 6)) - 0.04 * max(0, min(t-18, 6)) for t in times]
        axes[1, 0].plot(times, reference, "--", label="Position reference", color="#687785")
        axes[1, 0].set(title="Slow forward/backward travel", ylabel="Forward displacement (m)")
        axes[1, 0].legend(fontsize=8)
        st = [r["time_s"] for r in steering]
        axes[1, 1].plot(st, [r["left_command"] for r in steering], label="Left", color="#2563a6")
        axes[1, 1].plot(st, [r["right_command"] for r in steering], label="Right", color="#c25b22")
        for limit in [-params["wheel_torque_limit"], params["wheel_torque_limit"]]:
            axes[1, 1].axhline(limit, color="#89939c", linestyle="--", linewidth=1)
        axes[1, 1].set(title="Steering torque within assumed limits", ylabel="Wheel torque (N m)")
        axes[1, 1].legend(fontsize=8)
        for ax in axes.flat:
            ax.set_xlabel("Simulation time (s)")
            ax.grid(alpha=0.22)
            ax.spines[["top", "right"]].set_visible(False)
        fig.suptitle("Model Alpha: initial base-control validation", fontsize=16)
        fig.text(0.5, 0.015, "Idealized simulation | provisional mass layout | arm fixed | no hardware or full-workspace validation", ha="center", fontsize=9, color="#555f68")
        fig.tight_layout(rect=(0, 0.04, 1, 0.95))
        fig.savefig(RESULTS / "base-control-validation.png", dpi=160)
        plt.close(fig)
    print(json.dumps({"all_base_cases_passed": summary["all_base_cases_passed"], "tests": len(rows)}, indent=2))
    if not summary["all_base_cases_passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
