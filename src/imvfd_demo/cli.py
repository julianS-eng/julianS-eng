"""Command-line entry point that regenerates every figure and number in the README.

Run with ``generate-report`` (installed console script) or
``python -m imvfd_demo.cli``. Writes PNG figures into ``docs/img/`` and
prints a Markdown metrics table plus the Monte Carlo sensitivity summary to
stdout, so the numbers pasted into the README can be copied verbatim from a
real run instead of being retyped from memory.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use("Agg")

from imvfd_demo.control import VfControlLaw
from imvfd_demo.motor_model import EXAMPLE_MOTOR, MotorParameters
from imvfd_demo.simulate import (
    monte_carlo_breakdown_torque,
    operating_point_metrics,
    torque_speed_curve,
)

DEFAULT_FREQUENCIES_HZ = (15.0, 30.0, 45.0, 60.0)


def _plot_torque_speed_curves(
    motor: MotorParameters,
    control: VfControlLaw,
    frequencies_hz: tuple[float, ...],
    output_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(7.0, 5.0), dpi=150)
    for f in frequencies_hz:
        curve = torque_speed_curve(motor, control, f)
        ax.plot(curve.speed_rpm, curve.torque_nm, label=f"{f:.0f} Hz")
    ax.set_xlabel("Rotor speed (rpm)")
    ax.set_ylabel("Electromagnetic torque (N*m)")
    ax.set_title("Torque-speed curves under scalar V/Hz control")
    ax.axhline(0.0, color="black", linewidth=0.6)
    ax.legend(title="Drive frequency")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def _plot_sensitivity_histogram(
    torques_nm: np.ndarray, output_path: Path, frequency_hz: float
) -> None:
    fig, ax = plt.subplots(figsize=(7.0, 5.0), dpi=150)
    ax.hist(torques_nm, bins=60, color="#3B6FA0", edgecolor="white", linewidth=0.3)
    ax.set_xlabel("Breakdown torque (N*m)")
    ax.set_ylabel("Monte Carlo sample count")
    ax.set_title(f"Breakdown torque under +/-10% rotor R2/X2 tolerance @ {frequency_hz:.0f} Hz")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def _metrics_markdown_table(
    motor: MotorParameters, control: VfControlLaw, frequencies_hz: tuple[float, ...]
) -> str:
    header = (
        "| f (Hz) | V line (V) | n_sync (rpm) | Starting T (N*m) "
        "| Breakdown T (N*m) | Breakdown speed (rpm) | Breakdown slip |"
    )
    separator = "|---|---|---|---|---|---|---|"
    rows = [header, separator]
    for f in frequencies_hz:
        m = operating_point_metrics(motor, control, f)
        rows.append(
            f"| {m.frequency_hz:.0f} | {m.line_voltage_v:.1f} | "
            f"{m.synchronous_speed_rpm:.0f} | {m.starting_torque_nm:.2f} | "
            f"{m.breakdown_torque_nm:.2f} | {m.breakdown_speed_rpm:.0f} | "
            f"{m.breakdown_slip:.4f} |"
        )
    return "\n".join(rows)


def main(argv: list[str] | None = None) -> int:
    """Generate all report figures and print the metrics tables. Returns exit code 0."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--img-dir",
        type=Path,
        default=Path("docs/img"),
        help="Directory to write PNG figures into (default: docs/img)",
    )
    parser.add_argument(
        "--sensitivity-frequency-hz",
        type=float,
        default=60.0,
        help="Frequency at which to run the Monte Carlo breakdown-torque study",
    )
    parser.add_argument("--n-samples", type=int, default=5000, help="Monte Carlo sample count")
    parser.add_argument("--seed", type=int, default=42, help="Monte Carlo RNG seed")
    args = parser.parse_args(argv)

    args.img_dir.mkdir(parents=True, exist_ok=True)

    motor = EXAMPLE_MOTOR
    control = VfControlLaw(
        rated_voltage_v=motor.rated_voltage_v,
        rated_frequency_hz=motor.rated_frequency_hz,
        boost_voltage_v=8.0,
    )

    _plot_torque_speed_curves(
        motor, control, DEFAULT_FREQUENCIES_HZ, args.img_dir / "torque_speed_curves.png"
    )

    sensitivity = monte_carlo_breakdown_torque(
        motor,
        control,
        frequency_hz=args.sensitivity_frequency_hz,
        n_samples=args.n_samples,
        seed=args.seed,
    )
    _plot_sensitivity_histogram(
        sensitivity.breakdown_torques_nm,
        args.img_dir / "breakdown_torque_sensitivity.png",
        args.sensitivity_frequency_hz,
    )

    print("## Operating-point metrics (EXAMPLE_MOTOR, scalar V/Hz control)\n")
    print(_metrics_markdown_table(motor, control, DEFAULT_FREQUENCIES_HZ))
    print()
    print(
        f"## Monte Carlo breakdown-torque sensitivity @ "
        f"{sensitivity.frequency_hz:.0f} Hz "
        f"(n={sensitivity.n_samples}, seed={sensitivity.seed}, "
        f"R2/X2 tolerance +/-10%)\n"
    )
    print(f"- mean breakdown torque: {sensitivity.mean_nm:.3f} N*m")
    print(f"- std dev: {sensitivity.std_nm:.3f} N*m")
    print(f"- coefficient of variation: {sensitivity.coefficient_of_variation_pct:.2f}%")
    print(
        f"- min / max: {sensitivity.breakdown_torques_nm.min():.3f} / "
        f"{sensitivity.breakdown_torques_nm.max():.3f} N*m"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
