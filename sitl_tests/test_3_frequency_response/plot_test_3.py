###############################################################################
# Author: Luca Boninsegna
# Date:   29/07/2026
# Descr:  Post-processing for Test 3 — Frequency Response / Bode Plots.
#         Reads all CSV files produced by test_3_frequency_response.py,
#         computes gain and phase at each frequency, and generates Bode plots.
#
#         Method: For each frequency CSV, we extract the SWEEP phase, then
#         fit sinusoids to the input signal and the output response to compute
#         amplitude ratio (gain) and phase shift.
#
# Usage:  python graphs_generation/plot_test_3.py
#         python graphs_generation/plot_test_3.py --axis yaw
###############################################################################

import argparse
import glob
import os
import re
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


def find_bode_csvs(axis="yaw"):
    """Find all Test 3 CSV files for the given axis, sorted by frequency."""
    logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    pattern = os.path.join(logs_dir, f"test_3_bode_{axis}_*.csv")
    files = glob.glob(pattern)

    if not files:
        return [], []

    # Extract frequencies from filenames
    freq_files = []
    for f in files:
        match = re.search(r"_(\d+\.\d+)Hz\.csv$", f)
        if match:
            freq = float(match.group(1))
            freq_files.append((freq, f))

    freq_files.sort(key=lambda x: x[0])
    frequencies = [ff[0] for ff in freq_files]
    filepaths = [ff[1] for ff in freq_files]

    return frequencies, filepaths


def sine_model(t, amplitude, frequency, phase, offset):
    """Sinusoidal model for curve fitting."""
    return amplitude * np.sin(2 * np.pi * frequency * t + phase) + offset


def compute_gain_phase(csv_path, freq_hz, axis="yaw"):
    """
    Compute gain and phase for a single frequency experiment.

    Args:
        csv_path: Path to the frequency CSV file
        freq_hz: Expected oscillation frequency in Hz
        axis: Which axis was oscillated

    Returns:
        dict with: gain_linear, gain_db, phase_deg, input_amplitude, output_amplitude
    """
    df = pd.read_csv(csv_path)

    # Only use the SWEEP phase (skip settle time)
    sweep_mask = df["phase"] == "SWEEP"
    sweep_df = df[sweep_mask].copy()

    if len(sweep_df) < 20:
        print(f"  WARNING: Only {len(sweep_df)} samples in SWEEP phase for {freq_hz} Hz")
        return None

    # Time relative to sweep start
    t = sweep_df["time_s"].values
    t_rel = t - t[0]

    # Input signal (the sinusoidal target offset)
    input_signal = sweep_df["input_signal"].values

    # Output signal depends on the axis
    if axis == "yaw":
        # For yaw: the response is the horizontal error e_x
        # The drone tries to reduce e_x to 0, so the "output" is the actual
        # yaw tracking. We can also look at drone_y vs target_y directly.
        output_signal = sweep_df["e_x"].values
        output_label = "e_x (horizontal error)"
    elif axis == "altitude":
        output_signal = sweep_df["e_y"].values
        output_label = "e_y (vertical error)"
    else:  # distance
        output_signal = sweep_df["e_area"].values
        output_label = "e_area (area error)"

    # Fit sinusoid to input signal
    try:
        # Initial guess
        p0_in = [np.max(np.abs(input_signal)), freq_hz, 0.0, np.mean(input_signal)]
        popt_in, _ = curve_fit(sine_model, t_rel, input_signal, p0=p0_in,
                               maxfev=10000)
        input_amp = abs(popt_in[0])
        input_phase = popt_in[2]
    except Exception as e:
        print(f"  WARNING: Could not fit input sinusoid at {freq_hz} Hz: {e}")
        # Fallback: use FFT
        input_amp = np.max(np.abs(input_signal)) * 0.707  # RMS approximation
        input_phase = 0.0

    # Fit sinusoid to output signal
    try:
        p0_out = [np.max(np.abs(output_signal)) * 0.5, freq_hz, 0.0, np.mean(output_signal)]
        popt_out, _ = curve_fit(sine_model, t_rel, output_signal, p0=p0_out,
                                maxfev=10000)
        output_amp = abs(popt_out[0])
        output_phase = popt_out[2]
    except Exception as e:
        print(f"  WARNING: Could not fit output sinusoid at {freq_hz} Hz: {e}")
        # Fallback: FFT-based
        output_amp = np.std(output_signal) * np.sqrt(2)
        output_phase = 0.0

    # Compute gain and phase
    if input_amp > 1e-6:
        gain_linear = output_amp / input_amp
    else:
        gain_linear = 0.0

    gain_db = 20 * np.log10(gain_linear) if gain_linear > 1e-10 else -100.0
    phase_diff = np.degrees(output_phase - input_phase)

    # Wrap phase to [-180, 180]
    while phase_diff > 180:
        phase_diff -= 360
    while phase_diff < -180:
        phase_diff += 360

    return {
        "freq_hz": freq_hz,
        "input_amplitude": input_amp,
        "output_amplitude": output_amp,
        "gain_linear": gain_linear,
        "gain_db": gain_db,
        "phase_deg": phase_diff,
    }


def plot_bode(frequencies, gains_db, phases_deg, axis, output_dir):
    """Generate a Bode plot (gain + phase vs. frequency)."""

    fig, (ax_gain, ax_phase) = plt.subplots(2, 1, figsize=(12, 9), sharex=True)

    # ── Gain plot ───────────────────────────────────────────────────────────
    ax_gain.semilogx(frequencies, gains_db, 'b-o', linewidth=2, markersize=8,
                     markerfacecolor='white', markeredgewidth=2)
    ax_gain.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax_gain.axhline(y=-3, color='red', linestyle=':', alpha=0.5, label="-3 dB bandwidth")
    ax_gain.set_ylabel("Gain (dB)", fontsize=13)
    ax_gain.set_title(f"Bode Plot — {axis.upper()} Axis", fontsize=15, fontweight='bold')
    ax_gain.grid(True, which='both', alpha=0.3)
    ax_gain.legend(fontsize=10)

    # Find and annotate -3dB bandwidth
    gains_arr = np.array(gains_db)
    below_3db = np.where(gains_arr <= -3)[0]
    if len(below_3db) > 0:
        # Interpolate to find the -3dB frequency
        idx = below_3db[0]
        if idx > 0:
            # Linear interpolation between the two points
            f1, g1 = frequencies[idx - 1], gains_arr[idx - 1]
            f2, g2 = frequencies[idx], gains_arr[idx]
            if g1 != g2:
                f_3db = f1 + (f2 - f1) * (-3 - g1) / (g2 - g1)
            else:
                f_3db = f1
            ax_gain.axvline(x=f_3db, color='red', linestyle=':', alpha=0.3)
            ax_gain.annotate(f"BW ≈ {f_3db:.3f} Hz",
                           xy=(f_3db, -3), xytext=(f_3db * 2, -3 + 5),
                           arrowprops=dict(arrowstyle='->', color='red'),
                           fontsize=10, color='red')

    # ── Phase plot ──────────────────────────────────────────────────────────
    ax_phase.semilogx(frequencies, phases_deg, 'r-s', linewidth=2, markersize=8,
                      markerfacecolor='white', markeredgewidth=2)
    ax_phase.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax_phase.axhline(y=-90, color='orange', linestyle=':', alpha=0.5, label="-90° reference")
    ax_phase.axhline(y=-180, color='red', linestyle=':', alpha=0.5, label="-180° (instability)")
    ax_phase.set_ylabel("Phase (°)", fontsize=13)
    ax_phase.set_xlabel("Frequency (Hz)", fontsize=13)
    ax_phase.grid(True, which='both', alpha=0.3)
    ax_phase.legend(fontsize=10)

    # Find phase margin (phase at 0dB gain)
    zero_crossings = np.where(np.diff(np.sign(gains_arr)))[0]
    if len(zero_crossings) > 0:
        idx = zero_crossings[0]
        f1, g1 = frequencies[idx], gains_arr[idx]
        f2, g2 = frequencies[idx + 1], gains_arr[idx + 1]
        if g1 != g2:
            f_0db = f1 + (f2 - f1) * (0 - g1) / (g2 - g1)
            # Interpolate phase at this frequency
            p1, p2 = phases_deg[idx], phases_deg[idx + 1]
            phase_at_0db = p1 + (p2 - p1) * (f_0db - f1) / (f2 - f1)
            phase_margin = 180 + phase_at_0db

            ax_phase.annotate(f"PM ≈ {phase_margin:.1f}°\n@ {f_0db:.3f} Hz",
                            xy=(f_0db, phase_at_0db),
                            xytext=(f_0db * 0.3, phase_at_0db - 30),
                            arrowprops=dict(arrowstyle='->', color='orange'),
                            fontsize=10, color='orange', fontweight='bold')

    plt.tight_layout()

    plot_path = os.path.join(output_dir, f"test_3_bode_{axis}.pdf")
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.savefig(plot_path.replace('.pdf', '.png'), dpi=150, bbox_inches='tight')
    print(f"\nSaved: {plot_path}")
    print(f"Saved: {plot_path.replace('.pdf', '.png')}")
    plt.close()


def plot_individual_frequencies(frequencies, filepaths, axis, output_dir):
    """Generate per-frequency time-domain plots showing input vs output."""
    n_freqs = len(frequencies)
    if n_freqs == 0:
        return

    n_cols = min(3, n_freqs)
    n_rows = (n_freqs + n_cols - 1) // n_cols

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(6 * n_cols, 4 * n_rows),
                             squeeze=False)

    for i, (freq, filepath) in enumerate(zip(frequencies, filepaths)):
        row, col = divmod(i, n_cols)
        ax = axes[row][col]

        df = pd.read_csv(filepath)
        sweep = df[df["phase"] == "SWEEP"]

        if len(sweep) == 0:
            ax.set_title(f"{freq} Hz (no data)")
            continue

        t = sweep["time_s"].values - sweep["time_s"].values[0]
        input_sig = sweep["input_signal"].values

        if axis == "yaw":
            output_sig = sweep["e_x"].values
            out_label = "e_x"
        elif axis == "altitude":
            output_sig = sweep["e_y"].values
            out_label = "e_y"
        else:
            output_sig = sweep["e_area"].values
            out_label = "e_area"

        # Normalize both signals for comparison
        in_max = np.max(np.abs(input_sig)) if np.max(np.abs(input_sig)) > 1e-6 else 1
        out_max = np.max(np.abs(output_sig)) if np.max(np.abs(output_sig)) > 1e-6 else 1

        ax.plot(t, input_sig / in_max, 'b-', linewidth=1.2, alpha=0.7, label="Input (norm)")
        ax.plot(t, output_sig / out_max, 'r-', linewidth=1.2, alpha=0.7, label=f"{out_label} (norm)")
        ax.set_title(f"{freq} Hz", fontsize=11, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("Time (s)", fontsize=9)
        if col == 0:
            ax.set_ylabel("Normalized", fontsize=9)
        ax.legend(fontsize=7)

    # Hide unused subplots
    for i in range(n_freqs, n_rows * n_cols):
        row, col = divmod(i, n_cols)
        axes[row][col].set_visible(False)

    fig.suptitle(f"Test 3: Per-Frequency Input vs Output — {axis.upper()}", fontsize=14, fontweight='bold')
    plt.tight_layout()

    plot_path = os.path.join(output_dir, f"test_3_per_frequency_{axis}.pdf")
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.savefig(plot_path.replace('.pdf', '.png'), dpi=150, bbox_inches='tight')
    print(f"Saved: {plot_path}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(description="Generate Bode plots from Test 3 data")
    parser.add_argument("--axis", type=str, default="yaw",
                        choices=["yaw", "altitude", "distance"],
                        help="Axis to analyze (default: yaw)")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory for plots")
    args = parser.parse_args()

    frequencies, filepaths = find_bode_csvs(args.axis)

    if not frequencies:
        print(f"ERROR: No Test 3 CSVs found for axis '{args.axis}'.")
        print("  Run test_3_frequency_response.py first.")
        sys.exit(1)

    print(f"Found {len(frequencies)} frequency files for axis '{args.axis}':")
    for f, fp in zip(frequencies, filepaths):
        print(f"  {f:.3f} Hz → {fp}")

    output_dir = args.output_dir or os.path.dirname(filepaths[0])
    
    if os.path.basename(output_dir) == "logs":
        plots_dir = os.path.join(os.path.dirname(output_dir), "plots")
    else:
        plots_dir = os.path.join(output_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)
    # Compute gain/phase for each frequency
    results = []
    for freq, filepath in zip(frequencies, filepaths):
        print(f"\nProcessing {freq} Hz...")
        result = compute_gain_phase(filepath, freq, args.axis)
        if result is not None:
            results.append(result)
            print(f"  Gain: {result['gain_db']:.2f} dB | Phase: {result['phase_deg']:.1f}°")

    if not results:
        print("ERROR: Could not compute gain/phase for any frequency.")
        sys.exit(1)

    # Extract arrays for plotting
    freqs = [r["freq_hz"] for r in results]
    gains_db = [r["gain_db"] for r in results]
    phases_deg = [r["phase_deg"] for r in results]

    # Generate Bode plot
    plot_bode(freqs, gains_db, phases_deg, args.axis, plots_dir)

    # Generate per-frequency time domain plots
    plot_individual_frequencies(frequencies, filepaths, args.axis, plots_dir)

    # Save results table to CSV
    results_df = pd.DataFrame(results)
    results_csv = os.path.join(output_dir, f"test_3_bode_results_{args.axis}.csv")
    results_df.to_csv(results_csv, index=False, float_format="%.6f")
    print(f"\nResults table saved to: {results_csv}")

    # Print summary table
    print(f"\n{'=' * 70}")
    print(f"  BODE PLOT RESULTS — {args.axis.upper()} AXIS")
    print(f"{'=' * 70}")
    print(f"  {'Freq (Hz)':>10s}  {'Gain (dB)':>10s}  {'Phase (°)':>10s}  {'In Amp':>10s}  {'Out Amp':>10s}")
    print(f"  {'─' * 10}  {'─' * 10}  {'─' * 10}  {'─' * 10}  {'─' * 10}")
    for r in results:
        print(f"  {r['freq_hz']:10.3f}  {r['gain_db']:10.2f}  {r['phase_deg']:10.1f}  "
              f"{r['input_amplitude']:10.4f}  {r['output_amplitude']:10.4f}")
    print(f"{'=' * 70}")


if __name__ == "__main__":
    main()
