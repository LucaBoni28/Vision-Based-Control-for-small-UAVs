###############################################################################
# Author: Luca Boninsegna
# Date:   26/08/2026
# Descr:  NoisyVirtualCamera — wraps VirtualCamera and injects realistic
#         vision imperfections for robustness testing.
#
#         Imperfections:
#           1. Bounding box jitter (Gaussian noise on e_x, e_y, area)
#           2. Detection latency (ring buffer delay simulating YOLO inference)
#           3. Detection dropout (random frames with no detection)
#           4. Tracker ID switch (random ID changes that reset PD memory)
#
#         This is a pure-math module with NO hardware or MAVLink dependencies.
###############################################################################

import random
from collections import deque
from dataclasses import dataclass

from sitl_tests.utils.virtual_camera import VirtualCamera, VirtualCameraOutput


@dataclass
class NoisyCameraOutput:
    """Output of the noisy virtual camera projection."""
    # Noisy measurements (what the controller sees)
    e_x: float
    e_y: float
    fake_area: float
    distance: float
    in_fov: bool

    # Clean measurements (for logging/comparison)
    clean_e_x: float
    clean_e_y: float
    clean_fake_area: float
    clean_distance: float

    # Imperfection flags
    detected: bool       # False during dropout frames
    id_switched: bool    # True when tracker changed ID this frame


class NoisyVirtualCamera:
    """
    Wraps VirtualCamera and injects realistic vision imperfections.

    When noise is disabled (all parameters zero), this behaves identically
    to VirtualCamera, making it suitable for A/B comparison testing.
    """

    def __init__(self, vcam: VirtualCamera,
                 noise_sigma_xy: float = 0.015,
                 noise_sigma_area: float = 0.03,
                 latency_frames: int = 1,
                 dropout_prob: float = 0.02,
                 id_switch_prob: float = 0.005,
                 seed: int = 42):
        """
        Args:
            vcam: The underlying VirtualCamera for clean projections.
            noise_sigma_xy: Std dev of additive Gaussian noise on e_x, e_y.
                           0.015 ≈ 1.5% of frame width/height.
            noise_sigma_area: Std dev of multiplicative Gaussian noise on area.
                             0.03 = 3% area jitter.
            latency_frames: Number of frames of delay (simulates inference time).
                           1 frame at 20Hz = 50ms. 0 = no latency.
            dropout_prob: Probability of detection failure per frame.
                         0.02 = 2% of frames.
            id_switch_prob: Probability of tracker ID switch per frame.
                           0.005 = 0.5% of frames.
            seed: Random seed for reproducibility.
        """
        self._vcam = vcam
        self._noise_sigma_xy = noise_sigma_xy
        self._noise_sigma_area = noise_sigma_area
        self._latency_frames = latency_frames
        self._dropout_prob = dropout_prob
        self._id_switch_prob = id_switch_prob
        self._rng = random.Random(seed)

        # Latency ring buffer: oldest element = latency_frames ago
        self._buffer: deque = deque(maxlen=max(1, latency_frames + 1))

        # Statistics counters
        self.total_frames = 0
        self.dropout_count = 0
        self.id_switch_count = 0

    @property
    def optical_constant(self) -> float:
        return self._vcam.optical_constant

    @optical_constant.setter
    def optical_constant(self, value: float) -> None:
        self._vcam.optical_constant = value

    def project(self, drone_x: float, drone_y: float, drone_z: float,
                drone_yaw: float,
                target_x: float, target_y: float, target_z: float) -> NoisyCameraOutput:
        """
        Project a 3D target through the noisy virtual camera.

        Returns NoisyCameraOutput with both clean and noisy measurements,
        plus detection/tracking flags.
        """
        self.total_frames += 1

        # Get clean projection from the underlying VirtualCamera
        clean = self._vcam.project(drone_x, drone_y, drone_z, drone_yaw,
                                   target_x, target_y, target_z)

        # Push into latency buffer and retrieve delayed measurement
        self._buffer.append(clean)
        delayed: VirtualCameraOutput = self._buffer[0]  # Oldest = latency_frames ago

        # Check for detection dropout (higher priority than ID switch)
        detected = True
        if self._dropout_prob > 0 and self._rng.random() < self._dropout_prob:
            detected = False
            self.dropout_count += 1

        # Check for tracker ID switch (only if detection succeeded)
        id_switched = False
        if detected and self._id_switch_prob > 0 and self._rng.random() < self._id_switch_prob:
            id_switched = True
            self.id_switch_count += 1

        # Apply noise to the delayed measurement
        if detected:
            noisy_e_x = delayed.e_x + self._rng.gauss(0, self._noise_sigma_xy) if self._noise_sigma_xy > 0 else delayed.e_x
            noisy_e_y = delayed.e_y + self._rng.gauss(0, self._noise_sigma_xy) if self._noise_sigma_xy > 0 else delayed.e_y
            noisy_area = delayed.fake_area * (1.0 + self._rng.gauss(0, self._noise_sigma_area)) if self._noise_sigma_area > 0 else delayed.fake_area
            noisy_area = max(1.0, noisy_area)  # Area must be positive

            # Clamp noisy errors to [-1, 1]
            noisy_e_x = max(-1.0, min(1.0, noisy_e_x))
            noisy_e_y = max(-1.0, min(1.0, noisy_e_y))
        else:
            # During dropout, return the delayed values but flagged as not detected
            # (the controller should NOT use these — it should send stop)
            noisy_e_x = delayed.e_x
            noisy_e_y = delayed.e_y
            noisy_area = delayed.fake_area

        return NoisyCameraOutput(
            e_x=noisy_e_x,
            e_y=noisy_e_y,
            fake_area=noisy_area,
            distance=delayed.distance,
            in_fov=delayed.in_fov,
            clean_e_x=clean.e_x,
            clean_e_y=clean.e_y,
            clean_fake_area=clean.fake_area,
            clean_distance=clean.distance,
            detected=detected,
            id_switched=id_switched,
        )

    def stats_summary(self) -> str:
        """Return a human-readable summary of noise statistics."""
        if self.total_frames == 0:
            return "No frames processed."
        return (
            f"Noise Stats: {self.total_frames} frames | "
            f"{self.dropout_count} dropouts ({100*self.dropout_count/self.total_frames:.1f}%) | "
            f"{self.id_switch_count} ID switches ({100*self.id_switch_count/self.total_frames:.1f}%)"
        )


# ─── Ideal (no-noise) factory ────────────────────────────────────────────────

def create_ideal_camera(vcam: VirtualCamera) -> NoisyVirtualCamera:
    """Create a NoisyVirtualCamera with all imperfections disabled."""
    return NoisyVirtualCamera(
        vcam=vcam,
        noise_sigma_xy=0.0,
        noise_sigma_area=0.0,
        latency_frames=0,
        dropout_prob=0.0,
        id_switch_prob=0.0,
    )


# ─── Self-test ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    import os
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    import math

    vcam = VirtualCamera(hfov_deg=62.2, vfov_deg=48.8, optical_constant=7192.94)

    # Test 1: Ideal (no noise) should match VirtualCamera exactly
    ideal = create_ideal_camera(vcam)
    out = ideal.project(0, 0, -10, 0.0, 10, 0, -10)
    clean = vcam.project(0, 0, -10, 0.0, 10, 0, -10)
    assert abs(out.e_x - clean.e_x) < 1e-10, f"Ideal e_x mismatch: {out.e_x} vs {clean.e_x}"
    assert abs(out.e_y - clean.e_y) < 1e-10, f"Ideal e_y mismatch"
    assert abs(out.fake_area - clean.fake_area) < 1e-10, f"Ideal area mismatch"
    assert out.detected is True
    assert out.id_switched is False
    print("Test 1 (ideal mode): PASSED")

    # Test 2: Noisy camera should produce different values from clean
    noisy = NoisyVirtualCamera(vcam, noise_sigma_xy=0.05, seed=42)
    results = [noisy.project(0, 0, -10, 0.0, 10, 0, -10) for _ in range(100)]
    e_x_values = [r.e_x for r in results]
    mean_ex = sum(e_x_values) / len(e_x_values)
    e_x_std = (sum((x - mean_ex) ** 2 for x in e_x_values) / len(e_x_values)) ** 0.5
    assert e_x_std > 0.01, f"Noise std too low: {e_x_std}"
    print(f"Test 2 (noisy mode): PASSED (e_x std = {e_x_std:.4f})")

    # Test 3: Dropout probability
    noisy_dropout = NoisyVirtualCamera(vcam, dropout_prob=0.5, noise_sigma_xy=0.0, seed=42)
    results = [noisy_dropout.project(0, 0, -10, 0.0, 10, 0, -10) for _ in range(1000)]
    dropout_rate = sum(1 for r in results if not r.detected) / len(results)
    assert 0.4 < dropout_rate < 0.6, f"Dropout rate off: {dropout_rate:.2f}"
    print(f"Test 3 (dropout ~50%): PASSED (actual: {dropout_rate:.1%})")

    # Test 4: Clean values always match original regardless of noise
    noisy2 = NoisyVirtualCamera(vcam, noise_sigma_xy=0.1, latency_frames=0, seed=42)
    out = noisy2.project(0, 0, -10, 0.0, 10, 0, -10)
    assert abs(out.clean_e_x - clean.e_x) < 1e-10, "Clean values should match original"
    assert abs(out.clean_distance - clean.distance) < 1e-10, "Clean distance should match"
    print("Test 4 (clean values preserved): PASSED")

    # Test 5: Latency introduces delay
    noisy_latency = NoisyVirtualCamera(vcam, noise_sigma_xy=0.0, noise_sigma_area=0.0,
                                        latency_frames=2, dropout_prob=0.0, seed=42)
    # Frame 0: buffer has 1 element, output = frame 0 (not enough history yet)
    out0 = noisy_latency.project(0, 0, -10, 0.0, 10, 0, -10)
    # Frame 1: buffer has 2 elements, output = frame 0
    out1 = noisy_latency.project(0, 0, -10, 0.0, 10, 5, -10)  # Target moved East
    # Frame 2: buffer has 3 elements (maxlen=3), output = frame 0
    out2 = noisy_latency.project(0, 0, -10, 0.0, 10, 10, -10)  # Target moved more East
    # out2 should have the measurement from frame 0 (no lateral offset)
    assert abs(out2.e_x - out0.e_x) < 1e-10, f"Latency not working: out2.e_x={out2.e_x}, out0.e_x={out0.e_x}"
    print("Test 5 (latency delay): PASSED")

    print(f"\n{noisy.stats_summary()}")
    print(f"{noisy_dropout.stats_summary()}")
    print("\nAll self-tests passed!")
