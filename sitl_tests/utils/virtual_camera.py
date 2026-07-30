###############################################################################
# Author: Luca Boninsegna
# Date:   29/07/2026
# Descr:  Mathematical Virtual Camera for SITL closed-loop testing.
#         Projects a virtual 3D target into fake pixel errors and bounding box
#         area, enabling closed-loop PID testing without a 3D graphics engine.
#
#         This is a pure-math module with NO hardware or MAVLink dependencies.
###############################################################################

import math
from dataclasses import dataclass


@dataclass
class VirtualCameraOutput:
    """Output of the virtual camera projection."""
    e_x: float          # Normalized horizontal pixel error [-1, 1] (positive = target right of center)
    e_y: float          # Normalized vertical pixel error [-1, 1] (positive = target below center)
    fake_area: float    # Simulated bounding box area in pixels²
    distance: float     # 3D geometric distance to target in meters
    in_fov: bool        # True if the target is within the camera's field of view


class VirtualCamera:
    """
    Mathematical camera model that converts a 3D target position into
    normalized pixel errors and a fake bounding box area.

    Coordinate system: NED (North-East-Down)
        x = North, y = East, z = Down (negative z = above ground)

    Camera model:
        - Mounted facing forward along the drone's heading (yaw)
        - HFOV and VFOV define the angular field of view
        - Pixel errors are normalized to [-1, 1]

    Distance model:
        - Uses the inverted calibration formula: area = optical_constant / distance²
    """

    def __init__(self, hfov_deg: float = 62.2, vfov_deg: float = 48.8,
                 optical_constant: float = 7192.94):
        """
        Args:
            hfov_deg: Horizontal field of view in degrees (IMX219: 62.2°)
            vfov_deg: Vertical field of view in degrees (IMX219: 48.8°)
            optical_constant: Pre-calibrated constant for area = K / d²
        """
        self._hfov_rad = math.radians(hfov_deg)
        self._vfov_rad = math.radians(vfov_deg)
        self._half_hfov = self._hfov_rad / 2.0
        self._half_vfov = self._vfov_rad / 2.0
        self._optical_constant = optical_constant

    @property
    def optical_constant(self) -> float:
        return self._optical_constant

    @optical_constant.setter
    def optical_constant(self, value: float) -> None:
        self._optical_constant = value

    def project(self, drone_x: float, drone_y: float, drone_z: float,
                drone_yaw: float,
                target_x: float, target_y: float, target_z: float) -> VirtualCameraOutput:
        """
        Project a 3D target into virtual camera pixel errors and bounding box area.

        Args:
            drone_x, drone_y, drone_z: Drone position in NED frame (meters)
            drone_yaw: Drone heading in radians (0 = North, positive = clockwise)
            target_x, target_y, target_z: Target position in NED frame (meters)

        Returns:
            VirtualCameraOutput with normalized errors, fake area, distance, and FOV flag
        """
        # Relative position: target - drone (NED)
        dx = target_x - drone_x  # North component
        dy = target_y - drone_y  # East component
        dz = target_z - drone_z  # Down component

        # 3D geometric distance
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        # Edge case: drone is on top of the target
        if distance < 0.01:
            return VirtualCameraOutput(
                e_x=0.0, e_y=0.0,
                fake_area=self._clamp_area(self._optical_constant / 0.01**2),
                distance=distance,
                in_fov=True,
            )

        # --- Horizontal (yaw) error ---
        # Absolute bearing from drone to target in NED frame
        # atan2(East, North) gives bearing from North, clockwise positive
        bearing = math.atan2(dy, dx)

        # Relative bearing: how far the target is from the camera's heading
        relative_bearing = self._wrap_angle(bearing - drone_yaw)

        # Normalize to [-1, 1] using half the horizontal FOV
        e_x = relative_bearing / self._half_hfov

        # --- Vertical (pitch) error ---
        # Horizontal distance (ground plane)
        horizontal_distance = math.sqrt(dx**2 + dy**2)

        # Elevation angle: positive = target above drone
        # In NED, negative dz means target is higher (above), so we negate dz
        if horizontal_distance > 0.001:
            elevation = math.atan2(-dz, horizontal_distance)
        else:
            # Target is directly above or below
            elevation = math.copysign(math.pi / 2, -dz)

        # In screen coordinates, positive e_y means target is BELOW center
        # If the target is above the drone (positive elevation), it should be
        # above center on screen → negative e_y
        e_y = -elevation / self._half_vfov

        # --- Check if target is within FOV ---
        in_fov = abs(relative_bearing) <= self._half_hfov and abs(elevation) <= self._half_vfov

        # Clamp errors to [-1, 1] even if target is outside FOV
        # (the PID will just command max correction)
        e_x = max(-1.0, min(1.0, e_x))
        e_y = max(-1.0, min(1.0, e_y))

        # --- Fake bounding box area ---
        fake_area = self._optical_constant / (distance ** 2)
        fake_area = self._clamp_area(fake_area)

        return VirtualCameraOutput(
            e_x=e_x,
            e_y=e_y,
            fake_area=fake_area,
            distance=distance,
            in_fov=in_fov,
        )

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap an angle to [-π, π]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def _clamp_area(area: float) -> float:
        """Clamp the fake area to a reasonable range (1 to 1e6 px²)."""
        return max(1.0, min(1_000_000.0, area))


# ─── Self-test ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    cam = VirtualCamera(hfov_deg=62.2, vfov_deg=48.8, optical_constant=7192.94)

    # Test 1: Target directly ahead, 10m away, same altitude
    out = cam.project(0, 0, -10, 0.0, 10, 0, -10)
    print(f"Test 1 (10m North, same alt): e_x={out.e_x:.4f}, e_y={out.e_y:.4f}, "
          f"area={out.fake_area:.1f}, dist={out.distance:.2f}m, in_fov={out.in_fov}")
    assert abs(out.e_x) < 0.01, f"e_x should be ~0, got {out.e_x}"
    assert abs(out.e_y) < 0.01, f"e_y should be ~0, got {out.e_y}"

    # Test 2: Target 10m East, drone facing North → target should be to the right
    out = cam.project(0, 0, -10, 0.0, 0, 10, -10)
    print(f"Test 2 (10m East, facing N): e_x={out.e_x:.4f}, e_y={out.e_y:.4f}, "
          f"area={out.fake_area:.1f}, dist={out.distance:.2f}m, in_fov={out.in_fov}")
    assert out.e_x > 0, f"e_x should be positive (target right), got {out.e_x}"

    # Test 3: Target 5m above drone → should appear above center (negative e_y)
    out = cam.project(0, 0, -10, 0.0, 10, 0, -15)
    print(f"Test 3 (5m above): e_x={out.e_x:.4f}, e_y={out.e_y:.4f}, "
          f"area={out.fake_area:.1f}, dist={out.distance:.2f}m, in_fov={out.in_fov}")
    assert out.e_y < 0, f"e_y should be negative (target above), got {out.e_y}"

    # Test 4: Drone facing East (yaw=π/2), target 10m East
    out = cam.project(0, 0, -10, math.pi/2, 0, 10, -10)
    print(f"Test 4 (10m East, facing E): e_x={out.e_x:.4f}, e_y={out.e_y:.4f}, "
          f"area={out.fake_area:.1f}, dist={out.distance:.2f}m, in_fov={out.in_fov}")
    assert abs(out.e_x) < 0.01, f"e_x should be ~0 (facing target), got {out.e_x}"

    print("\nAll self-tests passed! ✓")
