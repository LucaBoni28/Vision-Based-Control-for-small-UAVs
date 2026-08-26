###############################################################################
# Author: Luca Boninsegna
# Date:   26/08/2026
# Descr:  Predefined target trajectories for Test 4 — Trajectory Scenario.
#
#         Each trajectory defines a realistic walking path as a function of
#         time, returning the target's offset from its starting position in the
#         drone's initial body frame:
#           dx_fwd: forward (positive = away from drone)
#           dx_lat: lateral (positive = right of drone heading)
#           dz:     vertical (NED convention: positive = down, negative = up)
#
#         This is a pure-math module with NO hardware or MAVLink dependencies.
###############################################################################

import math
from dataclasses import dataclass
from typing import Callable, Tuple, List


@dataclass
class Trajectory:
    """Definition of a target trajectory scenario."""
    name: str
    duration: float           # Total duration in seconds
    description: str
    walk_speed: float         # Characteristic walking speed (m/s)
    position_fn: Callable[[float], Tuple[float, float, float]]
    # position_fn(t) -> (dx_forward, dx_lateral, dz_vertical) in body frame


# ─── Interpolation Helper ────────────────────────────────────────────────────

def _interp_waypoints(t: float, waypoints: list) -> Tuple[float, float, float]:
    """
    Linear interpolation between waypoints.

    Args:
        t: Current time in seconds
        waypoints: List of (time, dx_fwd, dx_lat, dz) tuples, sorted by time

    Returns:
        (dx_fwd, dx_lat, dz) at time t
    """
    if t <= waypoints[0][0]:
        return waypoints[0][1], waypoints[0][2], waypoints[0][3]
    if t >= waypoints[-1][0]:
        return waypoints[-1][1], waypoints[-1][2], waypoints[-1][3]

    for i in range(len(waypoints) - 1):
        t0, f0, l0, a0 = waypoints[i]
        t1, f1, l1, a1 = waypoints[i + 1]
        if t <= t1:
            s = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
            return (
                f0 + (f1 - f0) * s,
                l0 + (l1 - l0) * s,
                a0 + (a1 - a0) * s,
            )

    return waypoints[-1][1], waypoints[-1][2], waypoints[-1][3]


# ─── Trajectory Definitions ──────────────────────────────────────────────────

def straight_walk(speed: float = 1.0) -> Trajectory:
    """Target walks 15m forward at constant speed, then stops for 5s."""
    distance = 15.0
    walk_time = distance / speed
    stop_time = 5.0
    total = walk_time + stop_time

    wps = [
        (0.0,       0.0,      0.0, 0.0),
        (walk_time, distance, 0.0, 0.0),
        (total,     distance, 0.0, 0.0),
    ]

    return Trajectory(
        name="straight_walk",
        duration=total,
        description=f"Walk {distance:.0f}m forward at {speed} m/s, stop {stop_time:.0f}s",
        walk_speed=speed,
        position_fn=lambda t, _wps=wps: _interp_waypoints(t, _wps),
    )


def l_shape(speed: float = 1.0) -> Trajectory:
    """Target walks 8m forward, turns right, walks 5m laterally while climbing 2m."""
    fwd_dist = 8.0
    lat_dist = 5.0
    climb_m = 2.0  # NED up = negative z
    stop_time = 5.0

    t1 = fwd_dist / speed           # End of forward walk
    t2 = t1 + lat_dist / speed      # End of lateral walk + climb
    total = t2 + stop_time

    wps = [
        (0.0, 0.0,      0.0,      0.0),
        (t1,  fwd_dist, 0.0,      0.0),
        (t2,  fwd_dist, lat_dist, -climb_m),  # Climb 2m (NED up = negative)
        (total, fwd_dist, lat_dist, -climb_m),
    ]

    return Trajectory(
        name="l_shape",
        duration=total,
        description=f"Walk {fwd_dist:.0f}m fwd -> {lat_dist:.0f}m right (climb {climb_m:.0f}m), stop {stop_time:.0f}s",
        walk_speed=speed,
        position_fn=lambda t, _wps=wps: _interp_waypoints(t, _wps),
    )


def circle(speed: float = 1.0, radius: float = 5.0) -> Trajectory:
    """Target walks in a circle of given radius, then stops for 3s."""
    omega = speed / radius           # Angular velocity (rad/s)
    period = 2.0 * math.pi / omega   # Time for one full circle
    stop_time = 3.0
    total = period + stop_time

    def position(t: float) -> Tuple[float, float, float]:
        if t <= 0:
            return 0.0, 0.0, 0.0
        if t >= period:
            return 0.0, 0.0, 0.0  # Completed circle, back at start
        angle = omega * t
        dx_fwd = radius * math.sin(angle)
        dx_lat = radius * (1.0 - math.cos(angle))
        return dx_fwd, dx_lat, 0.0

    return Trajectory(
        name="circle",
        duration=total,
        description=f"Circle R={radius:.0f}m at {speed} m/s ({period:.1f}s period), stop {stop_time:.0f}s",
        walk_speed=speed,
        position_fn=position,
    )


def stop_and_go(speed: float = 1.0) -> Trajectory:
    """Walk 5m, pause 3s, walk 5m, pause 3s, walk 5m. Gradual 1.5m climb."""
    seg = 5.0        # Distance per walking segment
    pause = 3.0      # Pause duration
    climb_total = -1.5   # Total altitude change (NED: negative = up)
    stop_time = 5.0

    seg_t = seg / speed
    t1 = seg_t                    # End walk 1
    t2 = t1 + pause               # End pause 1
    t3 = t2 + seg_t               # End walk 2
    t4 = t3 + pause               # End pause 2
    t5 = t4 + seg_t               # End walk 3
    total = t5 + stop_time

    # Altitude climbs proportionally during walking segments only
    wps = [
        (0.0, 0.0,     0.0, 0.0),
        (t1,  seg,     0.0, -0.5),     # Walk 1: climb 0.5m
        (t2,  seg,     0.0, -0.5),     # Pause 1: hold
        (t3,  2 * seg, 0.0, -1.0),     # Walk 2: climb to 1.0m
        (t4,  2 * seg, 0.0, -1.0),     # Pause 2: hold
        (t5,  3 * seg, 0.0, climb_total),  # Walk 3: climb to 1.5m
        (total, 3 * seg, 0.0, climb_total),  # Final stop
    ]

    return Trajectory(
        name="stop_and_go",
        duration=total,
        description=f"Walk-stop-walk ({seg:.0f}m x 3, {pause:.0f}s pauses, {abs(climb_total):.1f}m climb)",
        walk_speed=speed,
        position_fn=lambda t, _wps=wps: _interp_waypoints(t, _wps),
    )


def approach_retreat(speed: float = 1.0) -> Trajectory:
    """Target walks 2m toward drone, pauses, then walks 5m away. Includes altitude changes."""
    approach_dist = 2.0   # Distance toward drone (negative forward)
    retreat_dist = 5.0    # Distance away from drone (positive forward)
    pause = 2.0
    stop_time = 5.0

    t1 = approach_dist / speed        # End approach
    t2 = t1 + pause                   # End pause
    t3 = t2 + retreat_dist / speed    # End retreat
    total = t3 + stop_time

    # Approach: descend 0.8m (target goes down a ramp)
    # Retreat: climb 1.5m total from initial (target goes up a hill)
    final_fwd = retreat_dist - approach_dist  # Net forward distance

    wps = [
        (0.0,  0.0,        0.0, 0.0),
        (t1,  -approach_dist, 0.0, 0.8),       # Approach: negative fwd, descend 0.8m (NED down = positive)
        (t2,  -approach_dist, 0.0, 0.8),       # Pause: hold position
        (t3,   final_fwd,    0.0, -1.0),       # Retreat: past start, climb 1.8m from pause position
        (total, final_fwd,   0.0, -1.0),       # Final stop
    ]

    return Trajectory(
        name="approach_retreat",
        duration=total,
        description=f"Approach {approach_dist:.0f}m -> pause {pause:.0f}s -> retreat {retreat_dist:.0f}m (alt changes)",
        walk_speed=speed,
        position_fn=lambda t, _wps=wps: _interp_waypoints(t, _wps),
    )


# ─── Registry ────────────────────────────────────────────────────────────────

ALL_TRAJECTORIES = {
    "straight_walk": straight_walk,
    "l_shape": l_shape,
    "circle": circle,
    "stop_and_go": stop_and_go,
    "approach_retreat": approach_retreat,
}


def get_trajectory(name: str, speed: float = 1.0) -> Trajectory:
    """Get a trajectory by name."""
    if name not in ALL_TRAJECTORIES:
        raise ValueError(f"Unknown trajectory: {name}. Available: {list(ALL_TRAJECTORIES.keys())}")
    return ALL_TRAJECTORIES[name](speed=speed)


def get_all_trajectories(speed: float = 1.0) -> list:
    """Get all available trajectories."""
    return [factory(speed=speed) for factory in ALL_TRAJECTORIES.values()]


# ─── Self-test ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("  TRAJECTORY DEFINITIONS — Self Test")
    print("=" * 60)

    for name, factory in ALL_TRAJECTORIES.items():
        traj = factory(speed=1.0)
        start = traj.position_fn(0)
        mid = traj.position_fn(traj.duration / 2)
        end = traj.position_fn(traj.duration)

        print(f"\n  {name}: {traj.description}")
        print(f"    Duration: {traj.duration:.1f}s")
        print(f"    Start: fwd={start[0]:6.2f}m  lat={start[1]:6.2f}m  alt={start[2]:6.2f}m")
        print(f"    Mid:   fwd={mid[0]:6.2f}m  lat={mid[1]:6.2f}m  alt={mid[2]:6.2f}m")
        print(f"    End:   fwd={end[0]:6.2f}m  lat={end[1]:6.2f}m  alt={end[2]:6.2f}m")

        # Verify start is at origin
        assert abs(start[0]) < 1e-6 and abs(start[1]) < 1e-6 and abs(start[2]) < 1e-6, \
            f"{name}: Start should be (0, 0, 0), got {start}"

    # Test interpolation edge cases
    traj = straight_walk(speed=1.0)
    before = traj.position_fn(-1.0)
    assert abs(before[0]) < 1e-6, "Before start should return start position"
    after = traj.position_fn(traj.duration + 10.0)
    end = traj.position_fn(traj.duration)
    assert abs(after[0] - end[0]) < 1e-6, "After end should return end position"

    print(f"\n{'=' * 60}")
    print("  All self-tests passed!")
    print(f"{'=' * 60}")
