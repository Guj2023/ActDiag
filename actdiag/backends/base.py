from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class PhysicsBackend(Protocol):
    """Minimal interface every physics backend must satisfy.

    A backend owns one single-joint scene.  The caller drives it by reading
    state, computing a torque, and stepping — exactly what the simulation loop
    in simulate.py does.  All units are SI (rad, rad/s, N·m, s).
    """

    dt: float

    def get_state(self) -> tuple[float, float]:
        """Return (q, dq) — joint position [rad] and velocity [rad/s]."""
        ...

    def apply_torque_and_step(self, torque: float) -> None:
        """Apply *torque* [N·m] and advance physics by one dt."""
        ...
