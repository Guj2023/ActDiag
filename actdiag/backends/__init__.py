from __future__ import annotations

from actdiag.backends.base import PhysicsBackend
from actdiag.backends.mujoco_backend import MuJoCoBackend


def build_backend(scene_profile, dt: float, backend_name: str) -> PhysicsBackend:
    if backend_name == "mujoco":
        return MuJoCoBackend(scene_profile, dt)
    if backend_name == "physx":
        from actdiag.backends.physx_backend import PhysXBackend
        return PhysXBackend(scene_profile, dt)
    if backend_name == "openmodelica":
        from actdiag.backends.openmodelica_backend import OpenModelicaBackend
        return OpenModelicaBackend(scene_profile, dt)
    raise ValueError(
        f"unsupported backend {backend_name!r} — choose 'mujoco', 'physx', or 'openmodelica'"
    )


__all__ = ["PhysicsBackend", "MuJoCoBackend", "build_backend"]
