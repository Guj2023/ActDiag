from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field


class ParameterSearchSpace(BaseModel):
    min: float
    max: float


class ObjectiveWeights(BaseModel):
    q_weight: float = 1.0
    dq_weight: float = 0.2
    tau_weight: float = 0.0
    metric_weight: float = 0.5


class FitConfig(BaseModel):
    parameters: dict[str, ParameterSearchSpace]
    objective: ObjectiveWeights = ObjectiveWeights()
    samples: int = 200
    seed: int | None = None


def load_search_config(path: Path) -> FitConfig:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if data is None or "fit" not in data:
        raise ValueError(f"Missing 'fit' section in {path}")

    return FitConfig.model_validate(data["fit"])
