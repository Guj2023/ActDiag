from __future__ import annotations

import random
from typing import Any

from actdiag.fit.config import FitConfig


def sample_parameters(config: FitConfig) -> list[dict[str, float]]:
    if config.seed is not None:
        random.seed(config.seed)

    samples = []
    for _ in range(config.samples):
        sample = {}
        for param_name, space in config.parameters.items():
            sample[param_name] = random.uniform(space.min, space.max)
        samples.append(sample)
    return samples
