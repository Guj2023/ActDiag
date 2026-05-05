from __future__ import annotations

import math
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
            if space.log:
                if space.min <= 0 or space.max <= 0:
                    raise ValueError(
                        f"log-spaced parameter '{param_name}' needs strictly positive bounds"
                    )
                sample[param_name] = math.exp(
                    random.uniform(math.log(space.min), math.log(space.max))
                )
            else:
                sample[param_name] = random.uniform(space.min, space.max)
        samples.append(sample)
    return samples
