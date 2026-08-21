"""Small request-level RNG helper for paired NavDP evaluation.

The production server remains stochastic when no seed is supplied.  Formal
paired evaluations can instead attach one seed to every diffusion request so
two controller conditions receive the same initial DDPM noise independently
of earlier request counts.
"""

from __future__ import annotations

import random


MAX_TORCH_SEED = 2**63 - 1


def normalize_seed(value) -> int | None:
    """Parse an optional HTTP-form seed and reject ambiguous values."""
    if value is None or value == "":
        return None
    if isinstance(value, bool):
        raise ValueError("diffusion_seed must be an integer, not bool")
    try:
        seed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("diffusion_seed must be an integer") from exc
    if str(value).strip() != str(seed):
        raise ValueError("diffusion_seed must use canonical integer syntax")
    if not 0 <= seed <= MAX_TORCH_SEED:
        raise ValueError(
            f"diffusion_seed must be in [0, {MAX_TORCH_SEED}]")
    return seed


def apply_seed(value) -> int | None:
    """Apply an optional seed to every RNG used by the NavDP server."""
    seed = normalize_seed(value)
    if seed is None:
        return None
    # Keep parsing/test utilities importable without the heavy policy stack.
    # The server process already owns these dependencies when this path runs.
    import numpy as np
    import torch

    random.seed(seed)
    np.random.seed(seed % (2**32))
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    return seed
