import math
import time

import numpy as np

# TODO deprecate this and move to policy


def apply_lowpass_filter(action: np.ndarray, carry: dict, cutoff_hz: float) -> tuple[np.ndarray, dict]:
    x = np.asarray(action, dtype=float)
    now = time.perf_counter()
    if carry is None or 'prev' not in carry or getattr(carry['prev'], 'shape', None) != x.shape:
        return x, {'prev': x.copy(), 't': now}
    dt = max(now - carry.get('t', now), 0.0)
    carry['t'] = now
    if cutoff_hz <= 0 or dt <= 0:
        return x, carry
    alpha = 1.0 - math.exp(-2.0 * math.pi * cutoff_hz * dt)
    y = carry['prev'] + alpha * (x - carry['prev'])
    carry['prev'] = y
    return y, carry