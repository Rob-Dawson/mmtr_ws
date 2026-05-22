#!/usr/bin/env python3

import math
import numpy as np

class LowPassFilter():
    def __init__(self, cutoff_hz: float, sample_rate_hz: float, dim: int = 3):
        if cutoff_hz <= 0:
            raise ValueError("cutoff_hz must be > 0")
        if sample_rate_hz <= 0:
            raise ValueError("sample_rate_hz must be > 0")

        self.cutoff_hz = float(cutoff_hz)
        self.sample_rate_hz = float(sample_rate_hz)
        self.dim = dim

        self._y = np.zeros(dim, dtype=float)
        self._initialized = False
        self._alpha = self._compute_alpha(1.0 / self.sample_rate_hz)

    def _compute_alpha(self, dt: float) -> float:
        rc = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        return dt / (rc + dt)

    def reset(self) -> None:
        self._y[:] = 0.0
        self._initialized = False

    def update(self, x: np.ndarray, dt: float | None = None) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if x.shape != (self.dim,):
            raise ValueError(f"Expected shape ({self.dim},), got {x.shape}")

        alpha = self._alpha if dt is None else self._compute_alpha(dt)

        if not self._initialized:
            self._y = x.copy()
            self._initialized = True
            return self._y.copy()

        self._y = alpha * x + (1.0 - alpha) * self._y
        return self._y.copy()
