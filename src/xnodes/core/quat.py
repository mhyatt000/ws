"""Quaternion helpers."""

from __future__ import annotations

import numpy as np

__all__ = ["ema_quat", "ema_quat_slerp", "slerp"]


def _normalize(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm == 0.0:
        raise ValueError("zero-norm quaternion")
    return q / norm


def ema_quat(q_prev: np.ndarray, q_meas: np.ndarray, alpha: float) -> np.ndarray:
    """Blend quaternions with an exponential moving average."""
    q_prev = _normalize(np.asarray(q_prev, dtype=float))
    q_meas = _normalize(np.asarray(q_meas, dtype=float))

    a = float(alpha)
    if not np.isfinite(a):
        raise ValueError("alpha must be finite")
    if a <= 0.0:
        return q_prev
    if a >= 1.0:
        return q_meas

    if np.dot(q_prev, q_meas) < 0.0:
        q_meas = -q_meas

    q_new = (1.0 - a) * q_prev + a * q_meas
    return _normalize(q_new)


def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation."""
    q0 = _normalize(np.asarray(q0, dtype=float))
    q1 = _normalize(np.asarray(q1, dtype=float))

    if np.dot(q0, q1) < 0.0:
        q1 = -q1

    dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
    if dot > 0.9995:
        return ema_quat(q0, q1, float(t))

    theta = np.arccos(dot)
    sin_theta = np.sin(theta)
    if sin_theta == 0.0:
        return q0

    t = float(np.clip(t, 0.0, 1.0))
    w0 = np.sin((1.0 - t) * theta) / sin_theta
    w1 = np.sin(t * theta) / sin_theta
    return w0 * q0 + w1 * q1


def ema_quat_slerp(q_prev: np.ndarray, q_meas: np.ndarray, alpha: float) -> np.ndarray:
    """EMA that walks the geodesic exactly."""
    return slerp(q_prev, q_meas, float(alpha))
