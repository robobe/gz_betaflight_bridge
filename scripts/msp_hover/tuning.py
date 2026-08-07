from __future__ import annotations

import json
import statistics
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class TrialMetrics:
    rmse_m: float
    max_abs_error_m: float
    vertical_speed_rms_mps: float
    saturation_percent: float
    oscillations_per_minute: float


@dataclass(frozen=True)
class Comparison:
    accepted: bool
    reason: str
    baseline: TrialMetrics
    candidate: TrialMetrics


def median_metrics(paths: list[Path]) -> TrialMetrics:
    if not paths:
        raise ValueError("at least one tuning summary is required")
    trials: list[dict[str, object]] = []
    for path in paths:
        document = json.loads(path.read_text(encoding="utf-8"))
        safety = document.get("safety")
        hover = document.get("steady_hover")
        if not isinstance(safety, dict) or not safety.get("passed"):
            raise ValueError(f"unsafe or incomplete trial cannot be scored: {path}")
        if not isinstance(hover, dict):
            raise ValueError(f"steady_hover metrics missing: {path}")
        trials.append(hover)

    def median(name: str) -> float:
        values = [trial.get(name) for trial in trials]
        if any(not isinstance(value, (int, float)) for value in values):
            raise ValueError(f"metric '{name}' missing from one or more trials")
        return float(statistics.median(values))  # type: ignore[arg-type]

    return TrialMetrics(
        rmse_m=median("rmse_m"),
        max_abs_error_m=median("max_abs_error_m"),
        vertical_speed_rms_mps=median("vertical_speed_rms_mps"),
        saturation_percent=median("throttle_saturation_percent"),
        oscillations_per_minute=median("oscillations_per_minute"),
    )


def compare_trials(
    baseline_paths: list[Path],
    candidate_paths: list[Path],
    *,
    required_rmse_improvement: float = 0.05,
    allowed_regression: float = 0.10,
) -> Comparison:
    baseline = median_metrics(baseline_paths)
    candidate = median_metrics(candidate_paths)
    required_rmse = baseline.rmse_m * (1.0 - required_rmse_improvement)
    if candidate.rmse_m > required_rmse:
        return Comparison(False, "candidate did not improve median RMSE enough", baseline, candidate)

    guarded = (
        ("maximum error", baseline.max_abs_error_m, candidate.max_abs_error_m),
        ("vertical-speed RMS", baseline.vertical_speed_rms_mps, candidate.vertical_speed_rms_mps),
        ("throttle saturation", baseline.saturation_percent, candidate.saturation_percent),
        ("oscillation rate", baseline.oscillations_per_minute, candidate.oscillations_per_minute),
    )
    for name, old, new in guarded:
        limit = old * (1.0 + allowed_regression)
        if new > limit and new > old:
            return Comparison(False, f"candidate regressed {name}", baseline, candidate)
    return Comparison(True, "candidate improved RMSE without a guarded regression", baseline, candidate)
