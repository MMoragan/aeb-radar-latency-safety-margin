# AEB Radar Latency & Range Noise — Code and Data

Supporting code and data for:

**"Influence of Radar Detection Latency and Range Measurement Noise on
Automated Emergency Braking Safety Margin: A Synthetic-Sensor Simulation Study"**

Bocheng Jing, Yifan Wang (submitted to *Applied Sciences*)

## Requirements

- MATLAB **R2024b** with toolboxes:
  - Automated Driving Toolbox (`drivingScenario`, `drivingRadarDataGenerator`)
  - Statistics and Machine Learning Toolbox (`betaincinv`, `anova1`)
- Python 3.x with `ultralytics` (YOLOv8) — only needed for the real-camera
  control group (Figure 6)

## Contents

| File | Produces |
|---|---|
| `matlab/d5_full.m` | Tables 1–2: safety margin vs detection latency sweep (3 speeds × 0–1.5 s); critical latency vs braking build-up τ → `data/results_latency_v2.csv`, `data/results_critical_tau.csv` |
| `matlab/d6_full.m` | Tables 4–5: 80 km/h range-noise Monte Carlo (σ = 0/1/3 m, N = 1000, 40 m range trigger) incl. early/late trigger statistics; road-surface (wet/medium/dry) analytic critical latencies → `data/results_noise_mc_v2.csv`, `data/results_friction.csv` |
| `matlab/d8b_plateau.m` | Table 7 + Figure 7: 100 km/h joint delay–noise Monte Carlo on a 0.05 s fine grid (N = 1000 per cell, Clopper–Pearson 95% upper end point). Results are printed to the console (no CSV); expected staircase: 0% at Δt ≤ 0.10 s, 3.9–4.9% at 0.15–0.225 s, 33.6% at 0.25 s, 74.4% at 0.35 s (σ = 3 m) |
| `matlab/fig_export.m` | Figures 1–5 (scenario, margin vs latency, TTC vs latency, critical vs τ, noise boxplot) |
| `python/d9_fig6b.py` | Figure 6 panel frames: YOLOv8n frame-by-frame processing of dashcam footage (dense 720p / sparse 4K sequences) with injected-target trials |
| `python/d9_asm.py` | Figure 6 montage assembly (six-panel figure) |
| `data/` | Raw simulation outputs behind Tables 1, 2, 4, 5 (CSV) |
| `figures/` | Final figures 1–7 (300 dpi PNG) |

## Reproducibility

1. Run `matlab/d5_full.m`, then `matlab/d6_full.m`, then `matlab/d8b_plateau.m`
   in MATLAB R2024b (total ≈ 30 min). The console output of `d8b_plateau.m`
   reproduces Table 7; the CSV files reproduce Tables 1–2 and 4–5.
2. Figure 6 requires the dashcam video sources (720p dense: 660 frames @ 24 fps;
   4K sparse: 3677 frames @ 57.9 fps). The raw videos are too large for GitHub
   and are available from the authors on request; `d9_fig6b.py` documents the
   full processing pipeline and operational definitions (ego-lane-relevant
   target: car/bus/truck, ≥1% frame area, centre window).

## Notes

- The synthetic radar has no explicit clutter model; sparse returns on an empty
  road are suppressed by a two-frame persistence gate (Section 4.3 of the paper).
- The injected-target experiment measures appearance-to-first-box latency only
  (detection stage), not tracking/confirmation.

## License

MIT. Please cite the manuscript if you use this code or data.