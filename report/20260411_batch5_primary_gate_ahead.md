# 5-run Validation (prediction-primary + ahead hard constraint)

- Timestamp: 2026-04-11
- Command profile: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true`
- Hard constraint at release: `carrier ahead of mini` (`ahead_ok=1`)

| Run | Classification | first_non_idle_t (s) | START reason | Window accepted dist_xy (m) | pred_along | pred_lat | pred_score | Pre-accept ahead | Pre-accept ahead_ok |
|---|---|---:|---|---:|---:|---:|---:|---:|---:|
| `20260411_170150_px4_sih` | final-pass | 78.300 | window | 97.55 | -59.02 | -17.79 | 1.65 | 86.28 | 1 |
| `20260411_170436_px4_sih` | final-pass | 75.640 | window | 95.73 | -57.35 | -17.85 | 1.62 | 86.11 | 1 |
| `20260411_170717_px4_sih` | final-pass | 76.000 | window | 96.79 | -58.18 | -17.87 | 1.64 | 84.72 | 1 |
| `20260411_171002_px4_sih` | final-pass | 75.640 | window | 96.68 | -58.29 | -18.42 | 1.67 | 87.61 | 1 |
| `20260411_171246_px4_sih` | final-pass | 79.100 | window | 93.99 | -57.42 | -17.88 | 1.62 | 81.97 | 1 |
