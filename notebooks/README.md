# PDR Step Detection — Analysis Notebooks

## world_frame_test.ipynb

Detects walking steps from a waist-mounted VN-100 IMU using a **two-stage pipeline**: rule-based peak detection (high recall, ~50% precision) followed by ML classification to filter false peaks.

### Data

- 7 bags of walking data from a VN-100 IMU (3-axis accel/gyro, 40 Hz) mounted on the left waist
- Ground truth: keyboard tap on every left heel strike
- Bags range from 32 to 70 taps (after trimming), 40–92 seconds each
- Sensor tilt varies significantly across bags (63°–87° from Z-axis)

### What Was Done

1. **Adaptive stillness calibration** — Instead of a fixed calibration window, the notebook detects how long the person stood still at the start of each bag by monitoring a rolling standard deviation of acceleration magnitude. It uses 80% of the detected stillness for gravity estimation, avoiding contamination from the transition to walking. Detected stillness ranged from 2.25s to 4.85s across bags.

2. **Gravity subtraction** — Subtracts the estimated gravity vector from raw acceleration to isolate dynamic (motion-only) acceleration. The dynamic signal is centered near 0 m/s² instead of ~9.81 m/s², removing tilt dependence.

3. **70-tap trim on my_bag** — my_bag had 200 taps (62% of total training data). Trimming to the first 70 taps rebalanced training so it no longer dominated. This was the single biggest improvement for cross-bag generalization.

4. **12 shape-only features** — No timing features, so the classifier is pace-independent. Features describe the shape of each detected peak (height, prominence, skewness, kurtosis, gyro correlation, etc.).

5. **Leave-One-Bag-Out (LOBO) cross-validation** — Each of the 7 bags is held out in turn, training on the other 6. Tests generalization to unseen recording sessions.

6. **Three classifiers tested** — Logistic Regression (balanced class weights), Gradient Boosting (200 trees, sample-weighted), and Random Forest (200 trees, balanced weights).

### The Problem: Before

Without the 70-tap trim, my_bag dominated training with 62% of all peaks. The classifier learned my_bag's high-amplitude walking pattern (accel σ ≈ 3.2 m/s²) and failed on lower-amplitude bags.

| Bag | F1 (before trim) | Notes |
|---|---|---|
| 14_30_34 | 0.04–0.40 | Low accel variability (σ ≈ 0.8 m/s²) |
| 14_32_26 | 0.04–0.40 | Low accel variability (σ ≈ 0.7 m/s²) |
| Other bags | 0.80–0.95 | Higher variability, closer to my_bag |

The model had essentially memorized what my_bag's steps look like and rejected anything quieter.

### The Fix: After

Trimming my_bag to 70 taps + adaptive calibration + shape-only features:

| Bag | F1 (before) | F1 (after, body-frame RF) | Improvement |
|---|---|---|---|
| 14_30_34 | ~0.04–0.40 | 0.842+ | Massive |
| 14_32_26 | ~0.04–0.40 | 0.926+ | Massive |
| 14_14_01 | ~0.90 | 0.935 | Stable |
| 14_14_58 | ~0.90 | 0.947 | Stable |
| 14_21_45 | ~0.85 | 0.927 | Improved |
| 14_26_26 | ~0.85 | 0.891 | Improved |

### Results: All Classifiers × Both Signals

#### LOBO performance (excluding my_bag)

| Signal | Classifier | Step Count Error | F1 |
|---|---|---|---|
| Body-frame | Logistic Regression | 9.9% | 0.891 |
| Body-frame | Gradient Boosting | 10.3% | 0.910 |
| Body-frame | Random Forest | **6.7%** | **0.918** |
| Gravity-sub | Logistic Regression | 15.6% | 0.898 |
| Gravity-sub | Gradient Boosting | 7.4% | 0.915 |
| Gravity-sub | Random Forest | 6.8% | 0.918 |

#### Overfitting check (train–test F1 gap)

| Classifier | Body gap | Grav-sub gap | Verdict |
|---|---|---|---|
| Logistic Regression | 0.045 | 0.044 | Clean |
| Gradient Boosting | 0.117 | 0.151 | Mild–High |
| Random Forest | 0.137 | 0.145 | Mild |

LogReg has the smallest gap but lowest F1. Random Forest and GB achieve the best F1 with moderate overfitting — expected given only 7 bags of training data.

### Key Takeaways

- **The 70-tap trim was the biggest win.** It fixed the training imbalance that caused two bags to completely fail. Signal representation (body-frame vs gravity-subtracted) made little difference once the imbalance was addressed.
- **Body-frame and gravity-subtracted signals perform nearly identically** — both achieve ~0.918 F1 with Random Forest. Gravity subtraction is not necessary for this sensor setup.
- **Random Forest** is the best overall classifier (~6.7% step count error, 0.918 F1).
- **Logistic Regression** generalizes most cleanly (smallest train-test gap at 0.04) but is slightly less accurate.
- **Adaptive calibration** successfully detected per-bag stillness windows (2.25–4.85s) without any hardcoded assumptions.
- **More training data would help** — the learning curves show test F1 still climbing as bags are added, and the train-test gap is narrowing but not yet converged.
