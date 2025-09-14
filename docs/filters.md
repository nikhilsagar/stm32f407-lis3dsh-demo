# Filtering Pipeline (MA + LPF)

The accelerometer output is noisy. We use two simple filters that are inexpensive on a microcontroller and easy to reason about.

---

## 1) Moving Average (MA)
**What:** Average the most recent **N** samples (here, N=8).  
**Why:** Smooths random spikes and quantization noise before the LPF.

**Equation**
\[
y[n] = \frac{1}{N}\sum_{k=0}^{N-1} x[n-k]
\]

**Notes**
- Implemented with a ring buffer of `int16_t` and an integer sum.
- Output is promoted to `float` later for LPF and trig math.

---

## 2) First-Order Low-Pass Filter (LPF)
**What:** A 1-pole IIR that smooths signals via exponential averaging.  
**Why:** Produces stable values for tilt estimation while tracking slow motions.

**Form**
\[
y[n] = y[n-1] + \alpha \big( x[n] - y[n-1] \big)
\]

**Alpha from cutoff (`f_c`) and sampling rate (`f_s`)**
\[
\alpha = \frac{2\pi f_c}{2\pi f_c + f_s}
\]

**This project**
- `f_s = 100 Hz` (matches ODR)
- `f_c = 3 Hz`  → **strong smoothing** (α ≈ 0.16)

**Trade-offs**
- Smaller `f_c` → smoother but slower response.
- Larger `f_c` → faster response but more noise.

---

## 3) Angle calculation (axis-only tilt)
After filtering and offset removal:
- Convert to g: `a = LSB * 60e-6` (for ±2g FS).
- Clamp to [−1, +1] then `asin(a) * 180/π` to get degrees.
- We compute axis-only tilt (no sensor fusion).

---

## 4) Why MA + LPF together?
- **MA** quickly reduces random spikes.
- **LPF** provides continuous smoothing with little memory.
- Combined result is stable and responsive enough for LED orientation cues.

---

## 5) Where to tune
- **`AVG_SAMPLES` (N)**: more = smoother, slower.
- **`APP_FC_HZ` (f_c)**: higher = faster, noisier; lower = smoother, slower.
- **ODR / `APP_FS_HZ` (f_s)**: must match your data rate for correct α.

---

## 6) Minimal code (concept)

```c
// Alpha from fc, fs
static inline float lpf_alpha(float fc, float fs){
    const float two_pi_fc = 6.28318530718f * fc;
    return two_pi_fc / (two_pi_fc + fs);
}

// 1st-order LPF state
typedef struct { float a, y; } LPF1_t;

static inline void lpf_init(LPF1_t *f, float fc, float fs){
    f->a = lpf_alpha(fc, fs);
    f->y = 0.0f;
}

static inline float lpf_update(LPF1_t *f, float x){
    f->y += f->a * (x - f->y);
    return f->y;
}
