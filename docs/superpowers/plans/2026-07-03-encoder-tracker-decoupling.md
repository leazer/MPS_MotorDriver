# Encoder Tracker Decoupling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove blocking MA600A SPI reads from the 16 kHz FOC ISR and replace direct encoder speed usage with a lightweight angle tracker/PLL.

**Architecture:** A lower-priority encoder acquisition path reads MA600A raw angle and feeds `encoder_service` plus a new `encoder_tracker`. The FOC ISR only consumes the latest predicted electrical angle from `encoder_tracker`, so its execution time stays deterministic and independent of SPI latency. Speed-loop diagnostics and future speed control use tracker-derived speed instead of MA600A `speed_raw`.

**Tech Stack:** AT32M412 bare-metal/RT-Thread C, Keil ARMCC C90 constraints, existing `encoder_service`, `motor_control_isr`, Python static tests.

---

## File Structure

- Create `application/motor_control/encoder_tracker.h`: public API for tracker init, sample update, FOC tick prediction, angle/speed reads, and diagnostics.
- Create `application/motor_control/encoder_tracker.c`: fixed-cost angle PLL from raw mechanical angle to predicted electrical angle and filtered electrical speed.
- Modify `application/motor_control/encoder_service.h`: add a non-SPI sample ingestion API so service can accept raw samples from an acquisition path.
- Modify `application/motor_control/encoder_service.c`: split sample acceptance from SPI read; keep existing diagnostics and shell behavior.
- Modify `application/motor_control/motor_control_isr.c`: remove `encoder_service_update_from_isr()` from the FOC hot path and consume `encoder_tracker` instead.
- Modify `application/motor_app.c`: initialize `encoder_tracker`.
- Modify `application/motor_shell.c`: extend `enc_status` or `mc_debug` to print tracker angle/speed/sample age.
- Modify `CMakeLists.txt` and `project/MDK_V5/MPS_MotorDriver.uvprojx`: include `encoder_tracker.c`.
- Modify `tests/encoder_service/test_encoder_service_static.py`: enforce that `motor_control_isr.c` does not call SPI/service update and that tracker APIs exist.

## Task 1: Add Static Tests For Decoupling Contract

**Files:**
- Modify: `tests/encoder_service/test_encoder_service_static.py`

- [ ] **Step 1: Add failing static tests**

Append these checks:

```python
def test_foc_isr_does_not_read_encoder_spi():
    isr = read(ROOT / "application" / "motor_control" / "motor_control_isr.c")
    assert "encoder_service_update_from_isr" not in isr
    assert "motor_encoder_read_raw_frame" not in isr
    assert "ma600a_read_angle_and_speed_raw" not in isr
    assert "encoder_tracker_get_electrical_angle_rad" in isr


def test_encoder_tracker_module_exists_and_is_built():
    header = read(ROOT / "application" / "motor_control" / "encoder_tracker.h")
    source = read(ROOT / "application" / "motor_control" / "encoder_tracker.c")
    cmake = read(CMAKE)
    mdk = read(MDK_PROJECT)
    for token in [
        "encoder_tracker_init",
        "encoder_tracker_update_sample",
        "encoder_tracker_tick",
        "encoder_tracker_get_electrical_angle_rad",
        "encoder_tracker_get_speed_rad_s",
        "encoder_tracker_get_sample_age_ticks",
    ]:
        assert token in header + source
    assert "application/motor_control/encoder_tracker.c" in cmake
    assert "<FileName>encoder_tracker.c</FileName>" in mdk
```

Add these two functions to the `__main__` runner.

- [ ] **Step 2: Run test to verify it fails**

Run: `python MPS_MotorDriver\tests\encoder_service\test_encoder_service_static.py`

Expected: FAIL because `encoder_tracker.h/c` do not exist and `motor_control_isr.c` still calls `encoder_service_update_from_isr`.

## Task 2: Add Encoder Tracker Skeleton

**Files:**
- Create: `application/motor_control/encoder_tracker.h`
- Create: `application/motor_control/encoder_tracker.c`
- Modify: `CMakeLists.txt`
- Modify: `project/MDK_V5/MPS_MotorDriver.uvprojx`

- [ ] **Step 1: Create public header**

```c
#ifndef ENCODER_TRACKER_H
#define ENCODER_TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t raw16;
    int32_t  elec_mrad;
    int32_t  speed_mrad_s;
    uint32_t sample_count;
    uint32_t stale_ticks;
    uint8_t  valid;
} encoder_tracker_snapshot_t;

void encoder_tracker_init(void);
void encoder_tracker_update_sample(uint16_t raw16, uint8_t valid);
void encoder_tracker_tick(void);
float encoder_tracker_get_electrical_angle_rad(void);
float encoder_tracker_get_speed_rad_s(void);
uint32_t encoder_tracker_get_sample_age_ticks(void);
bool encoder_tracker_get_snapshot(encoder_tracker_snapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif
```

- [ ] **Step 2: Create minimal implementation**

Use electrical angle from raw sample and hold last angle. Keep PLL gains conservative for the first pass.

```c
#include "encoder_tracker.h"
#include "motor_params.h"
#include <string.h>

#define TWO_PI_F      6.28318530718f
#define ISR_DT_S      (1.0f / 16000.0f)
#define TRACKER_KP    0.20f
#define TRACKER_KI    200.0f

static float s_theta_e;
static float s_omega_e;
static uint16_t s_last_raw16;
static uint32_t s_sample_count;
static uint32_t s_age_ticks;
static uint8_t s_valid;

static float wrap_pm_pi(float x)
{
    while (x > 3.14159265359f) {
        x -= TWO_PI_F;
    }
    while (x < -3.14159265359f) {
        x += TWO_PI_F;
    }
    return x;
}

static float raw_to_elec_rad(uint16_t raw16)
{
    float theta;
    theta = ((float)raw16 * (float)MOTOR_POLE_PAIRS * TWO_PI_F) / 65536.0f;
    while (theta >= TWO_PI_F) {
        theta -= TWO_PI_F;
    }
    while (theta < 0.0f) {
        theta += TWO_PI_F;
    }
    return theta;
}

void encoder_tracker_init(void)
{
    s_theta_e = 0.0f;
    s_omega_e = 0.0f;
    s_last_raw16 = 0u;
    s_sample_count = 0u;
    s_age_ticks = 0u;
    s_valid = 0u;
}

void encoder_tracker_update_sample(uint16_t raw16, uint8_t valid)
{
    float measured;
    float err;

    if (valid == 0u) {
        return;
    }
    measured = raw_to_elec_rad(raw16);
    if (s_valid == 0u) {
        s_theta_e = measured;
        s_omega_e = 0.0f;
        s_valid = 1u;
    } else {
        err = wrap_pm_pi(measured - s_theta_e);
        s_theta_e += TRACKER_KP * err;
        s_omega_e += TRACKER_KI * err * ISR_DT_S;
    }
    while (s_theta_e >= TWO_PI_F) s_theta_e -= TWO_PI_F;
    while (s_theta_e < 0.0f) s_theta_e += TWO_PI_F;
    s_last_raw16 = raw16;
    s_sample_count++;
    s_age_ticks = 0u;
}

void encoder_tracker_tick(void)
{
    s_theta_e += s_omega_e * ISR_DT_S;
    while (s_theta_e >= TWO_PI_F) s_theta_e -= TWO_PI_F;
    while (s_theta_e < 0.0f) s_theta_e += TWO_PI_F;
    if (s_age_ticks < 0xFFFFFFFFu) {
        s_age_ticks++;
    }
}

float encoder_tracker_get_electrical_angle_rad(void)
{
    return s_theta_e;
}

float encoder_tracker_get_speed_rad_s(void)
{
    return s_omega_e;
}

uint32_t encoder_tracker_get_sample_age_ticks(void)
{
    return s_age_ticks;
}

bool encoder_tracker_get_snapshot(encoder_tracker_snapshot_t *out)
{
    if (out == 0) {
        return false;
    }
    out->raw16 = s_last_raw16;
    out->elec_mrad = (int32_t)(s_theta_e * 1000.0f);
    out->speed_mrad_s = (int32_t)(s_omega_e * 1000.0f);
    out->sample_count = s_sample_count;
    out->stale_ticks = s_age_ticks;
    out->valid = s_valid;
    return s_valid != 0u;
}
```

- [ ] **Step 3: Add source to build files**

Add `application/motor_control/encoder_tracker.c` to CMake and Keil application/motor_control group.

- [ ] **Step 4: Run test**

Run: `python MPS_MotorDriver\tests\encoder_service\test_encoder_service_static.py`

Expected: decoupling test still fails because ISR still calls service update, tracker build test passes.

## Task 3: Split Encoder Service Sample Ingestion From SPI Read

**Files:**
- Modify: `application/motor_control/encoder_service.h`
- Modify: `application/motor_control/encoder_service.c`

- [ ] **Step 1: Add service API test**

In `test_encoder_service_public_api_exists`, add token:

```python
"encoder_service_update_sample",
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python MPS_MotorDriver\tests\encoder_service\test_encoder_service_static.py`

Expected: FAIL because `encoder_service_update_sample` is missing.

- [ ] **Step 3: Add API declaration**

```c
int encoder_service_update_sample(uint16_t raw, int16_t speed, uint8_t bus_ok);
```

- [ ] **Step 4: Implement ingestion**

Refactor `encoder_service_update_from_isr()` so it calls:

```c
int encoder_service_update_sample(uint16_t raw, int16_t speed, uint8_t bus_ok)
{
    int16_t delta = 0;

    s_snapshot.sample_count++;
    if (bus_ok == 0u) {
        s_snapshot.bus_error_count++;
        s_consec_error_count++;
        return -1;
    }

    if (s_has_prev) {
        delta = encoder_raw_delta(s_snapshot.raw16, raw);
        if ((delta > ENC_MAX_DELTA_PER_TICK) || (delta < -ENC_MAX_DELTA_PER_TICK)) {
            s_snapshot.spike_count++;
            s_snapshot.spike_rejected = 1u;
            s_snapshot.last_rejected_raw16 = raw;
            s_snapshot.last_rejected_delta = delta;
            s_consec_error_count++;
            if (s_consec_error_count >= ENC_CONSEC_ERROR_THRESHOLD) {
                s_has_prev = false;
                return encoder_accept_sample(raw, speed, 0);
            }
            return -2;
        }
    }
    return encoder_accept_sample(raw, speed, delta);
}
```

Then make `encoder_service_update_from_isr()` only do SPI read and call the new API.

- [ ] **Step 5: Run test**

Run: `python MPS_MotorDriver\tests\encoder_service\test_encoder_service_static.py`

Expected: same ISR decoupling failure remains; service API tests pass.

## Task 4: Add Encoder Acquisition Function

**Files:**
- Modify: `application/motor_control/encoder_service.h`
- Modify: `application/motor_control/encoder_service.c`
- Modify: `application/motor_control/encoder_tracker.c`

- [ ] **Step 1: Define behavior**

For the first implementation, acquisition can be called from a lower-priority timer or thread:

```c
int encoder_service_acquire_once(void);
```

It reads angle-only first, sets `speed_raw` to zero, updates service, and feeds tracker. This removes speed_raw dependency and halves SPI traffic versus the current angle+speed frame.

- [ ] **Step 2: Add static test**

Assert that `encoder_service_acquire_once` calls `motor_encoder_read_angle_only` or `motor_encoder_read_raw_frame` depending on which platform API exists. Prefer adding `motor_encoder_read_angle_raw`.

- [ ] **Step 3: Add platform angle-only read**

In `platform/at32m412/motor_encoder_at32m412.h`:

```c
int motor_encoder_read_angle_raw(uint16_t *raw_angle_16);
```

In `motor_encoder_at32m412.c`:

```c
int motor_encoder_read_angle_raw(uint16_t *raw_angle_16)
{
    if (raw_angle_16 == 0) {
        s_error_count++;
        return -1;
    }
    if (ma600a_read_angle_raw(&s_ma600a, raw_angle_16) != 0) {
        s_error_count++;
        return -1;
    }
    s_last_raw16 = *raw_angle_16;
    return 0;
}
```

- [ ] **Step 4: Implement acquisition**

```c
int encoder_service_acquire_once(void)
{
    uint16_t raw = 0u;
    int ret;

    ret = motor_encoder_read_angle_raw(&raw);
    if (ret == 0) {
        encoder_tracker_update_sample(raw, 1u);
        return encoder_service_update_sample(raw, 0, 1u);
    }
    encoder_tracker_update_sample(0u, 0u);
    return encoder_service_update_sample(0u, 0, 0u);
}
```

- [ ] **Step 5: Run tests**

Run:

```text
python MPS_MotorDriver\tests\encoder_service\test_encoder_service_static.py
python MPS_MotorDriver\middlewares\msp\ma600\test_ma600a_static.py
```

Expected: service tests pass except ISR decoupling until Task 5.

## Task 5: Change FOC ISR To Consume Tracker Only

**Files:**
- Modify: `application/motor_control/motor_control_isr.c`
- Modify: `application/motor_app.c`

- [ ] **Step 1: Initialize tracker**

Add `#include "encoder_tracker.h"` to `motor_app.c`, then call:

```c
encoder_tracker_init();
```

after `encoder_service_init()`.

- [ ] **Step 2: Replace ISR encoder block**

In `motor_control_isr_tick()`, replace the full `encoder_service_update_from_isr()` block with:

```c
encoder_tracker_tick();
s_enc_theta_e = encoder_tracker_get_electrical_angle_rad();
s_dbg_enc_theta_mrad = (int32_t)(s_enc_theta_e * RAD_TO_MRAD_F);
s_dbg_enc_alive = (encoder_tracker_get_sample_age_ticks() < ENC_FAIL_THRESHOLD) ? 1u : 0u;
if (encoder_tracker_get_sample_age_ticks() >= ENC_FAIL_THRESHOLD) {
    fault_manager_set(FAULT_SENSOR);
}
```

Then keep mode branches unchanged: `OPEN_LOOP` and `CURRENT` still use `s_enc_theta_e`.

- [ ] **Step 3: Preserve ALIGN sampling**

Move ALIGN sample accumulation out of FOC ISR. In the acquisition path, if ALIGN is active, service snapshot raw can be accumulated from latest accepted raw. If direct access to `s_align_*` from service is too coupled, add a small function in `motor_control_isr.c`:

```c
void motor_control_isr_on_encoder_sample(uint16_t raw16)
{
    if (s_align_active) {
        s_align_tick_cnt++;
        if (s_align_tick_cnt > (ALIGN_SETTLE_MS * PWM_FREQUENCY_HZ / 1000u)) {
            s_align_sum += raw16;
            s_align_sample_cnt++;
        }
    }
}
```

Declare it in `motor_control_isr.h` and call it from `encoder_service_acquire_once()` after a valid sample.

- [ ] **Step 4: Run tests**

Run: `python MPS_MotorDriver\tests\encoder_service\test_encoder_service_static.py`

Expected: `test_foc_isr_does_not_read_encoder_spi` passes.

## Task 6: Add A Temporary Thread-Based Acquisition Path

**Files:**
- Modify: `application/motor_app.c`

- [ ] **Step 1: Add a low-priority acquisition thread**

Use RT-Thread thread context first to validate architecture before adding a hardware timer:

```c
static void encoder_acq_thread(void *parameter)
{
    (void)parameter;
    while (1) {
        (void)encoder_service_acquire_once();
        rt_thread_mdelay(1);
    }
}
```

Create it in `motor_app_init()` with a lower priority than the main control setup and finsh-safe stack.

- [ ] **Step 2: Validate timing manually**

Use LED/DWT around `motor_control_isr_tick()` after Task 5.

Expected:
- FOC ISR no longer includes the 18 us SPI cost.
- `mc_debug encoder alive=1` once acquisition thread runs.
- `enc_status` raw changes when rotating shaft.

## Task 7: Replace Thread Acquisition With Timer Acquisition

**Files:**
- Create: `platform/at32m412/encoder_acq_timer_at32m412.h`
- Create: `platform/at32m412/encoder_acq_timer_at32m412.c`
- Modify: `application/motor_app.c`
- Modify: build files

- [ ] **Step 1: Choose timer and frequency**

Use a spare general-purpose timer at 4 kHz initially. IRQ priority must be lower than `TMR1_OVF_TMR10_IRQn`.

- [ ] **Step 2: Implement timer ISR**

Timer ISR body:

```c
void ENCODER_ACQ_TIMER_IRQHandler(void)
{
    if (tmr_flag_get(ENCODER_ACQ_TIMER, TMR_OVF_FLAG) != RESET) {
        tmr_flag_clear(ENCODER_ACQ_TIMER, TMR_OVF_FLAG);
        (void)encoder_service_acquire_once();
    }
}
```

- [ ] **Step 3: Remove temporary thread**

Delete `encoder_acq_thread` after timer validates.

- [ ] **Step 4: Bench validate**

Measure:
- FOC ISR total high pulse.
- Encoder timer pulse.
- Sample age reported by tracker.
- Current loop stability in `enc` mode.

## Task 8: Tune Tracker And Remove MA600A Speed Dependency

**Files:**
- Modify: `application/motor_control/encoder_tracker.c`
- Modify: `application/motor_shell.c`

- [ ] **Step 1: Add diagnostic output**

In `enc_status`, print:

```c
encoder_tracker_snapshot_t trk;
if (encoder_tracker_get_snapshot(&trk)) {
    rt_kprintf("trk_theta : %ld mrad\n", (long)trk.elec_mrad);
    rt_kprintf("trk_speed : %ld mrad/s\n", (long)trk.speed_mrad_s);
    rt_kprintf("trk_age   : %lu ticks\n", trk.stale_ticks);
}
```

- [ ] **Step 2: Tune gains**

Start with:
- 4 kHz acquisition: `TRACKER_KP = 0.20f`, `TRACKER_KI = 200.0f`
- If angle lags under acceleration, increase KP.
- If speed is noisy, reduce KI or add a one-pole speed low-pass for diagnostics.

- [ ] **Step 3: Bench acceptance**

Expected:
- `trk_speed` smoother than MA600A `speed_raw`.
- `theta` remains stable at standstill.
- Current loop remains stable with encoder angle.
- FOC ISR total time no longer changes when encoder SPI timing changes.

## Self-Review

- Spec coverage: SPI is removed from FOC ISR in Task 5; tracker replaces speed_raw in Tasks 2 and 8; official mclib split is mirrored by acquisition vs FOC vs speed/diagnostics paths.
- Placeholder scan: no unfinished placeholder markers are present; timer choice is deferred to Task 7 with exact implementation pattern because current project timer availability must be checked before coding.
- Type consistency: tracker APIs are introduced in Task 2 and used with matching names in later tasks.
