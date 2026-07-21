# MotorDriver CAN Node Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and qualify one MPS MotorDriver as a persistent-ID, persistent-joint-zero, 1 Mbps/100 Hz CAN position-plus-velocity node with synchronized apply, local hold, and fatal communication timeout.

**Architecture:** Keep CAN framing pure and hardware-independent, keep joint-angle recovery separate from Flash I/O, and put the ARM/pending/SYNC/timeout rules in a host-tested `can_motion_service`. An always-running TMR6 1 kHz service drains the bounded CAN RX queue and publishes into the existing position cascade; normal telemetry is emitted from thread context through a non-blocking TX queue.

**Tech Stack:** ARMCC V5.06 C90-compatible embedded C, AT32M412 standard peripheral library, RT-Thread Nano/msh, GCC C11 host tests under WSL, Python static/bench tests, Keil MDK/J-Link, COM9.

## Global Constraints

- Protocol semantic source: `docs/superpowers/specs/2026-07-21-dual-node-can-trajectory-design.md` at commit `1234a03`.
- Begin execution with `superpowers:using-git-worktrees` and create `E:\WorkSpaces\2_MotorDriver\MPS_MotorDriver\.worktrees\motor-driver-can-node`; do not implement directly in `main`.
- Classic CAN, standard 11-bit IDs, exactly `1,000,000 bit/s`; MotorDriver pins remain PA12 TX and PA11 RX.
- Incoming trajectory period is `10 ms`; synchronized points use a `50 ms` position lease; unmatched pending points expire at `30 ms`; communication silence holds at `50 ms` and disables/latches at `500 ms`.
- TMR1 FOC remains `16 kHz`; position/speed control remains `1 kHz`; the new always-running motion service is `1 kHz` on TMR6.
- Preserve V2 U/V/W = TMR1 CH3/CH2/CH1, fixed ADC trigger `CCR4=5264`, current reconstruction, encoder calibration, direction normalization, speed/current tuning, and `0.5 A` position-mode Iq cap.
- CAN code must submit the existing `position_setpoint_t`; it must not duplicate or retune the position controller.
- RX ISR only validates/copies bounded frames; no RT-Thread call, Flash operation, control-loop reset, blocking TX, or formatted output is allowed in a CAN or timer ISR.
- `STOP` is session-independent. A latched fault is never cleared by STOP or by renewed traffic.
- Existing encoder calibration remains at `0x0801FC00`. Joint-config slots are independent pages `0x0801F400` and `0x0801F800`; application linkable Flash becomes exactly `0x1F400` bytes (125 KiB).
- `FAULT_CAN_TIMEOUT` becomes fatal for CAN motion. New `FAULT_CAN_BUS = 1u << 9` is fatal. `FAULT_CAL_INVALID` keeps its existing current-calibration warning meaning.
- ARMCC5 declarations stay at block starts. No heap allocation is introduced.
- Every powered motion test ends with `mc_stop` or broadcast STOP and verifies DISABLED, EN=LOW, duties 2812/2812/2812, `CCR4=5264`, and no unexplained fault.
- The companion X-Track plan is separate because it belongs to another repository; physical end-to-end qualification is not claimed until both plans reach their integration gate.

## File Map

### New focused files

- `communication/can_frame.h`: hardware-neutral 11-bit Classic CAN frame value type.
- `communication/can_protocol.h/.c`: byte-exact pure protocol codec and sequence/CRC helpers.
- `application/motor_control/joint_config.h/.c`: persistent record layout, CRC32, validation, generation selection, and single-turn boot-angle solver.
- `platform/at32m412/flash_joint_config_at32m412.h/.c`: two-page atomic record storage only.
- `application/motor_control/joint_config_service.h/.c`: boot restore, capture/save/erase, and position-origin installation.
- `communication/can_at32m412.h/.c`: 1 Mbps CAN1 hardware adapter with fixed RX/TX queues and diagnostics.
- `application/motor_control/can_motion_service.h/.c`: node state machine, pending/SYNC, timeouts, and feedback scheduling.
- `platform/at32m412/can_motion_timer_at32m412.h/.c`: always-running TMR6 1 kHz tick only.
- `tests/communication/test_can_protocol.c`: protocol golden vectors.
- `tests/motor_control/test_joint_config.c`: record and angle reconstruction tests.
- `tests/motor_control/test_can_motion_service.c`: state-machine tests with injected motor/transport operations.
- `tests/communication/test_can_at32m412_static.py`: hardware-driver and ISR contract checks.
- `tests/motor_control/test_joint_config_static.py`: Flash layout/service/shell contract checks.
- `tests/motor_control/test_can_motion_integration_static.py`: timer/application/project integration checks.
- `tests/stage8_can_bench.py` and `tests/motor_control/test_stage8_can_bench.py`: checked telemetry parser and hardware acceptance runner.

### Existing files changed

- `application/motor_control/motor_params.h`: CAN timing, service timeouts, and joint Flash addresses.
- `application/motor_control/fault_manager.h`: fatal CAN timeout and CAN bus fault.
- `application/motor_control/motor_control_isr.c`: safe first-position distance check and CAN stop reuse only.
- `application/motor_app.c`: initialize/poll joint config, CAN service, and telemetry.
- `application/motor_shell.c`: persistent joint config and CAN diagnostics commands.
- `AT32M412xB_FLASH.ld`: reserve the top three 1 KiB pages.
- `CMakeLists.txt`: add new production sources.
- `project/MDK_V5/MPS_MotorDriver.uvprojx`: link length and new source entries.
- `doc/调试记录.md`, `doc/FOC控制器开发记录.md`, `CLAUDE.md`: final measured evidence only.

---

### Task 1: Freeze the Pure CAN Byte Contract

**Files:**
- Create: `communication/can_frame.h`
- Replace: `communication/can_protocol.h`
- Replace: `communication/can_protocol.c`
- Create: `tests/communication/test_can_protocol.c`

**Interfaces:**
- Consumes: no hardware and no motor globals.
- Produces: `can_frame_t`, `can_trajectory_t`, `can_broadcast_t`, `can_feedback_t`, `can_health_t`, CRC/sequence helpers, and encode/decode functions used by all later MotorDriver tasks.

- [ ] **Step 1: Write the failing protocol golden-vector test**

Create `test_can_protocol.c` with assertions equivalent to:

```c
can_frame_t frame = {0};
can_trajectory_t trajectory;
can_broadcast_t broadcast;

frame.id = 0x101u;
frame.dlc = 8u;
frame.data[0] = 0x10u; frame.data[1] = 0x27u; /* 10000 mdeg */
frame.data[4] = 0x2cu; frame.data[5] = 0x01u; /* 300 * 10 mdeg/s */
frame.data[6] = 0x34u; frame.data[7] = 0x12u;
assert(can_protocol_decode_trajectory(&frame, 1u, &trajectory));
assert(trajectory.position_mdeg == 10000);
assert(trajectory.velocity_mdeg_s == 3000);
assert(trajectory.sequence == 0x1234u);
assert(!can_protocol_decode_trajectory(&frame, 2u, &trajectory));

assert(can_protocol_crc8((const uint8_t[]){
    0x01u, 0x01u, 0x34u, 0x12u, 0x78u, 0x56u, 0x00u
}, 7u) == 0x20u);
```

Also assert exact byte arrays and round trips for all five opcodes, negative position/velocity, feedback IDs `0x181/0x182`, health IDs `0x281/0x282`, wrong DLC, extended ID rejection, reserved flag rejection, protocol version rejection, bad CRC, exact duplicate, backward sequence, half-range ambiguity, and `65535 -> 0` wrap.

- [ ] **Step 2: Run the protocol test and verify RED**

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/motor-driver-can-node; gcc -std=c11 -Wall -Wextra -Werror -Icommunication tests/communication/test_can_protocol.c communication/can_protocol.c -o /tmp/test_can_protocol; /tmp/test_can_protocol"
```

Expected: compile failure because the new value types and pure codec functions do not exist.

- [ ] **Step 3: Define the exact public protocol interface**

Use this frame type and constants:

```c
typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

#define CAN_PROTOCOL_VERSION       1u
#define CAN_ID_BROADCAST           0x080u
#define CAN_ID_TRAJECTORY(node)    (0x100u + (uint16_t)(node))
#define CAN_ID_FEEDBACK(node)      (0x180u + (uint16_t)(node))
#define CAN_ID_HEALTH(node)        (0x280u + (uint16_t)(node))

typedef enum {
    CAN_OPCODE_ARM = 0x01u,
    CAN_OPCODE_SYNC = 0x02u,
    CAN_OPCODE_STOP = 0x03u,
    CAN_OPCODE_CLEAR_FAULT = 0x04u,
    CAN_OPCODE_DISCOVER = 0x05u
} can_opcode_t;
```

Expose exactly:

```c
uint8_t can_protocol_crc8(const uint8_t *data, uint8_t len);
bool can_protocol_sequence_newer(uint16_t candidate, uint16_t previous);
bool can_protocol_decode_trajectory(const can_frame_t *frame,
                                    uint8_t node_id,
                                    can_trajectory_t *out);
bool can_protocol_decode_broadcast(const can_frame_t *frame,
                                   can_broadcast_t *out);
bool can_protocol_encode_feedback(uint8_t node_id,
                                  const can_feedback_t *value,
                                  can_frame_t *out);
bool can_protocol_encode_health(uint8_t node_id,
                                const can_health_t *value,
                                can_frame_t *out);
bool can_protocol_encode_broadcast(const can_broadcast_t *value,
                                   can_frame_t *out);
```

`can_trajectory_t.velocity_mdeg_s` is expanded to `int32_t`; the wire `int16_t` is multiplied by exactly 10 after little-endian decode. Encoders reject null pointers and node IDs outside 1..2.

- [ ] **Step 4: Implement the minimal pure codec**

Implement explicit byte shifts; do not cast the payload to packed structs. CRC uses poly `0x07`, init/xorout `0`, no reflection, over bytes 0..6. Decode signed values by first assembling `uint16_t/uint32_t`, then casting to the signed type. Health state values are fixed at 0..5 from the spec.

- [ ] **Step 5: Verify GREEN and commit**

Run the command from Step 2. Expected: `can protocol: PASS`, exit 0, no warning.

```powershell
git add -- communication/can_frame.h communication/can_protocol.h communication/can_protocol.c tests/communication/test_can_protocol.c
git commit -m "feat: define dual-node CAN protocol"
```

### Task 2: Implement Persistent Joint Record and Boot-Angle Math

**Files:**
- Create: `application/motor_control/joint_config.h`
- Create: `application/motor_control/joint_config.c`
- Create: `tests/motor_control/test_joint_config.c`

**Interfaces:**
- Consumes: corrected MA600A `raw16` samples only.
- Produces: byte-stable 36-byte `joint_config_record_t`, software CRC32, record validation, wrap-safe generation comparison, and unique allowed-range angle reconstruction.

- [ ] **Step 1: Write the failing record and reconstruction test**

The test must require this layout and behavior:

```c
typedef char record_size_must_be_36[
    sizeof(joint_config_record_t) == 36u ? 1 : -1];

joint_config_record_t cfg;
int32_t joint_mdeg;
joint_config_make(&cfg, 7u, 1u, 1000u, 0, 1, -90000, 90000);
assert(joint_config_record_valid(&cfg));
assert(joint_config_restore_angle(&cfg, 900u, &joint_mdeg));
assert(joint_mdeg >= -550 && joint_mdeg <= -548);
assert(!joint_config_restore_angle(&cfg, 32768u, &joint_mdeg));
```

Add cases for Node 2, direction `-1`, raw wrap both ways, a non-zero known joint angle, exact min/max, range width `>=360000` rejection, known angle outside range, bad magic/version/size/CRC, generation wrap ordering, neither slot valid, and two valid slots selecting the newer generation.

- [ ] **Step 2: Compile and verify RED**

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/motor-driver-can-node; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/motor_control/test_joint_config.c application/motor_control/joint_config.c -o /tmp/test_joint_config; /tmp/test_joint_config"
```

Expected: compile failure because the module is absent.

- [ ] **Step 3: Define the record exactly**

```c
#define JOINT_CONFIG_MAGIC   0x4746434Au /* 'JCFG' */
#define JOINT_CONFIG_VERSION 1u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t generation;
    uint8_t node_id;
    int8_t joint_direction;
    uint16_t zero_corrected_raw16;
    int32_t known_joint_position_mdeg;
    int32_t min_joint_position_mdeg;
    int32_t max_joint_position_mdeg;
    uint32_t reserved;
    uint32_t crc32;
} joint_config_record_t;
```

Expose `joint_config_crc32`, `joint_config_make`, `joint_config_record_valid`, `joint_config_generation_newer`, `joint_config_select_latest`, and `joint_config_restore_angle` with fixed-width parameters and boolean results.

- [ ] **Step 4: Implement validation and the unique-candidate solver**

CRC is IEEE CRC-32 over the first 32 bytes. Restore angle by computing the modulo-65536 raw delta, applying direction, converting with a 64-bit intermediate, then evaluating candidate angles offset by `k * 360000` for `k=-2..2`. Return true only when exactly one candidate lies inside the configured inclusive limits.

- [ ] **Step 5: Verify GREEN and commit**

Run Step 2. Expected: `joint config: PASS` with no warning.

```powershell
git add -- application/motor_control/joint_config.h application/motor_control/joint_config.c tests/motor_control/test_joint_config.c
git commit -m "feat: add persistent joint configuration model"
```

### Task 3: Add Power-Loss-Safe Two-Page Joint Storage

**Files:**
- Create: `platform/at32m412/flash_joint_config_at32m412.h`
- Create: `platform/at32m412/flash_joint_config_at32m412.c`
- Modify: `application/motor_control/motor_params.h`
- Modify: `AT32M412xB_FLASH.ld`
- Modify: `project/MDK_V5/MPS_MotorDriver.uvprojx`
- Create: `tests/motor_control/test_joint_config_static.py`

**Interfaces:**
- Consumes: valid `joint_config_record_t` values from Task 2.
- Produces: `flash_joint_config_read_latest()`, `flash_joint_config_write_next()`, and `flash_joint_config_erase_all()` without touching encoder calibration storage.

- [ ] **Step 1: Write the failing static storage test**

Require all of these literals and relationships:

```python
assert "#define JOINT_CFG_FLASH_A_ADDR" in params
assert "0x0801F400u" in params
assert "0x0801F800u" in params
assert "#define CAL_FLASH_ADDR" in params and "0x0801FC00u" in params
assert "LENGTH = 125K" in linker
assert "0x1F400" in uvproj
assert "flash_sector_erase(target_addr)" in flash_source
assert "joint_config_record_valid" in flash_source
```

Also assert that the source selects the latest valid generation before choosing the inactive page, programs 32-bit words, locks Flash on every failure exit, and verifies by readback.

- [ ] **Step 2: Run and verify RED**

```powershell
python tests/motor_control/test_joint_config_static.py
```

Expected: failure because pages, adapter, and reduced link range do not exist.

- [ ] **Step 3: Reserve the exact Flash layout**

Add:

```c
#define JOINT_CFG_FLASH_A_ADDR 0x0801F400u
#define JOINT_CFG_FLASH_B_ADDR 0x0801F800u
#define JOINT_CFG_FLASH_PAGE_SIZE 1024u
```

Change GCC `FLASH LENGTH` from `127K` to `125K`. In `.uvprojx`, change the target CPU IROM length and `<IROM><Size>` from `0x1FC00` to `0x1F400`; leave the physical flash-programming range `0x20000` unchanged.

- [ ] **Step 4: Implement inactive-page write and readback**

`read_latest` validates both memory-mapped records and selects by 32-bit half-range generation. `write_next` requires a valid finalized record, selects the page not containing the current latest record, erases only that page, writes nine 32-bit words, locks Flash, then requires byte-for-byte readback and `joint_config_record_valid()`. `erase_all` erases A then B while the motor-disabled precondition is enforced by the caller.

- [ ] **Step 5: Verify storage contract and commit**

Run the static test and the Task 2 C test. Both must pass.

```powershell
git add -- platform/at32m412/flash_joint_config_at32m412.h platform/at32m412/flash_joint_config_at32m412.c application/motor_control/motor_params.h AT32M412xB_FLASH.ld project/MDK_V5/MPS_MotorDriver.uvprojx tests/motor_control/test_joint_config_static.py
git commit -m "feat: persist joint configuration atomically"
```

### Task 4: Restore and Configure the Joint Coordinate Through COM9

**Files:**
- Create: `application/motor_control/joint_config_service.h`
- Create: `application/motor_control/joint_config_service.c`
- Modify: `application/motor_app.c`
- Modify: `application/motor_shell.c`
- Extend: `tests/motor_control/test_joint_config_static.py`

**Interfaces:**
- Consumes: Task 2 record logic, Task 3 storage, valid encoder snapshot fields `corrected_raw16` and `control_position_mdeg`.
- Produces: boot-ready joint origin and exact shell commands `joint_cfg_set`, `joint_cfg_show`, and `joint_cfg_erase`.

- [ ] **Step 1: Extend the static test for RED service behavior**

Require these APIs:

```c
void joint_config_service_init(void);
void joint_config_service_poll(void);
bool joint_config_service_ready(void);
uint8_t joint_config_service_node_id(void);
bool joint_config_service_capture(uint8_t node_id,
                                  int32_t known_mdeg,
                                  int8_t direction,
                                  int32_t min_mdeg,
                                  int32_t max_mdeg);
bool joint_config_service_erase(void);
```

The static test must prove `motor_app_init()` calls service init after `position_loop_init()`, `motor_app_run()` polls before sleeping, capture is rejected while any motor mode is enabled, the raw angle comes from `corrected_raw16`, and successful restore/capture calls `position_loop_set_origin(control_position_mdeg, restored_joint_mdeg)`.

- [ ] **Step 2: Run and verify RED**

```powershell
python tests/motor_control/test_joint_config_static.py
```

Expected: service/shell assertions fail.

- [ ] **Step 3: Implement boot restore and capture**

`init` reads the latest record but leaves readiness false. `poll` waits for a valid encoder snapshot, calls `joint_config_restore_angle`, installs the origin exactly once, and then exposes the persisted node ID. Capture requires DISABLED, a valid encoder, valid fields, generation `latest+1`, finalized CRC, successful inactive-page write/readback, and immediate origin installation. Erase requires DISABLED, clears readiness/origin by reinitializing the position loop, and erases both joint pages only.

- [ ] **Step 4: Add exact shell commands**

```text
joint_cfg_set <node_id> <known_mdeg> <direction> <min_mdeg> <max_mdeg>
joint_cfg_show
joint_cfg_erase
```

`joint_cfg_show` prints version, generation, node, zero raw, known/min/max mdeg, direction, CRC validity, encoder readiness, restored joint angle, and service readiness. Parsing rejects overflow, node outside 1..2, direction other than ±1, invalid range, and known angle outside range.

- [ ] **Step 5: Verify, reboot-test on host contract, and commit**

Run Task 2 C test, the extended static test, and existing position shell/static tests. Expected: all pass.

```powershell
git add -- application/motor_control/joint_config_service.h application/motor_control/joint_config_service.c application/motor_app.c application/motor_shell.c tests/motor_control/test_joint_config_static.py
git commit -m "feat: restore persistent joint zero"
```

### Task 5: Implement the AT32M412 1 Mbps CAN Driver

**Files:**
- Replace: `communication/can_at32m412.h`
- Replace: `communication/can_at32m412.c`
- Modify: `project/src/at32m412_416_int.c`
- Create: `tests/communication/test_can_at32m412_static.py`

**Interfaces:**
- Consumes: `can_frame_t` and a configured node ID.
- Produces: bounded non-blocking RX/TX queues and `can_at32m412_diag_t` for the motion service.

- [ ] **Step 1: Write the failing hardware contract test**

Require this API:

```c
bool can_at32m412_init(uint8_t node_id);
bool can_at32m412_rx_pop(can_frame_t *out);
bool can_at32m412_tx_push(const can_frame_t *frame);
void can_at32m412_tx_kick(void);
void can_at32m412_get_diag(can_at32m412_diag_t *out);
bool can_at32m412_fatal_bus_error(void);
void can_at32m412_irq_rx(void);
void can_at32m412_irq_tx(void);
void can_at32m412_irq_status(void);
void can_at32m412_irq_error(void);
```

The test must require PA12/PA11 with mux 9, APB1/CAN/GPIO clocks, standard data frames only, DLC 8 filters for `0x080` and `0x100+node`, bitrate values `div=10`, `bts1=14`, `bts2=4`, `sjw=2` at 180 MHz, RX capacity at least 8, TX capacity at least 8, and no loop waiting for transmit completion.

- [ ] **Step 2: Run and verify RED**

```powershell
python tests/communication/test_can_at32m412_static.py
```

Expected: failure against the current blocking-stub API.

- [ ] **Step 3: Implement hardware initialization and exact filters**

Initialize `can_bittime_type` with the fixed values above, unlimited hardware retransmission, discard-new-on-full receive mode, one exact filter for broadcast `0x080`, and one exact filter for the configured trajectory ID. Enable RX, TX, status, and error interrupts at priorities below TMR1 and protection but above the TMR6 service.

- [ ] **Step 4: Implement bounded IRQ queues and diagnostics**

`CAN1_RX_IRQHandler`, `CAN1_TX_IRQHandler`, `CAN1_STAT_IRQHandler`, and `CAN1_ERR_IRQHandler` in `project/src/at32m412_416_int.c` call the four adapter IRQ functions above. RX IRQ drains available hardware receive buffers, rejects non-standard/non-data/non-DLC8 frames, and pushes to the SPSC ring; on full, it releases hardware RX, increments `rx_overflow`, and latches fatal bus diagnostics. TX push only copies to the software ring. TX kick writes the next frame when hardware has space and returns immediately. TX interrupt advances the queue. Error/status IRQs snapshot REC/TEC/error-passive/bus-off and increment counters; bus-off is latched for the application rather than silently auto-resuming motion.

- [ ] **Step 5: Verify static contract and commit**

```powershell
python tests/communication/test_can_at32m412_static.py
git add -- communication/can_at32m412.h communication/can_at32m412.c project/src/at32m412_416_int.c tests/communication/test_can_at32m412_static.py
git commit -m "feat: add nonblocking AT32 CAN transport"
```

### Task 6: Implement the Host-Tested CAN Motion State Machine

**Files:**
- Create: `application/motor_control/can_motion_service.h`
- Create: `application/motor_control/can_motion_service.c`
- Create: `tests/motor_control/test_can_motion_service.c`
- Modify: `application/motor_control/fault_manager.h`
- Modify: `tests/fault_manager/test_fault_manager.c`

**Interfaces:**
- Consumes: protocol frames, joint readiness/node ID, position start/submit/stop callbacks, motor feedback callbacks, fault callbacks, and non-blocking frame send/pop callbacks.
- Produces: state `UNCONFIGURED/READY/ARMED/RUNNING/HOLD/FAULT`, pending/applied sequence, watchdog behavior, and scheduled feedback/health frames.

- [ ] **Step 1: Write the failing state-machine test with fake operations**

Define a fake operation table matching:

```c
typedef struct {
    bool (*rx_pop)(can_frame_t *out);
    bool (*tx_push)(const can_frame_t *frame);
    int (*position_start)(const position_setpoint_t *setpoint);
    int (*position_submit)(const position_setpoint_t *setpoint);
    void (*position_stop)(void);
    int32_t (*position_mdeg)(void);
    int32_t (*velocity_mdeg_s)(void);
    uint16_t (*vbus_10mv)(void);
    uint32_t (*fault_get)(void);
    void (*fault_set)(uint32_t bits);
    void (*fault_clear_can)(void);
} can_motion_ops_t;
```

Test: invalid config stays UNCONFIGURED; ready config answers DISCOVER; ARM establishes session and expected first sequence; trajectory alone does not start; matching SYNC starts with `lease_ms=50`; next point submits without reset; duplicate is idempotent; wrong session/sequence and SYNC-before-point do nothing; pending expires at 30 ms; 50 ms silence enters HOLD while position remains active; valid new point resumes; 500 ms stops and sets timeout fault; STOP works with wrong session but leaves a latched FAULT in FAULT; CLEAR_FAULT cannot clear an active condition; sequence wraps; bus fault stops immediately; feedback/health IDs and applied sequence are correct.

- [ ] **Step 2: Compile and verify RED**

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/motor-driver-can-node; gcc -std=c11 -Wall -Wextra -Werror -Icommunication -Iapplication/motor_control tests/motor_control/test_can_motion_service.c application/motor_control/can_motion_service.c communication/can_protocol.c -o /tmp/test_can_motion_service; /tmp/test_can_motion_service"
```

Expected: compile failure because the service does not exist.

- [ ] **Step 3: Define the service interface and snapshot**

Expose `can_motion_service_init`, `can_motion_service_set_joint_config`, `can_motion_service_tick_1ms`, `can_motion_service_poll_tx`, `can_motion_service_get_snapshot`, and `can_motion_service_force_stop`. The snapshot contains node/state/session/pending/applied sequence, pending/sync ages, RX/TX/protocol counters, and joint readiness. Keep all state in one static service instance; no dynamic allocation.

- [ ] **Step 4: Implement the transition table and telemetry schedule**

Each 1 ms tick drains at most four RX frames, processes STOP before other queued control frames, advances saturating ages, performs HOLD/FAULT transitions, and records due flags. `poll_tx` emits feedback every 10 ticks and health every 50 ticks or on state/fault change; it may discard an old normal telemetry snapshot when TX is full, but records the failure. First SYNC calls position start only after comparing target and actual joint angle against `POSITION_MAX_ERROR_MDEG`; later SYNC calls submit.

- [ ] **Step 5: Make CAN faults fatal and verify GREEN**

Add:

```c
FAULT_CAN_BUS = 1u << 9
```

Include both `FAULT_CAN_TIMEOUT` and `FAULT_CAN_BUS` in `FAULT_FATAL_MASK`. Run the service test and fault-manager test; both must pass without warnings.

- [ ] **Step 6: Commit**

```powershell
git add -- application/motor_control/can_motion_service.h application/motor_control/can_motion_service.c application/motor_control/fault_manager.h tests/motor_control/test_can_motion_service.c tests/fault_manager/test_fault_manager.c
git commit -m "feat: add synchronized CAN motion service"
```

### Task 7: Integrate the Always-Running 1 kHz Service

**Files:**
- Create: `platform/at32m412/can_motion_timer_at32m412.h`
- Create: `platform/at32m412/can_motion_timer_at32m412.c`
- Modify: `application/motor_app.c`
- Modify: `application/motor_control/motor_control_isr.c`
- Modify: `CMakeLists.txt`
- Modify: `project/MDK_V5/MPS_MotorDriver.uvprojx`
- Create: `tests/motor_control/test_can_motion_integration_static.py`

**Interfaces:**
- Consumes: Tasks 4-6.
- Produces: production operation callbacks, TMR6 1 kHz service execution, deferred telemetry, and build-system inclusion.

- [ ] **Step 1: Write the failing integration static test**

Require TMR6 at exactly 1 kHz from the 180 MHz timer clock, a priority below CAN RX and above SysTick, `TMR6_DAC_GLOBAL_IRQHandler` calling only `can_motion_service_tick_1ms`, and `motor_app_run` calling joint restore plus `can_motion_service_poll_tx` before its existing delay. Require all new `.c` files in both CMake and uvprojx.

- [ ] **Step 2: Run and verify RED**

```powershell
python tests/motor_control/test_can_motion_integration_static.py
```

Expected: missing timer, callbacks, and project entries.

- [ ] **Step 3: Add the TMR6 adapter**

Use prescaler 179 and period 999 for `180 MHz / (180 * 1000) = 1 kHz`. Clear the overflow flag before enabling and use preemption priority 4; CAN RX remains priority 3, TMR1 remains 0, nFAULT remains 1, and SysTick remains 14. TMR7 may share priority 4 because neither same-priority handler nests. The ISR contains the flag clear and one service call only.

- [ ] **Step 4: Wire production operations and safe startup order**

Initialization order is: safe PWM/current/encoder, fault/control/calibration/loops, joint-config service, CAN motion service with production callbacks, CAN hardware only after a valid node ID exists, TMR6 service, encoder acquisition. While joint config is not ready, keep CAN uninitialized; after restore, initialize filters for persisted node ID once and call `can_motion_service_set_joint_config(true, node)`.

`motor_app_run` continues polling joint restore. Once ready it polls CAN TX/diagnostics every loop; fatal driver diagnostics call the service stop path. Do not start PWM during ARM—only the first matching SYNC may call position start.

- [ ] **Step 5: Add first-target distance defense**

Before `motor_control_isr_position_start()` enables outputs, convert current sensor position into joint coordinates and reject a setpoint whose absolute difference exceeds `POSITION_MAX_ERROR_MDEG`. Add a static assertion/test that this check occurs before `motor_pwm_at32m412_enable_output()`.

- [ ] **Step 6: Verify integration and commit**

Run integration static test, service C test, position ISR/static tests, and joint config tests. Expected: all pass.

```powershell
git add -- platform/at32m412/can_motion_timer_at32m412.h platform/at32m412/can_motion_timer_at32m412.c application/motor_app.c application/motor_control/motor_control_isr.c CMakeLists.txt project/MDK_V5/MPS_MotorDriver.uvprojx tests/motor_control/test_can_motion_integration_static.py
git commit -m "feat: integrate 1khz CAN motion scheduling"
```

### Task 8: Add Checked CAN Diagnostics and Bench Parser

**Files:**
- Modify: `application/motor_shell.c`
- Create: `tests/stage8_can_bench.py`
- Create: `tests/motor_control/test_stage8_can_bench.py`
- Extend: `tests/motor_control/test_can_motion_integration_static.py`

**Interfaces:**
- Consumes: motion-service/driver snapshots and existing position/current debug snapshots.
- Produces: `can_status`, `can_diag_reset`, a checksummed parser, and deterministic cleanup for powered tests.

- [ ] **Step 1: Write failing parser and shell-contract tests**

Freeze this compact line:

```text
cs id=1 s=3 se=4660 p=43 a=42 pa=0 sa=7 rx=120 tx=240 pe=0 ro=0 bo=0 te=0 f=00000000 k=89ABCDEF
```

Fields are node, state, session, pending/applied sequence, pending/sync ages, RX/TX counts, protocol errors, RX overflow, bus-off events, TX errors, faults, and XOR checksum seeded with `0x43414E31`. Tests reject missing/duplicate fields, bad hex, truncation, checksum changes, and impossible state/node values.

- [ ] **Step 2: Run and verify RED**

```powershell
python tests/motor_control/test_stage8_can_bench.py
python tests/motor_control/test_can_motion_integration_static.py
```

Expected: commands/parser absent.

- [ ] **Step 3: Implement shell diagnostics**

`can_status` only snapshots and prints; it never changes state. `can_diag_reset` is accepted only while DISABLED/READY and clears counters, not faults, session, or joint configuration. Keep the line below the proven UART DMA-safe size.

- [ ] **Step 4: Implement bench parser and guaranteed stop**

The script accepts a CAN-peer adapter object so the companion X-Track runner or a future USB-CAN adapter can provide `arm`, `submit`, `sync`, `stop`, and feedback reads. Its `finally` block sends broadcast STOP three times, sends `mc_stop` on COM9, and asserts the existing safe PWM state. It calculates point rate, missing/duplicate sequences, static/reversal/sine error, peak Iq, hold latency, fatal timeout latency, and driver counters.

- [ ] **Step 5: Verify and commit**

```powershell
python tests/motor_control/test_stage8_can_bench.py
python tests/motor_control/test_can_motion_integration_static.py
git add -- application/motor_shell.c tests/stage8_can_bench.py tests/motor_control/test_stage8_can_bench.py tests/motor_control/test_can_motion_integration_static.py
git commit -m "test: add CAN node qualification harness"
```

### Task 9: Full Software Regression and Firmware Build

**Files:**
- No new behavior files.

**Interfaces:**
- Consumes: Tasks 1-8.
- Produces: a reproducible firmware image and software-only CAN node qualification.

- [ ] **Step 1: Run every new focused test**

Run the exact compile commands from Tasks 1, 2, and 6, then:

```powershell
python -m pytest tests/communication/test_can_at32m412_static.py tests/motor_control/test_joint_config_static.py tests/motor_control/test_can_motion_integration_static.py tests/motor_control/test_stage8_can_bench.py -q
git diff --check
```

Expected: all PASS/exit 0.

- [ ] **Step 2: Run existing control regressions**

Run all Python tests and the focused existing C regressions:

```powershell
python -m pytest tests -q
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/motor-driver-can-node; gcc -std=c11 -Wall -Wextra -Werror -Itests/encoder_service -Iapplication/motor_control tests/encoder_service/test_encoder_direction.c application/motor_control/encoder_service.c -o /tmp/test_encoder_direction; /tmp/test_encoder_direction; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/current_sense/test_current_reconstruction.c application/motor_control/current_reconstruction.c -lm -o /tmp/test_current_reconstruction; /tmp/test_current_reconstruction; gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_current_loop.c -lm -o /tmp/test_current_loop; /tmp/test_current_loop; gcc -std=c11 -Wall -Wextra -Werror tests/motor_control/test_speed_loop.c -lm -o /tmp/test_speed_loop; /tmp/test_speed_loop; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/motor_control/test_position_loop.c application/motor_control/position_loop.c -lm -o /tmp/test_position_loop; /tmp/test_position_loop; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control tests/fault_manager/test_fault_manager.c application/motor_control/fault_manager.c -o /tmp/test_fault_manager; /tmp/test_fault_manager"
```

Expected: no regression and no compiler warning.

- [ ] **Step 3: Clean-build ARMCC5**

```powershell
project\MDK_V5\build.bat clean
```

Expected: HEX/AXF produced, `0 Error(s), 0 Warning(s)`, application image ends below `0x0801F400`.

- [ ] **Step 4: Flash and verify unconfigured safety before writing config**

Flash with `project\MDK_V5\flash.bat only`. On COM9 require DISABLED, EN low, no CAN motion, and `joint_cfg_show` reporting invalid/unconfigured until a record is written. Encoder calibration at `0x0801FC00` must remain valid.

- [ ] **Step 5: Commit only a necessary build fix if one was test-driven**

If Steps 1-4 required a production correction, first add a failing regression test, make the minimum correction, rerun Steps 1-4, and commit only those exact paths with a message describing the root cause. If no correction was required, create no empty commit.

### Task 10: Configure and Qualify MotorDriver Node 1

**Files:**
- Modify: `doc/调试记录.md`
- Modify: `doc/FOC控制器开发记录.md`
- Modify: `CLAUDE.md`

**Interfaces:**
- Consumes: companion X-Track CAN transport or another byte-compatible CAN peer, COM9 diagnostics, and the Stage 8 bench.
- Produces: measured single-node evidence; it does not claim dual-node or mechanism qualification.

- [ ] **Step 1: Capture the persistent zero safely**

With shaft free, output disabled, encoder valid, and the joint placed in the agreed known pose, run:

```text
joint_cfg_set 1 0 1 -90000 90000
joint_cfg_show
```

Power-cycle and require Node 1, CRC valid, READY, and restored angle within the encoder quantization of the pre-power-cycle value. Repeat once near the raw16 wrap boundary if the free shaft permits.

- [ ] **Step 2: Verify physical CAN without motion**

Connect the 1 Mbps peer and two 120 Ω end terminations. Send DISCOVER; require Node 1 health with version 1, READY, session 0, plausible VBUS, no bus-off/overflow. Send STOP with arbitrary session and require it remains safely READY.

- [ ] **Step 3: Bring up bounded motion**

ARM a fresh session, preload actual-position ±1000 mdeg points with zero feedforward, SYNC each point, then increase only after correct direction and applied sequence are observed. Abort on wrong sign, tracking fault, current saturation, missing feedback, or continued motion after STOP.

- [ ] **Step 4: Run the 100 Hz qualification**

Run at least 10 minutes at 100 Hz with 10° amplitude and peak 30°/s. Require no bus-off, RX overflow, unexplained sequence loss, or fault; P95 position error no worse than about 1.5°, and peak Iq at or below 0.5 A. Then test peak 60°/s without increasing the current limit and record the result rather than weakening the gate.

- [ ] **Step 5: Verify the independent watchdog**

During motion stop valid SYNC. Require HOLD at 50 ms with reference frozen/velocity feedforward zero; require output disabled and `FAULT_CAN_TIMEOUT` latched at 500 ms. Renewed traffic must not re-enable until CLEAR_FAULT, ARM, and a safe first SYNC.

- [ ] **Step 6: Repeat persistence and final safety**

Power-cycle, rediscover, ARM, and repeat a short sine without re-zeroing. Finish with broadcast STOP and `mc_stop`; verify DISABLED, EN low, PWM 2812/2812/2812, CCR4 5264, and only intentionally induced/cleared faults.

- [ ] **Step 7: Record and commit evidence**

Record firmware commit, supply, termination, node config, test duration, frame rate, error counters, static/dynamic metrics, 50/500 ms results, persistence result, and final safety. Explicitly state that Node 2, cross-node skew, and five-bar mechanics remain for the companion plan.

```powershell
git add -- 'doc/调试记录.md' 'doc/FOC控制器开发记录.md' CLAUDE.md
git commit -m "docs: qualify motor CAN node one"
```

### Task 11: Final MotorDriver Verification Gate

**Files:**
- No behavior changes.

**Interfaces:**
- Consumes: fresh evidence from all prior tasks.
- Produces: exact MotorDriver commit and a stable protocol handoff to X-Track.

- [ ] **Step 1: Run fresh complete verification**

Repeat all new and existing host/static tests, ARMCC5 clean build, Stage 7 local position smoke, Stage 8 CAN qualification subset, `git diff --check`, and `git status --short`.

- [ ] **Step 2: Audit every MotorDriver spec gate**

Map evidence to protocol bytes, joint persistence, unconfigured lockout, sequence/session rules, 30/50/500 ms timing, STOP priority, bus fault behavior, 100 Hz feedback, single-motor tracking, current cap, no regressions, and final disabled state. Mark any missing physical evidence as incomplete.

- [ ] **Step 3: Hand off exact constants**

Report the exact commit and frozen values: IDs, protocol version, opcodes, CRC parameters, 1 Mbps, units, node state values, fault bits, and timing. The X-Track implementation must consume these without changing MotorDriver control behavior.
