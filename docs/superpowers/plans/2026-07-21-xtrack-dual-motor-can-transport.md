# X-Track Dual-Motor CAN Transport Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace X-Track's blocked hardware transport with a tested 1 Mbps CAN transport that submits synchronized 100 Hz position-plus-velocity points, merges same-sequence dual-node feedback, and stops both motors on any node or communication failure.

**Architecture:** Mirror the frozen MotorDriver wire contract in a pure C++11 codec, hide `HardwareCAN` behind an injectable port interface, and implement discovery/ARM/submit/feedback/STOP in `CanMotorTransport`. Production uses Node 1 + Node 2; a conspicuous compile-time single-node commissioning mode exists only to qualify one free motor before the second node is admitted.

**Tech Stack:** ARMCC V5.06 C++11-compatible firmware, AT32F435 `HardwareCAN`, existing MotorDemo controller/service, GNU Make + g++ host tests under WSL, Python static tests, Keil MDK.

## Global Constraints

- Target repository root: `E:\WorkSpaces\5_CanSerialTool\X-TRACK` (implementation worktree to be selected at execution time).
- Protocol semantic source: MotorDriver `docs/superpowers/specs/2026-07-21-dual-node-can-trajectory-design.md` and `communication/can_protocol.{h,c}` at qualified commit `2a9a004646f63013a27f1574682cc2ecae5aad40`.
- Freeze check before implementation: SHA-256 is `D49657A9506FBA18EDCC446288EE761839B63EEC1D993D81C210E82C8C6CBA5F` for the design, `C98E8D101F5B8369F10A6BA579FB1C2CA84F7BF81634742AD6C7A228FFDD494A` for `can_protocol.h`, and `8E1A9FD6DCE07C3B09F2EB1D61A0CBB15E5F06936CB2CF64215CF8AF25C4E8E4` for `can_protocol.c`; if any differs, stop and reconcile both repositories' golden vectors before coding.
- Do not implement this plan in the currently dirty `five-bar-motor-demo-offline` worktree. Preserve its user/other-session edits and generated test directories; begin execution with `superpowers:using-git-worktrees` after that branch's intended changes are committed.
- Classic CAN, 11-bit IDs, `1,000,000 bit/s`, CAN1 PB9 TX/PB8 RX, two 120 Ω end terminations.
- Every 10 ms dual-node submission is Node 1 preload, Node 2 preload, then broadcast SYNC with the same 16-bit sequence.
- Protocol version/opcodes/state numbers/CRC/units/IDs/timeouts must exactly match the MotorDriver spec; do not create a second semantic variant.
- Production required-node mask is always `0x03`. Single-node mode is explicit commissioning-only, free-shaft-only, visibly reported, and final production verification enforces it is disabled.
- Any node fault, session/version mismatch, feedback loss over 30 ms, sequence mismatch over 30 ms, or CAN write/health failure triggers a three-frame broadcast STOP and blocks further Submit calls.
- `IMotorTransport` remains the controller boundary. Existing trajectory planning, 10 ms scheduler, geometry gates, simulator, storage, and UI behavior must continue passing.
- Host-testable protocol/transport code must not include `Arduino.h` or `HardwareCAN.h`.
- ARMCC5 compatibility remains mandatory: no C++14 features, exceptions, RTTI dependence, heap allocation, or scoped-enum assumptions outside existing compatibility macros.
- Hardware builds must call `setMode(CAN_MODE_COMMUNICATE)` after `HardwareCAN::begin`, because the current driver initializes in listen-only mode.
- Qualified MotorDriver Node 1 is persisted as node `1`, known pose `0 mdeg`, direction `+1`, limits `[-90000, 90000] mdeg`; its encoder calibration and joint-record CRC survive reset. It deliberately latches `FAULT_CAN_BUS` after bus-off and does not recover that hardware latch through `CLEAR_FAULT`; restore the physical bus and reboot the node before rediscovery.
- Bring-up order is physical bus plus both terminations, powered X-Track in communicate mode, then MotorDriver reboot. Never boot a configured MotorDriver on an unacknowledged bus and then treat its expected bus-off latch as an application protocol failure.

## File Map

Paths below are relative to the X-Track repository root.

### New focused files

- `Software/X-Track/USER/App/Features/MotorDemo/Transport/MotorCanProtocol.h/.cpp`: pure byte codec and constants.
- `Software/X-Track/USER/App/Features/MotorDemo/Transport/IMotorCanPort.h`: hardware-neutral port/frame/diagnostic interface.
- `Software/X-Track/USER/App/Features/MotorDemo/Transport/HardwareMotorCanPort.h/.cpp`: PB9/PB8 `HardwareCAN` adapter.
- `Software/X-Track/USER/App/Features/MotorDemo/Transport/CanMotorTransport.h/.cpp`: real transport and per-node state.
- `Software/X-Track/tests/motor_demo/test_motor_can_protocol.cpp`: golden vectors shared semantically with MotorDriver.
- `Software/X-Track/tests/motor_demo/test_can_motor_transport.cpp`: fake-port transport behavior.
- `Software/X-Track/tests/motor_demo/test_can_transport_static.py`: production wiring, commissioning guard, and build inclusion.

### Existing files changed

- `Software/X-Track/MDK-ARM_F435/Platform/Core/HardwareCAN.h/.cpp`: expose bus-off/RX-overflow diagnostics without changing queue semantics.
- `Software/X-Track/USER/App/Features/MotorDemo/Transport/IMotorTransport.h`: add precise CAN-related transport errors and the explicit `ClearFault()` operation.
- `Software/X-Track/USER/App/Common/DataProc/DP_MotorDemoService.h/.cpp`: own port + real transport and choose safe config.
- `Software/X-Track/USER/App/Features/MotorDemo/MotorDemoBuild.inc`: include real transport sources.
- `Software/X-Track/USER/App/Config/Config.h`: final-zero commissioning selector.
- `Software/X-Track/tests/motor_demo/test_transport.cpp`: remove hardware-stub expectations while retaining simulator coverage.
- `Software/X-Track/tests/motor_demo/test_controller.cpp`, `test_service.cpp`: real fake-port hardware discovery/arm/failure scenarios where needed.
- `Software/X-Track/tests/motor_demo/Makefile`: build new focused tests and real transport.
- `Software/X-Track/tests/motor_demo/test_integration_static.py`: replace stub-blocking assertions.
- `Software/X-Track/USER/App/Features/MotorDemo/Transport/CanMotorTransportStub.h/.cpp`: delete only after all consumers move.

---

### Task 1: Mirror the MotorDriver Wire Contract in Pure C++

**Files:**
- Create: `Software/X-Track/USER/App/Features/MotorDemo/Transport/MotorCanProtocol.h`
- Create: `Software/X-Track/USER/App/Features/MotorDemo/Transport/MotorCanProtocol.cpp`
- Create: `Software/X-Track/tests/motor_demo/test_motor_can_protocol.cpp`
- Modify: `Software/X-Track/tests/motor_demo/Makefile`

**Interfaces:**
- Consumes: fixed-width values only.
- Produces: `MotorCanFrame`, trajectory/broadcast encoders, feedback/health decoders, CRC8, and sequence helpers with no hardware include.

- [ ] **Step 1: Write the failing golden-vector test**

Use the same vectors as MotorDriver, including:

```cpp
MotorCanFrame point;
CHECK_TRUE(MotorCanProtocol::EncodeTrajectory(
    1U, 10000, 3000, 0x1234U, &point));
CHECK_EQ(point.id, 0x101U);
CHECK_EQ(point.dlc, 8U);
CHECK_EQ(point.data[0], 0x10U);
CHECK_EQ(point.data[1], 0x27U);
CHECK_EQ(point.data[4], 0x2cU);
CHECK_EQ(point.data[5], 0x01U);
CHECK_EQ(point.data[6], 0x34U);
CHECK_EQ(point.data[7], 0x12U);

const uint8_t arm_prefix[7] = {
    0x01U, 0x01U, 0x34U, 0x12U, 0x78U, 0x56U, 0x00U
};
CHECK_EQ(MotorCanProtocol::Crc8(arm_prefix, 7U), 0x20U);
```

Add exact negative-value, saturation rejection, all opcode, CRC/version/flags, feedback/health ID, state 0..5, wrong DLC/ID, sequence duplicate/backward/wrap, and radian conversion boundary tests.

- [ ] **Step 2: Add a Make target and verify RED**

Add `test_motor_can_protocol` to `.PHONY` and the aggregate `test` target, compiling only the codec plus its test with existing `CXXFLAGS`.

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/5_CanSerialTool/X-TRACK/.worktrees/dual-motor-can-transport/Software/X-Track/tests/motor_demo; make test_motor_can_protocol"
```

Expected: missing file/type compile failure.

- [ ] **Step 3: Define exact codec types and constants**

```cpp
struct MotorCanFrame {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
};

static const uint8_t kMotorCanProtocolVersion = 1U;
static const uint16_t kMotorCanBroadcastId = 0x080U;
static const uint16_t kMotorCanTrajectoryBase = 0x100U;
static const uint16_t kMotorCanFeedbackBase = 0x180U;
static const uint16_t kMotorCanHealthBase = 0x280U;
```

Use existing scoped-enum compatibility macros for `MotorCanOpcode` and `MotorCanNodeState`, with values exactly `1..5` and `0..5`. Expose `EncodeTrajectory`, `EncodeBroadcast`, `DecodeFeedback`, `DecodeHealth`, `Crc8`, `SequenceNewer16`, `RadiansToMdeg`, and `RadiansPerSecondTo10MdegPerSecond`.

Place the functions inside C++11-compatible nested blocks `namespace MotorDemo { namespace MotorCanProtocol { ... } }` so every call in this plan uses the same `MotorCanProtocol::Function` spelling.

- [ ] **Step 4: Implement explicit byte operations**

Do not cast payloads to structs. Reject non-finite radians, positions outside `int32_t`, velocity wire values outside `int16_t`, Node IDs outside 1..2, nonzero reserved flags, and malformed frames. Velocity rounds to nearest 10 mdeg/s; position rounds to nearest mdeg.

- [ ] **Step 5: Verify GREEN and commit in the X-Track worktree**

Expected: `motor CAN protocol: PASS`.

```powershell
git add -- Software/X-Track/USER/App/Features/MotorDemo/Transport/MotorCanProtocol.h Software/X-Track/USER/App/Features/MotorDemo/Transport/MotorCanProtocol.cpp Software/X-Track/tests/motor_demo/test_motor_can_protocol.cpp Software/X-Track/tests/motor_demo/Makefile
git commit -m "feat: add motor CAN protocol codec"
```

### Task 2: Add an Injectable CAN Port and Hardware Diagnostics

**Files:**
- Create: `Software/X-Track/USER/App/Features/MotorDemo/Transport/IMotorCanPort.h`
- Create: `Software/X-Track/USER/App/Features/MotorDemo/Transport/HardwareMotorCanPort.h`
- Create: `Software/X-Track/USER/App/Features/MotorDemo/Transport/HardwareMotorCanPort.cpp`
- Modify: `Software/X-Track/MDK-ARM_F435/Platform/Core/HardwareCAN.h`
- Modify: `Software/X-Track/MDK-ARM_F435/Platform/Core/HardwareCAN.cpp`
- Create: `Software/X-Track/tests/motor_demo/test_can_transport_static.py`

**Interfaces:**
- Consumes: `MotorCanFrame` and existing global `Can1`.
- Produces: a mockable `IMotorCanPort` and a production PB9/PB8 adapter with observable bus health.

- [ ] **Step 1: Write the failing static port test**

Require:

```cpp
struct MotorCanPortDiag {
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t tx_failures;
    uint32_t rx_overflows;
    uint32_t bus_off_events;
    uint8_t transmit_error_counter;
    uint8_t receive_error_counter;
};

class IMotorCanPort {
public:
    virtual ~IMotorCanPort() {}
    virtual bool Begin() = 0;
    virtual void End() = 0;
    virtual bool Write(const MotorCanFrame& frame) = 0;
    virtual bool Read(MotorCanFrame* frame) = 0;
    virtual void Poll() = 0;
    virtual MotorCanPortDiag Diagnostics() const = 0;
};
```

Static assertions require `Can1.begin(1000000U, PB9, PB8)`, four exact standard filters for `0x181`, `0x182`, `0x281`, `0x282`, and `Can1.setMode(CAN_MODE_COMMUNICATE)`.

- [ ] **Step 2: Run and verify RED**

```powershell
python Software/X-Track/tests/motor_demo/test_can_transport_static.py
```

Expected: missing interface/adapter/diagnostic assertions.

- [ ] **Step 3: Extend `HardwareCAN` diagnostics without changing clients**

Add `rx_overflows` and `bus_off_events` to `BusStatistics_t`, increment the former only when RX ISR discards due to a full software queue and the latter only on CAN_BOF. Keep existing `error_frames`, `getStatistics`, write/read signatures, and existing HAL polling behavior unchanged.

- [ ] **Step 4: Implement the production adapter**

`Begin` performs begin, four filters, communicate mode, and statistics start; if any step fails it calls `end` and returns false. `Write` maps exactly one standard data frame and uses the existing non-blocking `HardwareCAN::write`. `Read` rejects null and maps only standard data frames with DLC 8. `Poll` calls `processTxQueue`. Diagnostics copy statistics plus REC/TEC.

- [ ] **Step 5: Verify and commit**

```powershell
python Software/X-Track/tests/motor_demo/test_can_transport_static.py
git add -- Software/X-Track/USER/App/Features/MotorDemo/Transport/IMotorCanPort.h Software/X-Track/USER/App/Features/MotorDemo/Transport/HardwareMotorCanPort.h Software/X-Track/USER/App/Features/MotorDemo/Transport/HardwareMotorCanPort.cpp Software/X-Track/MDK-ARM_F435/Platform/Core/HardwareCAN.h Software/X-Track/MDK-ARM_F435/Platform/Core/HardwareCAN.cpp Software/X-Track/tests/motor_demo/test_can_transport_static.py
git commit -m "feat: add motor CAN hardware port"
```

### Task 3: Implement Discovery, ARM, Synchronized Submit, and Feedback Merge

**Files:**
- Create: `Software/X-Track/USER/App/Features/MotorDemo/Transport/CanMotorTransport.h`
- Create: `Software/X-Track/USER/App/Features/MotorDemo/Transport/CanMotorTransport.cpp`
- Create: `Software/X-Track/tests/motor_demo/test_can_motor_transport.cpp`
- Modify: `Software/X-Track/USER/App/Features/MotorDemo/Transport/IMotorTransport.h`
- Modify: `Software/X-Track/USER/App/Features/MotorDemo/Transport/SimMotorTransport.h`
- Modify: `Software/X-Track/USER/App/Features/MotorDemo/Transport/SimMotorTransport.cpp`
- Modify: `Software/X-Track/USER/App/Features/MotorDemo/MotorDemoController.cpp`
- Modify: `Software/X-Track/tests/motor_demo/test_controller.cpp`
- Modify: `Software/X-Track/tests/motor_demo/test_transport.cpp`
- Modify: `Software/X-Track/tests/motor_demo/Makefile`

**Interfaces:**
- Consumes: `IMotorCanPort`, pure codec, and existing `TrajectoryPoint`/`MotorFeedback`.
- Produces: a real `IMotorTransport` implementation and a deterministic fake-port test surface.

- [ ] **Step 1: Write the fake CAN port and failing lifecycle tests**

The fake stores written frames and queues received frames. Test all of:

- `Initialize(100)` calls Begin, sends DISCOVER, returns true, but remains disconnected until required READY health frames arrive.
- `EnterSafeMotion()` only succeeds after all required nodes are READY; it sends ARM with a nonzero session and expected first sequence 0.
- `Submit(point)` sends IDs `0x101`, `0x102`, `0x080` in that order with matching low-16 sequence.
- one node feedback does not publish combined feedback; matching Node 1/2 applied sequence does.
- mismatch for less than 30 ms waits; mismatch at 30 ms sends three STOP frames and latches an error.
- missing feedback, wrong session/version/state, nonzero fault, write failure, bus-off event, or RX overflow produces the same safe-stop behavior.
- `RequestStop()` is idempotent and session-independent.
- `ClearFault()` after a recoverable node timeout/fault sends STOP, CLEAR_FAULT, and DISCOVER, clears only transport-side discovery latches, and does not ARM or submit motion.
- X-Track port bus-off or MotorDriver `FAULT_CAN_BUS` is non-clearable in-session: `ClearFault()` returns false, keeps Submit blocked, and reports that the bus must be restored and the affected MotorDriver rebooted before rediscovery.
- pause/resume ARM uses `last_submitted+1` as its expected first sequence.
- `65535 -> 0` wire wrap works while 32-bit trajectory sequence remains monotonic within the 2048-point capacity.

- [ ] **Step 2: Add the Make target and verify RED**

Compile `MotorCanProtocol.cpp`, `CanMotorTransport.cpp`, and the new test. Expected: missing transport compile failure.

- [ ] **Step 3: Define exact config and per-node state**

```cpp
struct CanMotorTransportConfig {
    uint8_t required_node_mask;
    uint32_t feedback_timeout_ms;
    uint32_t sequence_mismatch_timeout_ms;
    static CanMotorTransportConfig DualNode();
    static CanMotorTransportConfig SingleNode(uint8_t node_id);
};

class CanMotorTransport : public IMotorTransport {
public:
    CanMotorTransport(IMotorCanPort* port,
                      const CanMotorTransportConfig& config);
    /* all existing IMotorTransport overrides */
};
```

Each node stores latest health, feedback, RX timestamps, seen flags, and fault/session/state. Add specific `TransportError` values `ProtocolMismatch`, `NodeFault`, `SequenceMismatch`, and `CanBusFault`; include them in the controller's serious-error classification.

Extend `IMotorTransport` with:

```cpp
virtual bool ClearFault() = 0;
```

`SimMotorTransport::ClearFault()` succeeds only when its active simulated condition no longer reports an error. Remove the current `IsConnected()` precondition from `MotorDemoController::ResetFault()`, because a faulted CAN transport is intentionally disconnected. ResetFault calls `ClearFault()` first; recoverable hardware faults return to `HwDiscovery` and wait for READY health, while a CAN bus-off keeps the controller faulted until the port and affected node have been restarted. Simulation may return to Disabled. No path enters safe motion or starts a trajectory.

- [ ] **Step 4: Implement lifecycle and stop priority**

Initialize resets all state, starts the port, and broadcasts DISCOVER. Update polls TX, drains all available frames with a fixed maximum of 16 per call, validates health/feedback, checks port diagnostics, and applies 30 ms deadlines with wrap-safe millisecond arithmetic. EnterSafeMotion increments a session generator, clears old feedback, and broadcasts ARM. Submit validates finite/range/sequence, emits active node preloads then SYNC, and only increments submitted stats after all writes succeed. For recoverable faults, ClearFault emits STOP three times, one CLEAR_FAULT, and one DISCOVER; it clears local error/seen state only after every write is accepted, then remains disconnected until required READY health is received. For port bus-off or node `FAULT_CAN_BUS`, it sends best-effort STOP but retains `CanBusFault`, refuses CLEAR_FAULT/re-ARM, and exposes a reboot-required status.

In dual mode, ReadLatest succeeds only after both nodes confirm the same sequence. In explicit single-node mode, the inactive joint feedback mirrors the last submitted command; the compile-time configuration and service status visibly mark commissioning mode. This is allowed only for a free-shaft commissioning build and never in final production.

- [ ] **Step 5: Verify GREEN and commit**

Run `make test_can_motor_transport test_motor_can_protocol`. Expected: both PASS.

```powershell
git add -- Software/X-Track/USER/App/Features/MotorDemo/Transport/CanMotorTransport.h Software/X-Track/USER/App/Features/MotorDemo/Transport/CanMotorTransport.cpp Software/X-Track/USER/App/Features/MotorDemo/Transport/IMotorTransport.h Software/X-Track/USER/App/Features/MotorDemo/Transport/SimMotorTransport.h Software/X-Track/USER/App/Features/MotorDemo/Transport/SimMotorTransport.cpp Software/X-Track/USER/App/Features/MotorDemo/MotorDemoController.cpp Software/X-Track/tests/motor_demo/test_can_motor_transport.cpp Software/X-Track/tests/motor_demo/test_controller.cpp Software/X-Track/tests/motor_demo/test_transport.cpp Software/X-Track/tests/motor_demo/Makefile
git commit -m "feat: add synchronized motor CAN transport"
```

### Task 4: Replace the Blocked Stub in Production Safely

**Files:**
- Modify: `Software/X-Track/USER/App/Common/DataProc/DP_MotorDemoService.h`
- Modify: `Software/X-Track/USER/App/Common/DataProc/DP_MotorDemoService.cpp`
- Modify: `Software/X-Track/USER/App/Features/MotorDemo/MotorDemoBuild.inc`
- Modify: `Software/X-Track/USER/App/Config/Config.h`
- Modify: `Software/X-Track/tests/motor_demo/test_transport.cpp`
- Modify: `Software/X-Track/tests/motor_demo/test_controller.cpp`
- Modify: `Software/X-Track/tests/motor_demo/test_service.cpp`
- Modify: `Software/X-Track/tests/motor_demo/test_integration_static.py`
- Delete: `Software/X-Track/USER/App/Features/MotorDemo/Transport/CanMotorTransportStub.h`
- Delete: `Software/X-Track/USER/App/Features/MotorDemo/Transport/CanMotorTransportStub.cpp`

**Interfaces:**
- Consumes: Task 2 hardware port and Task 3 transport.
- Produces: production-owned `HardwareMotorCanPort` and `CanMotorTransport` without changing controller public APIs.

- [ ] **Step 1: Update integration tests for RED production expectations**

Require `DP_MotorDemoService` members in construction order:

```cpp
MotorDemo::HardwareMotorCanPort can_port_;
MotorDemo::CanMotorTransport hardware_;
```

Require real source includes in `MotorDemoBuild.inc`, no stub include/reference, and this final-safe configuration:

```c
#define MOTOR_DEMO_CAN_COMMISSION_NODE 0u
```

The static test accepts only `0`, `1`, or `2`; value 0 must select `CanMotorTransportConfig::DualNode()`. Values 1/2 select `SingleNode(value)` and must set a visible status message containing `CAN COMMISSION NODE`.

- [ ] **Step 2: Run and verify RED**

Run integration static test and aggregate host Make test. Expected: stub assertions/current constructors fail.

- [ ] **Step 3: Wire production construction without disturbing other session work**

Use a file-local `ProductionCanConfig()` in `DP_MotorDemoService.cpp` selected by the macro. Initialize `hardware_(&can_port_, ProductionCanConfig())` before passing it to `controller_`. Preserve all teaching, storage, UI, and simulation members and their order-sensitive initialization.

- [ ] **Step 4: Update controller/service tests through injected fake transports**

Do not make host tests include `HardwareCAN`. Keep controller unit tests on their existing fake `IMotorTransport`. Guard the production port member and constructor wiring as follows:

```cpp
#ifndef MOTOR_DEMO_HOST_TEST
MotorDemo::HardwareMotorCanPort can_port_;
#endif
MotorDemo::CanMotorTransport hardware_;

#ifdef MOTOR_DEMO_HOST_TEST
explicit MotorDemoService(
    const MotorDemo::StoreFileOps* store_ops = NULL,
    MotorDemo::IMotorCanPort* motor_can_port = NULL);
#else
explicit MotorDemoService(
    const MotorDemo::StoreFileOps* store_ops = NULL);
#endif
```

Production constructs `hardware_(&can_port_, ProductionCanConfig())`; host constructs `hardware_(motor_can_port, ProductionCanConfig())`, so existing host tests passing no port remain safely disconnected while focused service tests inject a fake. Replace only expectations that hardware is permanently blocked with discovery/ready/failure expectations.

- [ ] **Step 5: Delete the stub and verify GREEN**

Run:

```powershell
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/5_CanSerialTool/X-TRACK/.worktrees/dual-motor-can-transport/Software/X-Track/tests/motor_demo; make test"
python Software/X-Track/tests/motor_demo/test_integration_static.py
```

Expected: all existing and new tests pass.

- [ ] **Step 6: Commit**

```powershell
git add -- Software/X-Track/USER/App/Common/DataProc/DP_MotorDemoService.h Software/X-Track/USER/App/Common/DataProc/DP_MotorDemoService.cpp Software/X-Track/USER/App/Features/MotorDemo/MotorDemoBuild.inc Software/X-Track/USER/App/Config/Config.h Software/X-Track/USER/App/Features/MotorDemo/Transport/CanMotorTransport.h Software/X-Track/USER/App/Features/MotorDemo/Transport/CanMotorTransport.cpp Software/X-Track/USER/App/Features/MotorDemo/Transport/CanMotorTransportStub.h Software/X-Track/USER/App/Features/MotorDemo/Transport/CanMotorTransportStub.cpp Software/X-Track/tests/motor_demo/test_transport.cpp Software/X-Track/tests/motor_demo/test_controller.cpp Software/X-Track/tests/motor_demo/test_service.cpp Software/X-Track/tests/motor_demo/test_integration_static.py
git commit -m "feat: enable X-Track motor CAN hardware"
```

Before committing, inspect `git diff --cached` and unstage any unrelated files inherited from the other X-Track session.

### Task 5: Full X-Track Regression and ARMCC5 Build

**Files:**
- Modify production files only through a new failing regression test if this task exposes a defect.

**Interfaces:**
- Consumes: Tasks 1-4.
- Produces: a production dual-node image and a separate explicitly marked Node 1 commissioning image.

- [ ] **Step 1: Run full host and static tests**

Run `make clean test`, `test_integration_static.py`, `test_keil_map.py` when a fresh map exists, and `git diff --check`. Expected: all pass with `-Werror`.

- [ ] **Step 2: Build the final dual-node image**

With `MOTOR_DEMO_CAN_COMMISSION_NODE 0u`, run:

```powershell
& 'C:\Keil_v5\UV4\UV4.exe' -b 'Software\X-Track\MDK-ARM_F435\proj.uvprojx' -t 'X-Track' -j0 -o 'Software\X-Track\MDK-ARM_F435\can_build.log'
```

Expected: output image produced and build log reports 0 errors; investigate every new warning before hardware use.

- [ ] **Step 3: Build the temporary Node 1 commissioning image**

Change only `MOTOR_DEMO_CAN_COMMISSION_NODE` from `0u` to `1u`, build, record the binary hash as commissioning-only, then restore the source to `0u` and rerun the static test. Do not commit the value `1u` and do not attach the mechanism while this image is installed.

- [ ] **Step 4: Rebuild final production and verify the guard**

Rebuild after restoring `0u`; require the UI/status lacks commissioning mode and the static test proves DualNode config. Record both hashes so the commissioning image cannot be confused with production.

- [ ] **Step 5: Commit a test-driven build correction or no commit**

If a correction was needed, add the failing test first, apply the minimum fix, rerun Steps 1-4, and commit exact paths. Otherwise create no empty commit.

### Task 6: Single-Motor Physical CAN Qualification

**Files:**
- Record evidence in the MotorDriver Stage 8 documents; do not commit a commissioning macro change.

**Interfaces:**
- Consumes: MotorDriver Node 1 firmware/plan and the X-Track Node 1 commissioning image.
- Produces: physical 1 Mbps/100 Hz single-node evidence while the motor is mechanically free.

- [ ] **Step 1: Verify wiring and safe discovery**

Use PB9/PB8 CAN1, two 120 Ω ends, common ground, and short cable. Keep the motor free. Power/boot X-Track first and verify it has entered 1 Mbps communicate mode; only then reboot MotorDriver Node 1 so its first health transmission receives an ACK. Require the visible commissioning banner, Node 1 READY health, correct version/session/VBUS, and no Node 2 requirement. If Node 1 reports `FAULT_CAN_BUS`, correct wiring/termination and reboot it; do not issue repeated CLEAR_FAULT attempts.

- [ ] **Step 2: Verify frame order and 100 Hz rate**

Run a short bounded trajectory. Require each 10 ms cycle to contain Node 1 preload then SYNC, feedback at 100 Hz, health at 20 Hz, matching applied sequence, and measured bus load below 10%. The inactive joint is shadow feedback only and must not be interpreted as dual-node qualification.

- [ ] **Step 3: Run MotorDriver Stage 8 single-node gates**

Execute static ±5°, reversal ±10°, 10°/30°/s sine, STOP, 50 ms HOLD, 500 ms fatal timeout, recoverable timeout clear/re-ARM, and 10-minute continuity. Separately force physical bus loss, require `FAULT_CAN_BUS`, restore the bus, reboot Node 1, rediscover, and prove that motion cannot resume without a fresh ARM. Require the MotorDriver metrics and final safety in its plan.

- [ ] **Step 4: Restore the final X-Track image immediately**

Flash the production image built with commissioning macro 0. Confirm X-Track now remains in hardware discovery when Node 2 is absent and cannot arm a one-node mechanism.

### Task 7: Admit Node 2 and Verify Dual-Node Synchronization

**Files:**
- Modify X-Track or MotorDriver production code only after reproducing a failure with a focused test.

**Interfaces:**
- Consumes: independently qualified Node 1, independently qualified Node 2, and final dual-node X-Track image.
- Produces: dual free-shaft synchronization and linked-stop evidence.

- [ ] **Step 1: Qualify Node 2 independently**

Configure the second MotorDriver through its COM port as Node 2 with its own zero/direction/range. Validate encoder direction, ± small static positions, CAN discovery, persistence, STOP, HOLD, and timeout before connecting both boards.

- [ ] **Step 2: Connect both nodes and discover safely**

Use one terminated main bus, not a long star. Require one valid READY response at IDs `0x281` and `0x282`, matching version, no arbitration/error counters, and commissioning macro 0. Do not claim duplicate-ID auto-detection; the two boards must already have been individually identified.

- [ ] **Step 3: Run dual free-shaft trajectories**

Test same direction, opposite direction, unequal positions, and zero-velocity holds. Require both feedback sequences remain equal and every controller progress update corresponds to a sequence confirmed by both nodes.

- [ ] **Step 4: Measure application skew**

Toggle one debug GPIO on each MotorDriver when a new applied sequence is published. Capture both with a scope/logic analyzer and require observed skew approximately no greater than 1 ms over representative points.

- [ ] **Step 5: Prove linked stop**

Disconnect or fault each node in turn. Require X-Track broadcasts STOP three times within the 30 ms transport deadline and the other motor ceases trajectory; independently require the surviving node's own 50/500 ms watchdog behavior.

- [ ] **Step 6: Document and commit final transport evidence**

Record X-Track/MotorDriver commits, firmware hashes, node configs, cable/termination, bus load, sequence statistics, skew capture, fault injection results, and final disabled state. Keep mechanism/MPS drawing qualification as the next explicit stage.

### Task 8: Final X-Track Verification Gate

**Files:**
- No behavior changes.

**Interfaces:**
- Consumes: fresh host, build, and hardware evidence.
- Produces: exact production commit ready for five-bar low-speed installation.

- [ ] **Step 1: Repeat fresh tests and final build**

Run aggregate Make tests, Python static tests, final ARMCC5 build with commissioning macro 0, `git diff --check`, and clean status checks.

- [ ] **Step 2: Audit every transport gate**

Map evidence to exact bytes, 1 Mbps initialization, filters, discovery, ARM/session, three-frame submit order, same-sequence merge, 30 ms linked stop, bus faults, single-node guard, dual-node skew, and controller/service regressions.

- [ ] **Step 3: Hand off to mechanism integration**

Report the exact X-Track commit and both MotorDriver node commits/configurations. State the allowed initial mechanism procedure: low speed, small angle, existing 0.5 A cap, verified zero/direction/workspace, then MPS path. Torque-ripple compensation remains a measured follow-up only if linked trajectory error shows a repeatable periodic component.
