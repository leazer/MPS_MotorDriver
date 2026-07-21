# CANalyst-II Stage 8 Peer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and use a tested Python CANalyst-II peer that safely qualifies MotorDriver Node 1 with STOP/DISCOVER, relative +1 degree, and relative +5 degree CAN motion gates.

**Architecture:** A narrow `ctypes` device wrapper owns the vendor x64 `ControlCAN.dll`; a protocol peer owns Motor CAN v1 bytes and ordering; a separate launcher owns COM9 preflight, powered phase selection, evidence, and guaranteed cleanup through the existing Stage 8 runner. Production firmware and X-Track behavior do not change.

**Tech Stack:** Python 3.14 x64, `ctypes`, vendor ControlCAN x64 API, pytest-compatible plain Python tests, existing `stage7_bench.py`/`stage8_can_bench.py`, COM9 pyserial.

## Global Constraints

- The active bus contains only CANalyst-II CH1 and MotorDriver Node 1; X-Track CANH/CANL remains physically disconnected.
- Both bus ends have exactly one 120-ohm termination.
- Use `VCI_USBCAN2 = 4`, device index `0`, CAN index `0`, `Timing0 = 0x00`, and `Timing1 = 0x14` for 1 Mbps.
- Use only 11-bit standard data frames and receive-all filtering.
- Never copy or commit the proprietary vendor DLL.
- Every powered path attempts broadcast STOP three times, COM9 `mc_stop`, and independent EN/PWM safety verification.
- The first hardware invocation is `--phase discover`; it cannot ARM or transmit trajectory frames.
- Motion targets are relative to fresh Node 1 feedback, remain within persisted joint limits, and advance only from +1 degree to +5 degrees.
- Abort on stale/malformed feedback, wrong direction, `abs(iqref) > 500 mA`, fault bits, encoder rejected-sample growth, protocol errors, RX overflow, bus-off, or TX error growth.
- CANalyst evidence qualifies MotorDriver Node 1 only; it is not X-Track dynamic transport evidence.
- Preserve the existing unrelated raw-line-ending modification of `project/MDK_V5/MPS_MotorDriver.uvprojx`.

---

### Task 1: Add the ControlCAN x64 Device Boundary

**Files:**
- Create: `tests/canalyst_controlcan.py`
- Create: `tests/motor_control/test_canalyst_controlcan.py`

**Interfaces:**
- Consumes: vendor `VCI_OpenDevice`, `VCI_InitCAN`, `VCI_StartCAN`, `VCI_ClearBuffer`, `VCI_Transmit`, `VCI_Receive`, `VCI_ResetCAN`, `VCI_CloseDevice`, and `VCI_ReadBoardInfo`.
- Produces: `CanFrame(can_id: int, data: bytes, timestamp: int = 0)` and `ControlCanDevice(dll_path, device_type=4, device_index=0, can_index=0)` with `open()`, `send(frame)`, `receive(max_frames=64, wait_ms=0)`, `reset()`, and `close()`.

- [ ] **Step 1: Write the failing fake-DLL tests**

Create a fake callable that records `argtypes`, `restype`, arguments, and configurable return values. Freeze these expectations:

```python
def test_confirmed_channel_configuration_and_structure_layout():
    assert ctypes.sizeof(MODULE.VCI_CAN_OBJ) == 24
    assert ctypes.sizeof(MODULE.VCI_INIT_CONFIG) == 16
    device = MODULE.ControlCanDevice("unused", library=FakeLibrary())
    device.open()
    assert device.library.init_config == {
        "AccCode": 0, "AccMask": 0xFFFFFFFF, "Filter": 1,
        "Timing0": 0x00, "Timing1": 0x14, "Mode": 0,
    }
    assert device.library.calls[:4] == [
        ("open", 4, 0, 0), ("init", 4, 0, 0),
        ("clear", 4, 0, 0), ("start", 4, 0, 0),
    ]

def test_context_cleanup_runs_after_transmit_failure():
    fake = FakeLibrary(transmit_result=0)
    try:
        with MODULE.ControlCanDevice("unused", library=fake) as device:
            device.send(MODULE.CanFrame(0x080, b"12345678"))
    except MODULE.ControlCanError:
        pass
    assert fake.calls[-2:] == [("reset", 4, 0, 0), ("close", 4, 0)]
```

Also test invalid ID/DLC rejection, standard data-frame fields, receive bounds, vendor short transmit, open/init/start failure cleanup, idempotent close, and a library object whose functions reject incorrectly declared signatures.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```powershell
python tests/motor_control/test_canalyst_controlcan.py
```

Expected: FAIL because `tests/canalyst_controlcan.py` does not exist.

- [ ] **Step 3: Implement the minimal wrapper**

Use Windows-width types, not platform-dependent `c_ulong`:

```python
DWORD = ctypes.c_uint32
UINT = ctypes.c_uint32
ULONG = ctypes.c_uint32
BYTE = ctypes.c_uint8

class VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", UINT), ("TimeStamp", UINT), ("TimeFlag", BYTE),
        ("SendType", BYTE), ("RemoteFlag", BYTE), ("ExternFlag", BYTE),
        ("DataLen", BYTE), ("Data", BYTE * 8), ("Reserved", BYTE * 3),
    ]

class VCI_INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", DWORD), ("AccMask", DWORD), ("Reserved", DWORD),
        ("Filter", BYTE), ("Timing0", BYTE), ("Timing1", BYTE),
        ("Mode", BYTE),
    ]
```

Load with `ctypes.WinDLL(str(path))`, declare every exact `argtypes`/`restype`, require vendor return `1` for open/init/clear/start/reset/close and exact frame count for transmit, and translate only standard data frames. `close()` always attempts reset before close and aggregates both errors without skipping either operation.

- [ ] **Step 4: Verify the wrapper and existing tests**

Run:

```powershell
python tests/motor_control/test_canalyst_controlcan.py
python tests/motor_control/test_stage8_can_bench.py
git diff --check
```

Expected: both tests print PASS and `git diff --check` exits 0.

- [ ] **Step 5: Commit Task 1**

```powershell
git add tests/canalyst_controlcan.py tests/motor_control/test_canalyst_controlcan.py
git commit -m "test: add CANalyst ControlCAN boundary"
```

### Task 2: Implement the Motor CAN v1 Peer

**Files:**
- Create: `tests/canalyst_motor_peer.py`
- Create: `tests/motor_control/test_canalyst_motor_peer.py`

**Interfaces:**
- Consumes: `CanFrame` and a device with `send(frame)`/`receive(max_frames, wait_ms)`.
- Produces: `Health`, `Feedback`, and `CanalystMotorPeer` with `discover()`, `arm(session, sequence=0)`, `submit(position_mdeg, velocity_mdeg_s, sequence)`, `sync(session, sequence)`, `stop(session=0, sequence=0)`, `read_health()`, and `read_feedback()`.

- [ ] **Step 1: Write failing protocol and ordering tests**

Freeze the existing C golden vectors and exact IDs:

```python
def test_broadcast_golden_vectors():
    assert MODULE.encode_broadcast(1, 0x1234, 0x5678).data == bytes.fromhex(
        "01 01 34 12 78 56 00 20"
    )
    assert MODULE.decode_health(CanFrame(0x281, bytes.fromhex(
        "01 05 34 12 78 56 BC 9A"
    ))) == MODULE.Health(1, 5, 0x1234, 0x5678, 0x9ABC)

def test_preload_must_succeed_before_matching_sync():
    device = FakeDevice(fail_send_ids={0x101})
    peer = MODULE.CanalystMotorPeer(device, node_id=1)
    try:
        peer.apply_point(0x2222, 7, 33596, 0)
    except RuntimeError:
        pass
    assert [frame.can_id for frame in device.sent] == [0x101]
```

Cover signed little-endian trajectory values, velocity `/10` wire scaling/range, feedback/health DLC and ID rejection, protocol version/state checks, stale session/sequence rejection, receive filtering, uint16 wrap behavior, and STOP sending exactly one frame per call so the Stage 8 runner controls the three attempts.

- [ ] **Step 2: Run and verify RED**

```powershell
python tests/motor_control/test_canalyst_motor_peer.py
```

Expected: FAIL because the peer module is absent.

- [ ] **Step 3: Implement exact protocol bytes**

Use `struct.pack("<i h H", position_mdeg, velocity_mdeg_s // 10, sequence)` for trajectory frames after validating exact integer/range semantics. Broadcast bytes are:

```python
body = struct.pack("<BBHHB", opcode, 1, sequence, session, 0)
return CanFrame(0x080, body + bytes((crc8(body),)))
```

`apply_point()` must call `submit()` and only then `sync()` with the same session and sequence. Decoders accept only IDs `0x181`/`0x281`, DLC 8, Node 1, protocol version 1, and known node states.

- [ ] **Step 4: Verify protocol parity**

```powershell
python tests/motor_control/test_canalyst_motor_peer.py
python tests/motor_control/test_can_motion_integration_static.py
wsl.exe -e bash -lc "set -e; cd /mnt/e/WorkSpaces/2_MotorDriver/MPS_MotorDriver/.worktrees/motor-driver-can-node; gcc -std=c11 -Wall -Wextra -Werror -Iapplication/motor_control -Icommunication tests/communication/test_can_protocol.c communication/can_protocol.c -o /tmp/test_can_protocol; /tmp/test_can_protocol"
git diff --check
```

Expected: Python peer PASS, static integration PASS, C protocol prints `can protocol: PASS`.

- [ ] **Step 5: Commit Task 2**

```powershell
git add tests/canalyst_motor_peer.py tests/motor_control/test_canalyst_motor_peer.py
git commit -m "test: add CANalyst Motor CAN peer"
```

### Task 3: Add the Gated Node 1 Launcher

**Files:**
- Create: `tests/stage8_canalyst_node1.py`
- Create: `tests/motor_control/test_stage8_canalyst_node1.py`

**Interfaces:**
- Consumes: `ControlCanDevice`, `CanalystMotorPeer`, `stage7_bench.read_position_status`, `stage8_can_bench.parse_can_status`, and `stage8_can_bench.run_can_bench`.
- Produces: CLI phases `probe`, `discover`, `step1`, and `step5`, timestamped JSON evidence, and deterministic cleanup.

- [ ] **Step 1: Write failing phase and cleanup tests**

Use fake device/peer/serial factories. Freeze these safety properties:

```python
def test_discover_phase_cannot_arm_or_send_trajectory():
    result = MODULE.run_phase("discover", peer=FakePeer(), serial=FakeSerial())
    assert result["node_state"] == "READY"
    assert all(event[0] not in ("arm", "submit", "sync") for event in result["events"])

def test_step_target_is_relative_and_workspace_checked():
    assert MODULE.relative_target(32596, 1000, -90000, 90000) == 33596
    for args in ((89500, 1000, -90000, 90000),
                 (0, 5000, -4000, 4000)):
        with pytest.raises(ValueError):
            MODULE.relative_target(*args)

def test_failure_still_runs_all_cleanup_layers():
    events = []
    peer = FakePeer(events, fail_after_arm=True)
    serial = FakeSerial(events)
    with pytest.raises(RuntimeError, match="injected after ARM"):
        MODULE.run_phase("step1", peer=peer, serial=serial)
    assert events[-5:] == ["stop", "stop", "stop", "mc_stop", "pwm_safe"]
```

Also test: `step5` is refused without a recorded passing `step1` evidence file for the same firmware/config; health must be READY/session 0/VBUS 6-30 V; COM9 preflight requires valid configuration and at least 10-degree margin; encoder/CAN counters may not grow; `abs(iqref) <= 500`; wrong movement sign aborts; evidence is written atomically after cleanup.

- [ ] **Step 2: Run and verify RED**

```powershell
python tests/motor_control/test_stage8_canalyst_node1.py
```

Expected: FAIL because the launcher is absent.

- [ ] **Step 3: Implement the launcher and strengthen bench cleanup ownership**

The parser exposes explicit phase choices:

```python
parser.add_argument("--phase", required=True,
                    choices=("probe", "discover", "step1", "step5"))
parser.add_argument("--dll", type=Path, required=True)
parser.add_argument("--port", default="COM9")
parser.add_argument("--evidence-dir", type=Path,
                    default=Path("artifacts/stage8-canalyst"))
```

`probe` loads the DLL and validates exported functions but never calls `VCI_OpenDevice`. `discover` opens CAN, sends three STOP frames before DISCOVER, validates Node 1 health, then exits through cleanup. `step1` and `step5` open fresh, derive the target from fresh feedback, ARM a fresh nonzero session, apply a zero-motion baseline point, apply the relative point, sample feedback plus COM9 `mc_pos_status`, STOP, and require DISABLED/neutral PWM.

Keep `run_can_bench()` unchanged. The launcher opens `ControlCanDevice` in an outer context manager, passes its peer into `run_can_bench()`, and therefore closes/reset the CAN channel only after the runner has completed its existing three STOP attempts plus COM9/PWM checks.

- [ ] **Step 4: Run the complete software gate**

```powershell
python tests/motor_control/test_canalyst_controlcan.py
python tests/motor_control/test_canalyst_motor_peer.py
python tests/motor_control/test_stage8_canalyst_node1.py
python tests/motor_control/test_stage8_can_bench.py
python tests/motor_control/test_can_motion_integration_static.py
python -m pytest tests -q
git diff --check
```

Expected: all tests PASS, no warning/error, clean diff check.

- [ ] **Step 5: Commit Task 3**

```powershell
git add tests/stage8_canalyst_node1.py tests/motor_control/test_stage8_canalyst_node1.py
git commit -m "test: add gated CANalyst Node 1 qualification"
```

### Task 4: Execute the Physical Gates and Record Evidence

**Files:**
- Modify: `doc/调试记录.md`
- Modify: `docs/superpowers/plans/2026-07-21-motor-driver-can-node.md`
- Record ignored evidence under: `artifacts/stage8-canalyst/`

**Interfaces:**
- Consumes: Tasks 1-3, CANalyst-II CH1, COM9, MotorDriver Node 1 firmware commit, and confirmed MAX3051 RS-to-ground hardware correction.
- Produces: reproducible discovery, +1 degree, and +5 degree evidence while ending DISABLED and safe.

- [ ] **Step 1: Run a no-open DLL probe**

```powershell
python tests/stage8_canalyst_node1.py --phase probe --dll "C:\Users\justb\Downloads\Compressed\CAN分析仪资料20250624\CAN分析仪资料20250618\二次开发库文件\x64(64bit)\ControlCAN.dll"
```

Expected: x64 DLL loads, all required exports resolve, and no CAN device is opened.

- [ ] **Step 2: Recheck physical and COM9 preflight**

Require X-Track CANH/CANL disconnected, two 120-ohm ends, Node 1 DISABLED/EN low/PWM neutral, encoder calibration/config valid, position inside limits with at least 10 degrees margin, fresh encoder reject counters stable, and no pre-existing fault.

- [ ] **Step 3: Run discovery-only hardware qualification**

```powershell
python tests/stage8_canalyst_node1.py --phase discover --port COM9 --dll "C:\Users\justb\Downloads\Compressed\CAN分析仪资料20250624\CAN分析仪资料20250618\二次开发库文件\x64(64bit)\ControlCAN.dll"
```

Expected: STOP/STOP/STOP then DISCOVER; one Node 1 READY health frame, protocol 1, session 0, plausible VBUS, no bus-off/overflow/error growth; final DISABLED/EN low/PWM neutral.

- [ ] **Step 4: Run relative +1 degree**

Run `--phase step1`. Require correct direction, applied sequence confirmation, peak `abs(iqref) <= 500 mA`, no fault/counter growth, and final safe state. Stop immediately on any violated gate.

- [ ] **Step 5: Run relative +5 degrees**

Only after Step 4 evidence passes, run `--phase step5` with the same requirements. Do not run reversal/sine/timeout/continuity in this task unless a later reviewed plan explicitly advances those gates.

- [ ] **Step 6: Document the physical result and commit only records**

Record commits, firmware build, DLL hash, USB VID/PID, topology, termination, MAX3051 RS root cause/fly-wire, raw frame evidence, COM9 metrics, encoder baseline/deltas, current peak, position error, and final safe state. Mark the original Stage 8 plan only for gates actually completed; leave X-Track dynamic transport and longer motion pending.

```powershell
git add doc/调试记录.md docs/superpowers/plans/2026-07-21-motor-driver-can-node.md
git commit -m "docs: record CANalyst Node 1 qualification"
```

Do not stage `project/MDK_V5/MPS_MotorDriver.uvprojx` or generated evidence.
