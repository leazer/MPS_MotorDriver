# CANalyst-II Stage 8 Peer Design

## Purpose

Provide a reproducible Windows CAN peer for the MotorDriver Stage 8 single-node qualification. The peer uses the user's CANalyst-II CH1 to issue byte-compatible Motor CAN protocol frames, collect feedback and health frames, and guarantee a safe stop on every success or failure path.

This tool qualifies MotorDriver Node 1 independently. It does not replace the later physical verification of X-Track's own 100 Hz preload/SYNC transport.

## Confirmed Hardware and Vendor Interface

- CANalyst-II is enumerated as `WinUSB Device`, USB `VID_04D8` / `PID_0053`.
- The vendor package supplies an x64 `ControlCAN.dll` compatible with the installed 64-bit Python runtime.
- `ControlCAN.h` defines `VCI_USBCAN2 = 4`.
- Device index is `0`; CH1 is CAN index `0`.
- Classic CAN is configured for `1,000,000 bit/s` with `Timing0 = 0x00` and `Timing1 = 0x14`.
- Frames are 11-bit standard data frames with receive-all filtering.
- The live bus contains only CANalyst-II CH1 and MotorDriver Node 1, with one 120-ohm termination at each end. X-Track CANH/CANL remains physically disconnected during this qualification.

## Architecture

The implementation has three small layers:

1. `ControlCanDevice` is a narrow `ctypes` wrapper around the vendor DLL. It owns open/init/start/reset/close, translates between Python frames and `VCI_CAN_OBJ`, validates every vendor return value, and never contains Motor protocol rules.
2. `CanalystMotorPeer` implements the peer operations expected by `tests/stage8_can_bench.py`: discover/health reads, ARM, trajectory preload, SYNC, STOP, and feedback reads. It reuses the frozen Motor CAN v1 byte layout and CRC-8 rules.
3. `stage8_canalyst_node1.py` orchestrates the powered qualification and COM9 evidence. It begins with three broadcast STOP frames, admits only Node 1 READY health, runs increasingly bounded motion, and delegates final cleanup to `run_can_bench()`.

The vendor DLL path is an explicit command-line argument with a default pointing to the confirmed x64 package. The adapter never copies or commits the proprietary DLL.

## Protocol and State Flow

The peer follows the existing protocol sources exactly:

- Broadcast command ID: `0x080`.
- Node 1 trajectory ID: `0x101`.
- Node 1 feedback ID: `0x181`.
- Node 1 health ID: `0x281`.
- Opcodes: ARM `0x01`, SYNC `0x02`, STOP `0x03`, DISCOVER `0x05`.
- Every command uses the existing little-endian layout and CRC-8 polynomial `0x07`.

The powered sequence is:

1. Assert COM9 reports DISABLED, EN low, neutral PWM, valid calibration, valid Node 1 configuration, and a position with at least 10 degrees of workspace margin.
2. Open CANalyst-II, initialize CH1 at 1 Mbps, clear receive buffers, and start the channel.
3. Send broadcast STOP three times before any discovery or ARM.
4. Send DISCOVER and require one valid Node 1 READY health frame with protocol version 1, session 0, plausible VBUS, and no bus-off/overflow evidence.
5. Send STOP with an arbitrary session and require Node 1 remains READY.
6. ARM a new nonzero session.
7. Preload and SYNC a zero-feedforward point at the actual position, then a relative `+1 degree` point. Require applied sequence confirmation, correct sign, current at or below 0.5 A, no CAN/encoder counter growth, and no fault.
8. STOP and return to DISABLED before advancing.
9. Repeat with relative `+5 degrees`, then the Stage 8 reversal, sine, timeout, and continuity gates only after each preceding gate passes.
10. In `finally`, send broadcast STOP three times, send COM9 `mc_stop`, verify EN low and neutral PWM, reset/close the CAN channel, and record final diagnostics.

The first hardware run is deliberately limited to STOP/DISCOVER. Motion is a separate explicit invocation so a successful adapter smoke test cannot accidentally enable the motor.

## Safety Rules

- No frame is transmitted until the exact DLL, device type, channel, bitrate, topology, configuration, current cap, encoder stability, and workspace gates are verified.
- Motion commands are rejected unless the target is relative to fresh feedback and remains inside configured joint limits with at least the required margin.
- The peer never sends a trajectory frame before ARM and never sends SYNC before its matching preload succeeds.
- Sequence/session mismatches, malformed frames, stale feedback, wrong direction, current saturation, encoder rejected-sample growth, protocol errors, receive overflow, bus-off, or a COM9 fault cause immediate cleanup.
- The existing firmware hard limit `SPEED_IQ_LIMIT_A = 0.5f` remains unchanged and is checked through static tests plus live `iq_ref` observations.
- CANalyst-II active-master evidence cannot be reported as proof of X-Track dynamic transport. That evidence remains mandatory in the later X-Track hardware task.

## Testing

All adapter behavior is developed test-first with a fake ControlCAN library and fake serial port.

The tests cover:

- Exact `ctypes` structure sizes/field order and x64 DLL function signatures.
- Open/init/start failure propagation and idempotent reset/close.
- Device type `4`, device index `0`, channel `0`, and timing `0x00/0x14`.
- Standard data-frame conversion, DLC validation, receive timeout behavior, and batch bounds.
- Golden byte vectors and CRC rejection for ARM, preload, SYNC, STOP, DISCOVER, feedback, and health.
- Required ordering: ARM, preload, matching SYNC; no SYNC after failed preload.
- Three STOP attempts plus COM9/PWM cleanup after normal completion and after every injected exception.
- Dry-run and discovery-only modes that cannot issue ARM or trajectory frames.
- Relative `+1 degree` and `+5 degree` workspace checks, direction checks, 0.5 A observation gate, and encoder/CAN counter invariants.

Only after the fake-backed suite and existing Stage 8 tests pass may the discovery-only hardware command open CANalyst-II. Each later powered phase requires fresh evidence from the prior phase.

## Evidence and Documentation

The launcher records timestamped JSON and a human-readable Markdown summary containing:

- MotorDriver commit and firmware build timestamp.
- DLL path/hash, device board information, channel and bitrate configuration.
- Bus topology and termination statement.
- Raw transmitted/received frames, health/feedback sequence statistics, COM9 snapshots, current/position metrics, induced timeout results, and final safe state.
- The MAX3051 `RS`-floating hardware root cause and the corrective RS-to-ground fly-wire, including the fact that existing schematic documentation described a different transceiver.

Generated bench evidence is kept out of source control unless explicitly promoted into the Stage 8 engineering records.
