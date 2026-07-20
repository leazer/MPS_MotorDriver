# Low-Side Current Polarity Fix Design

## Status

Approved by the user on 2026-07-21. This is a focused correction to the already approved current-sampling reconstruction design.

## Hardware Evidence

- With `CCR4=5264`, all three low-side windows are valid and `valid_mask=0x07` remains stable.
- `Iq_ref=-50mA` converged to approximately `Iq=+569mA, Id=-647mA`.
- `Iq_ref=+50mA` converged to approximately `Iq=-699mA, Id=-460mA`.
- Neither run accumulated invalid frames or PI freezes, and neither raised a fault.
- With `theta=0`, `Valpha=+500mV`, `Vbeta=0`, the expected physical phase-current signs are `Ia>0, Ib<0, Ic<0`; measured reconstructed signs were `Ia<0, Ib>0, Ic>0`.

The sampling window and reconstruction selection are operating, but MP6540H low-side device-current polarity is the inverse of the FOC phase-current convention. Feeding it directly to Clarke/Park turns the current PI into positive feedback.

## Chosen Design

Normalize polarity exactly once inside `current_reconstruction_run()` after a frame has at least two valid low-side samples and before reconstructing the discarded phase:

```c
out->ia = -raw_ia;
out->ib = -raw_ib;
out->ic = -raw_ic;
```

Then reconstruct the selected phase from the already normalized values using `Ia + Ib + Ic = 0`.

The result-field semantics are explicit:

- `raw_ia/raw_ib/raw_ic`: calibrated SOx amplifier values using the low-side device-current sign. These remain unchanged for hardware correlation.
- `ia/ib/ic`: physical motor phase currents using the FOC sign convention. Clarke/Park and corrected-current diagnostics consume these fields.
- `valid_mask`, margins, tracker state, reconstruction choice, blanking, and fault debounce are unchanged.

Overcurrent behavior remains equivalent because it compares absolute corrected phase current against 2.0A. Invalid frames continue to return `frame_valid=false` without publishing usable corrected currents.

## Rejected Alternatives

1. Invert `current_sense_calc()` globally: rejected because it changes the semantics of every calibrated ADC consumer and obscures the measured SOx polarity.
2. Negate only at the Clarke call site: rejected because protection and diagnostics would disagree with the control loop and a second consumer could miss the conversion.

## Verification Gates

1. Host regression must first fail against the old implementation for a known low-side current vector, then pass after the minimal fix.
2. Existing reconstruction, tracker, guard, current-loop, speed-loop, and static tests must pass.
3. CMake ARM and Keil builds must link successfully; Keil flashing may use J-Link Commander because the Keil `UL2CM3.DLL` download path is independently defective.
4. Hardware polarity gate: `mc_open 500 0` must produce corrected `Ia>0, Ib<0, Ic<0` with no fatal fault.
5. Closed-loop gates run in order: ±50mA, then ±100/200/500mA. Every point must meet the existing tolerance, `|Id|<=100mA`, no invalid/freeze growth, and no fatal fault.
6. The 1A supply current limit remains active. Stop immediately on abnormal sound, temperature, supply limiting, saturation, fault, or failed assertion.

## Out of Scope

- PI retuning
- Changing `CCR4=5264` or the 180-tick blanking threshold
- Raising the ±1.5A command limit or 2.0A protection threshold
- Speed-loop testing before the full low-current current-loop matrix passes
