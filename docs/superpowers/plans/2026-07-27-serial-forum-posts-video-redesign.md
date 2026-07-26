# V2 Serial Forum Posts and Video Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the single long V2 draft with four image-led serial posts, update the final competition submission, and rewrite the 3:10–3:30 PPT-led video script while preserving the old draft as an archive.

**Architecture:** Each serial post owns one engineering topic and links naturally to the next: coaxial encoder, three-loop LKS Scope tuning, CAN protocol/debugging, and the five-bar demo. The final competition post is a concise system overview, while the video script reuses the same evidence in a casual narrated slide sequence.

**Tech Stack:** Markdown, Mermaid source diagrams, existing C firmware/log/test evidence, LKS Scope screenshots supplied or captured later, CAD/PCB/real-product images supplied by the user.

## Global Constraints

- Preserve `doc/第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md` unchanged as an archive.
- Each new serial post targets 1500–2500 Chinese characters and 5–8 meaningful visuals.
- Post 4 must not describe the safe power-on inspection sequence.
- Post 4 uses the single uncalibrated V1 waveform only as background; the V2 waveform is the main evidence.
- Post 5 must describe the in-progress LKS Scope architecture accurately and must not claim final tuning completion.
- Post 6 may claim the verified X-Track single-node 60 s CAN result, but not completed dual-node/five-bar synchronization.
- Post 7 must state that dual-motor hardware is assembled and X-Track control remains under debug.
- The final video is 3:10–3:30, PPT-led, includes a short face introduction, and uses casual first-person narration.
- Missing final action media must use uniquely numbered placeholders rather than invented results.
- Do not modify the dirty motor-control parameters, `debug.lksscope`, Keil project, or the active CAN/LKS worktree.

---

## File Structure

### New files

- `doc/第四篇过程帖_V2同轴磁编改版与波形_论坛草稿.md` — V2 layout, acquisition, dragged waveform, and calibration usability.
- `doc/第五篇过程帖_三环LKS_Scope波形调试_论坛草稿.md` — cascade diagram and image-led LKS Scope tuning narrative.
- `doc/第六篇过程帖_CAN协议与单节点联调_论坛草稿.md` — protocol, code, historical debug logs, and verified single-node result.
- `doc/第七篇过程帖_五连杆轮腿Demo设计_论坛草稿.md` — motivation, mathematics, mechanics, and X-Track control architecture.
- `doc/CAN联调截图素材整理.md` — exact source snippets and composition notes for Post 6 screenshots.

### Modified files

- `doc/最终比赛作品提交帖_论坛草稿.md` — convert to a system-level competition submission using the four serial posts as supporting detail.
- `doc/项目演示视频脚本与制作建议.md` — replace the previous shoot-led structure with a PPT-led 3:10–3:30 script and casual narration.

### Read-only evidence

- `doc/第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md`
- `.worktrees/motor-driver-can-node/doc/调试记录.md`
- `.worktrees/motor-driver-can-node/docs/motor-control/test-reports/2026-07-22-xtrack-position-response.md`
- `.worktrees/motor-driver-can-node/docs/motor-control/evidence/*.jsonl`
- `.worktrees/motor-driver-can-node/communication/can_protocol.[ch]`
- `.worktrees/motor-driver-can-node/application/motor_control/can_motion_service.[ch]`

---

### Task 1: Write Post 4 — V2 Coaxial Encoder

**Files:**
- Create: `doc/第四篇过程帖_V2同轴磁编改版与波形_论坛草稿.md`
- Read: `doc/第三篇过程帖_论坛草稿.md`
- Read: `doc/第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md`

**Interfaces:**
- Consumes: verified V2 layout/acquisition/calibration facts and the user's single uncalibrated V1 waveform.
- Produces: the narrative bridge from the earlier V1 post to Post 5.

- [ ] **Step 1: Draft the opening and redesign motivation**

Write a first-person continuation that explains why coaxial placement was chosen, without restating the safe bring-up procedure.

- [ ] **Step 2: Add layout and physical implementation sections**

Explain the sensor/magnet/axis relationship and insert `[P4-01]` through `[P4-04]` exactly as defined in the spec.

- [ ] **Step 3: Add the acquisition chain**

Describe TMR7 4 kHz sampling and the 16 kHz FOC snapshot consumer, then include a compact Mermaid source block for `[P4-05]`.

- [ ] **Step 4: Add the waveform interpretation**

Use V1 only as a background image and give most of the text to V2 raw/corrected dragged-motion waveforms, continuity, wraparound, and spike count.

- [ ] **Step 5: State the calibration conclusion carefully**

Quote 256 bins and internal 115/60 mdeg metrics, explain their internal meaning, and phrase V2 as “标定更多是在做小误差修整” rather than claiming unverified absolute accuracy.

- [ ] **Step 6: Verify structure and length**

Run:

```powershell
$p='doc\第四篇过程帖_V2同轴磁编改版与波形_论坛草稿.md'
$t=Get-Content -LiteralPath $p -Raw
[PSCustomObject]@{
  Characters=$t.Length
  ImageMarkers=([regex]::Matches($t,'\[P4-\d\d\]')).Count
  HasSafetySequence=($t -match '上电检查顺序|安全上电顺序')
}
```

Expected: readable forum draft, 5–8 unique P4 visuals, and `HasSafetySequence=False`.

---

### Task 2: Write Post 5 — Three Loops and LKS Scope

**Files:**
- Create: `doc/第五篇过程帖_三环LKS_Scope波形调试_论坛草稿.md`
- Read: `.worktrees/motor-driver-can-node/application/motor_control/motor_tuning.[ch]`
- Read: `.worktrees/motor-driver-can-node/debug.lksscope`

**Interfaces:**
- Consumes: `g_motor_tuning`, `g_motor_loop_debug`, the approximately 137 numeric items, and the preserved 11 chart curves.
- Produces: a screenshot-ready guide for current, speed, and position tuning.

- [ ] **Step 1: Add the cascade block diagram**

Create a compact Mermaid diagram for position target → position loop → speed target → speed loop → Iq target → current loop → SVPWM/motor, with encoder and current feedback.

- [ ] **Step 2: Explain the online tuning design**

Describe centralized RAM tunables and runtime snapshots in plain language; state explicitly that reset restores defaults and there is no flash persistence.

- [ ] **Step 3: Add the current-loop image section**

Insert `[P5-04]` and `[P5-05]`, with short captions describing target, feedback, error, overshoot, settling, and output saturation.

- [ ] **Step 4: Add the speed- and position-loop image sections**

Insert `[P5-06]` and `[P5-07]`, with concise captions focused on tracking instead of a long PID tutorial.

- [ ] **Step 5: Preserve the in-progress boundary**

End with a natural transition to CAN and state that the screenshot set will be replaced as the LKS Scope workflow is finalized.

- [ ] **Step 6: Verify required facts**

Run:

```powershell
$p='doc\第五篇过程帖_三环LKS_Scope波形调试_论坛草稿.md'
$t=Get-Content -LiteralPath $p -Raw
@('g_motor_tuning','g_motor_loop_debug','RAM','电流环','速度环','位置环') |
  ForEach-Object { [PSCustomObject]@{Term=$_; Present=$t.Contains($_)} }
```

Expected: every term is present and no sentence claims completed final tuning.

---

### Task 3: Curate Post 6 CAN Screenshot Material

**Files:**
- Create: `doc/CAN联调截图素材整理.md`
- Read: `.worktrees/motor-driver-can-node/doc/调试记录.md:1547`
- Read: `.worktrees/motor-driver-can-node/docs/motor-control/test-reports/2026-07-22-xtrack-position-response.md:7`
- Read: `.worktrees/motor-driver-can-node/communication/can_protocol.h:14`
- Read: `.worktrees/motor-driver-can-node/application/motor_control/can_motion_service.c:18`

**Interfaces:**
- Consumes: exact code and historical logs.
- Produces: three screenshot compositions that Post 6 can cite without re-running hardware.

- [ ] **Step 1: Curate the protocol/code card**

Include only the four CAN IDs, five broadcast opcodes, and the 30/50/500 ms constants, with source paths and line references.

- [ ] **Step 2: Curate the bus-off debug card**

Show the before values `STS=0xC0000502`, `INTEN=0x000005CA` and the verified safe outcome: COM9 responsive, `FAULT_CAN_BUS`, `EN=0`, and safe CCR values.

- [ ] **Step 3: Curate the 60 s result card**

Show `sync_age=19 ms`, no reference freeze, and zero growth in the four CAN error counters; separately note that position response did not meet 9.5°/350 ms.

- [ ] **Step 4: Add capture instructions**

Specify a dark terminal theme, 16:9 or 4:3 crop, 32–40 px equivalent body text, red annotation only for the fault cause, and green annotation only for verified results.

- [ ] **Step 5: Verify all numbers against sources**

Run:

```powershell
rg -n "0x080|0x100|0x180|0x280|30ms|50ms|500ms|0xC0000502|0x000005CA|19 ms|9.5" `
  "doc/CAN联调截图素材整理.md"
```

Expected: every required source number appears and no dual-node completion claim appears.

---

### Task 4: Write Post 6 — CAN Protocol and Single-Node Debugging

**Files:**
- Create: `doc/第六篇过程帖_CAN协议与单节点联调_论坛草稿.md`
- Read: `doc/CAN联调截图素材整理.md`

**Interfaces:**
- Consumes: the curated CAN cards from Task 3.
- Produces: an image-led protocol/debug post that ends at the verified single-node boundary.

- [ ] **Step 1: Write the protocol overview**

Explain why the controller sends two node-specific trajectory frames before one broadcast SYNC.

- [ ] **Step 2: Add protocol and sequence diagrams**

Include Mermaid source for the X-Track/two-node frame map and the preload/SYNC timing.

- [ ] **Step 3: Add concise code commentary**

Use `[P6-04]` and `[P6-05]` to explain protocol definitions and the 1 kHz service without pasting large source files.

- [ ] **Step 4: Tell the no-ACK debugging story**

Use `[P6-06]` to connect symptom, root cause, fix, and safe outcome in four short paragraphs.

- [ ] **Step 5: Present the single-node result and boundary**

Use `[P6-07]` for the 60 s result; explicitly separate CAN timing/safety success from the unfinished position response and dual-node work.

- [ ] **Step 6: Verify claims**

Run:

```powershell
$p='doc\第六篇过程帖_CAN协议与单节点联调_论坛草稿.md'
$t=Get-Content -LiteralPath $p -Raw
[PSCustomObject]@{
  Has60s=$t.Contains('60 s')
  Has19ms=$t.Contains('19 ms')
  HasResponseBoundary=($t -match '9\.5°|350 ms')
  ClaimsDualComplete=($t -match '双节点.*(完成|通过|成功)')
}
```

Expected: first three fields are `True`; `ClaimsDualComplete=False`.

---

### Task 5: Write Post 7 — Five-Bar Wheel-Leg Demo

**Files:**
- Create: `doc/第七篇过程帖_五连杆轮腿Demo设计_论坛草稿.md`

**Interfaces:**
- Consumes: the assembled dual-motor hardware status and the unverified X-Track final-control status.
- Produces: the bridge from motor-driver engineering to the final competition demo.

- [ ] **Step 1: Write the selection story**

Mention the alternatives seen on the forum, then explain why a five-bar wheel-leg module better demonstrates two motors and synchronized joints.

- [ ] **Step 2: Add the kinematic explanation**

Define A/B/C/D/P and present the circle-intersection forward kinematics from the approved spec without inventing link dimensions.

- [ ] **Step 3: Add mechanical design and fabrication**

Insert `[P7-04]` through `[P7-06]` for CAD, drawing, printing, and assembly, with captions the user can replace directly.

- [ ] **Step 4: Add the X-Track architecture**

Create a compact diagram showing X-Track → two trajectory frames → broadcast SYNC → Node 1/2 cascaded loops → five-bar mechanism.

- [ ] **Step 5: State current progress honestly**

Say the dual-motor hardware is assembled and X-Track motion control remains under debug; reserve `[P7-08]` for the current hardware and the final action.

- [ ] **Step 6: Verify mathematical symbols and status**

Run:

```powershell
rg -n "A|B|C|D|P|l_1|l_2|l_3|l_4|正在.*debug|调试中|硬件.*搭建" `
  "doc/第七篇过程帖_五连杆轮腿Demo设计_论坛草稿.md"
```

Expected: the mechanism variables and current-state statement are present.

---

### Task 6: Rewrite the Final Competition Submission

**Files:**
- Modify: `doc/最终比赛作品提交帖_论坛草稿.md`
- Read: the four new serial posts.

**Interfaces:**
- Consumes: verified evidence and concise links/summary from Posts 4–7.
- Produces: the final competition-ready system overview.

- [ ] **Step 1: Rebuild the top-level outline**

Use sections for overview, system diagram, hardware, software modules, five-bar principle, mechanics, verified results, final media, source/video links, and conclusion.

- [ ] **Step 2: Add the motor-module architecture**

Show MP4583 → AT32M412/MP6540H/MA600A → motor and X-Track/CAN → two nodes → five-bar.

- [ ] **Step 3: Add the core-code module map**

Describe the exact modules listed in the spec in grouped form: sensing, FOC loops, online tuning, CAN motion, persistent joint configuration, and fault management.

- [ ] **Step 4: Condense the five-bar design**

Reuse only the essential geometry and CAD/assembly story; point readers to Post 7 for detail.

- [ ] **Step 5: Preserve final placeholders**

Keep unique markers for CAD, finished product, final action/GIF, video link, source link, and author name. Do not invent final performance.

- [ ] **Step 6: Verify competition coverage**

Run:

```powershell
$p='doc\最终比赛作品提交帖_论坛草稿.md'
$t=Get-Content -LiteralPath $p -Raw
@('MP4583','MP6540H','MA600A','AT32M412','encoder_service','can_motion_service','五连杆','视频链接','源码') |
  ForEach-Object { [PSCustomObject]@{Term=$_; Present=$t.Contains($_)} }
```

Expected: all required system and submission terms are present.

---

### Task 7: Rewrite the PPT-Led Video Script

**Files:**
- Modify: `doc/项目演示视频脚本与制作建议.md`

**Interfaces:**
- Consumes: the same evidence hierarchy and placeholders used by Posts 4–7.
- Produces: a 3:10–3:30 production script that needs minimal filming.

- [ ] **Step 1: Define the 10-slide visual spine**

Specify each slide's duration, title, visual composition, inserted evidence, and transition.

- [ ] **Step 2: Write casual narration**

Use short first-person sentences, conversational pauses, and natural phrases such as “我当时想了挺久” and “这个坑还挺有意思”, while avoiding formulaic transitions.

- [ ] **Step 3: Add the face introduction**

Write a 10–12 second opening that identifies the project and points naturally to the assembled demo.

- [ ] **Step 4: Add the final action segment**

Reserve 20–30 seconds for fixed-camera five-bar action and give a fallback edit if only one reliable motion is available.

- [ ] **Step 5: Add AI-PPT and editing instructions**

Provide a reusable visual prompt, slide-generation workflow, asset replacement checklist, voice recording approach, subtitle rules, and minimal shooting list.

- [ ] **Step 6: Verify timing**

Sum all segment durations manually in the timing table and ensure the planned result is between 190 and 210 seconds.

---

### Task 8: Cross-Document Verification

**Files:**
- Verify: all seven user-facing Markdown files.

**Interfaces:**
- Consumes: Tasks 1–7.
- Produces: a consistent, handoff-ready content package.

- [ ] **Step 1: Confirm the archive was not changed**

Run:

```powershell
git diff --no-index -- `
  "doc/第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md" `
  "doc/第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md"
```

Expected: exit code 0 and no output. Also confirm its last-write time did not change during execution.

- [ ] **Step 2: Scan for unsupported completion language**

Run:

```powershell
rg -n "双节点.*(已经|已).*通过|五连杆.*(已经|已).*跑通|最终效果.*已完成" `
  "doc/第四篇过程帖_V2同轴磁编改版与波形_论坛草稿.md" `
  "doc/第五篇过程帖_三环LKS_Scope波形调试_论坛草稿.md" `
  "doc/第六篇过程帖_CAN协议与单节点联调_论坛草稿.md" `
  "doc/第七篇过程帖_五连杆轮腿Demo设计_论坛草稿.md" `
  "doc/最终比赛作品提交帖_论坛草稿.md" `
  "doc/项目演示视频脚本与制作建议.md"
```

Expected: no unsupported completion claims.

- [ ] **Step 3: Verify unique visual markers**

Run:

```powershell
rg -o "\[(P4|P5|P6|P7|FINAL|VIDEO)-[0-9]{2}\]" doc |
  Sort-Object |
  Group-Object |
  Where-Object Count -gt 3
```

Expected: repeated markers occur only where the final post/video intentionally references a serial-post asset.

- [ ] **Step 4: Review image and replacement instructions**

Confirm every marker has a caption that states what image to use, what to crop, and what conclusion it supports.

- [ ] **Step 5: Report preserved dirty files**

Run:

```powershell
git status --short
```

Expected: the pre-existing edits to `motor_params.h`, `debug.lksscope`, and the Keil project remain untouched.
