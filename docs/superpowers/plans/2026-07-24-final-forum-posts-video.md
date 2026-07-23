# Final Forum Posts and Demo Video Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce two evidence-based Chinese forum drafts and one low-effort, time-coded project video script for the MPS motor-driver project.

**Architecture:** Keep the three artifacts independent but factually synchronized. The V2 post owns debugging evidence, the competition post owns the full product and five-link narrative, and the video document owns spoken copy, shot requirements, and editing guidance. Numbered image/result placeholders are the only intentionally incomplete content.

**Tech Stack:** UTF-8 Markdown, PowerShell content checks, Git, existing bench records and test reports.

## Global Constraints

- Preserve the user's first-person, candid Chinese engineering voice established in `doc/第一篇过程帖_论坛发布版.md`, `doc/ma600a_development_test_forum_post.md`, and `doc/第三篇过程帖_论坛草稿.md`.
- Treat `docs/superpowers/specs/2026-07-24-final-forum-posts-video-design.md` as the approved content boundary.
- Do not claim Node 2 permanent configuration, dual-node synchronization, five-link motion, or mechanism-load tracking before final evidence exists.
- State the 115 mdeg and 60 mdeg encoder values as internal calibration quality indicators, not external absolute-angle accuracy.
- Limit the verified current-loop range to `Iq_ref=±50/±100/±200/±500 mA`.
- Preserve the speed-loop boundary: 200 rpm_e is repeatable; ±60 rpm_e has usable long-window average but unresolved 4 ms instantaneous ripple.
- Preserve the CAN boundary: the 60 s single-node timing/safety run passed, while the 10°/350 ms response-amplitude gate did not.
- Use explicit numbered visual placeholders from `[V2-01]` through `[V2-08]` and `[FINAL-01]` through `[FINAL-08]`.
- Use Chinese punctuation and explicit engineering units.
- Do not invent image URLs, forum upload URLs, source-package links, video links, usernames, measurements, or final mechanism results.

---

### Task 1: Write the V2 Hardware, Three-Loop, and CAN Debug Post

**Files:**
- Create: `doc/第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md`
- Read: `doc/第三篇过程帖_论坛草稿.md`
- Read: `doc/FOC控制器开发记录.md`
- Read: `doc/调试记录.md`
- Read from branch: `feat/motor-driver-can-node:doc/调试记录.md`
- Read from branch: `feat/motor-driver-can-node:docs/motor-control/test-reports/2026-07-22-xtrack-position-response.md`

**Interfaces:**
- Consumes: approved evidence statements and placeholders `[V2-01]`–`[V2-08]`.
- Produces: a standalone forum draft whose summary facts may be reused by Task 2 and Task 3.

- [ ] **Step 1: Build an evidence ledger at the top of the working notes**

Before drafting, extract these exact claims from the source records:

```text
encoder: 4kHz acquisition, 16kHz FOC, 256 bins, 115/60mdeg, spike=0
current: ±50/100/200/500mA, final max Iq error about 2mA, max |Id| about 1mA
speed: 200rpm_e repeatable; ±60rpm_e long-window average usable; 4ms ripple unresolved
position: static <0.5deg; sine P95 best 1.329deg
CAN: 10ms X-Track timer; 60s max sync_age 19ms; no freeze/error growth
CAN response caveat: 10deg/350ms amplitude gate not met
Node 2: local calibration and ±5deg smoke test only; permanent joint record absent
five-link: dual-motor hardware assembled; X-Track mechanism motion still debugging
```

Reject any draft sentence that expands beyond these claims.

- [ ] **Step 2: Draft the opening and V2 encoder sections**

Write the title and Sections 1–3 with this narrative order:

```text
上一篇解释为什么从旁轴改同轴
→ V2 到板并完成安全 bring-up
→ encoder_service/TMR7 removes SPI from the FOC ISR
→ two-board calibration evidence
→ internal quality metric caveat
→ enc_status concurrent SPI read bug and snapshot-only fix
```

Insert `[V2-01]`, `[V2-02]`, and `[V2-03]` as blockquote placeholders with specific shot descriptions.

- [ ] **Step 3: Draft the current, speed, and position loop sections**

Preserve the process narrative:

```text
low-side current mirror semantics
→ wrong polarity creates positive feedback
→ stop the unsafe matrix
→ fix at the reconstruction boundary
→ verify the eight current points

encoder direction normalization
→ tune speed loop without hiding low-speed ripple
→ distinguish 200rpm_e from 60rpm_e

position/speed/current cascade
→ motion/static friction feedforward
→ lease timeout holds position
→ static, reversal, and sine evidence
```

Insert `[V2-04]`, `[V2-05]`, and `[V2-06]`.

- [ ] **Step 4: Draft the CAN, Node 2, AI, and conclusion sections**

Explain the transport in this exact logical order:

```text
trajectory point preload
→ matching broadcast SYNC
→ 1kHz node service
→ 100Hz feedback/trajectory objective
→ no-ACK error interrupt starvation
→ bounded polling fix
→ 60s X-Track single-node result
→ position-response caveat
```

Close with Node 2 and five-link status without declaring the mechanism complete. Insert `[V2-07]` and `[V2-08]`.

- [ ] **Step 5: Verify the V2 draft**

Run:

```powershell
$path='doc\第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md'
$text=Get-Content -LiteralPath $path -Raw
$required=@('4 kHz','16 kHz','115 mdeg','60 mdeg','±500 mA','200 rpm_e','1.329°','19 ms','[V2-01]','[V2-08]')
$missing=@($required | Where-Object { -not $text.Contains($_) })
if($missing.Count){ throw "Missing: $($missing -join ', ')" }
if($text -match '双节点.*(通过|完成)|五连杆.*(跑通|完成轨迹)|绝对精度.*0\.(115|060)'){ throw 'Unsupported completion claim' }
git diff --check
```

Expected: no exception and `git diff --check` exits 0.

- [ ] **Step 6: Commit Task 1**

```powershell
git add -- 'doc/第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md'
git commit -m "docs: draft V2 motor control process post"
```

---

### Task 2: Write the Final Competition Submission Post

**Files:**
- Create: `doc/最终比赛作品提交帖_论坛草稿.md`
- Read: `doc/作品提交素材清单.md`
- Read: `doc/第一篇过程帖_论坛发布版.md`
- Read: `doc/第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md`
- Read: `docs/superpowers/specs/2026-07-24-final-forum-posts-video-design.md`

**Interfaces:**
- Consumes: the technical summary from Task 1 and placeholders `[FINAL-01]`–`[FINAL-08]`.
- Produces: a complete competition-post scaffold requiring only final mechanism evidence, links, and username.

- [ ] **Step 1: Write the title, 100–200 Chinese-character introduction, and system overview**

The introduction must identify:

```text
the small dual-motor joint driver
two independent V2 nodes
AT32M412 + MP4583 + MP6540H + MA600A
CAN-synchronized five-link demo
the present boundary that mechanism motion is in final debugging
```

Follow with a system-flow block:

```text
X-Track
  → CAN trajectory preload + SYNC
  → Node 1 / Node 2
  → current-speed-position cascades
  → two motors
  → five-link mechanism
```

- [ ] **Step 2: Write the MPS device, PCB iteration, and control-system sections**

Give MP4583, MP6540H, and MA600A one focused subsection each. Summarize V1-to-V2 and the three-loop/CAN architecture without reproducing Task 1's full debug history.

Include a compact evidence table with these rows:

```text
coaxial encoder
current loop
speed loop
position loop
single-node CAN
second motor node
five-link mechanism
```

The final two rows must say “本地资格通过，机构配置待完成” and “硬件装配完成，X-Track 联动调试中”.

- [ ] **Step 3: Write the five-link design, printing, assembly, and debug sections**

Use the user's requested sequence:

```text
design purpose and two active joints
→ CAD and printable part split
→ printing and post-processing
→ bearings, links, motors, boards, and wiring
→ mechanical zero and soft-limit setup
→ single-board qualification
→ Node 1
→ Node 2
→ dual-node mechanism debug
```

Do not invent printer model, material, layer height, bearing model, link length, or print duration. Use `[FINAL-01]`–`[FINAL-05]` for missing media and final results.

- [ ] **Step 4: Write source, usage, AI collaboration, summary, and links**

Include:

```text
Keil ARMCC5 and CMake build paths
J-Link flashing
12V current-limited bring-up
CAN 1Mbps
explicit source-package placeholder [FINAL-07]
video placeholder [FINAL-06]
MPS username placeholder [FINAL-08]
links/placeholders for prior process posts
AI assists analysis/tests/records; hardware evidence remains authoritative
```

- [ ] **Step 5: Verify competition-submission coverage**

Run:

```powershell
$path='doc\最终比赛作品提交帖_论坛草稿.md'
$text=Get-Content -LiteralPath $path -Raw
$required=@('作品简介','系统总体方案','MP4583','MP6540H','MA600A','五连杆','打印','装配','作品源码','演示视频','项目总结','[FINAL-01]','[FINAL-08]')
$missing=@($required | Where-Object { -not $text.Contains($_) })
if($missing.Count){ throw "Missing: $($missing -join ', ')" }
$finalCount=([regex]::Matches($text,'\[FINAL-0[1-8]\]')).Count
if($finalCount -lt 8){ throw "Final placeholders incomplete: $finalCount" }
if($text -match '双节点.*(已经通过|同步完成)|五连杆.*(轨迹通过|演示完成)'){ throw 'Unsupported completion claim' }
git diff --check
```

Expected: no exception and `git diff --check` exits 0.

- [ ] **Step 6: Commit Task 2**

```powershell
git add -- 'doc/最终比赛作品提交帖_论坛草稿.md'
git commit -m "docs: draft final MPS competition submission"
```

---

### Task 3: Write the Time-Coded Low-Effort Video Script

**Files:**
- Create: `doc/项目演示视频脚本与制作建议.md`
- Read: `doc/最终比赛作品提交帖_论坛草稿.md`
- Read: `docs/superpowers/specs/2026-07-24-final-forum-posts-video-design.md`

**Interfaces:**
- Consumes: the competition post's product narrative and the exact evidence boundaries.
- Produces: spoken copy, shots, on-screen text, minimum pickup list, recording settings, and editing workflow.

- [ ] **Step 1: Write the recording assumptions and complete spoken script**

Create a table with columns:

```text
time
picture
spoken narration
on-screen text
source/new pickup
```

Cover `0:00` through approximately `3:30`. Write all narration verbatim, including:

```text
about 10 seconds of on-camera self-introduction
project origin
three MPS parts
V2 coaxial change
current/speed/position/CAN evidence
five-link design, printing, and assembly
final effect segment
short on-camera or product-shot conclusion
```

The final-effect narration must have two selectable versions:

```text
Version A: successful motion, with bracketed fields for actual trajectory/result
Version B: partial demonstration, honestly stating the achieved motion and remaining issue
```

- [ ] **Step 2: Write the minimum pickup list**

Limit required new footage to:

```text
one 25-second on-camera take
one 15-second tabletop wide shot
four approximately 5-second detail shots
one 10-second manual-movement/power-prep shot
one 30–60-second fixed-camera final run
```

For each shot specify framing, camera stability, visible hardware, and whether original audio is needed.

- [ ] **Step 3: Write the editing and audio workflow**

Specify:

```text
1080p, 16:9, 30fps
fixed exposure/focus where possible
record narration after the picture edit
use existing CAD/PCB/waveform/log assets with slow pan or scale
use Source Han Sans or another licensed font
no music is required
if music is used, keep it licensed and below narration
normalize narration before adding music
retain a clean full-length final-run clip before cutting close-ups
```

Add a one-session production sequence that lets the user shoot, record narration, and edit without reshooting.

- [ ] **Step 4: Verify the video script**

Run:

```powershell
$path='doc\项目演示视频脚本与制作建议.md'
$text=Get-Content -LiteralPath $path -Raw
$required=@('0:00','3:30','露脸','MP4583','MP6540H','MA600A','版本 A','版本 B','1080p','30 fps','补拍清单','制作流程')
$missing=@($required | Where-Object { -not $text.Contains($_) })
if($missing.Count){ throw "Missing: $($missing -join ', ')" }
if($text -notmatch '3 分钟|3分'){ throw 'Duration requirement missing' }
git diff --check
```

Expected: no exception and `git diff --check` exits 0.

- [ ] **Step 5: Commit Task 3**

```powershell
git add -- 'doc/项目演示视频脚本与制作建议.md'
git commit -m "docs: add low-effort project demo video script"
```

---

### Task 4: Cross-Artifact Fact and Handoff Review

**Files:**
- Modify if needed: `doc/第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md`
- Modify if needed: `doc/最终比赛作品提交帖_论坛草稿.md`
- Modify if needed: `doc/项目演示视频脚本与制作建议.md`

**Interfaces:**
- Consumes: all three completed drafts.
- Produces: a factually consistent handoff ready for the user's photos and final mechanism result.

- [ ] **Step 1: Check key-number consistency**

Run a PowerShell comparison that confirms all artifacts use the same form of:

```text
4 kHz / 16 kHz
±500 mA verified current limit
200 rpm_e repeatable speed point
1.329° best sine P95 when that metric is mentioned
19 ms maximum CAN sync age when that metric is mentioned
```

Do not force every number into every artifact; only reject contradictory values.

- [ ] **Step 2: Check completion-language consistency**

Search all three files for:

```powershell
rg -n "双节点|五连杆|Node 2|X-Track|完成|通过|跑通|调试中|待补" `
  'doc\第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md' `
  'doc\最终比赛作品提交帖_论坛草稿.md' `
  'doc\项目演示视频脚本与制作建议.md'
```

Every dual-node or five-link result must either be an explicitly numbered placeholder, a conditional video version, or a current-status statement.

- [ ] **Step 3: Check placeholder uniqueness and completeness**

Run:

```powershell
$paths=@(
  'doc\第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md',
  'doc\最终比赛作品提交帖_论坛草稿.md'
)
$all=(Get-Content -LiteralPath $paths -Raw) -join "`n"
1..8 | ForEach-Object {
  $v2='[V2-{0:D2}]' -f $_
  $final='[FINAL-{0:D2}]' -f $_
  if(-not $all.Contains($v2)){ throw "Missing $v2" }
  if(-not $all.Contains($final)){ throw "Missing $final" }
}
```

Expected: no exception.

- [ ] **Step 4: Run final repository checks**

Run:

```powershell
git diff --check
git status --short
git log -4 --oneline
```

Expected: no whitespace errors; status contains no unintended files; log shows the plan and three content commits.

- [ ] **Step 5: Commit review fixes only if files changed**

```powershell
git add -- `
  'doc/第四篇过程帖_V2硬件与三环CAN调试_论坛草稿.md' `
  'doc/最终比赛作品提交帖_论坛草稿.md' `
  'doc/项目演示视频脚本与制作建议.md'
git diff --cached --quiet
if($LASTEXITCODE -ne 0){
  git commit -m "docs: align final submission content"
}
```

