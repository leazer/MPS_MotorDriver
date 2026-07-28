# Final Submission MPS Writing Review Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a standalone review draft whose downstream sections match the updated introduction and the verified physical MPS-writing result.

**Architecture:** Use the user's current final-submission draft as the read-only source. Preserve its 13-section structure and verified motor-control evidence, while replacing stale five-bar status, adding the preset-trajectory/kinematics/control chain, and rewriting the final-result and conclusion sections.

**Tech Stack:** Markdown, Mermaid, existing project logs and forum-post writing conventions.

## Global Constraints

- Do not modify `doc/最终比赛作品提交帖_论坛草稿.md`.
- Create `doc/最终比赛作品提交帖_根据新版简介调整_审阅稿.md`.
- State that the pen physically left an MPS trace on paper.
- State that the complete motion ran smoothly, while handwriting quality and repeat stability remain average.
- Describe a preset MPS trajectory; do not claim a teaching function exists.
- Do not invent tracking error, duration, repeat count, success rate, or unverified source filenames.
- Preserve the user's first-person, candid engineering-forum voice.
- Preserve unrelated working-tree changes.

---

### Task 1: Build the Standalone Review Draft

**Files:**
- Read: `doc/最终比赛作品提交帖_论坛草稿.md`
- Create: `doc/最终比赛作品提交帖_根据新版简介调整_审阅稿.md`

**Interfaces:**
- Consumes: the user's revised introduction, the existing 13-section post, and the verified result boundary.
- Produces: one complete reviewable competition-submission draft.

- [ ] **Step 1: Preserve and polish Sections 1–5**

Copy the user's author name and revised project motivation. Normalize Chinese punctuation and change the top-level input label from “旋钮 / 示教轨迹 / 预设动作” to “预设 MPS 轨迹 / 预设动作”.

- [ ] **Step 2: Rewrite the five-bar purpose and control chain**

Keep the two-circle geometry and add the real data flow:

```text
预生成 MPS 末端坐标
  -> 轨迹缩放与离散
  -> 五连杆逆运动学
  -> 双关节目标
  -> CAN 预装 + 广播 SYNC
  -> 双节点本地三环
  -> 末端夹笔书写
```

- [ ] **Step 3: Update fabrication and assembly status**

Replace Node 2 pending-configuration language with completed mechanical-zero, direction, soft-limit, and whole-machine configuration wording, without inventing exact calibration values.

- [ ] **Step 4: Rewrite the result table and final-effect section**

State:

```text
结果：末端夹笔在纸面留下连续 MPS 笔迹
运动：整段轨迹能够流畅跑完
边界：字形效果一般，重复稳定性一般
```

Create distinct image placeholders for running process, final paper trace, and video link.

- [ ] **Step 5: Update source, AI, and conclusion sections**

Describe trajectory generation, discretization, inverse kinematics, and X-Track CAN transmission as functional responsibilities only. Retain the AI experiment theme and close with the completed end-to-end path from motor driver to physical writing.

---

### Task 2: Validate Content Consistency

**Files:**
- Verify: `doc/最终比赛作品提交帖_根据新版简介调整_审阅稿.md`
- Verify unchanged: `doc/最终比赛作品提交帖_论坛草稿.md`

**Interfaces:**
- Consumes: Task 1 draft.
- Produces: evidence that the review draft matches the approved design.

- [ ] **Step 1: Check required result language**

Run:

```powershell
$p='doc\最终比赛作品提交帖_根据新版简介调整_审阅稿.md'
$t=Get-Content -LiteralPath $p -Raw
@(
  '预生成',
  '逆运动学',
  '纸面',
  'MPS',
  '整体流畅',
  '字形效果一般',
  '重复稳定性一般'
) | ForEach-Object {
  [PSCustomObject]@{Term=$_; Present=$t.Contains($_)}
}
```

Expected: every term is present.

- [ ] **Step 2: Scan for stale or unsupported statements**

Run:

```powershell
rg -n "Node 2.*待|双节点.*联调中|最终轨迹.*待|示教功能.*实现|高精度书写|稳定重复|成功率|轨迹误差" `
  "doc/最终比赛作品提交帖_根据新版简介调整_审阅稿.md"
```

Expected: no stale completion state or invented metric claim.

- [ ] **Step 3: Check Markdown structure**

Run:

```powershell
$p='doc\最终比赛作品提交帖_根据新版简介调整_审阅稿.md'
$t=Get-Content -LiteralPath $p -Raw
[PSCustomObject]@{
  Sections=([regex]::Matches($t,'(?m)^## ')).Count
  Fences=([regex]::Matches($t,'```')).Count
  BalancedFences=((([regex]::Matches($t,'```')).Count % 2) -eq 0)
}
```

Expected: 13 top-level sections and balanced code fences.

- [ ] **Step 4: Confirm the original was not changed during execution**

Record the original file length and last-write time before Task 1, then compare them after Task 2.

- [ ] **Step 5: Confirm unrelated tracked changes remain untouched**

Run:

```powershell
git status --short
```

Expected: tracked working-tree status matches the status captured immediately before execution; no unrelated file is modified.
