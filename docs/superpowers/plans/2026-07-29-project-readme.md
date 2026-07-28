# MPS MotorDriver Project README Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Chinese repository landing README that opens with the V2 hardware photo and accurately introduces the MotorDriver hardware, firmware capabilities, build paths, debugging tools, project status, and detailed documentation.

**Architecture:** Create one root `README.md` as a concise navigation layer over the existing source tree and engineering records. Reuse `doc/pic/V2.jpg` through a repository-relative Markdown link and link to existing design/test documents instead of duplicating them.

**Tech Stack:** GitHub-flavored Markdown, CMake with `arm-none-eabi-gcc`, Keil MDK/ARMCC5, relative repository links.

## Global Constraints

- The first non-empty README content is `![MPS MotorDriver V2](doc/pic/V2.jpg)`.
- Write the README in Chinese; retain English identifiers, commands, and paths.
- Describe verified firmware capabilities without claiming the complete five-bar mechanism integration is finished.
- Use repository-relative paths only.
- Do not add badges, license claims, release downloads, or unsupported performance figures.

---

### Task 1: Create and verify the project landing README

**Files:**
- Create: `README.md`
- Reference: `doc/pic/V2.jpg`
- Reference: `CMakeLists.txt`
- Reference: `project/MDK_V5/MPS_MotorDriver.uvprojx`
- Reference: `doc/调试记录.md`
- Reference: `docs/superpowers/specs/2026-06-22-mps-foc-design.md`
- Reference: `docs/motor-control/test-reports/2026-07-22-xtrack-position-response.md`

**Interfaces:**
- Consumes: Existing repository paths, current motor-control constants, build entry points, and verified engineering records.
- Produces: A root `README.md` rendered correctly by GitHub/GitLab-style Markdown viewers.

- [ ] **Step 1: Create the README with the approved section order**

Use this section order and content contract:

```markdown
![MPS MotorDriver V2](doc/pic/V2.jpg)

# MPS MotorDriver

一句话说明这是基于 AT32M412、MP6540H 和 MA600A 的一体化有感 FOC 电机驱动模组。

## 项目简介
说明硬件、固件和双节点轨迹演示目标。

## 核心特性
列出三环 FOC、电流重构、编码器标定、CAN 同步、故障保护、RT-Thread Shell 和 LKS Scope 在线整定。

## 硬件组成
用表格介绍 MCU、功率级、编码器、电源与通信接口。

## 控制与软件架构
说明 16 kHz 电流环、1 kHz 速度环、1 kHz 位置环及 application/platform/communication 分层。

## 快速开始
给出 ARM GCC/CMake 命令和 Keil 工程路径。

## 调试方式
介绍串口 Shell、CAN、debug.lksscope 和测试目录。

## 当前状态
区分已验证的单节点/双节点固件能力与仍在推进的机构综合验收。

## 文档导航
链接设计文档、调试记录、测试报告和开发计划。
```

- [ ] **Step 2: Verify every referenced path**

Run:

```powershell
$paths = @(
  'doc\pic\V2.jpg',
  'CMakeLists.txt',
  'project\MDK_V5\MPS_MotorDriver.uvprojx',
  'debug.lksscope',
  'doc\调试记录.md',
  'docs\superpowers\specs\2026-06-22-mps-foc-design.md',
  'docs\motor-control\test-reports\2026-07-22-xtrack-position-response.md'
)
$paths | ForEach-Object {
  if (-not (Test-Path -LiteralPath $_)) { throw "Missing README target: $_" }
}
```

Expected: command exits successfully with no missing target.

- [ ] **Step 3: Verify the README structure and first content**

Run:

```powershell
$lines = Get-Content -LiteralPath README.md
$first = $lines | Where-Object { $_.Trim().Length -gt 0 } | Select-Object -First 1
if ($first -ne '![MPS MotorDriver V2](doc/pic/V2.jpg)') {
  throw "README does not start with the required image"
}
$required = @(
  '# MPS MotorDriver',
  '## 项目简介',
  '## 核心特性',
  '## 硬件组成',
  '## 控制与软件架构',
  '## 快速开始',
  '## 调试方式',
  '## 当前状态',
  '## 文档导航'
)
$text = $lines -join "`n"
$required | ForEach-Object {
  if (-not $text.Contains($_)) { throw "Missing README section: $_" }
}
```

Expected: command exits successfully.

- [ ] **Step 4: Inspect Markdown and Git hygiene**

Run:

```powershell
Select-String -LiteralPath README.md -Pattern 'E:\\', 'TBD', 'TODO'
git diff --check
git diff -- README.md
```

Expected: no absolute drive path, placeholder, or whitespace error; the diff contains only the intended README.

- [ ] **Step 5: Commit the README**

```powershell
git add README.md
git commit -m "docs: add project readme"
```
