# Keil 工程与构建脚本

本目录包含 MPS_MotorDriver 的 Keil MDK 工程及 Windows 命令行构建/烧录脚本。

## 环境要求

- **Keil MDK**: 安装在 `C:\Keil_v5\`(UV4.exe 路径硬编码在脚本里,如不同请改)
- **AT32 DFP**: `ArteryTek.AT32M412_416_DFP` Pack(Keil Pack Installer 安装)
- **调试器**: JLink(工程已配置,见 `keil_flash.log` 历史记录)
- **编译器**: ARMCC V5.06(Keil 自带,非 ARMClang v6)

## 快速使用

打开 `cmd` 或资源管理器双击:

| 命令 | 作用 |
| --- | --- |
| `build.bat` | 增量编译,生成 `objects\MPS_MotorDriver.axf` + `.hex` |
| `build.bat clean` | 全量重编译(先清后编) |
| `flash.bat` | 增量编译后烧录到板子(JLink) |
| `flash.bat rebuild` | 全量重编译后烧录 |
| `flash.bat only` | 跳过编译,直接烧录已有 axf |
| `clean.bat` | 清理 `objects/` 和 `listings/` 产物 |

## 退出码约定

| RC | 含义 |
| --- | --- |
| 0 | 成功,无警告 |
| 1 | 成功,有警告(常见: RT-Thread cpuport.c `context` 未用警告,可忽略) |
| 2+ | 失败,见 `keil_build.log` 或 `keil_flash.log` |

## 产物

- `objects\MPS_MotorDriver.axf` — 带调试符号的可执行文件(Keil 调试用)
- `objects\MPS_MotorDriver.hex` — Intel HEX 格式(独立烧录工具用)

## 日志

- `keil_build.log` — 最近一次构建日志
- `keil_flash.log` — 最近一次烧录日志

## 工程结构(Groups)

| Group | 内容 |
| --- | --- |
| `user` | `project/src/` 下的 main/int/wk_config/board/rtthread_app/ma600a_debug |
| `application/motor_control` | 状态机 + FOC 核心 + 各控制环 + 故障/标定 |
| `application` | motor_app 应用入口 |
| `platform/at32m412` | PWM/电流采样/编码器/保护/Flash/时钟 适配 |
| `communication` | CAN 协议 + 硬件适配 |
| `firmware` | AT32 标准外设库驱动 |
| `cmsis` | startup.s + system_at32m412_416.c |
| `middlewares/msp/ma600` | MA600A 驱动 + SPI2 适配 |
| `middlewares/rt-thread/*` | RT-Thread Nano 内核 + libcpu |

## 与 WSL CMake 构建的关系

| 维度 | Keil (本目录) | WSL CMake (工程根) |
| --- | --- | --- |
| 编译器 | ARMCC V5.06 | arm-none-eabi-gcc |
| 构建命令 | `build.bat` | `wsl cmake --build build/Debug` |
| 链接脚本 | Keil scatter(工程内配置) | `AT32M412xB_FLASH.ld` |
| 优化等级 | Optimize=1(-O1 等效) | -O0(Debug) |
| Code 大小 | ~8.7 KB | ~14.6 KB |

两者源文件列表保持同步(CMake 的 `target_sources` 与 .uvprojx 的 Groups 对应)。
新增源文件时,需同时更新 `CMakeLists.txt` 和 `.uvprojx`。

## 故障排查

**`UV4.exe not found`**: 修改脚本顶部 `UV4` 变量为你的 Keil 安装路径。

**编译错误找不到头文件**: 检查 .uvprojx 的 `IncludePath` 是否包含 `..\..\application`、`..\..\application\motor_control`、`..\..\platform\at32m412`、`..\..\communication`。

**Flash 失败**: 确认 JLink 已连接板子且上电,看 `keil_flash.log` 详细错误。首次烧录需在 Keil GUI 里配置好 JLink 调试器(Settings → Debug → J-LINK)。
