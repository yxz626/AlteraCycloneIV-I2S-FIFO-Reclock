##基于Altera Cyclone IV （EP4CE6F17C8）的 I2S / DoP / 原生 DSD FIFO 重时钟

本工程利用腾讯混元4+codebuddy进行建设和调试，并已经上板编程验证成功。本人不做任何技术支持，相关疑问和bug改进请咨询相应的AI agent。

本方案是面向 **EP4CE6F17C8** 核心板（Cyclone IV E，配 **W9825G6JH** SDRAM 与 **EPCS4**
配置芯片，[主板详见淘宝](https://item.taobao.com/item.htm?id=821637015352&mi_id=0000BXvZk1LRflSx63xT5gNr_1dwjKwPkOyyFr-JYrqp6P0&spm=tbpc.mytb_itemcollect.item.goods&upStreamPrice=2000)）的开源 FPGA 重时钟设计。它接收任意音源（例如树莓派）的 I2S 码流，
经 SDRAM 缓冲后，由本地低抖动音频时钟重新输出。

## 功能特性

- 自动识别输入格式与采样率：**8 档 PCM 速率**（44.1 kHz – 384 kHz），
  支持 **16 / 20 / 24 / 32 位**字长。
- 双晶振（**45.1584 MHz** 与 **49.152 MHz**），依据检测到的时钟族做**无毛刺
  全局时钟 MUX 切换**。
- SDRAM 环形缓冲与 Hi-end 措施：水位 slip 在平滑淡变内提交、20 位通路
  TPDF 抖动 + 舍入、每次切换淡入淡出，并完成全 IO 时序签核。
- **原生 DSD**（DSD64 / 128 / 256 / 512）直通，位精确 FIFO 重时钟。
- **DoP（DSD over PCM）v1.1** 自动识别并解码为原生 DSD，复用同一套
  SDRAM / FIFO 重时钟通路。
- **AD1862** 20 位串行输出模式，按 `K3` 键切换；LED 档位显示由常开时钟域驱动。
- **DSDOE**（`dsd_on`，引脚 F2）与 **F0–F3** 采样率指示
  （引脚 C2 / A2 / A3 / A4），编码见 `f0~F3.txt`，采样率指示与Amanero Combo usb interface相兼容。
- 稳健的流处理：短断流由 SDRAM 缓冲吸收；速率/格式变化需连续有效测量确认；
  PCM 已锁定状态受保护，避免误判为原生 DSD。

## 验证状态（发布时）

| 项目 | 结果 |
|---|---|
| Quartus Prime 17.1 全编译 | **0 errors / 0 warnings**，Flow Successful |
| TimeQuest | 最紧 setup **+0.709 ns**、hold **+0.132 ns**，三工艺角全正，未约束路径 0 项 |
| 资源占用 | LE 3,437 / 6,272（55%），存储位 51,200 / 276,480（19%），引脚 62 / 180 |
| Icarus Verilog 功能仿真 | **`ALL CHECKS PASSED (0 errors)`** |
| DoP 解码位精确性 | 3,849 / 3,872 / 3,848 bit，全部 **0 错** |
| 引脚核对 | **62 / 62 一致**（`python sim/check_pins.py`） |
| 编程文件 | `Cy4_I2Sq.jic`（EPCS4，ASx1） |

## 仓库结构

| 路径 | 内容 |
|---|---|
| `Cy4_I2S_reclock.v` | 完整 Verilog RTL（顶层 + 全部子模块） |
| `Cy4_I2Sq.qpf` / `.qsf` / `.sdc` | Quartus 工程、引脚分配、时序约束 |
| `Cy4_I2Sq.jic` | 面向 EPCS4 的发布固件 |
| `output_files/` | SOF 与全编译 / Fitter / 时序报告 |
| `sim/` | Icarus Verilog 测试平台、SDRAM / 时钟 MUX 模型、仿真日志、引脚核对脚本 |
| `f0~F3.txt` | DSDOE / F0–F3 编码定义 |
| `工程概述.md` | 完整设计文档（中文）——架构、引脚映射、全部修复记录 |
| `验证报告.md` | 需求到验证的对照报告（中文） |
| `工程要求说明.txt` | 原始需求（中文） |
| `归档备份/` | 归档清单，含逐文件 SHA256 |

本仓库**不再分发**厂商 datasheet 与核心板原理图 / PCB 图，请自行从
Intel（Cyclone IV）、Analog Devices（AD1862）、Winbond（W9825G6JH）
及核心板供应商获取。本地归档 ZIP 同样有意排除，版本记录以 Git 历史为准。

## 构建与复现

```sh
# 全编译
quartus_sh --flow compile Cy4_I2Sq

# 生成 EPCS4 编程文件
quartus_cpf -c -d EPCS4 -s EP4CE6 -m ASx1 \
    output_files/Cy4_I2Sq.sof Cy4_I2Sq.jic

# 功能仿真（Icarus Verilog）
iverilog -g2005 -o sim/tb.vvp -s tb \
    sim/altclkctrl_sim.v sim/sdram_model.v \
    sim/tb_Cy4_I2S_reclock.v Cy4_I2S_reclock.v
vvp sim/tb.vvp

# 引脚分配一致性核对
python sim/check_pins.py
```

通过 JTAG + Serial Flash Loader 把 `Cy4_I2Sq.jic` 写入 EPCS4，然后**断电重启**，
让 FPGA 从串行 Flash 自行配置。仅用 JTAG 烧 SOF 是易失的，掉电即失效。

## 硬件说明

- 需要两颗晶振：**E15 = 45.1584 MHz**（44.1 kHz 族）与 **E16 = 49.152 MHz**
  （48 kHz 族）。E16 同时为常开管理域提供时钟。
- LED 高电平有效；`K3` 低电平有效。
- 物理 `PIN_F2` 是 `DSD_ON`（DSDOE），而名为 F2 的**信号**（`fsel[2]`）
  在 `PIN_A3` —— 接线时不要混淆。
- 完整的 62 脚分配与时钟 / IO 约束见 `工程概述.md`。

完整的设计思路、调试历史与实测数据，见 [工程概述.md](./工程概述.md) 与
[验证报告.md](./验证报告.md)。
