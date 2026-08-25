# JESD204 PCS 链路训练模块文档

## 1. 概述

`jesd204_pcs_link_training` 是一个完整的带链路训练功能的 JESD204 PCS（物理编码子层）。它实现了完整的 TX/RX 数据路径，包括：

- **TX 端**：加扰器、字符替换（/A/ /F/）、8b10b 编码器
- **RX 端**：8b10b 解码器、解扰器、帧对齐监控、每通道弹性缓冲区（lane de-skew）、CGS 检测
- **链路训练 FSM**：CGS → ILAS → DATA
- **帧对齐错误检测**：通过 `ENABLE_FRAME_ALIGN_ERR_RESET` 实现自动重新同步

该设计运行在**单一时钟域**（`clk`）。在仿真中，serdes 模型会注入每通道的 skew（相位差）来模拟真实世界的通道失配。

---

## 2. 模块层级结构

```
jesd204_pcs_link_training  (顶层)
│
├── jesd204_lmfc  (1)
│   └── 生成本地多帧时钟（LMFC）边沿
│
├── jesd204_frame_mark  (1)
│   └── 生成帧/多帧边界标记（sof, eof, somf, eomf）
│
├── jesd204_tx_ctrl  (1)
│   ├── sync_bits  (1)  — sync_request_n 的 CDC
│   └── TX 状态机：WAIT → CGS → ILAS → DATA
│
├── jesd204_tx_lane  (NUM_LANES)
│   ├── jesd204_scrambler  (1, TX 模式)
│   ├── util_pipeline_stage  (1)
│   └── jesd204_frame_align_replace  (1, TX 模式)
│
├── jesd204_8b10b_encoder  (NUM_LANES × DATA_PATH_WIDTH)
│
├── jesd204_8b10b_decoder  (NUM_LANES × DATA_PATH_WIDTH)
│
├── jesd204_rx_ctrl  (1)
│   └── RX 状态机：RESET → WAIT_FOR_PHY → CGS → SYNCHRONIZED
│
└── jesd204_rx_lane  (NUM_LANES)
    ├── util_pipeline_stage  (2)
    ├── align_mux  (1)
    ├── jesd204_rx_frame_align  (1, 如果 ENABLE_FRAME_ALIGN_CHECK)
    │   ├── jesd204_frame_mark  (1)
    │   └── jesd204_frame_align_replace  (1, RX 模式)
    ├── jesd204_scrambler  (1, 解扰模式)
    ├── elastic_buffer  (1)
    ├── jesd204_ilas_monitor  (1)
    └── jesd204_rx_cgs  (1)
```

---

## 3. 顶层参数

| 参数 | 默认值 | 描述 |
|-----------|---------|-------------|
| `NUM_LANES` | 1 | serdes 通道数量 |
| `NUM_LINKS` | 1 | JESD204 链路数量 |
| `DATA_PATH_WIDTH` | 8 | 每个时钟周期每通道的 8b 符号数（2、4 或 8）。DPW=8 → 80b/通道 serdes；DPW=4 → 40b/通道；DPW=2 → 20b/通道 |
| `ENABLE_FRAME_ALIGN_CHECK` | 1 | 实例化 `rx_frame_align` 以监控 /A/ /F/ 的位置 |
| `ENABLE_CHAR_REPLACE` | 1 | TX 通过 `jesd204_frame_align_replace` 在 EOMF 插入 /A/，在 EOF 插入 /F/ |
| `ENABLE_FRAME_ALIGN_ERR_RESET` | 1 | 当帧对齐错误超过阈值时，rx_ctrl 重置到 CGS |
| `FRAME_ALIGN_ERR_THRESHOLD` | 16 | 触发重新同步的错误计数 |
| `ELASTIC_BUFFER_SIZE` | 256 | 每通道弹性缓冲区深度（单位：bit） |

---

## 4. 顶层端口定义

### 4.1 时钟 / 复位

| 端口 | 方向 | 位宽 | 描述 |
|------|-----|-------|-------------|
| `clk` | in | 1 | TX/RX 数据路径的单一时钟域 |
| `reset` | in | 1 | 同步高电平复位 |

### 4.2 配置

| 端口 | 方向 | 位宽 | 描述 |
|------|-----|-------|-------------|
| `cfg_octets_per_multiframe` | in | [9:0] | 一个多帧（ multiframe ）中的八进制字节数（K）。必须是 DATA_PATH_WIDTH 的倍数。典型值：32 |
| `cfg_octets_per_frame` | in | [7:0] | 一个帧（frame）中的八进制字节数（F） |
| `cfg_lanes_disable` | in | [NUM_LANES-1:0] | 每通道禁用（1 = 通道禁用） |
| `cfg_links_disable` | in | [NUM_LINKS-1:0] | 每链路禁用（1 = 链路禁用） |
| `cfg_disable_scrambler` | in | 1 | 1 = 旁路加扰/解扰器 |
| `cfg_disable_char_replacement` | in | 1 | 1 = 不插入 /A/ /F/ 对齐字符 |
| `cfg_mframes_per_ilas` | in | [7:0] | 一个 ILAS 序列中的多帧数（M） |
| `cfg_skip_ilas` | in | 1 | 1 = 跳过 ILAS，直接从 CGS 进入 DATA |
| `cfg_continuous_cgs` | in | 1 | 1 = 保持在 CGS（用于初始上电） |
| `cfg_continuous_ilas` | in | 1 | 1 = 连续重复 ILAS |

### 4.3 TX 应用接口（ready/valid 握手）

| 端口 | 方向 | 位宽 | 描述 |
|------|-----|-------|-------------|
| `tx_valid` | in | 1 | 应用层数据有效 strobe |
| `tx_ready` | out | 1 | PCS 准备接收 tx_data（仅在 DATA 阶段） |
| `tx_data` | in | [DATA_PATH_WIDTH×8×NUM_LANES-1:0] | 应用层有效负载，按通道打包：`[lane0_char0, lane0_char1, ...]` |
| `tx_charisk` | in | [DATA_PATH_WIDTH×NUM_LANES-1:0] | 每字符控制标志（1 = 该 8b 字符为控制字符 Kx.y） |

### 4.4 RX 应用接口（ready/valid 握手）

| 端口 | 方向 | 位宽 | 描述 |
|------|-----|-------|-------------|
| `rx_valid` | out | 1 | 本周期 rx_data / rx_charisk 有效 |
| `rx_ready` | in | 1 | 应用层准备接收 RX 数据 |
| `rx_data` | out | [DATA_PATH_WIDTH×8×NUM_LANES-1:0] | 解码后的应用层有效负载 |
| `rx_charisk` | out | [DATA_PATH_WIDTH×NUM_LANES-1:0] | 每字符控制标志 |
| `rx_notintable` | out | [DATA_PATH_WIDTH×NUM_LANES-1:0] | 每字符 8b10b 非法表错误 |
| `rx_disperr` | out | [DATA_PATH_WIDTH×NUM_LANES-1:0] | 每字符 8b10b 极性错误 |

### 4.5 Serdes 接口

| 端口 | 方向 | 位宽 | 描述 |
|------|-----|-------|-------------|
| `serdes_tx_data` | out | [DATA_PATH_WIDTH×10×NUM_LANES-1:0] | 到 serdes TX 的并行 8b10b 编码符号。按通道打包：`[lane0_sym0, lane0_sym1, ...]` |
| `serdes_rx_data` | in | [DATA_PATH_WIDTH×10×NUM_LANES-1:0] | 从 serdes RX 收到的并行 8b10b 符号 |

### 4.6 链路控制 / 状态

| 端口 | 方向 | 位宽 | 描述 |
|------|-----|-------|-------------|
| `sync_request_n` | in | [NUM_LINKS-1:0] | 来自远端 RX 设备。低电平有效。触发 TX 状态机离开 WAIT 并进入 CGS→ILAS→DATA |
| `status_ctrl_state` | out | [1:0] | 当前 TX 状态：2'b00=RESET, 2'b01=CGS, 2'b10=ILAS, 2'b11=DATA |
| `status_lane_cgs_state` | out | [2×NUM_LANES-1:0] | 每通道 CGS 检测器状态（每通道 2 bit） |
| `status_lane_ifs_ready` | out | [NUM_LANES-1:0] | 每通道 ILAS 监控器 IFS 就绪 |
| `sync_n` | out | [NUM_LINKS-1:0] | 到远端 TX 设备。低电平有效。低 = "我处于 CGS，请继续发送 K28.5" |

### 4.7 ILAS 配置（RX → 应用层）

| 端口 | 方向 | 位宽 | 描述 |
|------|-----|-------|-------------|
| `ilas_config_valid` | out | [NUM_LANES-1:0] | 单比特有效：通道 i 有有效的 ILAS 配置 |
| `ilas_config_addr` | out | [NUM_LANES×2-1:0] | 每通道 2 bit 配置地址（0=R/ADJCNT, 1=Q/ADJDIR, 2=F/MF, 3=K/CS） |
| `ilas_config_data` | out | [NUM_LANES×DATA_PATH_WIDTH×8-1:0] | 每通道配置数据 |

### 4.8 错误状态

| 端口 | 方向 | 位宽 | 描述 |
|------|-----|-------|-------------|
| `status_err_statistics_cnt` | out | [32×NUM_LANES-1:0] | 每通道 32 bit 错误计数器。在 8b10b 非法表/极性错误时递增。在 `event_data_phase` 时复位 |
| `status_frame_align_err_cnt_0` | out | [NUM_LANES-1:0] | 通道 0 的帧对齐错误计数器（每通道 8 bit） |
| `event_frame_alignment_error` | out | 1 | 每通道帧对齐错误阈值的 sticky OR。当任何使能通道的错误数 ≥ 阈值且 `ENABLE_FRAME_ALIGN_ERR_RESET=1` 时，脉冲为高 |

---

## 5. 功能描述

### 5.1 TX 数据路径

```
应用层数据
       │
       ▼
  [加扰器]
       │
       ▼
  [帧对齐替换]  ← 在 EOMF 插入 /A/，在 EOF 插入 /F/
       │
       ▼
  [8b10b 编码器]  ← 每字符，每通道
       │
       ▼
  serdes_tx_data
```

**TX 状态机（`jesd204_tx_ctrl`）**：

| 状态 | 描述 | tx_ready | serdes 输出 |
|-------|-------------|----------|---------------|
| WAIT | 等待 sync_request_n 解除 | 0 | 空闲 |
| CGS | 码组同步 | 0 | 所有通道输出 `/K28.5/`（0xBC） |
| ILAS | 初始通道对齐序列 | 0 | `/R/`、配置数据、`/Q/`、`/A/` |
| DATA | 正常数据传输 | 1 | 带 /A/ /F/ 的加扰应用层数据 |

### 5.2 RX 数据路径

```
serdes_rx_data
       │
       ▼
  [8b10b 解码器]
       │
       ▼
  [CGS 检测]  ← 检测 K28.5 是否存在
       │
       ▼
  [对齐 Mux]  ← 旋转数据，将第一个非 K 字符对齐到位置 0
       │
       ▼
  [帧对齐检查]  ← 验证 /A/ /F/ 位置
       │
       ▼
  [解扰器]
       │
       ▼
  [弹性缓冲区]  ← lane de-skew
       │
       ▼
  [ILAS 监控]  ← 提取配置数据
       │
       ▼
  rx_data / rx_charisk
```

**RX 状态机（`jesd204_rx_ctrl`）**：

| 状态 | 描述 | sync_n | buffer_release_n |
|-------|-------------|--------|------------------|
| RESET | 复位状态 | 高 | 高 |
| WAIT_FOR_PHY | 等待物理层就绪 | 高 | 高 |
| CGS | 码组同步 | 低 | 高 |
| SYNCHRONIZED | 链路建立，数据流动 | 高 | 受控释放 |

### 5.3 链路训练流程

1. **同步请求**：RX 将 `sync_request_n` 拉低（或 `cfg_continuous_cgs=1` 使 TX 保持在 CGS）
2. **CGS 阶段**：TX 发送连续的 `/K28.5/`（0xBC）。RX 的 CGS 检测器验证持续的 K28.5 无错误
3. **ILAS 阶段**：TX 发送配置数据。RX 的 ILAS 监控器捕获通道配置（ADJCNT、ADJDIR、F/MF、K/CS）
4. **DATA 阶段**：TX 发送加扰后的应用层数据。RX 弹性缓冲区同时释放以实现 lane de-skew
5. **错误恢复**：如果帧对齐错误超过阈值，`event_frame_alignment_error` 产生脉冲，RX 回退到 RESET/CGS

---

## 6. 子模块说明

### 6.1 `jesd204_lmfc`

生成本地多帧时钟（LMFC）。产生 `lmfc_edge`（每多帧一次）、`lmfc_clk`、`lmfc_counter` 以及与 SYSREF 相关的信号。

### 6.2 `jesd204_frame_mark`

生成帧/多帧边界标记（`sof`、`eof`、`somf`、`eomf`），以每通道位向量形式表示。支持 `DATA_PATH_WIDTH` = 4、6、8。

### 6.3 `jesd204_tx_ctrl`

TX 控制状态机。管理 WAIT → CGS → ILAS → DATA 转换。生成 `lane_cgs_enable`、`eof_reset`、`tx_ready` 和 ILAS 数据（`ilas_data`、`ilas_charisk`）。使用 `sync_bits` CDC 处理 `sync_request_n`。

### 6.4 `jesd204_tx_lane`

每通道 TX 数据路径。在 CGS（K28.5）、ILAS 和用户数据之间进行多路复用。应用加扰和字符替换。

### 6.5 `jesd204_8b10b_encoder`

带运行极性（running disparity）的单字符 8b10b 编码器。支持 K28.x 控制字符。

### 6.6 `jesd204_8b10b_decoder`

带运行极性的单字符 8b10b 解码器。检测 K28.X，报告非法表和极性错误。

### 6.7 `jesd204_rx_ctrl`

RX 控制状态机。驱动 `cgs_reset`、`ifs_reset`、`sync_n`、`phy_en_char_align`。转换流程：RESET → WAIT_FOR_PHY → CGS → SYNCHRONIZED。

### 6.8 `jesd204_rx_lane`

每通道 RX 数据路径。执行字符解码、IFS 检测、帧对齐、解扰、弹性缓冲、ILAS 监控和 CGS 检测。

### 6.9 `jesd204_rx_cgs`

每通道 CGS 检测器。检测无错误情况下持续的 K28.5 存在。状态：INIT → CHECK → DATA。

### 6.10 `jesd204_ilas_monitor`

从接收到的数据流中提取 ILAS 配置字。捕获 R/ADJCNT、Q/ADJDIR、F/MF、K/CS。

### 6.11 `jesd204_scrambler`

15 位 LFSR 风格的加扰/解扰器。支持加扰和解扰模式。

### 6.12 `jesd204_frame_align_replace`

在 EOMF 和 EOF 边界插入或验证对齐字符（/A/ = 0x7C，/F/ = 0xFC）。

### 6.13 `align_mux`

通过可编程偏移旋转输入数据总线，将第一个非 K28.5 字符对齐到位置 0。

### 6.14 `elastic_buffer`

用于 lane de-skew 的每通道弹性缓冲区。支持非对称输入/输出位宽。

### 6.15 `util_pipeline_stage`

带 `(* shreg_extract = "no" *)` 属性的可配置流水线寄存器级。

### 6.16 `sync_bits`

用于单比特变化的多比特信号的 CDC 同步器（2 级 FF 同步链）。

### 6.17 `sync_event`

使用基于 toggle 握手机制跨时钟域的事件同步器。

### 6.18 `jesd204_pattern_align`

用于原始 serdes 数据的 10 位模式对齐器。搜索已知的 10 位模式并旋转数据流。

### 6.19 `jesd204_rx_frame_align`

RX 帧对齐监控器。验证 /A/ 和 /F/ 的位置并计数对齐错误。

---

## 7. 信号参考

### 7.1 时钟 / 复位
- `clk`：TX 和 RX 数据路径的单一时钟域。在硅片中这通常是设备时钟；在仿真中，serdes 模型会注入每通道的 skew。

### 7.2 配置
- `cfg_octets_per_multiframe`：一个多帧中的八进制字节数（K）。必须是 DATA_PATH_WIDTH 的倍数。典型值：32。
- `cfg_octets_per_frame`：一个帧中的八进制字节数（F）。
- `cfg_lanes_disable`：每通道禁用（1 = 通道禁用）。
- `cfg_links_disable`：每链路禁用（1 = 链路禁用）。
- `cfg_disable_scrambler`：1 = 旁路加扰/解扰器。
- `cfg_disable_char_replacement`：1 = 不插入 /A/ /F/ 对齐字符。
- `cfg_mframes_per_ilas`：一个 ILAS 序列中的多帧数（M）。
- `cfg_skip_ilas`：1 = 跳过 ILAS，直接从 CGS 进入 DATA。
- `cfg_continuous_cgs`：1 = 保持在 CGS（用于初始上电）。
- `cfg_continuous_ilas`：1 = 连续重复 ILAS。

### 7.3 TX 应用接口
- `tx_valid`：应用层数据有效 strobe。
- `tx_ready`：PCS 准备接收 tx_data（仅在 DATA 阶段）。
- `tx_data`：应用层有效负载，打包格式为 `[lane0_char0, lane0_char1, ..., lane3_char7]`。
- `tx_charisk`：每字符控制标志。1 = 该 8b 字符是控制字符（Kx.y）。

### 7.4 RX 应用接口
- `rx_valid`：本周期 rx_data / rx_charisk 有效。
- `rx_ready`：应用层准备消费 RX 数据。
- `rx_data`：解码后的应用层有效负载。
- `rx_charisk`：每字符控制标志。
- `rx_notintable`：每字符 8b10b 非法表错误。
- `rx_disperr`：每字符 8b10b 极性错误。

### 7.5 Serdes 接口
- `serdes_tx_data`：到 serdes TX 的并行 8b10b 编码符号。按通道打包：`[lane0_sym0, lane0_sym1, ..., lane3_sym7]`。每个符号 10 bit 宽。
- `serdes_rx_data`：从 serdes RX 收到的并行 8b10b 符号。打包格式与 serdes_tx_data 相同。

### 7.6 链路控制 / 状态
- `sync_request_n`：来自远端 RX 设备。低电平有效。由 RX 的 rx_ctrl 驱动：低 = "请发送 CGS"。这是触发 TX 状态机离开 WAIT 并进入 CGS → ILAS → DATA 的输入。
- `status_ctrl_state`：当前 TX 状态机状态。2'b00 = RESET，2'b01 = CGS，2'b10 = ILAS，2'b11 = DATA。
- `status_lane_cgs_state`：每通道 CGS 检测器状态（每通道 2 bit）。
- `status_lane_ifs_ready`：每通道 ILAS 监控器 IFS 就绪。
- `sync_n`：到远端 TX 设备。低电平有效。由本地 RX 的 rx_ctrl 驱动：低 = "我处于 CGS，请继续发送 K28.5"。已注册用于 CDC。

### 7.7 ILAS 配置
- `ilas_config_valid`：单比特有效：通道 i 有有效的 ILAS 配置。
- `ilas_config_addr`：每通道 2 bit 配置地址（0=R/ADJCNT, 1=Q/ADJDIR, 2=F/MF, 3=K/CS）。
- `ilas_config_data`：每通道配置数据。

### 7.8 错误状态
- `status_err_statistics_cnt`：每通道 32 bit 错误计数器。在 8b10b 非法表/极性错误时递增。在每个 event_data_phase（DATA 开始）时复位。
- `status_frame_align_err_cnt_0`：通道 0 的帧对齐错误计数器（每通道 8 bit）。仅第一个通道的内部每通道 frame_align_err_cnt[i] 的镜像，用于快速调试可见性。
- `event_frame_alignment_error`：每通道 frame_align_err_thresh_met 的 sticky OR。当任何使能通道累积的帧对齐错误数 ≥ FRAME_ALIGN_ERR_THRESHOLD 且 ENABLE_FRAME_ALIGN_ERR_RESET=1 时，脉冲为高。这将触发 rx_ctrl 回退到 RESET/CGS。

---

## 8. 仿真验证

该设计使用 2-DUT 点对点测试平台（`jesd204_pcs_link_training_tb`）进行验证：

- **配置**：4 通道，DATA_PATH_WIDTH=8，最大 skew 16 个时钟周期
- **测试**：TX 发送 4 个标记 beat + 256 个有效负载 beat，每通道使用不同的递增数据
- **验证**：RX 捕获所有有效的应用层 beat，定位前导码，并验证每个有效负载字节在所有通道和八进制字节中的正确性
- **结果**：256/256 匹配，0 个不匹配，TX 达到 DATA 状态，RX de-skew 完成

```
2-DUT Test Results (DPW=8):
  Lane skews:  7, 8, 7, 8
  TX beats:    260
  RX beats:    900
  Anchor idx:  133
  Matches:     256
  Mismatches:  0
  Frame-align err cnt (lane 0): 3
  TX state:    11
========================================
SUCCESS: 2-DUT link training + data transfer verified across 4 lanes (DPW=8)
```

---

## 9. 设计注意事项

1. **单一时钟域**：顶层设计在单一时钟域中运行。在硅片中，`clk` 将是设备时钟；在仿真中，serdes 模型会注入每通道的 skew。

2. **低电平有效同步协议**：`sync_request_n`（TX 输入）和 `sync_n`（RX 输出）都是低电平有效。RX 将 `sync_n` 拉低表示它处于 CGS 状态，并请求 TX 继续发送 K28.5。

3. **帧对齐错误恢复**：当 `ENABLE_FRAME_ALIGN_ERR_RESET=1` 时，如果任何通道的帧对齐错误计数器超过 `FRAME_ALIGN_ERR_THRESHOLD`（默认 16），`event_frame_alignment_error` 会产生脉冲，`rx_ctrl` 回退到 RESET/CGS。

4. **弹性缓冲区释放协调**：`buffer_release_n` 由所有通道的协调延迟控制。顶层计算 LMFC 边沿，并且仅当 `all_buffer_ready_n` 被断言时才同时释放所有缓冲区。

5. **DATA_PATH_WIDTH 限制**：该设计主要支持 `DATA_PATH_WIDTH = 4` 和 `8`。对于 `DPW=8` 且 `F*K % 8 == 4` 的情况有特殊处理，此时多帧边界落在 beat 中间。

6. **字符替换与加扰的交互**：当加扰被禁用时，字符替换也被禁用，因为 TX 和 RX 的 `jesd204_frame_align_replace` 逻辑依赖于识别加扰器输出模式的能力。
