# SAS HBA + HDD TLM 模型 — 设计方案

[2026-10-08 建] 本文 = **对外交付件的设计规格**（与 `sw/*.h`、各模型头注释互为镜像，改动必须三处同步 ✓）。

---

## 1. 范围与定位

| 项 | 内容 |
|---|---|
| 做什么 | 一对**事务级/帧级** SystemC 模型：**SAS HBA**（hisi_sas v3 兼容发起端）+ **SAS HDD**（SSP 目标/盘），可单独或成对仿真 ✓ |
| 给谁用 | 系统级仿真（VDK/QEMU 协同、驱动 bring-up 预演、协议栈验证）；**不替代 RTL** ✓ |
| 依据 | 本参考实现 SAS RTL **已被参考平台的波形/采样记录 逐拍验证** ⇒ 直接作为模型的规格与对拍真值 ✓（出处逐条注在代码里 ✓） |
| 技术栈 | SystemC 2.3.4 + AMBA-PV（VDK 自带，header-only）+ g++ C++14 ✓ |
| 精度 | 帧级（dword 逐拍）+ 寄存器级；**无模拟时序/无 PHY** ✓ |

## 2. 目录与构件

```
sas_hba/tlm/
  model/hba/sas_hba_tlm.{h,cpp}    # HBA：CSR(AMBA-PV 从口) + DQ 引擎 + DMA(TLM 主口) + SSP 发/收
  model/link/sas_link_tlm.{h,cpp}  # SAS 链路：帧级直连 + 可配延迟/整帧丢弃（负控）
  model/top/sas_pair_top.h         # HBA ⇄ link ⇄ HDD + 内存 联合顶层
  model/top/sas_mem_tlm.h          # 通用 AXI 内存（AMBA-PV 从口，byte 阵列）
  model/tb/{apv_master,dummy_amba_master}.h   # TB 助手（真主端 / 占位主端）
  model/tb/tb_*.cpp                # 4 个 TB（见《验证方案》）
  sw/sas_hba_regs.h                # ★ 寄存器/DQ/CQE/SGE/帧常量（逐条带 RTL 出处）
  sw/sas_hba_hw.h                  # 序列层：DQ 条目/SGE/命令表/CQE/帧 编解码（纯函数）
sas_hdd/tlm/
  model/hdd/sas_hdd_tlm.{h,cpp}    # HDD：SSP 收/发 + SCSI 命令集 + 内存 LBA 存储
  model/tb/tb_hdd_scsi.cpp         # 字节级专项台
  sw/sas_hdd_regs.h                # HDD 侧帧常量 + 自有 CSR 表
```

## 3. HBA 模型

### 3.1 三个面（接口）

| 面 | 形式 | 说明 |
|---|---|---|
| 主机面 | `amba_pv::amba_pv_slave_socket<32> reg_s` + 直连 `reg_read/reg_write` | **同源**：socket 的 `read()/write()` 回调直接转调直连函数 ⇒ 两路逐寄存器一致 ✓（先例 `rdma400/tlm` ✓） |
| 内存面 | `amba_pv::amba_pv_master_socket<32> dma_s` | DQ 条目 / 命令表 / SGE / 数据 / CQE 全走 DMA；**零时 `b_transport`**、延迟自记拍数 ⇒ 可从 `SC_METHOD` 调用 ✓ |
| SAS 面 | 帧级 pin 口（`tx_*` / `rx_*`：data/valid/sof/eof/type + ready） | 与 `sas_link_tlm`、HDD 模型**同构** ✓（32-bit dword/拍 ✓） |

**地址口径**：CSR 低 20 位译码（`cfg_addr_mask`，默认 `0xFFFFF` ✓ 兼容任意映射基址）；段 = GLOBAL `<0x260` / DQ `<0x4E0` / CQ `<0x760` / PHY `0x2000..0x4400` / AXI_MASTER `0x5000..` / RAS `0x6000..` ✓（`sas_hba_regs.h` §1，出处见参考 RTL ✓）。未映射 ⇒ **DECERR**；只读写 ⇒ **OKAY + 忽略 + 计数**（照 RTL 静默语义 ✓）。

### 3.2 寄存器实现面

- **GLOBAL**：`DLVRY_QUEUE_ENABLE(0x000)`、IOST/ITCT 基址、`CFG_MAX_TAG(0x068)`、中断组（`OQ_INT_SRC(0x1B0)` / `MSK(0x1B4)` / ENT 组 / coalescing）、`CQE_SEND_CNT(0x248 只读)` ✓
- **DQ/CQ 各 32 组**：BASE_LO/HI、DEPTH、WR_PTR（软件写）、RD_PTR（硬件推进 ⇒ 软件只读 ✓）
- **PHY ×9**：CFG/LINKRATE/CTRL/SL_*/TX_ID_DW0..6/TXID_AUTO/RX_IDAF/CHL_INT0..2/MSK/ERR_CNT ✓；
  `CHL_INT0` = **全线唯一 W1C**（bit2 SL_PHY_ENABLE ✓ `regfile）；复位默认 bit2=1（点对点链路已建 ✓）
- **AXI_MASTER / RAS**：控制 + 掩码组 ✓

### 3.3 命令流（每拍一步的 `SC_METHOD` FSM）

```
H_IDLE ─(DLVRY_QUEUE_ENABLE[i] && wr_ptr≠rd_ptr)→ 读 DQ 条目 64B（4 拍 ✓）
 → H_PARSE（位域解析：iptt/dir/xfer_len/cmd_tbl/prd_tbl/sts_buf ✓）
 → 读命令表 64B（含 CDB@36 ✓）→ H_ISSUE（组 14 dword COMMAND 帧发出 + 取首 SGE 24B ✓）
 → [写方向：从 SGE 切帧发 DATA ✓] → H_WAIT_RESP
      ├ DATA 帧 ⇒ 按 DataOffset 落内存（SGE 目标 ✓；多帧自动拼 ✓）
      └ RESPONSE 帧 ⇒ 取 status（tag 须匹配 ✓；响应原文写 sts_buffer ✓）
 → H_CQE_WR：写 16B CQE 到 CQ 环 + 置 OQ pending + irq + rd_ptr++ ✓
```
- **CQE 布局**（依参考 RTL 行为 ✓）：`dw0[1:0]=cmplt`（0 正常/3 错）、`dw0[10]=rspns_xfrd`、`dw0[11]=rspns_good`、`dw1[15:0]=iptt`、`dw1[31:16]=dev_id` ✓
- **中断**：简化 = 一根电平线 `irq`（参考平台 MSI 两向量取 OR ✓）；**默认实现内核期望的 W1C**；`cfg_rtl_no_eoi=1` 可复现 RTL「无 EOI」（红线②：`intr_ack` 恒 0 ⇒ 写无效 ✓）
- **错误路径**（全部出错误 CQE，不静默 ✓）：等响应超时（`cfg_resp_timeout` ✓ 复现参考平台"帧发出去没回来"卡点场景 ✓）、DQ 读失败（DECERR）、命令表读失败、非 SSP 命令（SMP/STP/TMF）
- **配对口径**：逻辑 DQ i ↔ CQ i ✓（`dq_id↔cq_id` ✓）

### 3.4 SSP 帧（线上字节口径，**双侧逐字一致** ✓）

| 帧 | 长度 | 布局（帧字节 k = bits[8k+7:8k] ✓） |
|---|---|---|
| COMMAND | 14 dw | byte0=0x10；**tag@16-17（be16）**；**LUN@24-31**；**CDB@36-51**（9 头+4 CDB+1 CRC ✓）|
| DATA | 6 dw 头 + ≤250 dw 载荷 | byte0=0x60；DW4={0,tag be16}；DW5=**DataOffset be32** ✓ |
| RESPONSE | 12 dw（含 CRC 槽）| byte0=0x40；DW4 tag（be16）；**byte35=STATUS**；DW10=SENSE LEN（现状恒 0 ✓）|
| XFER_RDY | 4 dw | byte0=0x50 ✓（本工程读路径直发 DATA ⇒ 模型忽略 ✓）|
| OPEN | 6 dw | byte0=0x30（项目专有编码 ✓；TLM 点对点不发 ✓）|

> ⚠ 帧型半字节（侧带 `rx_type`/`tx_type`）：1=COMMAND 6=DATA 4=RESPONSE 5=XFER_RDY ✓；
> HBA RTL 的 `data_wr_fsm` 用 byte0[3:0] 与其余全部 [7:4] 相反（**模型一律 [7:4]** ✓ 差异注 D2 ✓）。

## 4. HDD 模型

- **收侧**：帧装配（SOF 锁存帧型 ✓）→ COMMAND ⇒ 取 tag（be16 ✓）与 LUN（帧字节 24-31 ✓）→ 执行
- **SCSI 命令集**（逐条照 RTL + 参考平台 采样记录 ✓）：
  | 命令 | 行为 |
  |---|---|
  | TUR(0x00) | GOOD、无数据 ✓ |
  | INQUIRY(0x12) | 36B：byte4=**32**（依参考 RTL 行为 ✓）、vendor `-UME MIS`、product `-SAS-DHVM652` ✓；**LUN≠0 ⇒ PQ=3（byte0=0x60 ✓ 缺陷#）** |
  | MODE SENSE(0x1A/0x5A) | **64B**：byte0=0x3F、byte4=0x08、byte5=0x12 ✓（缺陷# ✓）|
  | REQUEST SENSE(0x03) | **64B**：0x70 + 附加长度 0x0A ✓；可注入 key/ASC ✓ |
  | REPORT LUNS(0xA0) | **64B**、byte3=0x08（BE 长度 ✓ 缺陷#）|
  | READ CAPACITY(0x25/0x9E) | 8B：末 LBA **BE32@0-3** + 块长 **BE32@4-7**（512 ✓）；16 ⇒ 8B+4B ✓ |
  | READ(6/10/16) | DATA-in：按 CDB 取 LBA/块数 ✓；**长读自动分帧**（≤1000B/帧、DataOffset 递增 ✓）|
  | WRITE(6/10/16) | 立即 GOOD（**不消费 DATA-out** ✗ 明写简化 ✓）|
  | START STOP / SYNC CACHE / MODE SELECT / FORMAT(0x04) | GOOD、无数据 ✓ |
  | 未知 opcode | **CHECK CONDITION**（byte35=0x02 ✓ 缺陷#）；sense 经 REQ SENSE 提供 ✓ |
- **发侧**：DATA/RESPONSE 帧队列 + held-valid 逐拍 ✓
- **存储**：内存 LBA 阵列（构造期定容量 ✓）+ TB 钩子 `lba_ptr()` / `set_sense()` ✓

## 5. 链路与内存

- `sas_link_tlm`：帧级**全双工**直连；`cfg_delay`（在途拍数，默认 1 ✓）；`cfg_drop_every=N`（**每 N 帧整帧丢弃** = NAK 负控 ✓）
- `sas_mem_tlm`：`[base, base+size)` 窗口 byte 阵列；越界 ⇒ DECERR ✓；写支持 byte_enable ✓；**默认零延迟**（`SC_METHOD` 调用安全 ✓）

## 6. ⛔ 明写简化清单（不做的事）

1. **PHY/链路训练/拓扑发现/速率协商**（点对点 ✓）；PHY 段寄存器可读写但 CHL_INT0 只有 bit2=1
2. **SSP OPEN/CLOSE/TASK(TMF)** 不发；**SMP/STP 不实现**（⇒ 错误 CQE ✓）
3. **SGE 只取首项**（多 SGE 连续映射 = 扩展点 ✓）；**单命令在飞**（无并发 ✓）
4. **中断** = 一根电平线；**无 coalescing 定时器**；W1C/无-EOI 双口径可配 ✓
5. **DMA 延迟简化**（零时 b_transport + 自记拍数；RTL 的 63 拍命令表取指 ⇒ 模型 8 拍 ✓）
6. HDD 侧 **WRITE 不落盘**、无介质错误注入、无 DIF/PI、无 NCQ/STP
7. 帧的**目的 SAS 地址哈希**填 0（点对点不看 ✓；`sw` 里给了哈希函数供逐字节对拍 ✓）

## 7. 版本与文件基点

- 版本宏：HBA `SAS_HBA_TLM_VERSION_STR`、HDD `SAS_HDD_TLM_VERSION_STR` ✓
- **改 `sw/*.h` 必须同步本文件与两处头注释** ✓；改行为必须补《验证方案》判据 ✓
