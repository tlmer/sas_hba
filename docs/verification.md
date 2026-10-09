# SAS HBA + HDD TLM 模型 — 验证方案

[2026-10-08 建] 口径照先例 `rdma400/tlm` ✓：**只认 TB 自带终判行**（`TB_<NAME> PASS`）；
`regress.sh` 只收集/汇总、不重新解释判据 ✓；`make -q` 陈旧二进制守卫 ✓。

---

## 1. 怎么跑

```sh
cd sas_hba/tlm/model && make && sh regress.sh      # 3 台（regs / pair_bringup / neg）
cd sas_hdd/tlm/model && make && sh regress.sh      # 1 台（hdd_scsi）
```
环境变量可覆盖：`SYSTEMC_HOME`（默认 `/opt/tools/systemc2.34`）、
`AMBA_PV_HOME`（默认 VDK 2024.03 路径）、`HDD_DIR`（默认 `../../../sas_hdd/tlm`）✓。
单独跑一台：`./tb/tb_pair_bringup`（退出码 0 = 全 PASS ✓）。

## 2. 台清单与判据（**预登记**；期望值出处 = RTL/内核实证，逐条注在台头 ✓）

### 2.1 `tb_hba_regs`（10 条）— HBA 寄存器面

| 判据 | 内容 |
|---|---|
| C1 | 直连读写往返：DLQ_EN / DQ0 base+depth+wr_ptr / CQ0 / CFG_MAX_TAG ✓ |
| C2 | **两路同源**：socket 写 ⇒ 直连读一致；直连写 ⇒ socket 读一致 ✓ |
| C3 | 未映射（0x1000 空洞）读/写 ⇒ **DECERR** ✓（★ 本用例曾抓出模型 PHY 段译码下界缺失 ⇒ 段错误 ✓）|
| C4 | RO 写（CQE_SEND_CNT）⇒ OKAY + 值不变 + `ro_wr_cnt`+1 ✓ |
| C5 | DQ `rd_ptr` 写 ⇒ 忽略（硬件推进 ⇒ 软件只读 ✓）|
| C6 | PHY0 `CHL_INT0` 复位 bit2=1 → 写 bit2（W1C）⇒ 清 0 ✓；`CHNL_INT_STATUS` bit0 随之落 ✓ |
| C7 | PHY `TX_ID_DW0..6`（7 个）+ `PHY_CFG` 读写往返 ✓ |
| C8 | 段内未列地址（0x0F0）⇒ 读 0 且 OKAY（RTL case default 语义 ✓）|
| C9 | 空闲期 `irq` 恒低 ✓ |
| C10 | `reg_sock_n = 5`（成功 socket 访问计数；DECERR 不计 ✓）|

### 2.2 `tb_pair_bringup`（23 条）— ★ **HBA ⇄ link ⇄ HDD 交互性/全链路**

7 条命令（TUR / INQUIRY / MODE SENSE / **READ(10) 4 块=2048B** / 未知 opcode / REQ SENSE / **INQUIRY LUN≠0**）
经**寄存器 + 内存编程**驱动（DQ 写条目 + 命令表 + SGE ⇒ 门铃），判据：

| 组 | 内容 |
|---|---|
| A1 | 复位后 `CHNL_INT_STATUS` bit0 = PHY0 `SL_PHY_ENABLE` ✓ |
| A2 | TUR：CQE `iptt=1/dev_id=1/cmplt=0/rspns_good=1` ✓（依参考 RTL 行为 ✓）|
| A3 | INQUIRY 数据 36B：byte4=**32** ✓、vendor/product 逐字 ✓（差异注 D1 已闭 ✓）|
| A4 | MODE SENSE(6)：**64B**、byte0=0x3F/4=0x08/5=0x12 ✓（缺陷# ✓）|
| A5 | READ(10) 2048B：**数据逐字节 == 源模式** ✓、**多 DATA 帧**（DataOffset 递增 ✓）、`hdd.o_lba_read=4` ✓、HBA 收帧数 ≥6 ✓ |
| A6 | 未知 opcode ⇒ RESPONSE byte35=0x02 ⇒ CQE `cmplt=3` ✓；REQ SENSE 数据 0x70/0x0A/key/ASC ✓ |
| A7 | LUN≠0 的 INQUIRY ⇒ byte0 PQ=011b ✓（缺陷# ✓）|
| A8 | CQE 后 `irq=1`、`o_irq_cnt≥1` ✓；**W1C 清 ⇒ irq 落低**（内核口径 ✓）|
| A9 | 状态缓冲 48B：`dw0 byte0=0x40`、byte35=0x00 ✓ |
| A10 | `o_tag_mismatch=0`、`data_drop_cnt=0`、`hba.o_cmds_err=1`（仅未知 opcode）、`hdd.o_cmds_good=6/o_cmds_chk=1` ✓ |
| A11 | `link.o_dropped=0` ✓ |
| A12 | 7 条 CQE 的 iptt **顺序** = 1..7 ✓ |

### 2.3 `tb_neg`（12 条）— 负控（每组成对可判定 ✓）

| 组 | 内容 |
|---|---|
| B1 | **链路丢帧**（`cfg_drop_every=1` 全丢）⇒ HBA 等响应超时 ⇒ **错误 CQE**（`cmplt=3`、`rspns_xfrd=0` ✓）—— 复现参考平台「帧发出去没回来」卡点场景（本参考实现 缺陷#/缺陷# ✓）；HDD 侧 `o_rx_frames=0`（真没收到 ✓）；`link.o_dropped≥1` ✓ |
| B2 | **RTL 无 EOI 复现**（`cfg_rtl_no_eoi=1` 红线②）：CQE 后 irq=1 ✓；W1C 写 ⇒ **irq 仍高** ✓ + `ro_wr_cnt` 增 ✓（对照组：内核口径 W1C 真清 ⇒ irq 落低 ✓）|
| B3 | **DMA 越界**（DQ 基址改窗外）⇒ DECERR ⇒ 错误 CQE ✓、`dma_fail_cnt` 增 ✓、**iptt 不留残值（=0 ✓）** |

### 2.4 `tb_hdd_scsi`（25 条，在 `sas_hdd/tlm`）— HDD **字节级**专项

直驱 rx/采 tx（无 HBA/链路 ✓），判据 D1..D10：TUR / INQUIRY（36B、byte4=32、vendor/product、PQ=0）/
INQUIRY LUN≠0（PQ=3）/ MODE SENSE 64B / **RCAP10**（末 LBA BE32@0-3、块长 BE32@4-7 ✓）/ REPORT LUNS /
**READ 2048B**（3 DATA 帧、DataOffset=0/1000/2000、载荷逐字节 ✓）/ 未知 opcode（CHECK CONDITION）/
START STOP·SYNC CACHE·MODE SELECT / 计数（good=10、chk=1、lba_read=4）✓

## 3. 判据口径（十条纪律）

1. **预登记**：判据在台头先写死（含出处），跑完只对照 ✓
2. **终判行**：每台最后一行 `TB_<NAME> PASS|FAIL`；`regress.sh` 只认它 ✓
3. **陈旧二进制守卫**：`make -q` 检查后再跑 ✓
4. **每拍一步**：模型全部 `SC_METHOD@clk.pos`；TB 逐拍采样 ✓
5. **sink 不静默**：丢弃/失败/失配全部计数（`data_drop_cnt`/`dma_fail_cnt`/`o_tag_mismatch`/`o_dropped` ✓）
6. **不采半拍值**：读电平/计数前留一拍（`tick(2)` ✓ 实踩：irq 刷新滞后一拍 ✗）
7. **地址图无重叠**：TB 内存分区（DQ/CQ/CT/SGE/DAT/STS）分段独立 ✓（实踩：CT 伸进 SGE 区 ⇒ 数据全空 ✗）
8. **CDB 字节位置对**：LBA 大端（READ10 的 LSB 在 byte5 ✓ 实踩写错位 ⇒ 读错 LBA ✗）
9. **端口绑齐**：模型每拍写 `o_*` ⇒ 必须绑；AMBA-PV 从口**恰好绑 1**（VDK 此版 = ONE，多绑 E109 ✗）
10. **改动即重跑**：改模型/`sw` 后两工程 `regress.sh` 都必须全绿才允许提交 ✓

## 4. 回归快照（2026-10-08）

```
HBA：tb_hba_regs PASS 10/0 · tb_pair_bringup PASS 23/0 · tb_neg PASS 12/0  ⇒ 全部 PASS ✓ (3/3)
HDD：tb_hdd_scsi PASS 25/0                                                 ⇒ 全部 PASS ✓ (1/1)
合计 70 条判据全绿 ✓
```
（对拍基准与差异见《一致性核对报告》✓）
