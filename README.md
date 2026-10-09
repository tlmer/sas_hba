# SAS HBA — SystemC TLM 参考模型（含 HDD 对端与联合台 ✓）

**SAS HBA**（发起端，寄存器/队列口径兼容公开的 `hisi_sas` v3 风格）的**帧级 SystemC
事务级模型** ✓。本仓**同时带 HDD 对端模型**（`model/hdd/`）与**联合顶层**，因此可直接跑
`HBA ⇄ 链路 ⇄ HDD + 内存` 的**全链路**（7 命令端到端 ✓）；HDD 的**独立仓**见
[`tlmer/sas_hdd`](https://github.com/tlmer/sas_hdd) ✓

- 语言/工具：**SystemC 2.3.4**（`SYSTEMC_HOME` 可覆盖）+ **AMBA-PV**（VDK 自带，header-only）+ C++14
- 主机面：**AMBA-PV**（寄存器从口 + DMA 主口）；SAS 面：帧级 dword 口（SOF/EOF/type + ready）
- 行为基准：主线上游内核驱动口径 + SAS-5 SSP 帧布局（**逐字节**，含边界例）
- 交付含 **4 个自检台（70 条判据）+ `regress.sh`**：`make && sh regress.sh` 一条命令见绿 ✓

## 组成

```
model/hba/    SAS HBA：CSR(AMBA-PV 从口) + DQ 引擎 + DMA(TLM 主口) + SSP 发/收 + CQE/中断
model/hdd/    SAS HDD：SSP 收/发 + SCSI 命令集（TUR/INQUIRY/RCAP/MODE SENSE/…/READ）+ LBA 存储
model/link/   帧级链路：直连 + 可配延迟 / 整帧丢弃（负控注入）
model/top/    sas_pair_top（HBA⇄link⇄HDD + 内存联合顶层）+ sas_mem_tlm（AXI 内存从口）
model/tb/     4 个 TB + TB 助手（AMBA-PV 主机 / 占位主端）
sw/           寄存器/位域/帧常量共同基准（模型·TB·宿主序列共用，纯 C 头）
docs/         设计方案 / 验证方案 / 用户指南
```

## 快速开始

```sh
cd model
make                 # 构建全部 TB
sh regress.sh        # 回归：只认 TB 自带终判行（TB_* PASS）
```
环境变量：`SYSTEMC_HOME`（默认 `/opt/tools/systemc2.34`）、`AMBA_PV_HOME`（指向你的 AMBA-PV）。

最小集成示例见 `docs/user_guide.md`（5 分钟接进平台 + 发第一条 READ 命令）✓

## 判据概览（`docs/verification.md` 全文）

| 台 | 覆盖 | 结果 |
|---|---|---|
| `tb_hba_regs` | 寄存器面：直连/AMBA-PV 两路同源、DECERR、RO 忽略、W1C、PHY 段 | 10/10 ✓ |
| `tb_pair_bringup` | **HBA⇄link⇄HDD 全链路**：7 条命令（TUR/INQUIRY(+LUN 过滤)/MODE SENSE 64B/READ 2048B 多帧/未知 opcode/REQ SENSE）+ CQE/中断/W1C | 23/23 ✓ |
| `tb_neg` | 负控：丢帧⇒超时⇒错误 CQE；「无 EOI」口径复现；DMA 越界⇒错误 CQE | 12/12 ✓ |
| `tb_hdd_scsi` | HDD 字节级：各命令载荷/长度/字节序、RCAP 编码、多帧 DataOffset | 25/25 ✓ |

## 许可与声明

参考模型，**不替代 RTL/硅验证**；模型范围与明写简化见 `docs/design.md` §6 ✓
