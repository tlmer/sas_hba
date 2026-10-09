# SAS HBA + HDD TLM — 用户指南（集成与用例）

[2026-10-08 建]

---

## 1. 最小集成（5 分钟）

```cpp
#include <systemc.h>
#include "sas_pair_top.h"        // -Imodel/top

int sc_main(int argc, char **argv) {
    sc_core::sc_clock clk("clk", 10, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;

    SasPairTop pair("pair",          // HBA ⇄ link ⇄ HDD + 内存 ✓
                    0xfe400000ull,   // 内存窗基址（DMA 目标 ✓）
                    0x01000000ull,   // 16MB
                    4096);           // 盘容量（块 ✓）
    pair.clk(clk); pair.rst_n(rst_n);

    rst_n.write(false);
    sc_core::sc_start(200, sc_core::SC_NS);   // 20 拍复位
    rst_n.write(true);
    sc_core::sc_start();
    return 0;
}
```
编译（两个工程合编一个台 ✓）：
```sh
g++ -std=c++14 -I<sc>/include -I<amba_pv>/include \
    -Itlm/model/hba -Itlm/model/link -Itlm/model/top -Itlm/model/tb -Itlm/sw \
    -Ihdd_tlm/model/hdd my_tb.cpp \
    tlm/model/hba/sas_hba_tlm.cpp tlm/model/link/sas_link_tlm.cpp \
    hdd_tlm/model/hdd/sas_hdd_tlm.cpp \
    -L<sc>/lib-linux64 -lsystemc -Wl,-rpath,<sc>/lib-linux64 -o my_tb
```

## 2. 两个面怎么接

| 面 | 接法 |
|---|---|
| **CSR**（HBA 寄存器）| ① 平台里 `ApvMaster/*你的 master*`.m.bind(pair.hba.reg_s)（AMBA-PV ✓ **恰好绑 1 个主端** ✗ 多绑 E109）；② 或直接 `pair.hba.reg_write(addr, val)`（同源 ✓ 无需 socket；此时须绑一个占位件 `DummyAmbaMaster`）|
| **DMA** | 已接好：`hba.dma_s ⇄ mem.s` ✓；要换自己的内存模型 ⇒ 把 `SasPairTop` 里的 `mem` 换成你的从口（接口：`amba_pv_slave_base<32>` ✓）|
| **SAS** | 已接好：`hba ⇄ link ⇄ hdd` ✓；要单测 HBA 可只例化 `hba + link`，B 侧自己驱动帧 ✓ |

## 3. 发一条命令（完整流程模板）

```cpp
#include "sas_hba_hw.h"     // DQ/SGE/命令表 打包助手 ✓
// ① 寄存器编程（一次）
pair.hba.reg_write(SAS_HBA_CFG_DLVRY_QUEUE_ENABLE, sc_uint<32>(1));      // 使能 DQ0
pair.hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), sc_uint<32>(DQ & 0xffffffffu));
pair.hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_DEPTH),  sc_uint<32>(8));
pair.hba.reg_write(SAS_HBA_CQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), sc_uint<32>(CQ));
pair.hba.reg_write(SAS_HBA_CQ_REG(0, SAS_HBA_XX_DEPTH),  sc_uint<32>(8));
pair.hba.reg_write(SAS_HBA_INT_OQ_SRC_MSK, sc_uint<32>(0));              // 解掩（0 = 放行）

// ② 内存准备（命令表 + SGE + DQ 条目）
uint8_t cdb[16] = {0x28,0,0,0,0,0x10,0,0,0x01,0};      // READ10 @LBA 0x10 1 块
uint8_t lun[8]  = {0};
uint8_t tbl[SAS_HBA_CTBL_SSP_BYTES];                    // 1KB 命令表
sas_ctbl_pack_ssp(tbl, cdb, lun, /*tag*/0, /*tptt*/0, /*dest*/0);
pair.mem.poke(CT_ADDR, tbl, sizeof(tbl));
uint8_t sge[SAS_HBA_SGE_BYTES];                         // 首 SGE：DMA 目标 ✓
sas_sge_pack(sge, DAT_ADDR, 512, 0);
pair.mem.poke(SGE_ADDR, sge, sizeof(sge));
uint32_t dqw[SAS_HBA_DQ_ENTRY_DW];                      // 64B DQ 条目
sas_dq_pack_ssp(dqw, /*iptt*/1, /*dev_id*/1, SAS_HBA_DIR_TO_INI, 512, CT_ADDR, SGE_ADDR);
pair.mem.poke(DQ_BASE + 0, dqw, sizeof(dqw));

// ③ 门铃（wr_ptr++）
pair.hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_WR_PTR), sc_uint<32>(1));
// ④ 等 CQE（o_cqes 或轮询内存里的 CQE ✓）
while (pair.hba.o_cqes.read().to_uint() < 1) wait(clk.posedge_event());
uint32_t cqe[4]; pair.mem.peek(CQ, cqe, 16);
// sas_cqe_iptt/deivid/cmplt/rsp_good(...) 解码 ✓
```

## 4. 常用观测口（白盒，判据用 ✓）

| 口 | 含义 |
|---|---|
| `hba.o_cqes / o_cmds_done / o_cmds_err` | 完成/成功/错误计数 ✓ |
| `hba.o_irq_cnt`、`hba.irq` | 中断上升沿数 / 电平 ✓ |
| `hba.o_tag_mismatch`、`data_drop_cnt`、`dma_fail_cnt` | 异常计数（不静默 ✓）|
| `hba.o_last_iptt / o_last_ssp_status` | 最近一条命令的 iptt / SSP status ✓ |
| `hdd.o_cmds_good / o_cmds_chk / o_lba_read / o_last_opcode` | 盘侧计数 ✓ |
| `link.o_dropped`、`link.aq_occ_now()/bq_occ_now()` | 链路丢弃数 / 在途拍数 ✓ |

## 5. 配置旋钮（TB/集成可调 ✓）

| 项 | 默认 | 说明 |
|---|---|---|
| `hba.cfg_fetch_lat / cfg_ct_lat` | 4 / 8 | DQ 条目 / 命令表 读延迟（拍 ✓）|
| `hba.cfg_resp_timeout` | 4096 | 等响应超时（拍 ⇒ 错误 CQE ✓）|
| `hba.cfg_rtl_no_eoi` | 0 | 1 = 复现「无 EOI」RTL 口径（W1C 无效 ✓）|
| `hba.cfg_addr_mask` | 0xFFFFF | CSR 地址窗掩码 ✓ |
| `link.cfg_delay` | 1 | 链路在途拍数 ✓ |
| `link.cfg_drop_every` | 0 | N>0 ⇒ 每 N 帧丢 1 帧（负控 ✓）|
| `hdd.cfg_blocks`（构造参数 ✓）| 1024 | 盘容量（512B/块 ✓）|

## 6. 常见坑（实测清单 ✓）

1. **从口只能绑 1 个主端**（VDK 此版 ONE 策略）⇒ 不用 socket 的台要绑 `DummyAmbaMaster` ✓
2. 模型每拍写 `o_*` ⇒ **必须绑信号**，否则 elaboration 报错 / 运行崩 ✓
3. 读 `irq` 电平前**留一拍**（刷新滞后 ✓）
4. AMBA-PV 事务用 `pool.allocate(1, len, NULL, AMBA_PV_INCR)`（参数序照 BL ✓）
5. DMA 走**零时**b_transport ⇒ 内存模型别回非零延迟（或调用方用 SC_THREAD ✓）
6. HBA 的 `dir` 语义：1=读(TO_INI)、2=写(TO_DEVICE)、0=无数据 ✓（DQ dw1[6:5] ✓）
