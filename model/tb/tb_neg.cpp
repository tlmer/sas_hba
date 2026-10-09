//============================================================================
// tb_neg.cpp — 负控台（三组，全部**成对可判定** ✓）  [2026-10-08 建]
//   结构：SasPairTop（真 HBA + 真 link + 真 HDD ✓）
//
//   判据（**预登记**）：
//   B1  链路丢帧（link.cfg_drop_every=1 ⇒ 全丢）：HBA 发 COMMAND 但对侧收不到 ⇒
//       等响应超时 ⇒ **错误 CQE**（cmplt=3、rspns_xfrd=0 ✓ —— 复现参考平台「帧发出去
//       没回来」卡点场景 ✓ 本参考实现 缺陷#/缺陷# ✓）；HDD 侧 o_rx_frames=0 ✓（真没收到）
//   B2  **RTL 无 EOI 复现**（cfg_rtl_no_eoi=1 红线②）：CQE 后 irq=1 ✓；
//       W1C 写 OQ_INT_SRC ⇒ **irq 仍高** ✓ 且 ro_wr_cnt 增（写被忽略 ✓）
//       （对照组：B2 前置段先按内核口径 W1C 真清 —— irq 落低 ✓，两口径分明 ✓）
//   B3  DMA 越界（DQ 基址改到窗口外 0x10000000）：读条目 DECERR ⇒ **错误 CQE** ✓
//       且 dma_fail_cnt ≥1 ✓（不静默 ✗✓）
//============================================================================
#include <systemc.h>
#include "sas_pair_top.h"
#include "dummy_amba_master.h"
#include "sas_hba_hw.h"
#include <cstdio>
#include <cstring>

static const uint64_t MEM_BASE = 0xfe400000ull;
static const uint64_t DQ_BASE  = 0xfe440000ull;
static const uint64_t CQ_BASE  = 0xfe441000ull;
static const uint64_t CT_BASE  = 0xfe442000ull;
static const uint32_t DQ_DEPTH = 8, CQ_DEPTH = 8;

struct TbNeg : sc_core::sc_module {
    sc_core::sc_in<bool>  clk;
    sc_core::sc_out<bool> rst_n;
    SasPairTop *pair = nullptr;
    int pass_cnt = 0, fail_cnt = 0;

    SC_HAS_PROCESS(TbNeg);
    TbNeg(sc_core::sc_module_name nm) : sc_module(nm), clk("clk"), rst_n("rst_n") { SC_THREAD(run); }
    void tick(int n = 1) { for (int i = 0; i < n; i++) { wait(clk.posedge_event()); wait(sc_core::SC_ZERO_TIME); } }
    void chk(bool ok, const char *name) {
        printf("   %s %s\n", ok ? "✓" : "✗", name);
        if (ok) pass_cnt++; else fail_cnt++;
    }
    void issue_tur(int idx, uint16_t iptt) {
        uint8_t tbl[SAS_HBA_CTBL_SSP_BYTES];
        uint8_t cdb[16] = {0x00}, lun[8] = {0};
        sas_ctbl_pack_ssp(tbl, cdb, lun, 0, 0, 0);
        pair->mem.poke(CT_BASE + (uint64_t)idx * 0x400, tbl, sizeof(tbl));
        uint32_t dqw[SAS_HBA_DQ_ENTRY_DW];
        sas_dq_pack_ssp(dqw, iptt, 1, SAS_HBA_DIR_NO_DATA, 0, CT_BASE + (uint64_t)idx * 0x400, 0);
        pair->mem.poke(DQ_BASE + (uint64_t)idx * 64, dqw, sizeof(dqw));
    }
    uint32_t cqe_iptt(uint64_t cqe_index) {
        return pair->mem.rd32(CQ_BASE + cqe_index * 16 + 4) & 0xFFFFu;
    }
    uint32_t cqe_dw0(uint64_t cqe_index) { return pair->mem.rd32(CQ_BASE + cqe_index * 16); }
    void wait_cqes(uint32_t n, int cap = 50000) {
        while (pair->hba.o_cqes.read().to_uint() < n && cap-- > 0) tick(1);
    }

    void run() {
        printf("[TB] 预登记判据 B1..B3（负控 ✓）\n");
        pair->mem.fill(0x00);
        tick(4);
        while (!rst_n.read()) tick(1);
        tick(4);

        // 基础编程（DQ/CQ/掩码 ✓）
        pair->hba.reg_write(SAS_HBA_CFG_DLVRY_QUEUE_ENABLE, sc_uint<32>(1));
        pair->hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), sc_uint<32>(DQ_BASE & 0xFFFFFFFFu));
        pair->hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_DEPTH), sc_uint<32>(DQ_DEPTH));
        pair->hba.reg_write(SAS_HBA_CQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), sc_uint<32>(CQ_BASE & 0xFFFFFFFFu));
        pair->hba.reg_write(SAS_HBA_CQ_REG(0, SAS_HBA_XX_DEPTH), sc_uint<32>(CQ_DEPTH));
        pair->hba.reg_write(SAS_HBA_INT_OQ_SRC_MSK, sc_uint<32>(0));      // 解掩 ✓

        // ── B1 丢帧 ⇒ 超时 ⇒ 错误 CQE ──
        pair->link.cfg_drop_every = 1;                // 全丢 ✓
        pair->hba.cfg_resp_timeout = 500;             // 缩短超时 ✓
        issue_tur(0, 1);
        tick(2);
        pair->hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_WR_PTR), sc_uint<32>(1));
        wait_cqes(1);
        {
            uint32_t dw0 = cqe_dw0(0);
            chk(cqe_iptt(0) == 1, "B1 超时后仍出 CQE（iptt=1 ✓ 不静默 ✓）");
            chk((dw0 & 0x3u) == SAS_HBA_CQE_CMPLT_ERR, "B1 CQE cmplt=3（错误 ✓）");
            chk(((dw0 >> 10) & 1u) == 0u, "B1 rspns_xfrd=0（响应从没来过 ✓）");
            chk(pair->hba.o_cmds_err.read().to_uint() == 1u, "B1 hba.o_cmds_err = 1");
            chk(pair->hdd.o_rx_frames.read().to_uint() == 0u, "B1 HDD 侧 o_rx_frames=0（真没收到 ✓）");
            chk(pair->lk_dropped.read().to_uint() >= 1u, "B1 link.o_dropped ≥ 1");
            printf("   [dbg B1] err=%u done=%u cqes=%u | cqe0.iptt=%u dw0=%08x | cqe1.iptt=%u dw0=%08x | cqe2.iptt=%u | rd/tx=%u\n",
                   pair->hba.o_cmds_err.read().to_uint(), pair->hba.o_cmds_done.read().to_uint(),
                   pair->hba.o_cqes.read().to_uint(), cqe_iptt(0), cqe_dw0(0), cqe_iptt(1), cqe_dw0(1),
                   cqe_iptt(2), pair->hba.o_dq_fetches.read().to_uint());
        }
        pair->link.cfg_drop_every = 0;                // 恢复 ✓
        pair->hba.cfg_resp_timeout = 4096;

        // ── B2 RTL 无 EOI 复现（前段=内核口径对照组 ✓）──
        pair->hba.reg_write(SAS_HBA_INT_OQ_SRC, sc_uint<32>(0xFFFFFFFFu));  // 内核口径：真清 ✓
        tick(2);
        chk(!pair->hba.irq.read(), "B2 前置：内核口径 W1C 真清 ⇒ irq 落低（对照 ✓）");

        pair->hba.cfg_rtl_no_eoi = 1;                 // ★ 切 RTL 口径 ✓
        pair->hba.cfg_resp_timeout = 4096;
        issue_tur(1, 2);
        tick(2);
        pair->hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_WR_PTR), sc_uint<32>(2));
        wait_cqes(2);
        {
            tick(2);                              // ⚠ irq 电平在置位后下一拍刷新（读取须留一拍 ✓）
            unsigned long ro0 = pair->hba.ro_wr_cnt;
            chk(pair->hba.irq.read(), "B2 RTL 口径：CQE 后 irq=1 ✓");
            pair->hba.reg_write(SAS_HBA_INT_OQ_SRC, sc_uint<32>(0xFFFFFFFFu));  // 应被忽略 ✓
            tick(2);
            chk(pair->hba.irq.read(), "B2 RTL 无 EOI：W1C 后 irq 仍高 ✓（红线② 复现 ✓）");
            chk(pair->hba.ro_wr_cnt > ro0, "B2 该写被计为 RO 忽略 ✓（ro_wr_cnt 增 ✓）");
        }
        pair->hba.cfg_rtl_no_eoi = 0;
        pair->hba.reg_write(SAS_HBA_INT_OQ_SRC, sc_uint<32>(0xFFFFFFFFu));     // 清干净 ✓
        tick(2);

        // ── B3 DMA 越界 ⇒ 错误 CQE ──
        issue_tur(2, 3);
        tick(2);
        pair->hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), sc_uint<32>(0x10000000u));  // 窗外 ✓
        pair->hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_WR_PTR), sc_uint<32>(3));
        {
            unsigned long f0 = pair->hba.dma_fail_cnt;
            wait_cqes(3);
            chk(cqe_iptt(2) == 0 && (cqe_dw0(2) & 0x3u) == SAS_HBA_CQE_CMPLT_ERR,
                "B3 DMA 越界 ⇒ 错误 CQE（cmplt=3 ✓；iptt 不可知 ⇒ 0 ✓ 不留残值）");
            chk(pair->hba.dma_fail_cnt > f0, "B3 dma_fail_cnt 增（DECERR 被记账 ✓）");
        }

        printf("[合计] PASS=%d FAIL=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

int sc_main(int argc, char **argv) {
    (void)argc; (void)argv;
    sc_core::sc_clock clk("clk", 10, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;
    SasPairTop pair("pair", MEM_BASE, 0x01000000ull, 256);
    TbNeg tb("tb");
    DummyAmbaMaster stub("stub");              // 从口占位（ONE 策略 ✓）
    tb.clk(clk); tb.rst_n(rst_n); tb.pair = &pair;
    pair.clk(clk); pair.rst_n(rst_n);
    stub.m.bind(pair.hba.reg_s);
    rst_n.write(false);
    sc_core::sc_start(200, sc_core::SC_NS);
    rst_n.write(true);
    sc_core::sc_start();
    int rc = (tb.fail_cnt == 0) ? 0 : 1;
    printf("%s\n", rc == 0 ? "TB_NEG PASS" : "TB_NEG FAIL");
    return rc;
}
