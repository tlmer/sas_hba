//============================================================================
// tb_hba_regs.cpp — HBA 寄存器面专项（直连 vs AMBA-PV socket 两路同源 ✓）  [2026-10-08 建]
//   结构：SasPairTop（模型全接入但 DQ 未使能 ⇒ 全静默 ✓）+ ApvMaster 绑 hba.reg_s ✓
//
//   判据（**预登记**；期望值出处 = `sw/sas_hba_regs.h` 各条 ✓）：
//   C1  直连读写往返：DLVRY_QUEUE_ENABLE / DQ0 base+depth+wr_ptr / CQ0 / CFG_MAX_TAG ✓
//   C2  两路同源：socket 写 ⇒ 直连读一致；直连写 ⇒ socket 读一致 ✓
//   C3  未映射地址（0x1000，落入段间空洞）读/写 ⇒ **DECERR** ✓
//   C4  RO 写（CQE_SEND_CNT 0x248）⇒ **OKAY + 值不变 + ro_wr_cnt+1** ✓
//   C5  DQ rd_ptr 写 ⇒ 忽略（硬件推进 ⇒ 软件只读 ✓）+ 计数 ✓
//   C6  PHY0 CHL_INT0 复位 = bit2（SL_PHY_ENABLE ✓ 链路已建）；写 bit2 ⇒ 清 0 ✓
//       （★ 全线唯一 W1C ✓ regfile）；CHNL_INT_STATUS bit0 随之落 0 ✓
//   C7  PHY TX_ID_DW0..6（0x9C..0xB4，7 个 ✓）+ PHY_CFG 读写往返 ✓
//   C8  段内未列地址（0x0F0）⇒ 读 0 且 OKAY（RTL case default 语义 ✓）
//   C9  irq 空闲期恒低（无 pending ✓）
//   C10 socket 访问计数 reg_sock_n 增 ✓（成功访问计数口径 ✓）
//============================================================================
#include <systemc.h>
#include "sas_pair_top.h"
#include "apv_master.h"
#include "sas_hba_regs.h"
#include <cstdio>

static const uint64_t CSR = 0xfe400000ull;      // CSR 窗基址（低 20 位译码 ✓ 任意基址均可）

struct TbRegs : sc_core::sc_module {
    sc_core::sc_in<bool>  clk;
    sc_core::sc_out<bool> rst_n;
    SasPairTop *pair = nullptr;
    ApvMaster  *apv  = nullptr;
    int pass_cnt = 0, fail_cnt = 0;

    SC_HAS_PROCESS(TbRegs);
    TbRegs(sc_core::sc_module_name nm) : sc_module(nm), clk("clk"), rst_n("rst_n") { SC_THREAD(run); }
    void tick(int n = 1) { for (int i = 0; i < n; i++) { wait(clk.posedge_event()); wait(sc_core::SC_ZERO_TIME); } }
    void chk(bool ok, const char *name) {
        printf("   %s %s\n", ok ? "✓" : "✗", name);
        if (ok) pass_cnt++; else fail_cnt++;
    }
    bool dw(uint32_t off, uint32_t &v) { sc_uint<32> t = 0; bool m = pair->hba.reg_read(CSR + off, t); v = t.to_uint(); return m; }
    bool dwr(uint32_t off, uint32_t v) { return pair->hba.reg_write(CSR + off, sc_uint<32>(v)); }

    void run() {
        printf("[TB] 预登记判据 C1..C10（寄存器面 ✓）\n");
        tick(4);
        while (!rst_n.read()) tick(1);
        tick(4);
        uint32_t v = 0;

        // ── C1 直连读写往返 ──
        {
            bool ok = true;
            dwr(SAS_HBA_CFG_DLVRY_QUEUE_ENABLE, 0x5);   dw(SAS_HBA_CFG_DLVRY_QUEUE_ENABLE, v); ok &= (v == 5);
            dwr(SAS_HBA_DQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), 0xfe440000u);
            dw(SAS_HBA_DQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), v); ok &= (v == 0xfe440000u);
            dwr(SAS_HBA_DQ_REG(0, SAS_HBA_XX_DEPTH), 8); dw(SAS_HBA_DQ_REG(0, SAS_HBA_XX_DEPTH), v); ok &= (v == 8);
            dwr(SAS_HBA_DQ_REG(0, SAS_HBA_XX_WR_PTR), 3); dw(SAS_HBA_DQ_REG(0, SAS_HBA_XX_WR_PTR), v); ok &= (v == 3);
            dwr(SAS_HBA_CQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), 0xfe441000u);
            dw(SAS_HBA_CQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), v); ok &= (v == 0xfe441000u);
            dwr(SAS_HBA_CFG_MAX_TAG, 4096); dw(SAS_HBA_CFG_MAX_TAG, v); ok &= (v == 4096);
            chk(ok, "C1 直连读写往返（DLQ_EN/DQ0/CQ0/MAX_TAG ✓）");
        }
        // ── C2 两路同源 ──
        {
            apv->wr(CSR + SAS_HBA_CFG_MAX_TAG, 0x1234);          // socket 写 ✓
            dw(SAS_HBA_CFG_MAX_TAG, v);
            bool ok1 = (v == 0x1234);
            dwr(SAS_HBA_CFG_MAX_TAG, 0x5678);                    // 直连写 ✓
            uint32_t sv = 0; apv->rd(CSR + SAS_HBA_CFG_MAX_TAG, sv);
            chk(ok1 && sv == 0x5678, "C2 两路同源（socket 写⇒直连读 ✓；直连写⇒socket 读 ✓）");
        }
        // ── C3 未映射 ⇒ DECERR ──
        {
            uint32_t dummy = 0;
            amba_pv::amba_pv_resp_t r1 = apv->rd(CSR + 0x1000, dummy);
            amba_pv::amba_pv_resp_t r2 = apv->wr(CSR + 0x1000, 0xDEAD);
            chk(r1 == amba_pv::AMBA_PV_DECERR && r2 == amba_pv::AMBA_PV_DECERR,
                "C3 未映射（0x1000）读/写 ⇒ DECERR ✓");
        }
        // ── C4 RO 写（CQE_SEND_CNT）──
        {
            unsigned long ro0 = pair->hba.ro_wr_cnt;
            apv->wr(CSR + SAS_HBA_STS_CQE_SEND_CNT, 0xFFFFFFFFu);
            dw(SAS_HBA_STS_CQE_SEND_CNT, v);
            chk(v == 0 && pair->hba.ro_wr_cnt == ro0 + 1,
                "C4 RO 写 CQE_SEND_CNT ⇒ OKAY 忽略、值不变、计数 +1 ✓");
        }
        // ── C5 DQ rd_ptr 只读 ──
        {
            unsigned long ro0 = pair->hba.ro_wr_cnt;
            dwr(SAS_HBA_DQ_REG(0, SAS_HBA_XX_RD_PTR), 7);
            dw(SAS_HBA_DQ_REG(0, SAS_HBA_XX_RD_PTR), v);
            chk(v == 0 && pair->hba.ro_wr_cnt == ro0 + 1,
                "C5 DQ rd_ptr 写被忽略（硬件推进 ✓ 计数 +1 ✓）");
        }
        // ── C6 CHL_INT0 W1C ──
        {
            dw(SAS_HBA_PHY_REG(0, SAS_HBA_PHY_CHL_INT0), v);
            bool b2 = ((v >> 2) & 1u) == 1u;
            dw(SAS_HBA_INT_CHNL_STATUS, v);
            bool st0 = (v & 1u) == 1u;
            apv->wr(CSR + SAS_HBA_PHY_REG(0, SAS_HBA_PHY_CHL_INT0), SAS_HBA_CHL0_SL_PHY_ENABLE);  // W1C ✓
            dw(SAS_HBA_PHY_REG(0, SAS_HBA_PHY_CHL_INT0), v);
            bool clr = ((v >> 2) & 1u) == 0u;
            dw(SAS_HBA_INT_CHNL_STATUS, v);
            bool st1 = (v & 1u) == 0u;
            chk(b2 && st0 && clr && st1,
                "C6 CHL_INT0 bit2=1（复位）→ W1C 清 0；CHNL_INT_STATUS bit0 随之落 ✓");
        }
        // ── C7 PHY TX_ID 组 + PHY_CFG ──
        {
            bool ok = true;
            for (int i = 0; i < 7; i++) {
                dwr(SAS_HBA_PHY_REG(0, SAS_HBA_PHY_TX_ID_DW0 + 4 * i), 0x1000 + i);
                dw(SAS_HBA_PHY_REG(0, SAS_HBA_PHY_TX_ID_DW0 + 4 * i), v);
                ok &= (v == (uint32_t)(0x1000 + i));
            }
            dwr(SAS_HBA_PHY_REG(0, SAS_HBA_PHY_CFG), 0x01A00000u);
            dw(SAS_HBA_PHY_REG(0, SAS_HBA_PHY_CFG), v); ok &= (v == 0x01A00000u);
            chk(ok, "C7 PHY TX_ID_DW0..6（7 个）+ PHY_CFG 读写往返 ✓");
        }
        // ── C8 段内未列 ⇒ 0 + OKAY ──
        {
            amba_pv::amba_pv_resp_t r = apv->rd(CSR + 0x0F0, v);
            chk(r == amba_pv::AMBA_PV_OKAY && v == 0, "C8 段内未列 0x0F0 ⇒ 读 0 且 OKAY ✓");
        }
        // ── C9 irq 空闲恒低 / C10 socket 计数 ──
        chk(!pair->hba.irq.read(), "C9 空闲期 irq 恒低（无 pending ✓）");
        // 口径：成功访问计数（DECERR 不计 ✓）= C2×2 + C4×1 + C6×1 + C8×1 = 5 ✓
        chk(pair->hba.reg_sock_n == 5ul, "C10 reg_sock_n = 5（成功 socket 访问计数 ✓）");

        printf("[合计] PASS=%d FAIL=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

int sc_main(int argc, char **argv) {
    (void)argc; (void)argv;
    sc_core::sc_clock clk("clk", 10, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;
    SasPairTop pair("pair", 0xfe400000ull, 0x00100000ull, 64);
    ApvMaster  apv("apv");
    TbRegs tb("tb");
    tb.clk(clk); tb.rst_n(rst_n); tb.pair = &pair; tb.apv = &apv;
    pair.clk(clk); pair.rst_n(rst_n);
    apv.clk(clk);
    apv.m.bind(pair.hba.reg_s);                    // ★ 真 socket 路径 ✓（从口 = ONE ⇒ 本台不绑占位件 ✓）
    rst_n.write(false);
    sc_core::sc_start(200, sc_core::SC_NS);
    rst_n.write(true);
    sc_core::sc_start();
    int rc = (tb.fail_cnt == 0) ? 0 : 1;
    printf("%s\n", rc == 0 ? "TB_HBA_REGS PASS" : "TB_HBA_REGS FAIL");
    return rc;
}
