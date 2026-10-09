//=============================================================================
// sas_pair_top.h — HBA ⇄ 链路 ⇄ HDD（+ 内存）联合顶层  [2026-10-08 建]
//-----------------------------------------------------------------------------
// 角色：把三个模型 + 内存接成**点对点 SAS 域**，TB 直接例化即可跑全流程 ✓：
//   hba.dma_s ⇄ mem.s（AMBA-PV ✓）｜hba.reg_s = 留给 TB（另绑 ApvMaster 或直连 ✓）
//   hba.tx/rx ⇄ link.a_* ｜ link.b_* ⇄ hdd.tx/rx（帧级 ✓）
//
// ★ 例化用法（TB 的 sc_main）：
//     sc_clock clk("clk", 10, SC_NS);
//     SasPairTop pair("pair", 0xfe400000ull, 0x1000000ull, 4096);
//     pair.clk(clk); pair.rst_n(rst);
//     // 寄存器：pair.hba.reg_write(...)（直连 ✓）；走 socket 则绑 ApvMaster 到 pair.hba.reg_s ✓
//     // ⚠ 不用 socket 的 TB 必须绑 DummyAmbaMaster（否则 E109 端口未绑 ✗）
//     // 白盒：pair.hba.o_cqes.read() / pair.hdd.o_cmds_good.read() ✓
//
// ⚠ 所有模型 `o_*` 白盒口**必须绑定**（模型每拍写 ✓）—— 本顶层已绑齐 ✓
//=============================================================================
#ifndef SAS_PAIR_TOP_H
#define SAS_PAIR_TOP_H

#include <systemc.h>
#include "sas_hba_tlm.h"
#include "sas_hdd_tlm.h"
#include "sas_link_tlm.h"
#include "sas_mem_tlm.h"

struct SasPairTop : sc_core::sc_module {
    sc_core::sc_in<bool> clk, rst_n;

    SasHbaTlm       hba;
    SasLinkTlm      link;
    SasHddTlm       hdd;
    SasMemTlm       mem;

    // ── A 侧（HBA ↔ link）线路 ──
    sc_core::sc_signal<sc_uint<32>> a_tx_data;
    sc_core::sc_signal<bool>        a_tx_valid, a_tx_sof, a_tx_eof, a_tx_ready;
    sc_core::sc_signal<sc_uint<4>>  a_tx_type;
    sc_core::sc_signal<sc_uint<32>> a_rx_data;
    sc_core::sc_signal<bool>        a_rx_valid, a_rx_sof, a_rx_eof, a_rx_ready;
    sc_core::sc_signal<sc_uint<4>>  a_rx_type;
    // ── B 侧（link ↔ HDD）线路 ──
    sc_core::sc_signal<sc_uint<32>> b_tx_data;
    sc_core::sc_signal<bool>        b_tx_valid, b_tx_sof, b_tx_eof, b_tx_ready;
    sc_core::sc_signal<sc_uint<4>>  b_tx_type;
    sc_core::sc_signal<sc_uint<32>> b_rx_data;
    sc_core::sc_signal<bool>        b_rx_valid, b_rx_sof, b_rx_eof, b_rx_ready;
    sc_core::sc_signal<sc_uint<4>>  b_rx_type;
    // ── link 白盒 ──
    sc_core::sc_signal<sc_uint<32>> lk_dropped;
    // ── 中断线（简化：一根 ✓ 见模型头注释）──
    sc_core::sc_signal<bool>        h_irqline;
    // ── hba 白盒（必须绑 ✓）──
    sc_core::sc_signal<sc_uint<32>> h_done, h_err, h_cqes, h_irq, h_dqf, h_ctf;
    sc_core::sc_signal<sc_uint<32>> h_dmar, h_dmaw, h_frmtx, h_frmrx, h_tagmm, h_datrx;
    sc_core::sc_signal<sc_uint<16>> h_iptt;
    sc_core::sc_signal<sc_uint<8>>  h_sspst;
    sc_core::sc_signal<sc_uint<32>> h_dbg;
    // ── hdd 白盒（必须绑 ✓）──
    sc_core::sc_signal<sc_uint<32>> d_rxf, d_txf, d_good, d_chk, d_rd, d_wr, d_dbg2;
    sc_core::sc_signal<sc_uint<8>>  d_op, d_st;

    SasPairTop(sc_core::sc_module_name nm,
               uint64_t mem_base = 0xfe400000ull,
               uint64_t mem_size = 0x01000000ull,
               uint64_t hdd_blocks = 4096)
      : sc_core::sc_module(nm),
        clk("clk"), rst_n("rst_n"),
        hba("hba"), link("link"), hdd("hdd", hdd_blocks),
        mem("mem", mem_base, mem_size),
        a_tx_data("a_tx_data"), a_tx_valid("a_tx_valid"), a_tx_sof("a_tx_sof"),
        a_tx_eof("a_tx_eof"), a_tx_ready("a_tx_ready"), a_tx_type("a_tx_type"),
        a_rx_data("a_rx_data"), a_rx_valid("a_rx_valid"), a_rx_sof("a_rx_sof"),
        a_rx_eof("a_rx_eof"), a_rx_ready("a_rx_ready"), a_rx_type("a_rx_type"),
        b_tx_data("b_tx_data"), b_tx_valid("b_tx_valid"), b_tx_sof("b_tx_sof"),
        b_tx_eof("b_tx_eof"), b_tx_ready("b_tx_ready"), b_tx_type("b_tx_type"),
        b_rx_data("b_rx_data"), b_rx_valid("b_rx_valid"), b_rx_sof("b_rx_sof"),
        b_rx_eof("b_rx_eof"), b_rx_ready("b_rx_ready"), b_rx_type("b_rx_type"),
        lk_dropped("lk_dropped"), h_irqline("h_irqline"),
        h_done("h_done"), h_err("h_err"), h_cqes("h_cqes"), h_irq("h_irq"),
        h_dqf("h_dqf"), h_ctf("h_ctf"), h_dmar("h_dmar"), h_dmaw("h_dmaw"),
        h_frmtx("h_frmtx"), h_frmrx("h_frmrx"), h_tagmm("h_tagmm"), h_datrx("h_datrx"),
        h_iptt("h_iptt"), h_sspst("h_sspst"), h_dbg("h_dbg"),
        d_rxf("d_rxf"), d_txf("d_txf"), d_good("d_good"), d_chk("d_chk"),
        d_rd("d_rd"), d_wr("d_wr"), d_dbg2("d_dbg2"), d_op("d_op"), d_st("d_st")
    {
        // 时钟/复位 ✓
        hba.clk(clk);  hba.rst_n(rst_n);
        link.clk(clk); link.rst_n(rst_n);
        hdd.clk(clk);  hdd.rst_n(rst_n);

        // A 侧：HBA ⇄ link ✓
        hba.tx_data(a_tx_data);   link.a_tx_data(a_tx_data);
        hba.tx_valid(a_tx_valid); link.a_tx_valid(a_tx_valid);
        hba.tx_sof(a_tx_sof);     link.a_tx_sof(a_tx_sof);
        hba.tx_eof(a_tx_eof);     link.a_tx_eof(a_tx_eof);
        hba.tx_type(a_tx_type);   link.a_tx_type(a_tx_type);
        hba.tx_ready(a_tx_ready); link.a_tx_ready(a_tx_ready);
        hba.rx_data(a_rx_data);   link.a_rx_data(a_rx_data);
        hba.rx_valid(a_rx_valid); link.a_rx_valid(a_rx_valid);
        hba.rx_sof(a_rx_sof);     link.a_rx_sof(a_rx_sof);
        hba.rx_eof(a_rx_eof);     link.a_rx_eof(a_rx_eof);
        hba.rx_type(a_rx_type);   link.a_rx_type(a_rx_type);
        hba.rx_ready(a_rx_ready); link.a_rx_ready(a_rx_ready);

        // B 侧：link ⇄ HDD ✓
        hdd.tx_data(b_tx_data);   link.b_tx_data(b_tx_data);
        hdd.tx_valid(b_tx_valid); link.b_tx_valid(b_tx_valid);
        hdd.tx_sof(b_tx_sof);     link.b_tx_sof(b_tx_sof);
        hdd.tx_eof(b_tx_eof);     link.b_tx_eof(b_tx_eof);
        hdd.tx_type(b_tx_type);   link.b_tx_type(b_tx_type);
        hdd.tx_ready(b_tx_ready); link.b_tx_ready(b_tx_ready);
        hdd.rx_data(b_rx_data);   link.b_rx_data(b_rx_data);
        hdd.rx_valid(b_rx_valid); link.b_rx_valid(b_rx_valid);
        hdd.rx_sof(b_rx_sof);     link.b_rx_sof(b_rx_sof);
        hdd.rx_eof(b_rx_eof);     link.b_rx_eof(b_rx_eof);
        hdd.rx_type(b_rx_type);   link.b_rx_type(b_rx_type);
        hdd.rx_ready(b_rx_ready); link.b_rx_ready(b_rx_ready);

        // 白盒口（每拍写 ⇒ 必须绑 ✓）
        hba.o_cmds_done(h_done);   hba.o_cmds_err(h_err);
        hba.o_cqes(h_cqes);        hba.o_irq_cnt(h_irq);
        hba.o_dq_fetches(h_dqf);   hba.o_ct_fetches(h_ctf);
        hba.o_dma_reads(h_dmar);   hba.o_dma_writes(h_dmaw);
        hba.o_frames_tx(h_frmtx);  hba.o_frames_rx(h_frmrx);
        hba.o_tag_mismatch(h_tagmm); hba.o_data_rx_bytes(h_datrx);
        hba.o_last_iptt(h_iptt);   hba.o_last_ssp_status(h_sspst);
        hba.o_dbg_state(h_dbg);
        hba.irq(h_irqline);
        hdd.o_rx_frames(d_rxf);    hdd.o_tx_frames(d_txf);
        hdd.o_cmds_good(d_good);   hdd.o_cmds_chk(d_chk);
        hdd.o_lba_read(d_rd);      hdd.o_lba_write(d_wr);
        hdd.o_last_opcode(d_op);   hdd.o_last_status(d_st);
        hdd.o_dbg_state(d_dbg2);
        link.o_dropped(lk_dropped);

        // DMA 面 ✓（CSR 从口由 TB 自行绑：真 ApvMaster 或 DummyAmbaMaster ✓
        //   ⚠ VDK 此版从口绑策略 = **ONE**（多绑报 E109「2 binds exceeds maximum」✗ 实踩）
        //   ⇒ 凡例化本顶层而不走 socket 的 TB 必须绑占位件；用真主端的 TB 则不绑占位 ✓）
        hba.dma_s.bind(mem.s);
    }
};

#endif // SAS_PAIR_TOP_H
