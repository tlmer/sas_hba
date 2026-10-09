//=============================================================================
// sas_hba_tlm.cpp — SAS HBA（hisi_sas v3 兼容）事务级模型实现  [2026-10-08 建]
//-----------------------------------------------------------------------------
// 逐拍时序（step() 一次 = 一拍 ✓；全部端口为**寄存器输出**、构造期不写 ✓）：
//   ⓪ 复位：寄存器堆回默认（掩码全 1 = 全掩 ✓）、端口确定态 ✓
//   ① TX 子引擎（帧队列 → held-valid 发出 ✓）
//   ② RX 子引擎（逐拍装配；帧尾回调 on_rx_frame ✓）
//   ③ 主状态机：IDLE → DQ 读 → 解析 → 命令表读 → 组帧发 → （写方向：发 DATA）
//      → 等响应（DATA 落内存 / RESPONSE 取 status ✓）→ 写 CQE + 中断 → rd_ptr++ ✓
//   ⚠ DMA = **零时 b_transport**（内存模型默认 0 延迟 ✓）⇒ 可从 SC_METHOD 调 ✓；
//     返回非零延迟时记 `dma_late_cnt`（本模型忽略之，明写 ✗ 简化）
//=============================================================================
#include "sas_hba_tlm.h"
#include "sas_hba_hw.h"
#include <cstring>

SasHbaTlm::SasHbaTlm(sc_core::sc_module_name nm)
: sc_core::sc_module(nm),
  amba_pv::amba_pv_slave_base<32>("sas_hba_csr"),     // 从端宿主名（照先例 ✓）
  amba_pv::amba_pv_master_base("sas_hba_dma"),
  reg_s("reg_s"), dma_s("dma_s"),
  clk("clk"), rst_n("rst_n"),
  tx_data("tx_data"), tx_valid("tx_valid"), tx_sof("tx_sof"), tx_eof("tx_eof"),
  tx_type("tx_type"), tx_ready("tx_ready"),
  rx_data("rx_data"), rx_valid("rx_valid"), rx_sof("rx_sof"), rx_eof("rx_eof"),
  rx_type("rx_type"), rx_ready("rx_ready"),
  irq("irq"),
  o_cmds_done("o_cmds_done"), o_cmds_err("o_cmds_err"), o_cqes("o_cqes"),
  o_irq_cnt("o_irq_cnt"), o_dq_fetches("o_dq_fetches"), o_ct_fetches("o_ct_fetches"),
  o_dma_reads("o_dma_reads"), o_dma_writes("o_dma_writes"),
  o_frames_tx("o_frames_tx"), o_frames_rx("o_frames_rx"),
  o_tag_mismatch("o_tag_mismatch"), o_data_rx_bytes("o_data_rx_bytes"),
  o_last_iptt("o_last_iptt"), o_last_ssp_status("o_last_ssp_status"),
  o_dbg_state("o_dbg_state")
{
    reg_s(*this);                 // ★ CSR 从口绑到本类（基类转 read()/write() ✓ 照先例 ✓）
    dma_s(*this);                 // DMA 主口反向路径绑到自己（照 VDK dma 例子 ✓）
    // 配置默认 ✓
    cfg_fetch_lat    = 4;         // DQ 条目 4 拍（RTL `dq_engine ✓）
    cfg_ct_lat       = 8;         // 命令表延迟（RTL 63 拍 ⇒ 本模型 8 拍，简化 ✓）
    cfg_resp_timeout = 4096;      // 等响应超时（拍 ✓；TB 可调小做负控 ✓）
    cfg_rtl_no_eoi   = 0;         // 0 = 内核期望的 W1C ✓（1 = 复现 RTL 无 EOI ✓ 红线②）
    cfg_addr_mask    = 0xFFFFFu;  // CSR 地址窗（低 20 位 ✓ 兼容任意映射基址 ✓）
    reg_sock_n = ro_wr_cnt = decerr_cnt = 0;
    dma_fail_cnt = dma_late_cnt = 0; data_drop_cnt = 0;
    dq_cq_defaults();
    st = H_IDLE; lat_cnt = 0; cur_dqi = 0; resp_cnt = 0;
    wd_left = 0; wd_off = 0;
    cur_iptt = cur_dev_id = cur_tag = 0; cur_dir = 0; cur_xfer_len = 0;
    cur_ct_addr = cur_prd_addr = cur_sts_addr = 0;
    cur_sge_addr = 0; cur_sge_len = 0; tag_gen = 0;
    cmpl_err = false; cmpl_got_resp = false; cmpl_status = 0;
    tx_active = false; tx_presented = false; tx_total = tx_idx = 0; tx_ft = 0;
    rx_dwc = 0; rx_ft_lat = 0;
    c_done = c_err = c_cqes = c_irq = c_dq_fetches = c_ct_fetches = 0;
    c_dma_r = c_dma_w = c_frames_tx = c_frames_rx = c_tag_mm = c_data_rx = 0;
    last_iptt = 0; last_ssp_status = 0;
    SC_METHOD(step);
    sensitive << clk.pos();
    dont_initialize();            // 首拍照常 step；端口逐拍写、构造期不碰 ✓
}

void SasHbaTlm::dq_cq_defaults()
{
    memset(dq, 0, sizeof(dq));
    memset(cq, 0, sizeof(cq));
    memset(phy, 0, sizeof(phy));
    for (int i = 0; i < SAS_HBA_MAX_PHY; i++) {
        phy[i].chl_int0 = SAS_HBA_CHL0_SL_PHY_ENABLE;      // 链路已建立（点对点 ✓）
        phy[i].msk0 = 0xFFFFFFFFu; phy[i].msk1 = 0xFFFFFFFFu; phy[i].msk2 = 0xFFFFFFFFu;
    }
    r_dlq_en = 0; r_iost_lo = r_iost_hi = r_itct_lo = r_itct_hi = 0;
    r_broken_lo = r_broken_hi = 0;
    r_phy_ctx = r_phy_state = r_phy_port_ma = r_phy_conn_rate = 0;
    r_itct_clr = r_sata_lo = r_sata_hi = r_max_tag = 0;
    r_nexus_t = r_maxtime_t = r_inactive_t = r_reject_t = 0;
    r_converge_en = r_abt_query = r_abt_done = 0;
    r_coal_en = r_oq_coal_t = r_oq_coal_c = r_ent_coal_t = r_ent_coal_c = 0;
    r_oq_int_src = 0;
    r_oq_int_src_msk = 0xFFFFFFFFu;   // 复位全掩 ✓（0 位 = 未掩 = 放行 ✓）
    r_ent_msk1 = r_ent_msk2 = r_ent_msk3 = 0xFFFFFFFFu;
    r_phyupdown_msk = r_chnl_ent_msk = r_hgc_com_msk = 0xFFFFFFFFu;
    r_ecc_intr = 0; r_ecc_msk = 0xFFFFFFFFu; r_hgc_en = 0;
    r_am_ctrl = r_am_max_trans = 0;
    r_ras0 = r_ras1 = 0; r_ras0_msk = r_ras1_msk = 0xFFFFFFFFu;
}

// ═════════════ 寄存器面（直连与 socket 同源 ✓）═════════════
bool SasHbaTlm::reg_read(sc_dt::uint64 addr, sc_uint<32> &data)
{
    return reg_read_off((uint32_t)(addr & (sc_dt::uint64)cfg_addr_mask), data);
}

bool SasHbaTlm::reg_write(sc_dt::uint64 addr, sc_uint<32> v)
{
    return reg_write_off((uint32_t)(addr & (sc_dt::uint64)cfg_addr_mask), (uint32_t)v.to_uint());
}

// 读：true = 映射（段级）；段内未列子偏移 ⇒ 0（RTL case default 同 ✓）
bool SasHbaTlm::reg_read_off(uint32_t off, sc_uint<32> &data)
{
    data = 0;
    // ── GLOBAL 段（`RTL 源` ✓）──
    if (off < SAS_HBA_SEG_GLOBAL_END) {
        switch (off) {
        case SAS_HBA_CFG_DLVRY_QUEUE_ENABLE: data = r_dlq_en; return true;
        case SAS_HBA_CFG_IOST_BASE_LO:       data = r_iost_lo; return true;
        case SAS_HBA_CFG_IOST_BASE_HI:       data = r_iost_hi; return true;
        case SAS_HBA_CFG_ITCT_BASE_LO:       data = r_itct_lo; return true;
        case SAS_HBA_CFG_ITCT_BASE_HI:       data = r_itct_hi; return true;
        case SAS_HBA_CFG_BROKEN_MSG_LO:      data = r_broken_lo; return true;
        case SAS_HBA_CFG_BROKEN_MSG_HI:      data = r_broken_hi; return true;
        case SAS_HBA_CFG_PHY_CONTEXT:        data = r_phy_ctx; return true;
        case SAS_HBA_CFG_PHY_STATE:          data = r_phy_state; return true;
        case SAS_HBA_CFG_PHY_PORT_NUM_MA:    data = r_phy_port_ma; return true;
        case SAS_HBA_CFG_PHY_CONN_RATE:      data = r_phy_conn_rate; return true;
        case SAS_HBA_CFG_ITCT_CLR:           data = r_itct_clr; return true;
        case SAS_HBA_CFG_SATA_INIT_D2H_LO:   data = r_sata_lo; return true;
        case SAS_HBA_CFG_SATA_INIT_D2H_HI:   data = r_sata_hi; return true;
        case SAS_HBA_CFG_MAX_TAG:            data = r_max_tag; return true;
        case SAS_HBA_CFG_ITY_NEXUS_LOSS_T:   data = r_nexus_t; return true;
        case SAS_HBA_CFG_MAX_CON_TIME_LIMIT: data = r_maxtime_t; return true;
        case SAS_HBA_CFG_BUS_INACTIVE_LIMIT: data = r_inactive_t; return true;
        case SAS_HBA_CFG_REJECT_TO_OPEN_T:   data = r_reject_t; return true;
        case SAS_HBA_CFG_CQ_INT_CONVERGE_EN: data = r_converge_en; return true;
        case SAS_HBA_CFG_ABT_SET_QUERY_IPTT: data = r_abt_query; return true;
        case SAS_HBA_CFG_ABT_SET_IPTT_DONE:  data = r_abt_done; return true;
        case SAS_HBA_INT_CHNL_STATUS: {     // PHY0..7 汇总（⚠ 只到 PHY7 = 红线④ ✓）
            uint32_t v = 0;
            for (unsigned p = 0; p < 8; p++) {
                uint32_t c0 = phy[p].chl_int0;
                v |= ((c0 >> 2) & 1u) << (4 * p + 0);   // SL_PHY_ENABLE ✓
                v |= ((c0 >> 0) & 1u) << (4 * p + 1);   // HOTPLUG_TOUT ✓
                v |= ((c0 >> 4) & 1u) << (4 * p + 2);   // NOT_RDY ✓
                v |= ((c0 >> 5) & 1u) << (4 * p + 3);   // PHY_RDY ✓
            }
            data = v; return true;
        }
        case SAS_HBA_INT_COAL_EN:      data = r_coal_en; return true;
        case SAS_HBA_INT_OQ_COAL_TIME: data = r_oq_coal_t; return true;
        case SAS_HBA_INT_OQ_COAL_CNT:  data = r_oq_coal_c; return true;
        case SAS_HBA_INT_ENT_COAL_TIME:data = r_ent_coal_t; return true;
        case SAS_HBA_INT_ENT_COAL_CNT: data = r_ent_coal_c; return true;
        case SAS_HBA_INT_OQ_SRC:       data = r_oq_int_src; return true;   // 只读位图 ✓
        case SAS_HBA_INT_OQ_SRC_MSK:   data = r_oq_int_src_msk; return true;
        case SAS_HBA_INT_ENT_SRC1:
        case SAS_HBA_INT_ENT_SRC2:
        case SAS_HBA_INT_ENT_SRC3:     data = 0; return true;              // 无拓扑事件 ✓
        case SAS_HBA_INT_ENT_SRC_MSK1: data = r_ent_msk1; return true;
        case SAS_HBA_INT_ENT_SRC_MSK2: data = r_ent_msk2; return true;
        case SAS_HBA_INT_ENT_SRC_MSK3: data = r_ent_msk3; return true;
        case SAS_HBA_INT_PHYUPDOWN_MSK:data = r_phyupdown_msk; return true;
        case SAS_HBA_INT_CHNL_ENT_MSK: data = r_chnl_ent_msk; return true;
        case SAS_HBA_INT_HGC_COM_MSK:  data = r_hgc_com_msk; return true;
        case SAS_HBA_INT_SAS_ECC_INTR: data = r_ecc_intr; return true;
        case SAS_HBA_INT_SAS_ECC_INTR_MSK: data = r_ecc_msk; return true;
        case SAS_HBA_CFG_HGC_ERR_STAT_EN:  data = r_hgc_en; return true;
        case SAS_HBA_STS_CQE_SEND_CNT: data = c_cqes; return true;         // 只读 ✓
        default: return true;                                              // 段内未列 ⇒ 0 ✓
        }
    }
    // ── DQ 段（`RTL 源` ✓；32 项全译 ✓）──
    if (off < SAS_HBA_SEG_CQ_BASE) {
        unsigned i = SAS_HBA_DQ_IDX(off), r = SAS_HBA_DQ_OFF(off);
        if (i >= SAS_HBA_MAX_DQ) return false;
        switch (r) {
        case SAS_HBA_XX_BASE_ADDR_LO: data = dq[i].base_lo; return true;
        case SAS_HBA_XX_BASE_ADDR_HI: data = dq[i].base_hi; return true;
        case SAS_HBA_XX_DEPTH:        data = dq[i].depth; return true;
        case SAS_HBA_XX_WR_PTR:       data = dq[i].wr_ptr; return true;
        case SAS_HBA_XX_RD_PTR:       data = dq[i].rd_ptr; return true;
        default: return true;
        }
    }
    // ── CQ 段（模型 = 32 项全译；⚠ RTL 上界 0x6D0 ⇒ CQ25+ 不可达 = 红线① ✓）──
    if (off < SAS_HBA_SEG_CQ_BASE + (uint32_t)SAS_HBA_MAX_DQ * SAS_HBA_CQ_STRIDE) {
        unsigned i = SAS_HBA_CQ_IDX(off), r = SAS_HBA_CQ_OFF(off);
        switch (r) {
        case SAS_HBA_XX_BASE_ADDR_LO: data = cq[i].base_lo; return true;
        case SAS_HBA_XX_BASE_ADDR_HI: data = cq[i].base_hi; return true;
        case SAS_HBA_XX_DEPTH:        data = cq[i].depth; return true;
        case SAS_HBA_XX_WR_PTR:       data = cq[i].wr_ptr; return true;
        case SAS_HBA_XX_RD_PTR:       data = cq[i].rd_ptr; return true;
        default: return true;
        }
    }
    // ── PHY 段（0x2000..0x4400，9 个 ✓）──
    if (off >= SAS_HBA_SEG_PHY_BASE &&
        off <  SAS_HBA_SEG_PHY_BASE + (uint32_t)SAS_HBA_MAX_PHY * SAS_HBA_PHY_STRIDE) {  // ★ 须查下界 ✗ 否则空洞段下溢（实踩段错误 ✓）
        unsigned i = SAS_HBA_PHY_IDX(off), r = SAS_HBA_PHY_OFF(off);
        PhyReg &p = phy[i];
        if (r >= SAS_HBA_PHY_TX_ID_DW0 && r <= SAS_HBA_PHY_TX_ID_DW6 && (r & 3u) == 0) {
            data = p.tx_id[(r - SAS_HBA_PHY_TX_ID_DW0) / 4]; return true;
        }
        switch (r) {
        case SAS_HBA_PHY_CFG:            data = p.cfg; return true;
        case SAS_HBA_PHY_HARD_LINKRATE:  data = p.hard_linkrate; return true;
        case SAS_HBA_PHY_PROG_LINK_RATE: data = p.prog_rate; return true;
        case SAS_HBA_PHY_CTRL:           data = p.ctrl; return true;
        case SAS_HBA_PHY_SERDES_CFG:     data = p.serdes; return true;
        case SAS_HBA_PHY_SL_CFG:         data = p.sl_cfg; return true;
        case SAS_HBA_PHY_SL_CONTROL:     data = p.sl_ctl; return true;
        case SAS_HBA_PHY_TXID_AUTO:      data = p.txid_auto; return true;
        case SAS_HBA_PHY_RX_IDAF_DW0:    data = p.rx_idaf; return true;
        case SAS_HBA_PHY_CON_CFG_DRIVER: data = p.con_cfg_drv; return true;
        case SAS_HBA_PHY_SSP_CON_TIMER:  data = p.ssp_timer; return true;
        case SAS_HBA_PHY_SMP_CON_TIMER:  data = p.smp_timer; return true;
        case SAS_HBA_PHY_STP_CON_TIMER:  data = p.stp_timer; return true;
        case SAS_HBA_PHY_CHL_INT0:       data = p.chl_int0; return true;
        case SAS_HBA_PHY_CHL_INT1:       data = p.chl_int1; return true;
        case SAS_HBA_PHY_CHL_INT2:       data = p.chl_int2; return true;
        case SAS_HBA_PHY_CHL_INT_MSK0:   data = p.msk0; return true;
        case SAS_HBA_PHY_CHL_INT_MSK1:   data = p.msk1; return true;
        case SAS_HBA_PHY_CHL_INT_MSK2:   data = p.msk2; return true;
        case SAS_HBA_PHY_ERR_CNT_DWS_LOST: data = p.err_dws; return true;
        case SAS_HBA_PHY_ERR_CNT_CODE:   data = p.err_code; return true;
        case SAS_HBA_PHY_ERR_CNT_DISP:   data = p.err_disp; return true;
        default: return true;
        }
    }
    // ── AXI_MASTER / RAS 段 ✓ ──
    if (off >= SAS_HBA_SEG_AXI_BASE && off < SAS_HBA_SEG_AXI_BASE + 0x20) {
        switch (off) {
        case SAS_HBA_AM_CTRL_GLOBAL:   data = r_am_ctrl; return true;
        case SAS_HBA_AM_CFG_MAX_TRANS: data = r_am_max_trans; return true;
        default: return true;
        }
    }
    if (off >= SAS_HBA_SEG_RAS_BASE && off < SAS_HBA_SEG_RAS_BASE + 0x30) {
        switch (off) {
        case SAS_HBA_RAS_INTR0:      data = r_ras0; return true;
        case SAS_HBA_RAS_INTR1:      data = r_ras1; return true;
        case SAS_HBA_RAS_INTR0_MASK: data = r_ras0_msk; return true;
        case SAS_HBA_RAS_INTR1_MASK: data = r_ras1_msk; return true;
        default: return true;
        }
    }
    decerr_cnt++;
    return false;                      // 未映射 ⇒ DECERR ✓
}

// 写：true = 地址映射（RO 写 ⇒ 忽略 + 计数，仍 OKAY ✓ 照 RTL 静默语义）
bool SasHbaTlm::reg_write_off(uint32_t off, uint32_t v)
{
    if (off < SAS_HBA_SEG_GLOBAL_END) {
        switch (off) {
        case SAS_HBA_CFG_DLVRY_QUEUE_ENABLE: r_dlq_en = v; return true;
        case SAS_HBA_CFG_IOST_BASE_LO:       r_iost_lo = v; return true;
        case SAS_HBA_CFG_IOST_BASE_HI:       r_iost_hi = v; return true;
        case SAS_HBA_CFG_ITCT_BASE_LO:       r_itct_lo = v; return true;
        case SAS_HBA_CFG_ITCT_BASE_HI:       r_itct_hi = v; return true;
        case SAS_HBA_CFG_BROKEN_MSG_LO:      r_broken_lo = v; return true;
        case SAS_HBA_CFG_BROKEN_MSG_HI:      r_broken_hi = v; return true;
        case SAS_HBA_CFG_PHY_CONTEXT:        r_phy_ctx = v; return true;
        case SAS_HBA_CFG_PHY_STATE:          r_phy_state = v; return true;
        case SAS_HBA_CFG_PHY_PORT_NUM_MA:    r_phy_port_ma = v; return true;
        case SAS_HBA_CFG_PHY_CONN_RATE:      r_phy_conn_rate = v; return true;
        case SAS_HBA_CFG_ITCT_CLR:           r_itct_clr = v; return true;
        case SAS_HBA_CFG_SATA_INIT_D2H_LO:   r_sata_lo = v; return true;
        case SAS_HBA_CFG_SATA_INIT_D2H_HI:   r_sata_hi = v; return true;
        case SAS_HBA_CFG_MAX_TAG:            r_max_tag = v; return true;
        case SAS_HBA_CFG_ITY_NEXUS_LOSS_T:   r_nexus_t = v; return true;
        case SAS_HBA_CFG_MAX_CON_TIME_LIMIT: r_maxtime_t = v; return true;
        case SAS_HBA_CFG_BUS_INACTIVE_LIMIT: r_inactive_t = v; return true;
        case SAS_HBA_CFG_REJECT_TO_OPEN_T:   r_reject_t = v; return true;
        case SAS_HBA_CFG_CQ_INT_CONVERGE_EN: r_converge_en = v; return true;
        case SAS_HBA_CFG_ABT_SET_QUERY_IPTT: r_abt_query = v; return true;
        case SAS_HBA_INT_CHNL_STATUS:        ro_wr_cnt++; return true;    // 只读 ✓
        case SAS_HBA_INT_COAL_EN:      r_coal_en = v; return true;
        case SAS_HBA_INT_OQ_COAL_TIME: r_oq_coal_t = v; return true;
        case SAS_HBA_INT_OQ_COAL_CNT:  r_oq_coal_c = v; return true;
        case SAS_HBA_INT_ENT_COAL_TIME:r_ent_coal_t = v; return true;
        case SAS_HBA_INT_ENT_COAL_CNT: r_ent_coal_c = v; return true;
        case SAS_HBA_INT_OQ_SRC:                          // pending 位图
            if (cfg_rtl_no_eoi) { ro_wr_cnt++; return true; }   // ⚠ 红线②：RTL 无 EOI，写无效 ✓
            r_oq_int_src &= ~v; return true;                    // W1C（内核期望 ✓）
        case SAS_HBA_INT_OQ_SRC_MSK:   r_oq_int_src_msk = v; return true;
        case SAS_HBA_INT_ENT_SRC1:
        case SAS_HBA_INT_ENT_SRC2:
        case SAS_HBA_INT_ENT_SRC3:     ro_wr_cnt++; return true;          // 只读 ✓
        case SAS_HBA_INT_ENT_SRC_MSK1: r_ent_msk1 = v; return true;
        case SAS_HBA_INT_ENT_SRC_MSK2: r_ent_msk2 = v; return true;
        case SAS_HBA_INT_ENT_SRC_MSK3: r_ent_msk3 = v; return true;
        case SAS_HBA_INT_PHYUPDOWN_MSK:r_phyupdown_msk = v; return true;
        case SAS_HBA_INT_CHNL_ENT_MSK: r_chnl_ent_msk = v; return true;
        case SAS_HBA_INT_HGC_COM_MSK:  r_hgc_com_msk = v; return true;
        case SAS_HBA_INT_SAS_ECC_INTR: r_ecc_intr &= ~v; return true;     // W1C ✓
        case SAS_HBA_INT_SAS_ECC_INTR_MSK: r_ecc_msk = v; return true;
        case SAS_HBA_CFG_HGC_ERR_STAT_EN:  r_hgc_en = v; return true;
        case SAS_HBA_STS_CQE_SEND_CNT: ro_wr_cnt++; return true;          // 只读 ✓
        default: return true;                                             // 段内未列 ⇒ 忽略 ✓
        }
    }
    if (off < SAS_HBA_SEG_CQ_BASE) {                    // DQ：wr_ptr = 软件生产 ✓；rd_ptr 只读 ✓
        unsigned i = SAS_HBA_DQ_IDX(off), r = SAS_HBA_DQ_OFF(off);
        if (i >= SAS_HBA_MAX_DQ) return false;
        switch (r) {
        case SAS_HBA_XX_BASE_ADDR_LO: dq[i].base_lo = v; return true;
        case SAS_HBA_XX_BASE_ADDR_HI: dq[i].base_hi = v; return true;
        case SAS_HBA_XX_DEPTH:        dq[i].depth = v; return true;
        case SAS_HBA_XX_WR_PTR:       dq[i].wr_ptr = v; return true;
        case SAS_HBA_XX_RD_PTR:       ro_wr_cnt++; return true;           // 硬件推进 ⇒ 软件只读 ✓
        default: return true;
        }
    }
    if (off < SAS_HBA_SEG_CQ_BASE + (uint32_t)SAS_HBA_MAX_DQ * SAS_HBA_CQ_STRIDE) {
        unsigned i = SAS_HBA_CQ_IDX(off), r = SAS_HBA_CQ_OFF(off);
        switch (r) {
        case SAS_HBA_XX_BASE_ADDR_LO: cq[i].base_lo = v; return true;
        case SAS_HBA_XX_BASE_ADDR_HI: cq[i].base_hi = v; return true;
        case SAS_HBA_XX_DEPTH:        cq[i].depth = v; return true;
        case SAS_HBA_XX_WR_PTR:       cq[i].wr_ptr = v; return true;      // 软件可复位 ✓
        case SAS_HBA_XX_RD_PTR:       cq[i].rd_ptr = v; return true;      // 软件确认 ✓
        default: return true;
        }
    }
    if (off >= SAS_HBA_SEG_PHY_BASE &&
        off <  SAS_HBA_SEG_PHY_BASE + (uint32_t)SAS_HBA_MAX_PHY * SAS_HBA_PHY_STRIDE) {  // ★ 须查下界 ✗ 否则空洞段下溢（实踩段错误 ✓）
        unsigned i = SAS_HBA_PHY_IDX(off), r = SAS_HBA_PHY_OFF(off);
        PhyReg &p = phy[i];
        if (r >= SAS_HBA_PHY_TX_ID_DW0 && r <= SAS_HBA_PHY_TX_ID_DW6 && (r & 3u) == 0) {
            p.tx_id[(r - SAS_HBA_PHY_TX_ID_DW0) / 4] = v; return true;
        }
        switch (r) {
        case SAS_HBA_PHY_CFG:            p.cfg = v; return true;
        case SAS_HBA_PHY_HARD_LINKRATE:  p.hard_linkrate = v; return true;
        case SAS_HBA_PHY_PROG_LINK_RATE: p.prog_rate = v; return true;
        case SAS_HBA_PHY_CTRL:           p.ctrl = v; return true;
        case SAS_HBA_PHY_SERDES_CFG:     p.serdes = v; return true;
        case SAS_HBA_PHY_SL_CFG:         p.sl_cfg = v; return true;
        case SAS_HBA_PHY_SL_CONTROL:     p.sl_ctl = v; return true;
        case SAS_HBA_PHY_TXID_AUTO:      p.txid_auto = v; return true;
        case SAS_HBA_PHY_RX_IDAF_DW0:    p.rx_idaf = v; return true;
        case SAS_HBA_PHY_CON_CFG_DRIVER: p.con_cfg_drv = v; return true;
        case SAS_HBA_PHY_SSP_CON_TIMER:  p.ssp_timer = v; return true;
        case SAS_HBA_PHY_SMP_CON_TIMER:  p.smp_timer = v; return true;
        case SAS_HBA_PHY_STP_CON_TIMER:  p.stp_timer = v; return true;
        case SAS_HBA_PHY_CHL_INT0:                       // ★ 全线唯一 W1C 通路（regfile ✓）
            p.chl_int0 = (p.chl_int0 & ~(v & SAS_HBA_CHL0_SL_PHY_ENABLE))  // bit2：写 1 清 ✓
                       | (v & ~SAS_HBA_CHL0_SL_PHY_ENABLE);                // 其余：写 1 置（RAW ✓）
            return true;
        case SAS_HBA_PHY_CHL_INT1:       p.chl_int1 = v; return true;
        case SAS_HBA_PHY_CHL_INT2:       p.chl_int2 = v; return true;
        case SAS_HBA_PHY_CHL_INT_MSK0:   p.msk0 = v; return true;
        case SAS_HBA_PHY_CHL_INT_MSK1:   p.msk1 = v; return true;
        case SAS_HBA_PHY_CHL_INT_MSK2:   p.msk2 = v; return true;
        case SAS_HBA_PHY_ERR_CNT_DWS_LOST: p.err_dws = v; return true;
        case SAS_HBA_PHY_ERR_CNT_CODE:   p.err_code = v; return true;
        case SAS_HBA_PHY_ERR_CNT_DISP:   p.err_disp = v; return true;
        default: return true;
        }
    }
    if (off >= SAS_HBA_SEG_AXI_BASE && off < SAS_HBA_SEG_AXI_BASE + 0x20) {
        switch (off) {
        case SAS_HBA_AM_CTRL_GLOBAL:   r_am_ctrl = v; return true;
        case SAS_HBA_AM_CFG_MAX_TRANS: r_am_max_trans = v; return true;
        default: return true;
        }
    }
    if (off >= SAS_HBA_SEG_RAS_BASE && off < SAS_HBA_SEG_RAS_BASE + 0x30) {
        switch (off) {
        case SAS_HBA_RAS_INTR0:      r_ras0 &= ~v; return true;   // W1C ✓
        case SAS_HBA_RAS_INTR1:      r_ras1 &= ~v; return true;
        case SAS_HBA_RAS_INTR0_MASK: r_ras0_msk = v; return true;
        case SAS_HBA_RAS_INTR1_MASK: r_ras1_msk = v; return true;
        default: return true;
        }
    }
    decerr_cnt++;
    return false;                       // 未映射 ⇒ DECERR ✓
}

// ── AMBA-PV 从口（b_transport 经基类转此处 ⇒ 与直连同源 ✓）──
amba_pv::amba_pv_resp_t SasHbaTlm::read(int, const sc_dt::uint64 &addr, unsigned char *data,
                                         unsigned int len, const amba_pv::amba_pv_control *,
                                         sc_core::sc_time &)
{
    if (len != 4 || data == nullptr) return amba_pv::AMBA_PV_SLVERR;
    sc_uint<32> v = 0;
    if (!reg_read(addr, v)) return amba_pv::AMBA_PV_DECERR;
    data[0] = (unsigned char)v.range(7, 0);
    data[1] = (unsigned char)v.range(15, 8);
    data[2] = (unsigned char)v.range(23, 16);
    data[3] = (unsigned char)v.range(31, 24);
    reg_sock_n++;
    return amba_pv::AMBA_PV_OKAY;
}

amba_pv::amba_pv_resp_t SasHbaTlm::write(int, const sc_dt::uint64 &addr, unsigned char *data,
                                         unsigned int len, const amba_pv::amba_pv_control *,
                                         unsigned char *byte_en, sc_core::sc_time &)
{
    (void)byte_en;                      // 寄存器面 = 4B 全字节（照先例明写 ✓）
    if (len != 4 || data == nullptr) return amba_pv::AMBA_PV_SLVERR;
    sc_uint<32> v = 0;
    v.range(7, 0)   = (unsigned)data[0];
    v.range(15, 8)  = (unsigned)data[1];
    v.range(23, 16) = (unsigned)data[2];
    v.range(31, 24) = (unsigned)data[3];
    if (!reg_write(addr, v)) return amba_pv::AMBA_PV_DECERR;   // 未映射 ⇒ DECERR ✓
    reg_sock_n++;
    return amba_pv::AMBA_PV_OKAY;                              // RO 写 = 忽略式 OKAY ✓
}

// ═════════════ DMA（零时 b_transport ✓；非 OKAY/非零延迟 ⇒ 记数不静默 ✗✓）═════════════
bool SasHbaTlm::dma_read(uint64_t addr, void *dst, unsigned len)
{
    amba_pv::amba_pv_trans_ptr tr(pool.allocate(1, len,
        (const amba_pv::amba_pv_control *)NULL, amba_pv::AMBA_PV_INCR));
    tr->set_command(tlm::TLM_READ_COMMAND);
    tr->set_address(addr);
    tr->set_data_length(len);
    tr->set_data_ptr((unsigned char *)dst);
    amba_pv::amba_pv_extension *ex = NULL;
    tr->get_extension(ex);
    sc_core::sc_time t = sc_core::SC_ZERO_TIME;
    dma_s.b_transport(*tr, t);
    c_dma_r++;
    if (t != sc_core::SC_ZERO_TIME) dma_late_cnt++;
    bool ok = (!ex) || (ex->get_resp() == amba_pv::AMBA_PV_OKAY);
    if (!ok) dma_fail_cnt++;
    return ok;
}

bool SasHbaTlm::dma_write(uint64_t addr, const void *src, unsigned len)
{
    amba_pv::amba_pv_trans_ptr tr(pool.allocate(1, len,
        (const amba_pv::amba_pv_control *)NULL, amba_pv::AMBA_PV_INCR));
    tr->set_command(tlm::TLM_WRITE_COMMAND);
    tr->set_address(addr);
    tr->set_data_length(len);
    tr->set_data_ptr((unsigned char *)src);
    amba_pv::amba_pv_extension *ex = NULL;
    tr->get_extension(ex);
    sc_core::sc_time t = sc_core::SC_ZERO_TIME;
    dma_s.b_transport(*tr, t);
    c_dma_w++;
    if (t != sc_core::SC_ZERO_TIME) dma_late_cnt++;
    bool ok = (!ex) || (ex->get_resp() == amba_pv::AMBA_PV_OKAY);
    if (!ok) dma_fail_cnt++;
    return ok;
}

// ═════════════ SSP 发送（帧队列 + held-valid ✓）═════════════
// ⚠ 纪律（实踩修正 ✗→✓）：**推进与呈现必须分拍** —— 每个 beat 给对端一个
//   **完整窗口**（本拍呈现 → 下一拍才按 ready 推进/完成）；否则末拍会与
//   「完成清 valid」同拍双写 ⇒ 末拍窗口消失 ⇒ 对端永远收不到 EOF ✗
//   （曾致 link 只送到 13/14 拍、HDD 无帧成帧 ⇒ 全链路超时 ✓ 插桩实证 ✓）
void SasHbaTlm::tx_begin(uint8_t ft, int ndw)
{
    tx_active    = true;
    tx_total     = ndw;
    tx_idx       = 0;
    tx_ft        = ft;
    tx_presented = false;
}

bool SasHbaTlm::tx_run()
{
    if (!tx_active) { tx_valid.write(false); tx_sof.write(false); tx_eof.write(false); return true; }
    if (tx_presented) {
        if (tx_ready.read()) {                 // ready = 上一窗口对端的受理 ✓
            tx_idx++;
            if (tx_idx >= tx_total) {          // 帧完成 ⇒ **本拍**清 valid（与呈现分拍 ✓）
                tx_active = false; tx_presented = false;
                tx_valid.write(false); tx_sof.write(false); tx_eof.write(false);
                return true;
            }
        }
        // 未受理 ⇒ 原 beat 重呈现（held-valid ✓）
    }
    tx_data.write(sc_uint<32>(tx_dw[tx_idx]));
    tx_type.write(sc_uint<4>(tx_ft));
    tx_valid.write(true);
    tx_sof.write(tx_idx == 0);
    tx_eof.write(tx_idx == tx_total - 1);
    tx_presented = true;
    return false;
}

void SasHbaTlm::queue_frame(uint8_t ft, int ndw)
{
    if (ndw > FRAME_MAX_DW) ndw = FRAME_MAX_DW;
    TlsFrame f; f.n = ndw; f.ft = ft;
    for (int i = 0; i < ndw; i++) f.dw[i] = tx_dw[i];
    txq.push_back(f);
}

void SasHbaTlm::tx_engine()
{
    if (!tx_active && !txq.empty()) {
        TlsFrame f = txq.front(); txq.erase(txq.begin());
        for (int i = 0; i < f.n; i++) tx_dw[i] = f.dw[i];
        tx_begin(f.ft, f.n);
    }
    if (tx_active && tx_run()) {
        c_frames_tx++;          // ⛔ 勿在此再清 valid（tx_run 完成分支已写 ✓ 防同拍双写 ✗）
    }
}

// ═════════════ SSP 接收装配（每拍 1 dword ✓）═════════════
void SasHbaTlm::rx_engine()
{
    if (rx_valid.read() && rx_ready.read()) {
        if (rx_sof.read()) { rx_dwc = 0; rx_ft_lat = (uint8_t)rx_type.read().to_uint(); }
        if (rx_dwc < FRAME_MAX_DW) rx_dw[rx_dwc] = rx_data.read().to_uint();
        rx_dwc++;
        if (rx_eof.read()) {
            c_frames_rx++;
            on_rx_frame(rx_ft_lat);
            rx_dwc = 0;
        }
    }
}

// 帧尾处理：DATA ⇒ 落内存（SGE ✓）；RESPONSE ⇒ 取 status（tag 须匹配 ✓）
void SasHbaTlm::on_rx_frame(uint8_t ft)
{
    if (st != H_WAIT_RESP && st != H_WRDATA) return;         // 非法态收帧 ⇒ 忽略（计数已加 ✓）
    if (ft == SAS_FT_DATA) {
        if (cur_dir == SAS_HBA_DIR_TO_INI && rx_dwc >= SAS_DATA_HDR_DW) {
            uint32_t dw5 = rx_dw[SAS_DATA_DW_OFFSET];
            uint32_t off = ((dw5 & 0xFFu) << 24) | (((dw5 >> 8) & 0xFFu) << 16)
                         | (((dw5 >> 16) & 0xFFu) << 8) | ((dw5 >> 24) & 0xFFu);   // be32 ✓
            unsigned nbytes = (unsigned)(rx_dwc - SAS_DATA_HDR_DW) * 4;
            if (cur_sge_addr && ((uint64_t)off + nbytes) <= (uint64_t)cur_xfer_len) {
                dma_write(cur_sge_addr + off, &rx_dw[SAS_DATA_HDR_DW], nbytes);    // 帧字节口径 ✓
                c_data_rx += nbytes;
            } else {
                data_drop_cnt++;                     // 无处可落（无 SGE / 越界）⇒ 计数不静默 ✗✓
            }                                        // （⛔ 不计入 tag 失配 —— 二者语义不同 ✓）
        }
    } else if (ft == SAS_FT_RESPONSE && rx_dwc >= SAS_RESP_FRAME_DW) {
        // tag = be16 @DW4（帧字节 16-17 ✓，与 `RTL 源` 同 ✓）
        uint16_t tag = (uint16_t)(((rx_dw[SAS_RESP_DW_TAG] & 0xFFu) << 8) |
                                   ((rx_dw[SAS_RESP_DW_TAG] >> 8) & 0xFFu));
        uint8_t  status = (uint8_t)(rx_dw[SAS_RESP_DW_STATUS] >> 24);   // byte35 ✓
        if (tag != cur_tag) { c_tag_mm++; return; }                     // 失配 ⇒ 不认 ✓
        last_ssp_status = status;
        if (cur_sts_addr) dma_write(cur_sts_addr, rx_dw, SAS_RESP_FRAME_DW * 4);  // 响应原文留档 ✓
        cmpl_got_resp = true;
        cmpl_status   = status;
        cmpl_err      = (status != SAS_SCSI_GOOD);
        st = H_CQE_WR;
    }
    // XFER_RDY/TASK/其它 ⇒ 忽略 ✓（本工程读路径直发 DATA ✓）
}

// ═════════════ 完成：CQE + 指针推进 + 中断 ═════════════
void SasHbaTlm::build_cqe(uint32_t cqe[4], bool err, uint8_t /*ssp_status*/)
{
    for (int i = 0; i < 4; i++) cqe[i] = 0;
    cqe[0] = (err ? SAS_HBA_CQE_CMPLT_ERR : SAS_HBA_CQE_CMPLT_OK)
           | (cmpl_got_resp ? (1u << SAS_HBA_CQE0_RSPNS_XFRD) : 0u)
           | ((cmpl_got_resp && !err) ? (1u << SAS_HBA_CQE0_RSPNS_GOOD) : 0u);
    cqe[1] = ((uint32_t)cur_dev_id << 16) | (uint32_t)cur_iptt;   // iptt@[15:0] ✓
    // dw2/dw3 = 0（SSP 恒 0 ✓ `RTL 源`）
}

void SasHbaTlm::finish_cmd(bool err, uint8_t ssp_status)
{
    uint32_t cqe[4];
    build_cqe(cqe, err, ssp_status);
    int ci = (cur_dqi >= 0 && cur_dqi < SAS_HBA_MAX_DQ) ? cur_dqi : 0;
    CqReg &c = cq[ci];
    if (c.depth) {                                        // CQ 写 ✓（模型口径：DQ i ↔ CQ i ✓）
        uint64_t a = cq_base(ci) + (uint64_t)c.wr_ptr * SAS_HBA_CQE_BYTES;
        dma_write(a, cqe, SAS_HBA_CQE_BYTES);
        c.wr_ptr = (c.wr_ptr + 1u) % c.depth;
    }
    c_cqes++;
    uint32_t bit = 1u << ((unsigned)cur_dqi & 31u);
    if (!(r_oq_int_src & bit)) c_irq++;                   // 中断按上升沿计数 ✓
    r_oq_int_src |= bit;
    DqReg &d = dq[ci];
    if (d.depth) d.rd_ptr = (d.rd_ptr + 1u) % d.depth;    // rd_ptr 推进 ✓（RTL `dq_engine）
}

void SasHbaTlm::refresh_irq()
{
    irq.write((r_oq_int_src & ~r_oq_int_src_msk) != 0);   // 0 位 = 放行 ✓
}

// ═════════════ 主状态机（每拍一步 ✓）═════════════
void SasHbaTlm::step()
{
    if (!rst_n.read()) {
        tx_valid.write(false); tx_sof.write(false); tx_eof.write(false);
        tx_data.write(0); tx_type.write(0);
        rx_ready.write(false);
        irq.write(false);
        dq_cq_defaults();
        st = H_IDLE; lat_cnt = 0; cur_dqi = 0; resp_cnt = 0;
        wd_left = 0; wd_off = 0; tag_gen = 0;
        tx_active = false; tx_presented = false; txq.clear();
        rx_dwc = 0; rx_ft_lat = 0;
        cmpl_err = false; cmpl_got_resp = false; cmpl_status = 0;
        c_done = c_err = c_cqes = c_irq = c_dq_fetches = c_ct_fetches = 0;
        c_dma_r = c_dma_w = c_frames_tx = c_frames_rx = c_tag_mm = c_data_rx = 0;
        last_iptt = 0; last_ssp_status = 0;
        o_cmds_done.write(0); o_cmds_err.write(0); o_cqes.write(0); o_irq_cnt.write(0);
        o_dq_fetches.write(0); o_ct_fetches.write(0);
        o_dma_reads.write(0); o_dma_writes.write(0);
        o_frames_tx.write(0); o_frames_rx.write(0);
        o_tag_mismatch.write(0); o_data_rx_bytes.write(0);
        o_last_iptt.write(0); o_last_ssp_status.write(0);
        o_dbg_state.write(0);
        return;
    }

    rx_ready.write(true);       // 恒可收（简化 ✓）
    refresh_irq();

    tx_engine();                // ① TX（与 RX 全双工 ✓）
    rx_engine();                // ② RX 装配

    switch (st) {
    // ── 空闲：扫 DQ（使能 + 非空 ⇒ 起取指 ✓；优先级 = 序号小者 ✓ 简化）──
    case H_IDLE: {
        o_dbg_state.write(0x10);
        for (int i = 0; i < SAS_HBA_MAX_DQ; i++) {
            if (!((r_dlq_en >> i) & 1u)) continue;
            if (!dq[i].depth) continue;
            if (dq[i].wr_ptr == dq[i].rd_ptr) continue;
            cur_dqi = i;
            uint64_t a = dq_base(i) + (uint64_t)dq[i].rd_ptr * SAS_HBA_DQ_ENTRY_BYTES;
            if (!dma_read(a, dq_entry, SAS_HBA_DQ_ENTRY_BYTES)) {
                cmpl_err = true; cmpl_got_resp = false; cmpl_status = 0xFF;
                cur_iptt = 0; cur_dev_id = 0;        // ⚠ 取指失败 ⇒ iptt 不可知 ⇒ 显式清 0
                                                     //   （⛔ 勿留上一条残值 —— 台曾抓出 ✗）
                st = H_CQE_WR;                       // DMA 失败 ⇒ 错误 CQE（不静默 ✓）
                break;
            }
            c_dq_fetches++;
            lat_cnt = cfg_fetch_lat;
            st = H_DQ_FETCH;
            break;
        }
        break;
    }
    // ── DQ 条目读延迟（RTL 4 拍 ✓）──
    case H_DQ_FETCH: {
        o_dbg_state.write(0x20);
        if (lat_cnt) lat_cnt--;
        if (!lat_cnt) st = H_PARSE;
        break;
    }
    // ── 解析（位域照 `sw/sas_hba_regs.h` §6 ✓）──
    case H_PARSE: {
        o_dbg_state.write(0x30);
        cur_iptt     = (uint16_t)sas_bf_get_dw(dq_entry, 3, SAS_HBA_DQ3_IPTT_LO, 16);
        unsigned cmd = sas_bf_get_dw(dq_entry, 0, SAS_HBA_DQ0_CMD_LO, 3);
        cur_dir      = (uint8_t)sas_bf_get_dw(dq_entry, 1, SAS_HBA_DQ1_DIR_LO, 2);
        cur_dev_id   = (uint16_t)sas_bf_get_dw(dq_entry, 1, SAS_HBA_DQ1_DEV_ID_LO, 16);
        cur_xfer_len = dq_entry[4];
        cur_ct_addr  = sas_dw_get_u64(dq_entry, SAS_HBA_DQ_CMD_TABLE_LO_DW);
        cur_prd_addr = sas_dw_get_u64(dq_entry, SAS_HBA_DQ_PRD_TABLE_LO_DW);
        cur_sts_addr = sas_dw_get_u64(dq_entry, SAS_HBA_DQ_STS_BUF_LO_DW);
        last_iptt = cur_iptt;
        cmpl_err = false; cmpl_got_resp = false; cmpl_status = 0;
        if (cmd != SAS_HBA_CMD_SSP || !cur_ct_addr) {      // 非 SSP / 无命令表 ⇒ 错误 CQE ✓（简化）
            cmpl_err = true; cmpl_status = 0xFF;
            st = H_CQE_WR;
            break;
        }
        if (!dma_read(cur_ct_addr, cmd_tbl, sizeof(cmd_tbl))) {
            cmpl_err = true; cmpl_status = 0xFF;
            st = H_CQE_WR;
            break;
        }
        c_ct_fetches++;
        lat_cnt = cfg_ct_lat;
        st = H_CT_FETCH;
        break;
    }
    // ── 命令表读延迟（RTL 63 拍 ⇒ 模型 8 拍，简化 ✓）──
    case H_CT_FETCH: {
        o_dbg_state.write(0x40);
        if (lat_cnt) lat_cnt--;
        if (!lat_cnt) st = H_ISSUE;
        break;
    }
    // ── 组 COMMAND 帧并发出（14 dword ✓ CDB@36 ✓ LUN@24 ✓）──
    case H_ISSUE: {
        o_dbg_state.write(0x50);
        cur_tag = ++tag_gen;                               // 发起方 tag（递增；参考平台取池 ✓ 简化）
        for (int i = 0; i < SAS_CMD_FRAME_DW; i++) tx_dw[i] = 0;
        tx_dw[0] = SAS_FRAME_B0_COMMAND;                   // 0x10 ✓（目的哈希 lo24 = 0，点对点简化 ✓）
        tx_dw[SAS_CMD_DW_TAG]  = ((uint32_t)(cur_tag & 0xFFu) << 8) |
                                 ((uint32_t)(cur_tag >> 8) & 0xFFu);   // tag be16 ✓
        tx_dw[SAS_CMD_DW_LUN_LO]     = cmd_tbl[SAS_CMD_DW_LUN_LO];    // LUN@24-27 ✓
        tx_dw[SAS_CMD_DW_LUN_LO + 1] = cmd_tbl[SAS_CMD_DW_LUN_LO + 1];// LUN@28-31 ✓
        tx_dw[SAS_CMD_DW_ATTR]       = cmd_tbl[SAS_CMD_DW_ATTR];      // attr/TMF@32-35 ✓
        for (int i = 0; i < 4; i++) tx_dw[SAS_CMD_DW_CDB_LO + i] = cmd_tbl[SAS_CMD_DW_CDB_LO + i]; // CDB@36-51 ✓
        queue_frame(SAS_FT_COMMAND, SAS_CMD_FRAME_DW);
        // 数据路径：取首 SGE（⚠ 单 SGE 简化 ✓）
        cur_sge_addr = 0; cur_sge_len = 0;
        if (cur_dir != SAS_HBA_DIR_NO_DATA && cur_prd_addr && cur_xfer_len) {
            uint8_t sge[SAS_HBA_SGE_BYTES];
            if (dma_read(cur_prd_addr, sge, SAS_HBA_SGE_BYTES)) {
                memcpy(&cur_sge_addr, sge + SAS_HBA_SGE_ADDR_LO, 8);      // 内核布局 ✓（红线⑤）
                memcpy(&cur_sge_len,  sge + SAS_HBA_SGE_DATA_LEN, 4);
            }
        }
        resp_cnt = 0; wd_off = 0;
        if (cur_dir == SAS_HBA_DIR_TO_DEVICE && cur_xfer_len) {
            wd_left = cur_xfer_len;
            st = H_WRDATA;
        } else {
            st = H_WAIT_RESP;
        }
        break;
    }
    // ── 写方向：从内存取、切帧发（每拍最多排一帧 ✓ ≤250 dword ✓）──
    case H_WRDATA: {
        o_dbg_state.write(0x70);
        if (!txq.empty()) break;                           // 等队列排干 ✓
        uint32_t chunk = wd_left;
        if (chunk > 1000u) chunk = 1000u;                  // 250 dword ✓
        if (!dma_read(cur_sge_addr + wd_off, wd_buf, chunk)) {
            cmpl_err = true; cmpl_status = 0xFF;
            st = H_CQE_WR;
            break;
        }
        int pldw = (int)(chunk / 4);
        for (int i = 0; i < SAS_DATA_HDR_DW; i++) tx_dw[i] = 0;
        tx_dw[0] = SAS_FRAME_B0_DATA;                      // 0x60 ✓
        tx_dw[SAS_DATA_DW_TAG] = ((uint32_t)(cur_tag & 0xFFu) << 8) |
                                 ((uint32_t)(cur_tag >> 8) & 0xFFu);
        tx_dw[SAS_DATA_DW_OFFSET] = sas_bswap32(wd_off);   // DataOffset be32 ✓
        for (int i = 0; i < pldw; i++)
            tx_dw[SAS_DATA_HDR_DW + i] = (uint32_t)wd_buf[i * 4 + 0]
                                       | ((uint32_t)wd_buf[i * 4 + 1] << 8)
                                       | ((uint32_t)wd_buf[i * 4 + 2] << 16)
                                       | ((uint32_t)wd_buf[i * 4 + 3] << 24);
        queue_frame(SAS_FT_DATA, SAS_DATA_HDR_DW + pldw);
        wd_left -= chunk; wd_off += chunk;
        if (!wd_left) { resp_cnt = 0; st = H_WAIT_RESP; }
        break;
    }
    // ── 等响应（超时 ⇒ 错误 CQE ✓ —— 复现参考平台"帧发出去没回"卡点场景 ✓）──
    case H_WAIT_RESP: {
        o_dbg_state.write(0x60);
        if (resp_cnt < 0xFFFFFFFFu) resp_cnt++;
        if (cfg_resp_timeout && resp_cnt > cfg_resp_timeout) {
            cmpl_err = true; cmpl_got_resp = false; cmpl_status = 0xFF;
            st = H_CQE_WR;
        }
        break;
    }
    // ── 完成：写 CQE、置 pending、rd_ptr++ ✓ ──
    case H_CQE_WR: {
        o_dbg_state.write(0x80);
        finish_cmd(cmpl_err, cmpl_status);
        if (cmpl_err) c_err++; else c_done++;      // ★ 统一在此计数（勿在错误路径内联 ✗）
        st = H_IDLE;
        break;
    }
    default: st = H_IDLE; break;
    }

    // ── 观测口（每拍刷新 ✓）──
    o_cmds_done.write(c_done);   o_cmds_err.write(c_err);
    o_cqes.write(c_cqes);        o_irq_cnt.write(c_irq);
    o_dq_fetches.write(c_dq_fetches); o_ct_fetches.write(c_ct_fetches);
    o_dma_reads.write(c_dma_r);  o_dma_writes.write(c_dma_w);
    o_frames_tx.write(c_frames_tx); o_frames_rx.write(c_frames_rx);
    o_tag_mismatch.write(c_tag_mm); o_data_rx_bytes.write(c_data_rx);
    o_last_iptt.write(last_iptt); o_last_ssp_status.write(last_ssp_status);
}
