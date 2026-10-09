//=============================================================================
// sas_hba_tlm.h — SAS HBA（hisi_sas v3 兼容）事务级模型  [2026-10-08 建]
//-----------------------------------------------------------------------------
// 规格出处（全部 RTL/内核实证，逐条见 `sw/sas_hba_regs.h` 与各行为注释 ✓）：
//   · 寄存器面 = `RTL 源` + `RTL 源`
//   · DQ 引擎  = `RTL 源`（64B 条目 / 4 拍读 / rd_ptr 推进 ✓）
//   · 命令表  = `RTL 源`（CDB@36 ✓ LUN@24 ✓）
//   · SSP 发  = `RTL 源`（COMMAND 14 dword ✓ tag@16-17 be16 ✓）
//   · SSP 收  = `RTL 源`（RESPONSE 12 dword / status@byte35 ✓）
//   · 完成    = `RTL 源`（CQE 16B 布局 ✓ 见 regs.h §7）
//
// ★ 结构（照先例 `rdma400/tlm/model/device/device_tlm.h` ✓）：
//   · 主机面 = **AMBA-PV 从口**（`reg_s`；b_transport 经基类转 read()/write()
//     ⇒ 与直连 `reg_read/reg_write` **同源** ✓ 两路逐寄存器一致 ✓）
//   · 内存面 = **AMBA-PV 主口**（`dma_s`；DQ/命令表/SGE/CQE/数据 全走它 ✓）
//     ⚠ DMA 用**零时 b_transport**、延迟自记拍数 ⇒ 可从 SC_METHOD 调用（节气纪律 ✓）
//   · SAS 面 = 帧级 pin 口（与 `sas_link_tlm`/HDD 模型同构 ✓）
//
// ★ 流程（每拍一步 ✓）：DLVRY_QUEUE_ENABLE[DQ] 使能 → 扫 wr_ptr≠rd_ptr →
//   读 DQ 条目（64B）→ 读命令表（64B，含 CDB@36 ✓）→ 组 COMMAND 帧发 SSP →
//   收 DATA（写内存 SGE ✓）/ RESPONSE（取 status ✓）→ 写 CQE 到 CQ 环 →
//   置 OQ pending + IRQ → rd_ptr++ ✓
//
// ⛔ 简化（明写，见 md/设计方案.md）：
//   · 无 PHY/链路训练/拓扑发现（点对点 ✓）；PHY 段寄存器可读可写、CHL_INT0 bit2=1
//   · SSP OPEN/CLOSE/TASK(TMF) 帧不发；SMP/STP 不实现（cmd≠SSP ⇒ 错误 CQE ✓）
//   · SGE 只取**首个**（多 SGE 连续映射 = 扩展点 ✓）；单命令在飞（无多命令并发 ✓）
//   · 中断 = 一根电平线（参考平台 MSI 两向量 —— 本模型取 OR ✓）；无 coalescing 定时器
//   · 超时/TAG 失配/坏响应 ⇒ 错误 CQE（判据可测 ✓），无链路重传（见 sas_link_tlm ✓）
//
// ⚠ 红线②（RTL 无 EOI）：模型默认实现**内核期望的 W1C**；置 `cfg_rtl_no_eoi=1`
//   复现 RTL（OQ_INT_SRC 只读、写无效 ✓）—— 对拍与负控都要用 ✓
//=============================================================================
#ifndef SAS_HBA_TLM_H
#define SAS_HBA_TLM_H

#include <systemc.h>
#include <amba_pv.h>
#include <cstdint>
#include <vector>
#include "sas_hba_regs.h"

#define SAS_HBA_TLM_VERSION_MAJOR 1
#define SAS_HBA_TLM_VERSION_MINOR 0
#define SAS_HBA_TLM_VERSION_STR   "1.0"

struct SasHbaTlm : sc_core::sc_module,
                   amba_pv::amba_pv_slave_base<32>,      // CSR 从口 ✓
                   amba_pv::amba_pv_master_base {        // DMA 主口 ✓
    // 帧缓冲上限（dword ✓；与 HDD 模型 256 一致 ✓ 见 sas_hdd_tlm.h）
    static const int FRAME_MAX_DW = 256;

    // ── 主机面：CSR（AMBA-PV 从口；平台/TB 的 master 绑此处 ✓）──
    amba_pv::amba_pv_slave_socket<32>  reg_s;
    // ── 内存面：DMA（AMBA-PV 主口；绑内存/SasMemTlm ✓）──
    amba_pv::amba_pv_master_socket<32> dma_s;
    amba_pv::amba_pv_trans_pool        pool;

    // ── 时钟/复位（端口，构造期不写 ✓）──
    sc_core::sc_in<bool> clk, rst_n;

    // ── SSP 帧级口（与 link/HDD 同构 ✓；帧型常量见 sas_hba_regs.h §10 ✓）──
    sc_core::sc_out<sc_uint<32>> tx_data;
    sc_core::sc_out<bool>        tx_valid, tx_sof, tx_eof;
    sc_core::sc_out<sc_uint<4>>  tx_type;
    sc_core::sc_in<bool>         tx_ready;
    sc_core::sc_in<sc_uint<32>>  rx_data;
    sc_core::sc_in<bool>         rx_valid, rx_sof, rx_eof;
    sc_core::sc_in<sc_uint<4>>   rx_type;
    sc_core::sc_out<bool>        rx_ready;

    // ── 中断（简化：一根电平线 = OQ pending & ~掩码 ✓）──
    sc_core::sc_out<bool> irq;

    // ── 白盒观测（⛔ 只 TB 用 ✓）──
    sc_core::sc_out<sc_uint<32>> o_cmds_done;      // 成功完成命令数
    sc_core::sc_out<sc_uint<32>> o_cmds_err;       // 错误/超时命令数
    sc_core::sc_out<sc_uint<32>> o_cqes;           // 写出的 CQE 数
    sc_core::sc_out<sc_uint<32>> o_irq_cnt;        // 中断上升沿数
    sc_core::sc_out<sc_uint<32>> o_dq_fetches;     // DQ 条目读次数
    sc_core::sc_out<sc_uint<32>> o_ct_fetches;     // 命令表读次数
    sc_core::sc_out<sc_uint<32>> o_dma_reads;      // DMA 读次数
    sc_core::sc_out<sc_uint<32>> o_dma_writes;     // DMA 写次数
    sc_core::sc_out<sc_uint<32>> o_frames_tx;      // 发出的 SSP 帧数
    sc_core::sc_out<sc_uint<32>> o_frames_rx;      // 收完的 SSP 帧数
    sc_core::sc_out<sc_uint<32>> o_tag_mismatch;   // 响应 tag 失配数（不静默 ✗✓）
    sc_core::sc_out<sc_uint<32>> o_data_rx_bytes;  // DATA-in 累计字节
    sc_core::sc_out<sc_uint<16>> o_last_iptt;      // 最近命令 iptt
    sc_core::sc_out<sc_uint<8>>  o_last_ssp_status;// 最近 SSP status（响应帧 byte35 ✓）
    sc_core::sc_out<sc_uint<32>> o_dbg_state;      // 状态机（调试）

    // ── 配置（构造后改可 ✓ 逐拍生效；0 = 关/默认语义见注释）──
    unsigned cfg_fetch_lat;     // DQ 条目读延迟（拍；默认 4 = RTL 4 拍 ✓）
    unsigned cfg_ct_lat;        // 命令表读延迟（拍；默认 8 —— RTL 63 拍，见简化清单 ✓）
    unsigned cfg_resp_timeout;  // 等响应超时（拍；默认 4096 ✓ TB 可调小做负控 ✓）
    unsigned cfg_rtl_no_eoi;    // 1 = 复现 RTL「无 EOI」（W1C 写无效 ✓ 红线②）；0 = 内核期望 ✓
    uint32_t cfg_addr_mask;     // CSR 地址窗掩码（默认 0xFFFFF ✓ 兼容任意映射基址）

    SC_HAS_PROCESS(SasHbaTlm);
    SasHbaTlm(sc_core::sc_module_name nm);

    // ── 直连寄存器（与 socket 同源 ✓；addr 取低 cfg_addr_mask 位 ✓）──
    bool reg_read (sc_dt::uint64 addr, sc_uint<32> &data);
    bool reg_write(sc_dt::uint64 addr, sc_uint<32> v);

    // ── AMBA-PV 从口覆写（与先例同签名 ✓）──
    virtual amba_pv::amba_pv_resp_t read(int, const sc_dt::uint64 &addr, unsigned char *data,
                                         unsigned int len, const amba_pv::amba_pv_control *,
                                         sc_core::sc_time &delay) override;
    virtual amba_pv::amba_pv_resp_t write(int, const sc_dt::uint64 &addr, unsigned char *data,
                                          unsigned int len, const amba_pv::amba_pv_control *,
                                          unsigned char *byte_en, sc_core::sc_time &delay) override;

    // 观测计数（TB 直读 ✓）
    unsigned long reg_sock_n;   // 经 socket 的成功访问数
    unsigned long ro_wr_cnt;    // RO 写被忽略次数
    unsigned long decerr_cnt;   // 未映射访问数
    unsigned long dma_fail_cnt; // DMA 非 OKAY 次数（不静默 ✗✓）
    unsigned long dma_late_cnt; // DMA 返回非零延迟次数（本模型忽略延迟 ⇒ 记数 ✓）
    unsigned long data_drop_cnt;// DATA 帧无处可落（无 SGE/越界）次数（不静默 ✗✓）

    static const char *version() { return SAS_HBA_TLM_VERSION_STR; }

private:
    // ── 每拍一步（SC_METHOD@clk.pos ✓ 非阻塞、无 wait ✓）──
    void step();
    void tx_engine();           // SSP 发送子引擎（每拍一步 ✓）
    void rx_engine();           // SSP 接收装配（每拍一步 ✓ 帧尾回调 on_rx_frame ✓）
    void on_rx_frame(uint8_t ft);
    void refresh_irq();         // 电平 = OQ pending & ~掩码 ✓

    // ── 寄存器堆（未列寄存器 = 读了 0、写忽略 ✓ 见 reg_read_off/reg_write_off）──
    struct DqReg { uint32_t base_lo, base_hi, depth, wr_ptr, rd_ptr; };
    struct CqReg { uint32_t base_lo, base_hi, depth, wr_ptr, rd_ptr; };
    struct PhyReg {
        uint32_t cfg, hard_linkrate, prog_rate, ctrl, serdes, sl_cfg, sl_ctl;
        uint32_t tx_id[7]; uint32_t txid_auto; uint32_t rx_idaf;
        uint32_t con_cfg_drv, ssp_timer, smp_timer, stp_timer;
        uint32_t chl_int0, chl_int1, chl_int2, msk0, msk1, msk2;
        uint32_t err_dws, err_code, err_disp;
    };
    DqReg  dq[SAS_HBA_MAX_DQ];
    CqReg  cq[SAS_HBA_MAX_DQ];
    PhyReg phy[SAS_HBA_MAX_PHY];
    uint32_t r_dlq_en, r_iost_lo, r_iost_hi, r_itct_lo, r_itct_hi;
    uint32_t r_broken_lo, r_broken_hi, r_phy_ctx, r_phy_state, r_phy_port_ma, r_phy_conn_rate;
    uint32_t r_itct_clr, r_sata_lo, r_sata_hi, r_max_tag;
    uint32_t r_nexus_t, r_maxtime_t, r_inactive_t, r_reject_t;
    uint32_t r_converge_en, r_abt_query, r_abt_done;
    uint32_t r_coal_en, r_oq_coal_t, r_oq_coal_c, r_ent_coal_t, r_ent_coal_c;
    uint32_t r_oq_int_src;          // 完成中断位图（每 CQ 一位 ✓）
    uint32_t r_oq_int_src_msk;      // 1 = 掩码（默认全掩 ✓ TB 须解掩）
    uint32_t r_ent_msk1, r_ent_msk2, r_ent_msk3;
    uint32_t r_phyupdown_msk, r_chnl_ent_msk, r_hgc_com_msk;
    uint32_t r_ecc_intr, r_ecc_msk, r_hgc_en;
    uint32_t r_am_ctrl, r_am_max_trans;
    uint32_t r_ras0, r_ras1, r_ras0_msk, r_ras1_msk;
    uint64_t dq_base(int i) const { return ((uint64_t)dq[i].base_hi << 32) | dq[i].base_lo; }
    uint64_t cq_base(int i) const { return ((uint64_t)cq[i].base_hi << 32) | cq[i].base_lo; }

    bool     reg_read_off (uint32_t off, sc_uint<32> &data);   // true = 映射 ✓
    bool     reg_write_off(uint32_t off, uint32_t v);          // true = 真正写入 ✓
    void     dq_cq_defaults();                                 // 复位值（全 0 + 掩码全 1 ✓）

    // ── 命令流状态机 ──
    enum HSt { H_IDLE = 0, H_DQ_FETCH, H_PARSE, H_CT_FETCH, H_ISSUE,
               H_WRDATA, H_WAIT_RESP, H_CQE_WR };
    HSt      st;
    unsigned lat_cnt;                       // 读延迟倒计时
    int      cur_dqi;                       // 当前 DQ 号
    uint32_t dq_entry[SAS_HBA_DQ_ENTRY_DW]; // DQ 条目（64B ✓）
    uint32_t cmd_tbl[16];                   // 命令表 64B（含 CDB@36 ✓）
    uint16_t cur_iptt, cur_dev_id, cur_tag;
    uint8_t  cur_dir;
    uint32_t cur_xfer_len;
    uint64_t cur_ct_addr, cur_prd_addr, cur_sts_addr;
    uint64_t cur_sge_addr;                  // 首 SGE 目标/源地址（DMA ✓）
    uint32_t cur_sge_len;
    uint16_t tag_gen;
    unsigned resp_cnt;                      // 等响应超时计数
    uint32_t wd_left;                       // WRITE 路径：剩余待发字节
    uint32_t wd_off;
    uint8_t  wd_buf[1024];                  // WRITE 分块读缓冲（≤ 250 dword ✓）
    bool     cmpl_err;                      // 本次完成是否错误 ✓
    bool     cmpl_got_resp;                 // 是否收到过响应（超时 ⇒ false ✓）
    uint8_t  cmpl_status;                   // SSP status ✓

    // ── SSP 发送（帧队列 + held-valid ✓）──
    struct TlsFrame { uint32_t dw[FRAME_MAX_DW]; int n; uint8_t ft; };
    std::vector<TlsFrame> txq;
    uint32_t tx_dw[FRAME_MAX_DW];
    int      tx_total, tx_idx;
    bool     tx_active;
    bool     tx_presented;                  // 本拍已呈现过 beat（推进与呈现分拍 ✓ 见 .cpp）
    uint8_t  tx_ft;
    void tx_begin(uint8_t ft, int ndw);
    bool tx_run();                          // 返回 true = 该帧发完 ✓
    void queue_frame(uint8_t ft, int ndw);  // 把 tx_dw[0..ndw) 入队 ✓

    // ── SSP 接收装配 ──
    int      rx_dwc;
    uint32_t rx_dw[FRAME_MAX_DW];
    uint8_t  rx_ft_lat;

    // ── DMA（零时 b_transport；返回 OKAY 与否 ✓）──
    bool dma_read (uint64_t addr, void *dst, unsigned len);
    bool dma_write(uint64_t addr, const void *src, unsigned len);

    // ── 完成（CQE + 指针推进 + 中断 ✓）──
    void finish_cmd(bool err, uint8_t ssp_status);
    void build_cqe(uint32_t cqe[4], bool err, uint8_t ssp_status);

    // ── 计数（白盒 ✓）──
    uint32_t c_done, c_err, c_cqes, c_irq, c_dq_fetches, c_ct_fetches;
    uint32_t c_dma_r, c_dma_w, c_frames_tx, c_frames_rx, c_tag_mm, c_data_rx;
    uint16_t last_iptt; uint8_t last_ssp_status;
};

#endif // SAS_HBA_TLM_H
