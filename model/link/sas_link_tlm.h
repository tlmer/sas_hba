//=============================================================================
// sas_link_tlm.h — SAS 帧级链路简化模型  [2026-10-08 建]
//-----------------------------------------------------------------------------
// 角色：把两侧（HBA 的 SSP 发/收 ↔ HDD 的 SSP 收/发）在**帧级**直连，
//   并可选注入**逐拍转发延迟 / 整帧丢弃（=NAK 语义）** 作负控 ✓。
//
// 为什么是帧级（⛔ 简化，真实实现在 `sas_hdd/rtl/link_layer/` 与 `transport/`）：
//   · 真实链路 = 8b/10b、原语、速率协商、ACK/NAK 双向、信用…… 对**系统级**建模
//     是噪声；本参考实现已验证：驱动/内核可以只看"帧完整性 + 顺序 + 反压" ✓
//   · 本模型保留：**逐拍反压**（ready/valid ✓）、**帧边界**（SOF/EOF ✓）、
//     可配置**转发延迟**（模拟链路往返 ✓）、**整帧丢弃**（负控：帧发出去对侧
//     收不到、发送侧靠超时发现 —— 与本参考实现 RTL 卡点场景同型 ✓）
//
// 两侧契约（与两个 IP 模型的端口签名逐字一致 ✓）：
//   每拍 1 dword：{data, valid, sof, eof, type} + ready 反向 ✓
//   `valid && ready` 为拍完成 ✓；valid 保持到 ready ✓（held-valid 纪律 ✓）
//   帧型常量见 `sas_hdd_tlm.h` FT_* ✓
//
// ⛔ 不做：OPEN/CLOSE 连接建立、速率/SAS 地址协商、ACK/NAK 原语**定时**（以整帧
//   丢弃近似 ✓）、位翻/CRC（可用丢弃注入近似 ✓）、8b/10b、扩频时钟（SSC）
//=============================================================================
#ifndef SAS_LINK_TLM_H
#define SAS_LINK_TLM_H

#include <systemc.h>
#include <cstdint>

struct SasLinkTlm : sc_core::sc_module {
    // ── A 侧（如 HBA）──
    sc_core::sc_in<sc_uint<32>> a_tx_data;
    sc_core::sc_in<bool>        a_tx_valid;
    sc_core::sc_in<bool>        a_tx_sof;
    sc_core::sc_in<bool>        a_tx_eof;
    sc_core::sc_in<sc_uint<4>>  a_tx_type;
    sc_core::sc_out<bool>       a_tx_ready;
    sc_core::sc_out<sc_uint<32>> a_rx_data;
    sc_core::sc_out<bool>        a_rx_valid;
    sc_core::sc_out<bool>        a_rx_sof;
    sc_core::sc_out<bool>        a_rx_eof;
    sc_core::sc_out<sc_uint<4>>  a_rx_type;
    sc_core::sc_in<bool>         a_rx_ready;

    // ── B 侧（如 HDD）── 同构（口径与 A 侧一致：`tx_*`=读对端发送、`rx_*`=驱动对端接收 ✓）
    sc_core::sc_in<sc_uint<32>>  b_tx_data;
    sc_core::sc_in<bool>         b_tx_valid;
    sc_core::sc_in<bool>         b_tx_sof;
    sc_core::sc_in<bool>         b_tx_eof;
    sc_core::sc_in<sc_uint<4>>   b_tx_type;
    sc_core::sc_out<bool>        b_tx_ready;
    sc_core::sc_out<sc_uint<32>> b_rx_data;
    sc_core::sc_out<bool>        b_rx_valid;
    sc_core::sc_out<bool>        b_rx_sof;
    sc_core::sc_out<bool>        b_rx_eof;
    sc_core::sc_out<sc_uint<4>>  b_rx_type;
    sc_core::sc_in<bool>         b_rx_ready;

    // ── 配置（TB 注入 ✓；构造后改也可，逐拍生效 ✓）──
    unsigned cfg_delay;        // 在途拍数：入队后经过几拍才可呈现（0 与 1 同效、
                               //   最小 1 拍 ✓；>250 截到 250 ✓）默认 1 ✓
    unsigned cfg_drop_every;   // N>0 ⇒ 每方向第 N、2N、… 帧**整帧丢弃**（负控 ✓）；
                               //   0 = 不丢（默认 ✓）

    // ── 白盒观测（⛔ 只 TB 用 ✓）──
    sc_core::sc_out<sc_uint<32>> o_dropped;   // 触发丢弃的帧数（按发起计 ✓）

    sc_core::sc_in<bool> clk;
    sc_core::sc_in<bool> rst_n;

    SC_HAS_PROCESS(SasLinkTlm);
    SasLinkTlm(sc_core::sc_module_name nm);
    static const char* version() { return "1.0"; }

    // ── 白盒观测（⛔ 只 TB 用 ✓）：在途拍数（调试/判据 ✓）
    int aq_occ_now() const { return aq_t - aq_h; }
    int bq_occ_now() const { return bq_t - bq_h; }
    // ⚠ 端口呈现纪律（全工程统一 ✓）：**每拍推进与呈现必须分拍** —— 一次 step 里
    //   对同一端口只写一次；末拍若在同拍被 valid=0 覆盖 ⇒ 对端窗口缺失（曾实踩 ✗）。
    //   本链路的「呈现+出队」同拍是安全的（改写发生在其**下一拍** ✓ 见 .cpp 注释 ✓）

private:
    struct Beat { bool v, sof, eof; uint32_t d; uint8_t t; uint8_t age; };
    static const int QCAP = 256;          // 单方向在途拍上限（64 dword 帧也够 ✓）
    Beat aq[QCAP]; int aq_h, aq_t;        // A→B 在途
    Beat bq[QCAP]; int bq_h, bq_t;        // B→A 在途
    unsigned frame_cnt_a, frame_cnt_b;    // 各方向已完成帧数（被丢的也计 ✓）
    bool drop_a, drop_b;                  // 正在整帧丢弃（用 SOF..EOF 圈定 ✓）
    bool a_tx_acc, b_tx_acc;              // 本拍入队位置允许（含队列余量 ✓）

    void step();
    void accept_side(bool is_a);
    void age_q(Beat* q, int h, int t, unsigned eff);
    static int q_occ(int h, int t) { return t - h; }
};

#endif // SAS_LINK_TLM_H
