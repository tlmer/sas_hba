//=============================================================================
// sas_link_tlm.cpp — SAS 帧级链路简化模型实现  [2026-10-08 建]
//-----------------------------------------------------------------------------
// 逐拍时序（step() 一次 = 一拍 ✓，全部输出为**寄存器**、构造期不写端口 ✓）：
//   ⓪ 复位：清链路、输出回确定态 ✓
//   ① 呈现队头：B→A 取 bq、A→B 取 aq —— 被反压即保持（held-valid ✓），全双工
//      两方向同拍各出 1 拍 ✓
//   ② 消费：呈现且对侧 ready ⇒ 出队（下拍换队头 ✓）
//   ③ 接收：tx_valid && 队列有位置 ⇒ 入队（内含**整帧丢弃**判定 ✓）
//   ④ 老化：在途拍 age++；age 到 eff 即"到达"、下一拍可呈现 ✓
// 语义要点：
//   · cfg_delay 拍 = 入队后经过几拍才可呈现（0 与 1 同效、最小 1 拍 ✓）
//   · cfg_drop_every=N ⇒ 每方向第 N、2N、… 帧**整帧丢弃**：**仍照常 ready**
//     （发送侧无感 ✓ —— 与真实 NAK 不同：真实 NAK 会让发送侧重传；本模型的
//     用途是复现"帧发出去对侧没收到 ⇒ 上层靠超时死等"的卡点负控 ✓，见本参考实现
//     缺陷#/缺陷# 场景 ✓）
//   · 丢帧用 SOF..EOF 圈定；发送侧恒以 EOF 收尾（本工程两侧模型都保证 ✓）
//=============================================================================

#include "sas_link_tlm.h"

SasLinkTlm::SasLinkTlm(sc_core::sc_module_name nm)
  : sc_core::sc_module(nm),
    cfg_delay(1), cfg_drop_every(0),
    aq_h(0), aq_t(0), bq_h(0), bq_t(0),
    frame_cnt_a(0), frame_cnt_b(0),
    drop_a(false), drop_b(false),
    a_tx_acc(false), b_tx_acc(false)
{
    SC_METHOD(step);
    sensitive << clk.pos();
    dont_initialize();   // 首拍照常 step；端口逐拍写、构造期不碰 ✓
}

void SasLinkTlm::step() {
    // ── ⓪ 复位 ──
    if (!rst_n.read()) {
        aq_h = aq_t = bq_h = bq_t = 0;
        frame_cnt_a = frame_cnt_b = 0;
        drop_a = drop_b = false;
        a_tx_acc = b_tx_acc = false;
        o_dropped.write(0);
        a_tx_ready.write(false);
        b_tx_ready.write(false);
        a_rx_data.write(0);      a_rx_valid.write(false);
        a_rx_sof.write(false);   a_rx_eof.write(false);  a_rx_type.write(0);
        b_rx_data.write(0);      b_rx_valid.write(false);
        b_rx_sof.write(false);   b_rx_eof.write(false);  b_rx_type.write(0);
        return;
    }

    // 有效延迟（cfg_delay 0 与 1 同效 —— 最小 1 拍 ✓；上限 250 防 age 溢出 ✓）
    const unsigned eff = (cfg_delay == 0) ? 1u : (cfg_delay > 250u ? 250u : cfg_delay);

    // ── ① 呈现队头（全双工 ✓）──
    const bool bq_present = (bq_h != bq_t) && (bq[bq_h % QCAP].age >= eff);
    const bool aq_present = (aq_h != aq_t) && (aq[aq_h % QCAP].age >= eff);

    if (bq_present) {   // B→A ⇒ 呈到 a_rx ✓
        const Beat& x = bq[bq_h % QCAP];
        a_rx_data.write(x.d);    a_rx_valid.write(true);
        a_rx_sof.write(x.sof);   a_rx_eof.write(x.eof);  a_rx_type.write(x.t);
    } else {
        a_rx_data.write(0);      a_rx_valid.write(false);
        a_rx_sof.write(false);   a_rx_eof.write(false);  a_rx_type.write(0);
    }
    if (aq_present) {   // A→B ⇒ 呈到 b_rx ✓
        const Beat& x = aq[aq_h % QCAP];
        b_rx_data.write(x.d);    b_rx_valid.write(true);
        b_rx_sof.write(x.sof);   b_rx_eof.write(x.eof);  b_rx_type.write(x.t);
    } else {
        b_rx_data.write(0);      b_rx_valid.write(false);
        b_rx_sof.write(false);   b_rx_eof.write(false);  b_rx_type.write(0);
    }

    // ── ② 消费 ──
    if (bq_present && a_rx_ready.read()) bq_h++;
    if (aq_present && b_rx_ready.read()) aq_h++;

    // ── ③ 接收（ready = 队列余量 ✓；丢帧不占队列、但照常 ready ✓）──
    a_tx_acc = (q_occ(aq_h, aq_t) < QCAP);
    b_tx_acc = (q_occ(bq_h, bq_t) < QCAP);
    a_tx_ready.write(a_tx_acc);
    b_tx_ready.write(b_tx_acc);
    accept_side(true);    // A→B（入 aq ✓）
    accept_side(false);   // B→A（入 bq ✓）

    // ── ④ 老化 ──
    age_q(aq, aq_h, aq_t, eff);
    age_q(bq, bq_h, bq_t, eff);
}

// 单侧接收：A 侧（is_a=true）↔ aq，B 侧 ↔ bq ✓
void SasLinkTlm::accept_side(bool is_a) {
    const bool v = is_a ? a_tx_valid.read() : b_tx_valid.read();
    if (!v || !(is_a ? a_tx_acc : b_tx_acc)) return;

    Beat x;
    x.d   = is_a ? (uint32_t)a_tx_data.read() : (uint32_t)b_tx_data.read();
    x.sof = is_a ? a_tx_sof.read()             : b_tx_sof.read();
    x.eof = is_a ? a_tx_eof.read()             : b_tx_eof.read();
    x.t   = is_a ? (uint8_t)a_tx_type.read()   : (uint8_t)b_tx_type.read();
    x.v   = true;
    x.age = 0;

    unsigned& fcnt = is_a ? frame_cnt_a : frame_cnt_b;
    bool&     drp  = is_a ? drop_a      : drop_b;

    // 丢帧判定（只在帧首 SOF 拍决断 ✓）：该帧序号命中 N 的倍数 ⇒ 整帧吞掉 ✓
    if (!drp && x.sof && cfg_drop_every > 0 &&
        ((fcnt + 1u) % cfg_drop_every) == 0u) {
        drp = true;
        o_dropped.write((uint32_t)o_dropped.read() + 1u);
    }
    if (!drp) {
        if (is_a) { aq[aq_t % QCAP] = x; aq_t++; }
        else      { bq[bq_t % QCAP] = x; bq_t++; }
    }
    if (x.eof) {          // 帧尾：计数并解除丢弃态 ✓（被丢的帧也计数 ✓）
        fcnt++;
        drp = false;
    }
}

// 在途拍老化（到 eff 即"到达"、可呈现 ✓）
void SasLinkTlm::age_q(Beat* q, int h, int t, unsigned eff) {
    for (int i = h; i != t; i++) {
        Beat& b = q[i % QCAP];
        if (b.age < eff) b.age++;
    }
}
