//=============================================================================
// apv_master.h — TB 侧 AMBA-PV 主机（4B 寄存器访问；判据用）  [2026-10-08 建]
//-----------------------------------------------------------------------------
// 出处：照先例 `rdma400/tlm/model/tb/tb_device_amba_pv.cpp:22-69` 的 ApvMaster ✓
//   · 主机模块须继承 `amba_pv_master_base` + `m(*this)`（照 VDK dma 例子 ✓）
//   · `pool.allocate(1, 4, NULL, AMBA_PV_INCR)`（⚠ BL `amba_pv_mm.h 口径 ✓）
//   · 读回经 `amba_pv_extension::get_resp()` ⇒ DECERR/SLVERR 可判 ✓
// 用法（TB 的 SC_THREAD 里）：`uint32_t v; apv.rd(addr, v); apv.wr(addr, v);` ✓
//=============================================================================
#ifndef APV_MASTER_H
#define APV_MASTER_H

#include <systemc.h>
#include <amba_pv.h>

struct ApvMaster : sc_core::sc_module,
                   amba_pv::amba_pv_master_base {
    sc_core::sc_in<bool> clk;
    amba_pv::amba_pv_master_socket<32> m;
    amba_pv::amba_pv_trans_pool pool;

    SC_HAS_PROCESS(ApvMaster);
    ApvMaster(sc_core::sc_module_name nm)
      : sc_core::sc_module(nm), amba_pv::amba_pv_master_base("apv_mb"),
        clk("clk"), m("m") {
        m(*this);                       // 反向路径接口绑到自己（照 VDK dma 例子 ✓）
    }

    // 单笔 4B 事务 ✓
    amba_pv::amba_pv_resp_t xfer(bool is_wr, sc_dt::uint64 addr, uint32_t &vref) {
        unsigned char buf[4];
        if (is_wr) {
            buf[0] = (unsigned char)vref;       buf[1] = (unsigned char)(vref >> 8);
            buf[2] = (unsigned char)(vref >> 16); buf[3] = (unsigned char)(vref >> 24);
        }
        amba_pv::amba_pv_trans_ptr tr(pool.allocate(1, 4,
            (const amba_pv::amba_pv_control *)NULL, amba_pv::AMBA_PV_INCR));
        tr->set_command(is_wr ? tlm::TLM_WRITE_COMMAND : tlm::TLM_READ_COMMAND);
        tr->set_address(addr);
        tr->set_data_length(4);
        tr->set_data_ptr(buf);
        amba_pv::amba_pv_extension *ex = NULL;
        tr->get_extension(ex);
        sc_core::sc_time t = sc_core::SC_ZERO_TIME;
        m.b_transport(*tr, t);
        wait(t);
        if (!is_wr && ex) {
            vref = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
                 | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
        }
        return ex ? ex->get_resp() : amba_pv::AMBA_PV_OKAY;
    }
    amba_pv::amba_pv_resp_t wr(sc_dt::uint64 a, uint32_t v) { uint32_t t = v; return xfer(true, a, t); }
    amba_pv::amba_pv_resp_t rd(sc_dt::uint64 a, uint32_t &v) { return xfer(false, a, v); }

    void tick(int n = 1) { for (int i = 0; i < n; i++) { wait(clk.posedge_event()); wait(sc_core::SC_ZERO_TIME); } }
};

#endif // APV_MASTER_H
