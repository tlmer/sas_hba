//============================================================================
// dummy_amba_master.h — AMBA-PV **占位主端**（只为满足 socket 绑定；不发事务）  [2026-10-08 建]
//
// ⚠ 起因（借先例实踩教训 `rdma400/tlm/model/tb/dummy_amba_master.h` ✓）：
//   `amba_pv::amba_pv_slave_socket` 内部 `sc_port` 绑定策略 = **ONE_OR_MORE**
//   ⇒ **不用它的 TB 不绑就 elaboration 报 E109** ✗（且旧二进制不报 —— 重链才现形，
//     属「陈旧二进制读数」陷阱家族 ✓）
// ⇒ 纪律：凡例化 `SasHbaTlm` / `SasMemTlm` 而不走其 socket 的 TB，
//   **一律绑本占位件**（`stub.m.bind(dev.reg_s)`）✓
//   （真事务路径 = `apv_master.h` 的 `ApvMaster` ✓；本件绑上即满足契约、不发任何事务 ✓）
//============================================================================
#ifndef DUMMY_AMBA_MASTER_H
#define DUMMY_AMBA_MASTER_H

#include <systemc.h>
#include <amba_pv.h>

struct DummyAmbaMaster : sc_core::sc_module,
                         amba_pv::amba_pv_master_base {
    amba_pv::amba_pv_master_socket<32> m;

    SC_HAS_PROCESS(DummyAmbaMaster);
    DummyAmbaMaster(sc_core::sc_module_name nm)
      : sc_core::sc_module(nm), amba_pv::amba_pv_master_base("dummy_mb"), m("m") {
        m(*this);                    // 反向路径接口绑到自己（照 VDK dma 例子 ✓）
    }
    // ⛔ 无行为：只为 socket 绑定；⛔ 不发任何事务 ✗
};

#endif // DUMMY_AMBA_MASTER_H
