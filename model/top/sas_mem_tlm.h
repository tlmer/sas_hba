//=============================================================================
// sas_mem_tlm.h — 通用 AXI 内存模型（AMBA-PV 从口，byte 阵列）  [2026-10-08 建]
//-----------------------------------------------------------------------------
// 角色：给 HBA 的 DMA（TLM 主端）当**宿主内存** —— DQ 环 / 命令表 / SGE 表 /
//   数据缓冲 / CQ 环 全部落在这里 ✓（参考平台对应 guest CMA 区 ✓）
//
// ★ 接口契约（照先例 `rdma400/tlm/model/device/device_tlm.h:73-87` ✓）：
//   · 从口 = `amba_pv::amba_pv_slave_socket<32>` + 本类继承 `amba_pv_slave_base<32>`
//   · 内部：`[base, base+size)` 窗口命中 ⇒ OKAY；越界 ⇒ **DECERR** ✓
//   · 写口 **支持 byte_enable**（`byte_en[i]!=0` 才写第 i 字节 ✓；nullptr = 全写 ✓）
//     —— 寄存器面可忽略 byte_en（先例 ✓），但**内存**必须处理（CQE 等结构化写 ✓）
//   · 延迟：默认 `lat = SC_ZERO_TIME`（**SC_METHOD 调用方安全** ✓ —— 本 HBA 的
//     逐拍 FSM 就靠这个）；要加延迟的仿真请置非 0 并保证调用方是 SC_THREAD ✓
//
// ★ TB 直访钩子（预置命令表/SGE/校验 CQE —— 判据直接读 ✓）：
//   poke/peek/rd32/wr32/fill + `raw()`（返回 window 内首字节指针 ✓）
//=============================================================================
#ifndef SAS_MEM_TLM_H
#define SAS_MEM_TLM_H

#include <systemc.h>
#include <amba_pv.h>
#include <vector>
#include <cstring>
#include <cstdint>

struct SasMemTlm : sc_core::sc_module,
                   amba_pv::amba_pv_slave_base<32> {
    amba_pv::amba_pv_slave_socket<32> s;    // 平台/TA 的 master 绑此处 ✓
    uint64_t          base;                 // 窗口基址（如 0xfe400000 ✓）
    uint64_t          size;                 // 窗口大小（字节 ✓）
    sc_core::sc_time  lat;                  // 访问延迟（默认 0 ✓）
    unsigned long     n_reads, n_writes;    // 观测计数 ✓
    unsigned long     n_decerr;             // 越界访问计数（不静默 ✗✓）

    SC_HAS_PROCESS(SasMemTlm);
    SasMemTlm(sc_core::sc_module_name nm, uint64_t base_ = 0xfe400000ull,
              uint64_t size_ = 0x01000000ull)
      : sc_core::sc_module(nm), amba_pv::amba_pv_slave_base<32>((const char *)nm),
        s("s"), base(base_), size(size_), lat(sc_core::SC_ZERO_TIME),
        n_reads(0), n_writes(0), n_decerr(0)
    {
        s(*this);          // ★ 从口绑到本类（基类转 read()/write() ✓ 照先例 device_tlm.cpp:63 ✓）
        mem.assign((size_t)size_, 0x00);
    }

    // ── AMBA-PV 从口（b_transport 经基类转此处 ✓）──
    virtual amba_pv::amba_pv_resp_t read(int, const sc_dt::uint64 &addr,
                                         unsigned char *data, unsigned int len,
                                         const amba_pv::amba_pv_control *,
                                         sc_core::sc_time &delay) override {
        if (data == nullptr) return amba_pv::AMBA_PV_SLVERR;
        if (!in_win(addr, len)) { n_decerr++; delay += lat; return amba_pv::AMBA_PV_DECERR; }
        memcpy(data, &mem[(size_t)(addr - base)], len);
        n_reads++; delay += lat;
        return amba_pv::AMBA_PV_OKAY;
    }
    virtual amba_pv::amba_pv_resp_t write(int, const sc_dt::uint64 &addr,
                                          unsigned char *data, unsigned int len,
                                          const amba_pv::amba_pv_control *,
                                          unsigned char *byte_en,
                                          sc_core::sc_time &delay) override {
        if (data == nullptr) return amba_pv::AMBA_PV_SLVERR;
        if (!in_win(addr, len)) { n_decerr++; delay += lat; return amba_pv::AMBA_PV_DECERR; }
        uint8_t *p = &mem[(size_t)(addr - base)];
        if (byte_en) {
            for (unsigned int i = 0; i < len; i++) if (byte_en[i]) p[i] = data[i];
        } else {
            memcpy(p, data, len);
        }
        n_writes++; delay += lat;
        return amba_pv::AMBA_PV_OKAY;
    }

    // ── TB 直访（⛔ 只 TB 用 ✓；不计数 ✓）──
    uint8_t *raw() { return mem.data(); }
    void poke(uint64_t a, const void *p, size_t n) {
        if (!in_win(a, (unsigned)n)) return;
        memcpy(&mem[(size_t)(a - base)], p, n);
    }
    void peek(uint64_t a, void *p, size_t n) const {
        memset(p, 0, n);
        if (!in_win(a, (unsigned)n)) return;
        memcpy(p, &mem[(size_t)(a - base)], n);
    }
    uint32_t rd32(uint64_t a) const { uint32_t v = 0; peek(a, &v, 4); return v; }
    void     wr32(uint64_t a, uint32_t v) { poke(a, &v, 4); }
    void     fill(uint8_t v) { memset(mem.data(), v, mem.size()); }

private:
    std::vector<uint8_t> mem;
    bool in_win(uint64_t a, unsigned len) const {
        return a >= base && (a - base) + len <= size;
    }
};

#endif // SAS_MEM_TLM_H
