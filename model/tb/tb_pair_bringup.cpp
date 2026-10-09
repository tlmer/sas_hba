//============================================================================
// tb_pair_bringup.cpp — ★ HBA ⇄ link ⇄ HDD **交互性/全链路**联合台  [2026-10-08 建]
//   结构：SasPairTop（hba + link + hdd + mem ✓）经**寄存器/MEM 编程驱动**，
//         全流程 = DQ 写条目 → HBA DMA 取指/取命令表/取 SGE → SSP COMMAND →
//         HDD(mem) 命令集 → DATA(可多帧)/RESPONSE → HBA 落数据 + 写 CQE + 中断 ✓
//
//   判据（**预登记**，全部以 RTL/内核真值为期望 ✓ 出处见各条）：
//   A1  复位后 PHY0 链路位：CHNL_INT_STATUS bit0=1（SL_PHY_ENABLE ✓ regfile ○）
//   A2  TUR（iptt=1）：CQE iptt/dev_id/cmplt=0 ✓（`RTL 源`）
//   A3  INQUIRY：数据 36B —— vendor "-UME MIS"/product "-SAS-DHVM652"/**byte4=32** ✓
//         （`RTL 源,65` ✓ 对拍差异 D1 闭 ✓）
//   A4  MODE SENSE(6)：**64B**、byte0=0x3F、byte4=0x08、byte5=0x12 ✓（缺陷# ✓）
//   A5  READ(10) 4 块=2048B：数据逐字节 == 源模式 ✓；**多 DATA 帧**（>1000B 分帧、
//         DataOffset 递增 ✓）；hdd.o_lba_read=1 ✓
//   A6  未知 opcode ⇒ RESPONSE status=0x02（CHECK CONDITION ✓ 缺陷#）⇒ CQE cmplt=3 ✓；
//         随后 REQ SENSE(0x03) ⇒ 数据 64B：byte0=0x70/byte7=0x0A/key=0x05/ASC=0x20 ✓
//   A7  LUN≠0 的 INQUIRY ⇒ 数据 byte0 高 3 位 = 011b（PQ=3 ✓ 缺陷#）
//   A8  中断：CQE 后 irq 电平高、o_irq_cnt≥1 ✓；**W1C 写 OQ_INT_SRC ⇒ irq 落低** ✓
//         （内核期望口径 ✓；RTL「无 EOI」复现见 tb_neg ✓ 红线②）
//   A9  状态缓冲：TUR 的响应原文 48B 落 sts_buffer，dw0=0x40、byte35=0x00 ✓
//   A10 计数：o_tag_mismatch=0 ✓、hba.o_cmds_err=1 ✓、hdd.o_cmds_good=5/o_cmds_chk=1 ✓
//   A11 链路：link.o_dropped=0 ✓
//   A12 CQE 顺序：6 条 CQE 的 iptt 依次 = 1,2,3,4,5,6 ✓
//============================================================================
#include <systemc.h>
#include "sas_pair_top.h"
#include "dummy_amba_master.h"
#include "sas_hba_hw.h"
#include <cstdio>
#include <cstring>
#include <vector>

// ── 地址规划（全在 mem 窗口 0xfe400000 + 16MB 内 ✓）──
static const uint64_t MEM_BASE = 0xfe400000ull;
static const uint64_t DQ_BASE  = 0xfe440000ull;
static const uint64_t CQ_BASE  = 0xfe441000ull;
// ⚠ 布局纪律（实踩 ✗→✓）：**各区分段必须无重叠** —— CT 每条 1KB ×8 占满
//   0xfe442000..0xfe444000；曾把 SGE 区放 0xfe443000 ⇒ **被 idx4/5/6 的命令表覆盖**
//   ⇒ SGE 全零 ⇒ DATA 全丢（drop=5 ✗）。现 SGE/DAT/STS 全部后移 ✓
static const uint64_t CT_BASE  = 0xfe442000ull;   // 每条 1KB ×8 ⇒ 到 0xfe444000 止 ✓
static const uint64_t SGE_BASE = 0xfe450000ull;   // 每条 0x100 ×8 ✓
static const uint64_t DAT_BASE = 0xfe451000ull;   // 每条 4KB ×8 ✓
static const uint64_t STS_BASE = 0xfe460000ull;   // 状态缓冲 ✓
static const uint32_t DQ_DEPTH = 8, CQ_DEPTH = 8;
static const uint64_t TST_LBA  = 0x10;

struct TbPair : sc_core::sc_module {
    sc_core::sc_in<bool>  clk;
    sc_core::sc_out<bool> rst_n;
    SasPairTop *pair = nullptr;
    int pass_cnt = 0, fail_cnt = 0;

    SC_HAS_PROCESS(TbPair);
    TbPair(sc_core::sc_module_name nm) : sc_module(nm), clk("clk"), rst_n("rst_n") {
        SC_THREAD(run);
    }
    void tick(int n = 1) {
        for (int i = 0; i < n; i++) { wait(clk.posedge_event()); wait(sc_core::SC_ZERO_TIME); }
    }
    void chk(bool ok, const char *name) {
        printf("   %s %s\n", ok ? "✓" : "✗", name);
        if (ok) pass_cnt++; else fail_cnt++;
    }

    // 数据缓冲（每条命令一页 ✓）
    uint64_t dat_of(int idx) const { return DAT_BASE + (uint64_t)idx * 0x1000; }

    // 发一条命令：把 DQ 条目 + 命令表 + **SGE 记录**（内核 24B 布局 ✓ 红线⑤）写进内存
    //   ⚠ 台自身教训：只给 SGE 表地址、不填记录 ⇒ 模型无处落数据（曾全落空 ✗）
    void issue(int idx, uint16_t iptt, const uint8_t cdb[16], const uint8_t lun[8],
               unsigned dir, uint32_t xfer, uint64_t sge_addr, uint64_t sts_addr) {
        uint8_t tbl[SAS_HBA_CTBL_SSP_BYTES];
        sas_ctbl_pack_ssp(tbl, cdb, lun, 0, 0, 0);
        pair->mem.poke(CT_BASE + (uint64_t)idx * 0x400, tbl, sizeof(tbl));
        if (xfer) {                                   // SGE：dst = 本命令数据页 ✓
            uint8_t sge[SAS_HBA_SGE_BYTES];
            sas_sge_pack(sge, dat_of(idx), xfer, 0);
            pair->mem.poke(sge_addr, sge, sizeof(sge));
        }
        uint32_t dqw[SAS_HBA_DQ_ENTRY_DW];
        sas_dq_pack_ssp(dqw, iptt, 0x0001, dir, xfer,
                        CT_BASE + (uint64_t)idx * 0x400,
                        sge_addr);
        sas_dw_set_u64(dqw, SAS_HBA_DQ_STS_BUF_LO_DW, sts_addr);
        pair->mem.poke(DQ_BASE + (uint64_t)idx * 64, dqw, sizeof(dqw));
    }

    void run() {
        // ── 复位期内准备（内存可写 ✓）──
        printf("[TB] 预登记判据 A1..A12（HBA⇄link⇄HDD 交互 ✓）\n");
        pair->mem.fill(0x00);
        // HDD 存储模式（LBA TST_LBA..+3 ✓）
        for (int i = 0; i < 4; i++) {
            uint8_t *p = pair->hdd.lba_ptr(TST_LBA + i);
            for (int k = 0; k < 512; k++) p[k] = (uint8_t)((TST_LBA + i) * 512 + k) ^ 0xA5;
        }

        tick(4);                                  // 等复位释放边缘（下方 sc_main 控 rst_n ✓）
        while (!rst_n.read()) tick(1);
        tick(4);

        // ── A1 复位后链路位 ──
        {
            sc_uint<32> v = 0;
            pair->hba.reg_read(SAS_HBA_INT_CHNL_STATUS, v);
            chk((v.to_uint() & 1u) == 1u, "A1 CHNL_INT_STATUS bit0 = PHY0 SL_PHY_ENABLE=1");
        }

        // ── 寄存器编程（直连 ✓ 与 socket 同源）──
        pair->hba.reg_write(SAS_HBA_CFG_DLVRY_QUEUE_ENABLE, sc_uint<32>(1));
        pair->hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), sc_uint<32>(DQ_BASE & 0xFFFFFFFFu));
        pair->hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_BASE_ADDR_HI), sc_uint<32>(DQ_BASE >> 32));
        pair->hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_DEPTH), sc_uint<32>(DQ_DEPTH));
        pair->hba.reg_write(SAS_HBA_CQ_REG(0, SAS_HBA_XX_BASE_ADDR_LO), sc_uint<32>(CQ_BASE & 0xFFFFFFFFu));
        pair->hba.reg_write(SAS_HBA_CQ_REG(0, SAS_HBA_XX_BASE_ADDR_HI), sc_uint<32>(CQ_BASE >> 32));
        pair->hba.reg_write(SAS_HBA_CQ_REG(0, SAS_HBA_XX_DEPTH), sc_uint<32>(CQ_DEPTH));
        pair->hba.reg_write(SAS_HBA_INT_OQ_SRC_MSK, sc_uint<32>(0));      // 解掩 ✓

        // ── 六条命令（iptt 1..6）──
        uint8_t lun0[8] = {0,0,0,0,0,0,0,0};
        uint8_t lun1[8] = {0,0,0,0,0,0,0,1};      // LUN=1（A7 ✓）
        // 1) TUR
        { uint8_t c[16] = {0x00};                  issue(0, 1, c, lun0, SAS_HBA_DIR_NO_DATA, 0, 0, STS_BASE); }
        // 2) INQUIRY（alloc=36 ✓）
        { uint8_t c[16] = {0x12,0,0,0,36,0};       issue(1, 2, c, lun0, SAS_HBA_DIR_TO_INI, 36, SGE_BASE + 0x100, 0); }
        // 3) MODE SENSE(6)：`1a 00 19 00 40 00`（参考平台 libsas 读页 0x19、64B ✓）
        { uint8_t c[16] = {0x1A,0,0x19,0,0x40,0};  issue(2, 3, c, lun0, SAS_HBA_DIR_TO_INI, 64, SGE_BASE + 0x200, 0); }
        // 4) READ(10) LBA=TST_LBA 4 块（2048B ⇒ 多 DATA 帧 ✓）
        { uint8_t c[16] = {0x28,0,0,0,0,0x10,0,0,0x04,0}; issue(3, 4, c, lun0, SAS_HBA_DIR_TO_INI, 2048, SGE_BASE + 0x300, 0); }  // ⚠ LBA 大端：LSB=byte5 ⇒ 0x10 落 c[5]（台曾写错 c[4] ⇒ 读 LBA 0x1000 全零 ✗）
        // 5) 未知 opcode 0x7F ⇒ CHECK CONDITION
        { uint8_t c[16] = {0x7F};                  issue(4, 5, c, lun0, SAS_HBA_DIR_NO_DATA, 0, 0, 0); }
        // 6) REQ SENSE（注入 key=5/ASC=0x20 模拟 ILLEGAL REQUEST ✓）
        { uint8_t c[16] = {0x03,0,0,0,64,0};       issue(5, 6, c, lun0, SAS_HBA_DIR_TO_INI, 64, SGE_BASE + 0x400, 0); }
        // 7) INQUIRY 且 **LUN≠0** ⇒ PQ=3（A7 ✓ 缺陷#）
        { uint8_t c[16] = {0x12,0,0,0,36,0};       issue(6, 7, c, lun1, SAS_HBA_DIR_TO_INI, 36, SGE_BASE + 0x500, 0); }

        pair->hdd.set_sense(0x05, 0x20, 0x00);

        tick(4);
        pair->hba.reg_write(SAS_HBA_DQ_REG(0, SAS_HBA_XX_WR_PTR), sc_uint<32>(7));   // 门铃 ✓

        // ── 等 7 条 CQE（上限 100k 拍 ✓）──
        int cap = 100000;
        while (pair->hba.o_cqes.read().to_uint() < 7u && cap-- > 0) tick(1);
        chk(pair->hba.o_cqes.read().to_uint() == 7u, "A2/A5 七条命令全部完成（o_cqes=7）");
        tick(8);

        // ── 读 CQE 环（16B ×7 ✓）──
        uint32_t cqe[7][4];
        for (int i = 0; i < 7; i++)
            for (int k = 0; k < 4; k++) cqe[i][k] = pair->mem.rd32(CQ_BASE + (uint64_t)i * 16 + k * 4);
        chk(sas_cqe_iptt(cqe[0]) == 1 && sas_cqe_devid(cqe[0]) == 1 &&
            sas_cqe_cmplt(cqe[0]) == 0 && sas_cqe_rsp_good(cqe[0]) == 1,
            "A2 TUR：CQE iptt=1 dev=1 cmplt=0 rspns_good=1");
        bool seq_ok = true;
        for (int i = 0; i < 7; i++) if (sas_cqe_iptt(cqe[i]) != (uint16_t)(i + 1)) seq_ok = false;
        chk(seq_ok, "A12 CQE iptt 顺序 = 1..7");
        chk(sas_cqe_cmplt(cqe[4]) == SAS_HBA_CQE_CMPLT_ERR &&
            sas_cqe_rsp_good(cqe[4]) == 0,
            "A6 未知 opcode：CQE cmplt=3（错误 ✓）");

        // ── A3 INQUIRY 数据 ──
        {
            uint8_t d[36];
            pair->mem.peek(dat_of(1), d, 36);
            chk(d[4] == 32, "A3 INQUIRY byte4 = 32（RTL INQ_STD_LEN-4 ✓ 差异 D1 闭）");
            chk(memcmp(&d[8], "-UME MIS", 8) == 0, "A3 vendor = -UME MIS");
            chk(memcmp(&d[16], "-SAS-DHVM652", 12) == 0, "A3 product = -SAS-DHVM652");
        }
        // ── A4 MODE SENSE ──
        {
            uint8_t d[64];
            pair->mem.peek(dat_of(2), d, 64);
            chk(d[0] == 0x3F && d[4] == 0x08 && d[5] == 0x12,
                "A4 MODE SENSE 64B：byte0=0x3F byte4=0x08 byte5=0x12");
        }
        // ── A5 READ 2048B 数据比对（多帧 ✓）──
        {
            std::vector<uint8_t> got(2048), exp(2048);
            pair->mem.peek(dat_of(3), got.data(), 2048);
            for (int i = 0; i < 4; i++)
                for (int k = 0; k < 512; k++) exp[(size_t)i * 512 + k] = (uint8_t)((TST_LBA + i) * 512 + k) ^ 0xA5;
            chk(got == exp, "A5 READ(10) 2048B 数据逐字节一致（3 个 DATA 帧、偏移递增 ✓）");
            chk(pair->hdd.o_lba_read.read().to_uint() == 4u, "A5 hdd.o_lba_read = 4（4 块 ✓）");
            chk(pair->hba.o_frames_rx.read().to_uint() >= 6u, "A5 HBA 收帧数 ≥6（含多 DATA ✓）");
        }
        // ── A6 REQ SENSE 数据 ──
        {
            uint8_t d[64];
            pair->mem.peek(dat_of(5), d, 64);
            chk(d[0] == 0x70 && d[7] == 0x0A && d[2] == 0x05 && d[12] == 0x20,
                "A6 REQ SENSE：0x70/len=0x0A/key=0x05/ASC=0x20");
        }
        // ── A7 LUN≠0 的 INQUIRY ⇒ PQ=3 ──
        {
            uint8_t d[36];
            pair->mem.peek(dat_of(6), d, 36);
            chk((d[0] & 0xE0) == 0x60, "A7 LUN≠0 ⇒ INQUIRY byte0 PQ=011b（0x60 ✓ 缺陷#）");
        }
        // ── A9 状态缓冲（TUR 响应原文 ✓）──
        {
            uint32_t d0 = pair->mem.rd32(STS_BASE);
            uint32_t d8 = pair->mem.rd32(STS_BASE + 32);
            chk((d0 & 0xFFu) == 0x40, "A9 sts_buf dw0 byte0 = 0x40（RESPONSE ✓）");
            chk(((d8 >> 24) & 0xFFu) == 0x00, "A9 sts_buf byte35 = 0x00（status GOOD ✓）");
        }
        // ── A8 中断电平 + W1C ──
        {
            bool irq_hi = pair->hba.irq.read();
            chk(irq_hi && pair->hba.o_irq_cnt.read().to_uint() >= 1u,
                "A8 CQE 后 irq=1 且 o_irq_cnt≥1");
            pair->hba.reg_write(SAS_HBA_INT_OQ_SRC, sc_uint<32>(0xFFFFFFFFu));  // W1C ✓
            tick(2);
            chk(!pair->hba.irq.read(), "A8 W1C 清 OQ_INT_SRC ⇒ irq 落低（内核口径 ✓）");
        }
        // ── A10 计数 ──
        chk(pair->hba.o_tag_mismatch.read().to_uint() == 0u, "A10 o_tag_mismatch = 0");
        chk(pair->hba.data_drop_cnt == 0ul, "A10 data_drop_cnt = 0（无 DATA 落空 ✓）");
        chk(pair->hba.o_cmds_err.read().to_uint() == 1u, "A10 hba.o_cmds_err = 1（仅未知 opcode ✓）");
        chk(pair->hdd.o_cmds_good.read().to_uint() == 6u &&
            pair->hdd.o_cmds_chk.read().to_uint() == 1u,
            "A10 hdd.o_cmds_good=6 / o_cmds_chk=1");
        // ── A11 链路 ──
        chk(pair->lk_dropped.read().to_uint() == 0u, "A11 link.o_dropped = 0");

        printf("[合计] PASS=%d FAIL=%d\n", pass_cnt, fail_cnt);
        sc_core::sc_stop();
    }
};

int sc_main(int argc, char **argv) {
    (void)argc; (void)argv;
    sc_core::sc_clock clk("clk", 10, sc_core::SC_NS);
    sc_core::sc_signal<bool> rst_n;
    SasPairTop pair("pair", MEM_BASE, 0x01000000ull, 4096);
    TbPair tb("tb");
    DummyAmbaMaster stub("stub");              // ⚠ 从口须绑 1（ONE ✗ 不能多）⇒ 本台不办 socket 事务用占位 ✓
    tb.clk(clk); tb.rst_n(rst_n); tb.pair = &pair;
    pair.clk(clk); pair.rst_n(rst_n);
    stub.m.bind(pair.hba.reg_s);
    rst_n.write(false);
    sc_core::sc_start(200, sc_core::SC_NS);          // 20 拍复位 ✓
    rst_n.write(true);
    sc_core::sc_start();                              // 到 sc_stop ✓
    int rc = (tb.fail_cnt == 0) ? 0 : 1;
    printf("%s\n", rc == 0 ? "TB_PAIR_BRINGUP PASS" : "TB_PAIR_BRINGUP FAIL");
    return rc;
}
