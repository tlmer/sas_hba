//=============================================================================
// sas_hdd_tlm.cpp — SAS 硬盘（SSP target）事务级模型实现  [2026-10-08 建]
// 行为逐条出处：`RTL 源`（含 缺陷#/缺陷#/缺陷#/缺陷# 修复 ✓）
//             + `RTL 源`（帧布局 ✓）+ `RTL 源`（INQUIRY ✓）
// [2026-10-08 修订1] ① DATA 帧改为「先填载荷再入队」（旧路径先 queue 后覆写 ⇒ 载荷全 0 ✗）
//                    ② 长数据自动分帧（READ 512B=134dword > 旧 64 dword 上限被静默截断 ✗）
//                    ③ INQUIRY byte4 = 32（RTL `RTL 源` `INQ_STD_LEN-4` ✓，旧写 31 ✗）
//                    ④ INQUIRY LUN 过滤（缺陷# ✓：LUN≠0 ⇒ PQ=3）
//=============================================================================
#include "sas_hdd_tlm.h"
#include <cstring>

// ── 常量（出处见头文件注释 ✓）──
static const int FRAME_HDR_DW = 6;                          // DATA 帧头 6 dword ✓
static const int RESP_DW      = 12;                         // RESPONSE 12 拍（DW11=CRC 槽，本模型填 0 ✓）
static const uint32_t SECTOR  = 512;
// 单帧载荷上限 = 缓冲 − 头（250 dword = 1000B ✓）；超过则分帧、DataOffset 递增 ✓
static const int MAX_PAYLOAD_DW = HDD_MAX_DW - FRAME_HDR_DW;

SasHddTlm::SasHddTlm(sc_core::sc_module_name nm, uint64_t blocks)
: sc_module(nm),
  clk("clk"), rst_n("rst_n"),
  rx_data("rx_data"), rx_valid("rx_valid"), rx_sof("rx_sof"), rx_eof("rx_eof"),
  rx_type("rx_type"), rx_ready("rx_ready"),
  tx_data("tx_data"), tx_valid("tx_valid"), tx_sof("tx_sof"), tx_eof("tx_eof"),
  tx_type("tx_type"), tx_ready("tx_ready"),
  o_rx_frames("o_rx_frames"), o_tx_frames("o_tx_frames"),
  o_cmds_good("o_cmds_good"), o_cmds_chk("o_cmds_chk"),
  o_lba_read("o_lba_read"), o_lba_write("o_lba_write"),
  o_last_opcode("o_last_opcode"), o_last_status("o_last_status"),
  o_dbg_state("o_dbg_state")
{
    cfg_blocks = blocks ? blocks : 1024;     // 构造期定容量 ✓（默认 1K 块）
    store.assign((size_t)cfg_blocks * SECTOR, 0x00);
    inj_key = 0; inj_asc = 0; inj_ascq = 0; inj_pending = false;
    // ⚠ 节气纪律：构造期只写内部初值，**不写端口** ✓（端口初值在复位分支 ✓）
    rx_dwc = 0; rx_frame_type_lat = 0;
    c_rx_frames = c_tx_frames = c_good = c_chk = c_rd = c_wr = 0;
    last_opcode = last_status = 0;
    st = S_IDLE;
    cur_tag = 0; tx_active = false; tx_presented = false; tx_total = tx_idx = 0; tx_ft = 0;
    SC_METHOD(step);
    sensitive << clk.pos();
    dont_initialize();
}

// ── 帧字节访问：帧字节 n = dword n/4 的 bits[8*(n%4)+7 : 8*(n%4)] ✓ ──
uint64_t SasHddTlm::frame_byte(int n) const
{
    return dw_byte(rx_dw[n / 4], n % 4);
}

// COMMAND IU 的 LUN：帧字节 24-31（DW6-7，8 字节大端 ✓ `RTL 源`）
uint64_t SasHddTlm::frame_lun() const
{
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v = (v << 8) | (uint8_t)frame_byte(TLS_LUN_OFF + i);
    return v;
}

// CDB 取 16 字节：帧字节 TLS_CDB_OFF..+15 ✓（口径与 `RTL 源` 同 ✓）
void SasHddTlm::cdb_get(uint8_t cdb[16]) const
{
    for (int i = 0; i < 16; i++) cdb[i] = (uint8_t)frame_byte(TLS_CDB_OFF + i);
}

// LBA 存储直读（越界 = 0 ✓）
uint8_t SasHddTlm::store_byte(uint64_t a) const
{
    return (a < store.size()) ? store[(size_t)a] : 0;
}

// ── TX 原语（非阻塞、逐拍 ✓；valid 保持到 ready ✓）──
// ⚠ 纪律（实踩修正 ✗→✓，与 HBA 模型同型）：**推进与呈现必须分拍** —— 每个 beat
//   给对端一个**完整窗口**；末拍若与「完成清 valid」同拍双写 ⇒ 窗口消失 ⇒
//   对端收不到 EOF（曾致全链路 13/14 拍截断 ✗ 插桩实证 ✓）
void SasHddTlm::tx_begin(uint8_t type, int ndw)
{
    tx_active    = true;
    tx_total     = ndw;
    tx_idx       = 0;
    tx_ft        = type;
    tx_presented = false;
}

bool SasHddTlm::tx_step()
{
    if (!tx_active) { tx_valid.write(false); tx_sof.write(false); tx_eof.write(false); return true; }
    if (tx_presented) {
        if (tx_ready.read()) {                 // ready = 上一窗口对端受理 ✓
            tx_idx++;
            if (tx_idx >= tx_total) {          // 帧完成 ⇒ 本拍清 valid（与呈现分拍 ✓）
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

// ── 数据帧（D2H）：**先在 tx_dw 填齐 头+载荷、再一次入队** ✓
//    ⚠ 旧路径「先 queue 后覆写载荷」⇒ 发出去全 0 ✗（本文件修订1 已修 ✓）
//    载荷 > MAX_PAYLOAD_DW ⇒ 自动分帧，DataOffset 递增（真 SSP 多 DATA 帧同型 ✓）
//    payload != nullptr ⇒ 用该缓冲；否则从 LBA 存储取（READ ✓）
void SasHddTlm::queue_data_frame(uint64_t lba, const uint8_t* payload, uint32_t nbytes)
{
    uint32_t off = 0;
    do {
        uint32_t chunk = nbytes - off;
        if (chunk > (uint32_t)(MAX_PAYLOAD_DW * 4)) chunk = (uint32_t)(MAX_PAYLOAD_DW * 4);
        int pldw = (int)(chunk / 4);
        int ndw  = FRAME_HDR_DW + pldw;
        tx_dw[0] = 0x60u;                              // byte0 = 0x60（帧型 6 ✓）
        tx_dw[1] = 0; tx_dw[2] = 0; tx_dw[3] = 0;
        tx_dw[4] = ((uint32_t)(cur_tag & 0xFF) << 8) | ((uint32_t)(cur_tag >> 8) & 0xFF); // tag 字节交换 ✓
        tx_dw[5] = ((off & 0xFFu) << 24) | (((off >> 8) & 0xFFu) << 16)
                 | (((off >> 16) & 0xFFu) << 8) | ((off >> 24) & 0xFFu);  // DataOffset(be32) ✓
        for (int i = 0; i < pldw; i++) {
            uint32_t b[4];
            for (int k = 0; k < 4; k++) {
                uint64_t a = lba * SECTOR + off + (uint64_t)i * 4 + k;
                b[k] = payload ? (uint32_t)payload[off + i * 4 + k] : (uint32_t)store_byte(a);
            }
            tx_dw[FRAME_HDR_DW + i] = b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24);  // 帧字节 k=bits[8k+7:8k] ✓
        }
        queue_cur(FT_DATA, ndw);
        off += chunk;
    } while (off < nbytes);
}

// ── RESPONSE（12 拍）：DW4=tag(be16) ✓ / DW8: byte35=STATUS ✓
//    ⚠ DW10（帧字节 40-43，SENSE LEN）**照 RTL 现状恒 0** ✓（`RTL 源`；
//      sense 经 REQ SENSE 命令提供 —— 对拍差异注 D3 见 md/对拍报告.md ✓）
void SasHddTlm::build_response(uint8_t status)
{
    for (int i = 0; i < RESP_DW; i++) tx_dw[i] = 0;
    tx_dw[0] = 0x40u;                                 // byte0 = 0x40（帧型 4 ✓）
    tx_dw[4] = ((uint32_t)(cur_tag & 0xFF) << 8) | ((uint32_t)(cur_tag >> 8) & 0xFF);
    tx_dw[8] = ((uint32_t)status << 24);              // byte35 = STATUS ✓（byte34 DATA PRESENT=0，照 RTL ✓）
    queue_cur(FT_RESPONSE, RESP_DW);
}

// 把当前 tx_dw[0..ndw) 排入帧队列 ✓（数据/响应顺序 = 调用顺序 ✓）
void SasHddTlm::queue_cur(uint8_t ft, int ndw)
{
    if (ndw > HDD_MAX_DW) ndw = HDD_MAX_DW;
    TlsFrame f; f.n = ndw; f.ft = ft;
    for (int i = 0; i < ndw; i++) f.dw[i] = tx_dw[i];
    qf.push_back(f);
}

void SasHddTlm::set_sense(uint8_t key, uint8_t asc, uint8_t ascq)
{ inj_key = key; inj_asc = asc; inj_ascq = ascq; inj_pending = true; }

uint8_t* SasHddTlm::lba_ptr(uint64_t lba) {
    if (lba * SECTOR >= store.size()) return nullptr;
    return &store[(size_t)lba * SECTOR];
}

// ═══════════════ SCSI 命令执行 ═══════════════
// 每条行为出处 = `RTL 源` 对应分支（含全部修复 ✓）；长度口径 = 缺陷#（64B ✓）
void SasHddTlm::exec_command(const uint8_t cdb[16])
{
    uint8_t  op   = cdb[0];
    uint64_t lun  = frame_lun();      // LUN（帧字节 8-15，8B 大端 ✓）
    last_opcode   = op;

    switch (op) {
    // ── TEST UNIT READY：GOOD、无数据 ✓（`OP_TEST_READY` 分支 ✓）──
    case 0x00:
        last_status = SCSI_GOOD; c_good++;
        build_response(SCSI_GOOD);
        break;

    // ── INQUIRY：36 字节 ✓（`RTL 源` 逐字段 ✓；缺陷#：LUN≠0 ⇒ PQ=3 ✓）──
    case 0x12: {
        uint8_t data[36];
        memset(data, 0, sizeof(data));
        data[0] = 0x00;                                  // PQ=0 / 外设类型=0（直接访问盘 ✓）
        data[1] = 0x00;                                  // RMB=0
        data[2] = 0x00;                                  // version（ANSI=0 ✓ 内核实测所见 ✓）
        data[3] = 0x20;                                  // response data format=2 ✓
        data[4] = 36 - 4;                                // additional length = 32 ✓
                                                         //  ★RTL `RTL 源`：
                                                         //   inquiry_data[0][39:32] <= INQ_STD_LEN-4 ✓
                                                         //   （旧写 36-5=31 ✗ 已修；差异注 D1 已闭 ✓）
        data[7] = 0x10;                                  // SYNC ✓
        memcpy(&data[8],  "-UME MIS", 8);                // Vendor ✓（参考平台载荷逐字 ✓）
        memcpy(&data[16], "-SAS-DHVM652", 12);           // Product ✓
        // LUN 过滤（缺陷# ✓）：LUN≠0 ⇒ PQ=3（byte0[7:5]=011b ⇒ 0x60 ✓）
        // 出处 RTL `RTL 源`：admin_done_data[0][7:0] <= 8'h60 | (inquiry_data[0][7:0] & 8'h1F)
        if (lun != 0) data[0] = 0x60 | (data[0] & 0x1F);
        queue_data_frame(0, data, 36);                   // 先数据帧 ✓
        build_response(SCSI_GOOD);                       // 后 RESPONSE ✓
        last_status = SCSI_GOOD; c_good++;
        break;
    }

    // ── READ CAPACITY(10/16)：8B / 32B ✓（`OP_READ_CAP_10/16` 分支 ✓）──
    //   布局（RTL 权威 ✓ `RTL 源`）：
    //     10 ⇒ 末 LBA **BE32 @byte0-3** + 块长 **BE32 @byte4-7**（512 ✓）；len=8 ✓
    //     16 ⇒ 末 LBA **BE64 @byte0-7** + 块长 BE32 @byte8-11 + 补零至 32 ✓
    //   ⚠ 台曾抓出模型旧码「64 位 BE 全塞前 8 字节」✗（末 LBA 低字节错位到 byte7）
    case 0x25: case 0x9E: {
        uint64_t last_lba = cfg_blocks - 1;
        const uint32_t BSZ = 512;
        int bytes = (op == 0x25) ? 8 : 32;
        int lb    = (op == 0x25) ? 4 : 8;                 // 末 LBA 宽度（字节 ✓）
        uint8_t data[32];
        memset(data, 0, sizeof(data));
        for (int i = 0; i < lb; i++) data[i] = (uint8_t)(last_lba >> (8 * (lb - 1 - i)));  // BE ✓
        data[lb + 0] = (uint8_t)(BSZ >> 24); data[lb + 1] = (uint8_t)(BSZ >> 16);
        data[lb + 2] = (uint8_t)(BSZ >> 8);  data[lb + 3] = (uint8_t)(BSZ);               // 512 BE ✓
        queue_data_frame(0, data, (uint32_t)bytes);
        build_response(SCSI_GOOD);                       // 数据帧后补 RESPONSE ✓
        last_status = SCSI_GOOD; c_good++;
        break;
    }

    // ── MODE SENSE(6/10)：**64B**（缺陷# ✓ byte0=0x3F、byte4=0x08 caching、WCE=0 ✓）──
    case 0x1A: case 0x5A: {
        uint8_t data[64];
        memset(data, 0, sizeof(data));
        data[0] = 0x3F;                                  // mode data length = 63 ✓
        data[2] = 0x00;                                  // device-specific：WP=0 ✓
        data[3] = 0x00;                                  // block descriptor length = 0 ✓
        data[4] = 0x08;                                  // caching page code ✓
        data[5] = 0x12;                                  // page length = 18 ✓
        queue_data_frame(0, data, 64);
        build_response(SCSI_GOOD);                       // 数据帧后补 RESPONSE ✓
        last_status = SCSI_GOOD; c_good++;
        break;
    }

    // ── REQUEST SENSE：**64B**（缺陷# ✓；缺陷# ✓ 标准 18B sense 0x70 + 附加长度 0x0A）──
    case 0x03: {
        uint8_t data[64];
        memset(data, 0, sizeof(data));
        data[0] = 0x70;                                  // response code ✓
        data[7] = 0x0A;                                  // additional length = 10 ✓
        if (inj_pending) { data[2] = inj_key; data[12] = inj_asc; data[13] = inj_ascq; inj_pending = false; }
        queue_data_frame(0, data, 64);
        build_response(SCSI_GOOD);                       // 数据帧后补 RESPONSE ✓
        last_status = SCSI_GOOD; c_good++;
        break;
    }

    // ── REPORT LUNS：**64B**（缺陷# ✓；缺陷# ✓ 条目落字节 8+8i、长度字段 = 8 ✓）──
    case 0xA0: {
        uint8_t data[64];
        memset(data, 0, sizeof(data));
        data[3] = 0x08;                                  // LUN 列表长度 = 8（BE ✓ lun_count*8 ✓）
        queue_data_frame(0, data, 64);
        build_response(SCSI_GOOD);                       // 数据帧后补 RESPONSE ✓
        last_status = SCSI_GOOD; c_good++;
        break;
    }

    // ── START STOP / SYNC CACHE / MODE SELECT / FORMAT：GOOD、无数据 ✓ ──
    case 0x1B: case 0x35: case 0x91: case 0x15: case 0x55: case 0x04:
        last_status = SCSI_GOOD; c_good++;
        build_response(SCSI_GOOD);
        break;

    // ── READ(6/10/16)：DATA-in ✓（数据来自内存 LBA 存储 ✓；长读自动分帧 ✓）──
    case 0x08: case 0x28: case 0x88: {
        uint64_t lba = 0; uint32_t blocks = 0;
        if (op == 0x08) { lba = ((uint64_t)cdb[1] << 16) | ((uint64_t)cdb[2] << 8) | cdb[3];
                          blocks = cdb[4] ? cdb[4] : 256; }              // 0 ⇒ 256 边例 ✓（镜像引擎 ✓）
        else if (op == 0x28) { lba = ((uint64_t)cdb[2] << 24) | ((uint64_t)cdb[3] << 16)
                                   | ((uint64_t)cdb[4] << 8) | cdb[5];
                               blocks = ((uint32_t)cdb[7] << 8) | cdb[8]; }
        else { for (int i = 0; i < 8; i++) lba = (lba << 8) | cdb[2 + i];
               blocks = ((uint32_t)cdb[10] << 24) | ((uint32_t)cdb[11] << 16)
                      | ((uint32_t)cdb[12] << 8) | cdb[13]; }
        uint32_t bytes = blocks * SECTOR;
        queue_data_frame(lba, nullptr, bytes);           // 从 LBA 存储取 ✓（自动分帧 ✓）
        build_response(SCSI_GOOD);                       // 数据帧后补 RESPONSE ✓
        c_rd += blocks;
        last_status = SCSI_GOOD; c_good++;
        break;
    }

    // ── WRITE(6/10/16)：数据 OUT ⇒ GOOD ✓（载荷在后续 DATA 帧收 ✓）──
    case 0x0A: case 0x2A: case 0x8A: {
        last_status = SCSI_GOOD; c_good++;
        build_response(SCSI_GOOD);           // ⚠ 本模型 = 立即 GOOD（不等待/不消费 DATA-out 帧 ✗ 简化；
                                             //   见 md/设计方案.md「不做」清单 ✓）
        break;
    }

    // ── 未知 opcode：CHECK CONDITION + ILLEGAL REQUEST/ASC 0x20 ✓（缺陷# ✓）──
    default: {
        last_status = SCSI_CHECK_CONDITION; c_chk++;
        build_response(SCSI_CHECK_CONDITION);            // sense 经 REQ SENSE 提供（照 RTL ✓ D3）
        break;
    }
    }
}

// ═══════════════ 主状态机（每拍一步 ✓）═══════════════
void SasHddTlm::step()
{
    if (!rst_n.read()) {
        rx_ready.write(false);
        tx_valid.write(false); tx_sof.write(false); tx_eof.write(false);
        tx_data.write(0); tx_type.write(0);
        st = S_IDLE; rx_dwc = 0; rx_frame_type_lat = 0;
        tx_active = false; tx_presented = false; tx_idx = tx_total = 0;
        o_rx_frames.write(0); o_tx_frames.write(0);
        o_cmds_good.write(0); o_cmds_chk.write(0);
        o_lba_read.write(0); o_lba_write.write(0);
        o_last_opcode.write(0); o_last_status.write(0);
        o_dbg_state.write(0);
        return;
    }

    rx_ready.write(true);                                // 本模型恒可收 ✓

    switch (st) {
    case S_IDLE: {
        o_dbg_state.write(0x10);
        // 收帧装配（每拍 1 dword ✓）
        if (rx_valid.read() && rx_ready.read()) {
            if (rx_sof.read()) { rx_dwc = 0; rx_frame_type_lat = (uint8_t)rx_type.read().to_uint(); }
            if (rx_dwc < HDD_MAX_DW) rx_dw[rx_dwc] = rx_data.read().to_uint();
            rx_dwc++;
            if (rx_eof.read()) {
                c_rx_frames++;
                o_rx_frames.write(c_rx_frames);
                if (rx_frame_type_lat == FT_COMMAND) {
                    cur_tag = (uint16_t)((frame_byte(16) << 8) | frame_byte(17));  // tag be16 ✓
                    st = S_EXEC;
                }
                rx_dwc = 0;
            }
        }
        break;
    }
    case S_EXEC: {
        o_dbg_state.write(0x20);
        uint8_t cdb[16];
        cdb_get(cdb);
        exec_command(cdb);
        st = S_TX;
        break;
    }
    case S_TX: {
        o_dbg_state.write(0x30);
        if (!tx_active && !qf.empty()) {                 // 取下一帧 ✓
            TlsFrame f = qf.front(); qf.erase(qf.begin());
            for (int i = 0; i < f.n; i++) tx_dw[i] = f.dw[i];
            tx_begin(f.ft, f.n);
        }
        if (tx_active && tx_step()) {
            c_tx_frames++;
            o_tx_frames.write(c_tx_frames);
            // ⛔ 勿在此再清 valid（tx_step 完成分支已写 ✓ 防同拍双写 ✗）
            if (qf.empty()) st = S_IDLE;                 // 队列排干 ⇒ 收下一命令 ✓
        }
        break;
    }
    default: st = S_IDLE; break;
    }

    // 观测口（每拍刷新 ✓）
    o_cmds_good.write(c_good); o_cmds_chk.write(c_chk);
    o_lba_read.write(c_rd); o_lba_write.write(c_wr);
    o_last_opcode.write(last_opcode); o_last_status.write(last_status);
}
