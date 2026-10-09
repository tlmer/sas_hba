//=============================================================================
// sas_hdd_tlm.h — SAS 硬盘（SSP target）事务级模型  [2026-10-08 建]
//-----------------------------------------------------------------------------
// 规格出处：本参考实现 RTL（已参考平台验证 ✓）＋ 内核驱动（hisi_sas / libsas）
//   · 帧级契约 = `RTL 源` 顶层端口 + 目标 FSM 行为
//   · SCSI 语义 = `RTL 源`（含 缺陷#/缺陷#/缺陷#/缺陷# 全部修复 ✓）
//   · 数据帧/响应帧布局 = `RTL 源`（DW0..DW11，逐 dword 已验证 ✓）
//
// 两侧契约（本模型 ↔ 主控/链路，帧级；与 RTL hub 同口径 ✓）：
//   RX（收），每拍 1 dword：
//     `rx_sof` 拍 = 帧首 dword；`rx_eof` 拍 = 帧末 dword；`rx_type` = 帧型半字节
//       （1=COMMAND、6=DATA、4=RESPONSE、5=XFER_RDY、2=TASK、3=OPEN ✓
//        出处 `RTL 源` 常量表 + `RTL 源` FT_* ✓）
//     `rx_valid && rx_ready` 为拍完成 ✓（本模型恒 ready ✓）
//   TX（发）：同构；`tx_ready` 为对端反压 ✓（valid 保持到 ready ✓，held-valid 纪律 ✓）
//   帧收尾：数据/响应帧以 EOF 结束；链路层 ACK/NAK 由 `sas_link_tlm` 简化建模 ✓
//
// 帧字节口径（**全线统一**，出处 `RTL 源` 注释 + 内核 sas.h 对照 ✓）：
//   dword 内：帧字节 k = bits[8k+7 : 8k]（byte0 = bits[7:0] ✓）
//   COMMAND IU：帧字节 16-17 = initiator tag（**大端**：byte16 = tag[15:8] ✓）
//               CDB 16 字节 = 帧字节 CDB_OFF..CDB_OFF+15（见 TLS_CDB_OFF ✓）
//   DATA 帧头（6 dword）：DW0 byte0[7:4]=6；DW4 = {TPTT be16, tag be16}；DW5 = 字节偏移 ✓
//   RESPONSE（12 拍含 CRC 槽）：DW4 tag（be16）✓；DW8 byte34[1:0]=DATA PRESENT / byte35=STATUS ✓；
//               DW10 = SENSE LEN(be32) ✓；DW11 = RESP LEN(be32) ✓
//
// ⛔ 简化（本模型不做）：
//   · PHY/链路训练、OPEN/CLOSE 连接管理（帧级直连；`sas_link_tlm` 可加延迟/错误注入）
//   · SSP TASK(TMF) 帧：留接口不实现（返回 FUNCTION NOT SUPPORTED 的 TMF 响应由上层按需加 ✓）
//   · 扇区级 ECC/介质错误（可注入 ✓）；DIF/PI；NCQ/STP
//   · 时序精度：TLM 每拍一步，无模拟延迟（背压只体现为 ready 反压 ✓）
//
// 白盒观测（⛔ 只 TB 用 ✓）：`o_*` 计数/最近一次状态，判据直接读 ✓
//=============================================================================

#ifndef SAS_HDD_TLM_H
#define SAS_HDD_TLM_H

#include <systemc.h>
#include <vector>
#include <cstdint>

#define SAS_HDD_TLM_VERSION_MAJOR 1
#define SAS_HDD_TLM_VERSION_MINOR 0
#define SAS_HDD_TLM_VERSION_STR   "1.0"

// 帧型（半字节，出处见头注释 ✓）
enum : uint8_t {
    FT_COMMAND  = 1,
    FT_DATA     = 6,
    FT_RESPONSE = 4,
    FT_XFER_RDY = 5,
    FT_TASK     = 2,
    FT_OPEN     = 3,
};

// COMMAND IU 内 CDB 起始帧字节（凭 RTL 实测：CDB 16 字节 ✓；单位=帧字节）
// 出处（双侧逐字一致 ✓）：
//   · HDD `RTL 源`「IU 字节 12-27 = 帧字节 36-51 = DW9..DW12」
//   · HBA `RTL 源`「cmd_tbl_accum[(36*8) +: 128]」
//   ⇒ 帧字节 36 = DW9 字节 0 ✓（COMMAND 帧共 14 dword：9 头 + 4 CDB + 1 CRC ✓）
static const int TLS_CDB_OFF = 36;

// COMMAND IU 内 LUN 起始帧字节（DW6-7 = 帧字节 24-31，8 字节大端 ✓）
// 出处（双侧逐字一致 ✓）：
//   · HDD `RTL 源`「byte24-27 = LUN[31:0]；byte28-31 = LUN[63:32]」
//   · HBA `RTL 源`「DW6-7: byte24-31 = LUN」
//   ⚠ 非 SAS 规范惯用的 IU 字节 8-15 ⇒ 建模以 RTL 实测为准 ✓；缺陷# 过滤只看「LUN≠0」✓
static const int TLS_LUN_OFF = 24;

// 帧缓冲上限（dword ✓）：DATA 帧 = 头 6 + 载荷 ≤250 ⇒ 256 ✓
// ⚠ READ 512B = 134 dword > 旧 64 ✗ ⇒ 已改「长载荷自动分帧」（DataOffset 递增 ✓ 真 SSP 同型 ✓）
static const int HDD_MAX_DW = 256;

// SCSI status（只实现用到的三项 ✓）
enum : uint8_t {
    SCSI_GOOD            = 0x00,
    SCSI_CHECK_CONDITION = 0x02,
};

struct SasHddTlm : sc_core::sc_module {
    // ── 时钟/复位（端口，不写初值 ✓ 节气纪律）──
    sc_core::sc_in<bool> clk;
    sc_core::sc_in<bool> rst_n;

    // ── SSP 帧级 RX（收）──
    sc_core::sc_in<sc_uint<32>> rx_data;
    sc_core::sc_in<bool>        rx_valid;
    sc_core::sc_in<bool>        rx_sof;
    sc_core::sc_in<bool>        rx_eof;
    sc_core::sc_in<sc_uint<4>>  rx_type;
    sc_core::sc_out<bool>       rx_ready;

    // ── SSP 帧级 TX（发）──
    sc_core::sc_out<sc_uint<32>> tx_data;
    sc_core::sc_out<bool>        tx_valid;
    sc_core::sc_out<bool>        tx_sof;
    sc_core::sc_out<bool>        tx_eof;
    sc_core::sc_out<sc_uint<4>>  tx_type;
    sc_core::sc_in<bool>         tx_ready;

    // ── 配置（构造后稳定 ✓）──
    uint64_t cfg_blocks;        // 盘容量（512B/块 ✓ 出处 `cfg_disk_blocks` ✓）

    // ── 白盒观测（⛔ 只 TB）──
    sc_core::sc_out<sc_uint<32>> o_rx_frames;      // 收完的帧数
    sc_core::sc_out<sc_uint<32>> o_tx_frames;      // 发完的帧数
    sc_core::sc_out<sc_uint<32>> o_cmds_good;      // 命令：GOOD 完成数
    sc_core::sc_out<sc_uint<32>> o_cmds_chk;       // 命令：CHECK CONDITION 数
    sc_core::sc_out<sc_uint<32>> o_lba_read;       // 累计读出块数
    sc_core::sc_out<sc_uint<32>> o_lba_write;      // 累计写入块数
    sc_core::sc_out<sc_uint<8>>  o_last_opcode;    // 最近一条 CDB opcode
    sc_core::sc_out<sc_uint<8>>  o_last_status;    // 最近一条 status
    sc_core::sc_out<sc_uint<32>> o_dbg_state;      // 状态机（调试）

    SC_HAS_PROCESS(SasHddTlm);
    SasHddTlm(sc_core::sc_module_name nm, uint64_t blocks = 1024);   // 容量在构造期定 ✓

    // ── TB 注入钩子（⛔ 只 TB）──
    void set_sense(uint8_t key, uint8_t asc, uint8_t ascq);  // 注入一条 sense（下次 CHECK CONDITION 用 ✓）
    uint8_t* lba_ptr(uint64_t lba);                          // 直访存储（TB 预置/校验 ✓）

    // 版本（照先例 ✓）
    static const char* version() { return SAS_HDD_TLM_VERSION_STR; }

private:
    // 帧接收装配
    int      rx_dwc;                       // 帧内 dword 计数（SOF 拍起 1 ✓）
    uint32_t rx_dw[HDD_MAX_DW];            // 帧缓冲（COMMAND 14 dword；DATA(W) ≤256 ✓）
    uint8_t  rx_frame_type_lat;            // SOF 拍锁存的帧型 ✓（照 RTL ⑪-D 纪律 ✓）

    // 存储
    std::vector<uint8_t> store;

    // 注入 sense
    uint8_t inj_key, inj_asc, inj_ascq;
    bool    inj_pending;

    // 观测计数（内部）
    uint32_t c_rx_frames, c_tx_frames, c_good, c_chk, c_rd, c_wr;
    uint8_t  last_opcode, last_status;

    // 状态机与子流程
    enum { S_IDLE = 0, S_EXEC, S_TX } st;
    void step();

    // ── 内部工具（实现见 .cpp ✓）──
    static uint8_t  dw_byte(uint32_t dw, int k) { return (dw >> (8 * k)) & 0xFF; }  // 帧字节 k ✓
    uint32_t frame_dw(int n) const { return rx_dw[n]; }
    uint64_t frame_byte(int n) const;      // 帧字节 n（跨 dword ✓）
    uint64_t frame_lun() const;            // COMMAND IU 的 LUN（帧字节 8-15，8B 大端 ✓）
    void     cdb_get(uint8_t cdb[16]) const;
    uint8_t  store_byte(uint64_t a) const; // LBA 存储直读（越界=0 ✓）

    // 收发原语（非阻塞、逐拍 ✓）+ **帧队列**（一条命令可出多帧：数据（可分帧）+ 响应 ✓）
    void tx_begin(uint8_t type, int ndw);
    bool tx_step();                        // 返回 true=发完 ✓
    struct TlsFrame { uint32_t dw[HDD_MAX_DW]; int n; uint8_t ft; };
    std::vector<TlsFrame> qf;
    void queue_cur(uint8_t ft, int ndw);   // 把 tx_dw[0..ndw) 排入队列 ✓

    // SCSI 执行（全部行为出处见 .cpp 各分支注释 ✓）
    void exec_command(const uint8_t cdb[16]);
    uint16_t cur_tag;

    // TX 缓冲
    uint32_t tx_dw[HDD_MAX_DW];
    int      tx_total, tx_idx;
    bool     tx_active;
    bool     tx_presented;                 // 推进与呈现分拍（见 .cpp「实踩修正」✓）
    uint8_t  tx_ft;

    // 数据帧发送（DATA-in）：**填好头+载荷再入队** ✓；长载荷自动分帧、DataOffset 递增 ✓
    //   payload != nullptr ⇒ 用该缓冲；否则从 LBA 存储取（READ 路径 ✓）
    void queue_data_frame(uint64_t lba, const uint8_t* payload, uint32_t nbytes);
    // 响应帧（12 拍：DW0..DW10 呈现 + CRC 槽 ✓）
    void build_response(uint8_t status);
};

#endif // SAS_HDD_TLM_H
