/*============================================================================
 * sas_hba_hw.h — SAS HBA 序列层：DQ 条目 / SGE / 命令表 / CQE 编解码  [2026-10-08 建]
 *
 * ★★ 用途 = **模型与 TB（以及任何宿主序列）共用的纯函数库** —— 只依赖
 *    `sas_hba_regs.h`，无动态内存、无 SystemC 依赖（纯 C/C++ ✓）。
 *    ⚠ 每条布局与 `sas_hba_regs.h` 红线注一一对应；改这里必须同步两处 ✓
 *
 * ★ 字节序总规则（全工程 ✓）：
 *    · 帧字节 k = dword 内 bits[8k+7:8k]（byte0 = bits[7:0] ✓）
 *    · 线上 be16 字段（tag/tptt）：帧字节 16-17 ⇒ byte16=tag[15:8] ✓
 *    · 内存结构（DQ 条目/CQE/SGE）= **小端 32/64 位**（AXI LE ✓）
 *============================================================================*/
#ifndef SAS_HBA_HW_H
#define SAS_HBA_HW_H

#include <stdint.h>
#include <string.h>
#include "sas_hba_regs.h"

/*--------------------------------------------------------------------------*/
/* §1 位域读写（**dw 索引 + dw 内位偏移**，与 regs.h 的 DQ0_/DQ1_/… 宏同口径 ✓）*/
/*--------------------------------------------------------------------------*/
static inline void sas_bf_set_dw(uint32_t *dw, unsigned dwi, unsigned lo, unsigned width, uint32_t v) {
    uint32_t m = (width >= 32) ? 0xFFFFFFFFu : ((1u << width) - 1u);
    dw[dwi] = (dw[dwi] & ~(m << lo)) | ((v & m) << lo);
}
static inline uint32_t sas_bf_get_dw(const uint32_t *dw, unsigned dwi, unsigned lo, unsigned width) {
    uint32_t m = (width >= 32) ? 0xFFFFFFFFu : ((1u << width) - 1u);
    return (dw[dwi] >> lo) & m;
}
static inline void sas_dw_set_u64(uint32_t *dw, unsigned lo_dw, uint64_t v) {
    dw[lo_dw] = (uint32_t)v; dw[lo_dw + 1] = (uint32_t)(v >> 32);
}
static inline uint64_t sas_dw_get_u64(const uint32_t *dw, unsigned lo_dw) {
    return (uint64_t)dw[lo_dw] | ((uint64_t)dw[lo_dw + 1] << 32);
}

/*--------------------------------------------------------------------------*/
/* §2 DQ 64B 命令条目（16 dword ✓；字段出处见 regs.h §6）                      */
/*--------------------------------------------------------------------------*/
/* 组装一条 SSP 命令条目（最常用 ✓）；未列字段保持 0 */
static inline void sas_dq_pack_ssp(uint32_t dw[SAS_HBA_DQ_ENTRY_DW],
                                   uint16_t iptt, uint16_t dev_id,
                                   unsigned dir,          /* SAS_HBA_DIR_* ✓ */
                                   uint32_t xfer_len,     /* 0 = 无数据 ✓ */
                                   uint64_t cmd_tbl_addr, uint64_t prd_tbl_addr) {
    memset(dw, 0, sizeof(uint32_t) * SAS_HBA_DQ_ENTRY_DW);
    sas_bf_set_dw(dw, 0, SAS_HBA_DQ0_CMD_LO,       3, SAS_HBA_CMD_SSP);    /* dw0 cmd=1 ✓ */
    sas_bf_set_dw(dw, 1, SAS_HBA_DQ1_DIR_LO,       2, dir);                /* dw1 dir ✓ */
    sas_bf_set_dw(dw, 1, SAS_HBA_DQ1_FRAME_TY_LO,  5, SAS_FT_COMMAND);     /* dw1 FT ✓ */
    sas_bf_set_dw(dw, 1, SAS_HBA_DQ1_DEV_ID_LO,   16, dev_id);             /* dw1 dev ✓ */
    sas_bf_set_dw(dw, 2, SAS_HBA_DQ2_CFL_LO,       9, SAS_CMD_FRAME_DW);   /* dw2 cfl=14 ✓ */
    sas_bf_set_dw(dw, 3, SAS_HBA_DQ3_IPTT_LO,     16, iptt);               /* dw3 iptt ✓ */
    dw[4] = xfer_len;                                                      /* dw4 ✓ */
    sas_dw_set_u64(dw, SAS_HBA_DQ_CMD_TABLE_LO_DW, cmd_tbl_addr);          /* dw8-9 ✓ */
    sas_dw_set_u64(dw, SAS_HBA_DQ_PRD_TABLE_LO_DW, prd_tbl_addr);          /* dw12-13 ✓ */
}

/*--------------------------------------------------------------------------*/
/* §3 SGE（24B，**内核布局** ✓ 红线⑤）                                        */
/*--------------------------------------------------------------------------*/
static inline void sas_sge_pack(uint8_t *sge24, uint64_t addr, uint32_t len, uint32_t off) {
    memcpy(sge24 + SAS_HBA_SGE_ADDR_LO, &addr, 8);        /* le64 ✓ */
    memset(sge24 + SAS_HBA_SGE_PGCTRL0, 0, 8);            /* pgctrl 置 0（整页连续 ✓）*/
    memcpy(sge24 + SAS_HBA_SGE_DATA_LEN, &len, 4);        /* le32 ✓ */
    memcpy(sge24 + SAS_HBA_SGE_DATA_OFF, &off, 4);
}

/*--------------------------------------------------------------------------*/
/* §4 命令表（SSP：头 24B + LUN@24 + TMF@34 + CDB@36 ✓ regs.h §9）            */
/*--------------------------------------------------------------------------*/
/* 目的地址哈希（**仅逐字节对拍用**；点对点链路不看 ✓ `RTL 源`）
   h60 = a[59:0] ^ (a[55:0]<<4) ^ (a[51:0]<<8)；帧取 h[23:0] ✓ */
static inline uint32_t sas_dest_hash_lo24(uint64_t sas_addr) {
    uint64_t a = sas_addr & 0x0FFFFFFFFFFFFFFFull;
    uint64_t h = (a & 0x0FFFFFFFFFFFFFFFull)
               ^ ((a & 0x00FFFFFFFFFFFFFFull) << 4)
               ^ ((a & 0x000FFFFFFFFFFFFFull) << 8);
    return (uint32_t)(h & 0xFFFFFFu);
}
/* 组装 SSP 命令表映像（tb 侧填 1024B 缓冲；模型取 [0..23] + [34] + [36..51] ✓）*/
static inline void sas_ctbl_pack_ssp(uint8_t *tbl, const uint8_t cdb[16],
                                     const uint8_t lun[8], uint16_t tag, uint16_t tptt,
                                     uint64_t dest_sas) {
    memset(tbl, 0, SAS_HBA_CTBL_SSP_BYTES);
    /* 帧头 DW0：byte0 = FT<<4（0x10 ✓）、[31:8] = 目的哈希 lo24 ✓ */
    tbl[0] = SAS_FRAME_B0_COMMAND;
    uint32_t h = sas_dest_hash_lo24(dest_sas);
    tbl[1] = (uint8_t)(h); tbl[2] = (uint8_t)(h >> 8); tbl[3] = (uint8_t)(h >> 16);
    /* DW4（帧字节 16-19）：tag/tptt 各 be16 ✓ */
    tbl[SAS_CMD_TAG_BYTE + 0] = (uint8_t)(tag >> 8);  tbl[SAS_CMD_TAG_BYTE + 1] = (uint8_t)tag;
    tbl[SAS_CMD_TAG_BYTE + 2] = (uint8_t)(tptt >> 8); tbl[SAS_CMD_TAG_BYTE + 3] = (uint8_t)tptt;
    memcpy(tbl + SAS_HBA_CTBL_LUN_OFF, lun, 8);          /* LUN（**大端 8 字节** ✓）*/
    memcpy(tbl + SAS_HBA_CTBL_CDB_OFF, cdb, 16);         /* CDB@36 ✓ */
}

/*--------------------------------------------------------------------------*/
/* §5 CQE 解码（16B；regs.h §7 ✓）                                           */
/*--------------------------------------------------------------------------*/
static inline unsigned sas_cqe_cmplt(const uint32_t c[4])   { return c[0] & 0x3u; }
static inline unsigned sas_cqe_erx(const uint32_t c[4])     { return (c[0] >> 12) & 1u; }
static inline unsigned sas_cqe_rsp_xfrd(const uint32_t c[4]){ return (c[0] >> 10) & 1u; }
static inline unsigned sas_cqe_rsp_good(const uint32_t c[4]){ return (c[0] >> 11) & 1u; }
static inline uint16_t sas_cqe_iptt(const uint32_t c[4])    { return (uint16_t)(c[1] & 0xFFFFu); }
static inline uint16_t sas_cqe_devid(const uint32_t c[4])   { return (uint16_t)(c[1] >> 16); }

/*--------------------------------------------------------------------------*/
/* §6 帧头构建（两侧模型共用；**帧字节 k = bits[8k+7:8k]** ✓）                  */
/*--------------------------------------------------------------------------*/
/* be32（帧字节 n..n+3 大端）⇒ dword 需整体反序 ✓ */
static inline uint32_t sas_bswap32(uint32_t v) {
    return ((v & 0xFFu) << 24) | (((v >> 8) & 0xFFu) << 16)
         | (((v >> 16) & 0xFFu) << 8) | ((v >> 24) & 0xFFu);
}
/* RESPONSE 12 dword：DW0=0x40；DW4 tag(be16)；DW8：byte34[1:0]=DATA PRESENT、
   byte35=STATUS；DW10=SENSE LEN(be32@byte40-43) ✓ `RTL 源` */
static inline void sas_resp_pack(uint32_t dw[SAS_RESP_FRAME_DW], uint16_t tag,
                                 uint8_t status, uint32_t sense_len, unsigned data_present) {
    memset(dw, 0, sizeof(uint32_t) * SAS_RESP_FRAME_DW);
    dw[0] = SAS_FRAME_B0_RESPONSE;                                        /* 0x40 ✓ */
    dw[SAS_RESP_DW_TAG] = ((uint32_t)(tag & 0xFF) << 8) | ((uint32_t)tag >> 8); /* be16 交换 ✓ */
    dw[SAS_RESP_DW_STATUS] = ((uint32_t)status << 24) | ((uint32_t)(data_present & 3u) << 16);
    dw[SAS_RESP_DW_DATA_LEN] = sas_bswap32(sense_len);                    /* byte40-43 be32 ✓ */
}
/* DATA 6 dword 头 + 调用方续 payload：DW0=0x60；DW4 tag(be16)；DW5 offset(be32) ✓ */
static inline void sas_data_hdr_pack(uint32_t dw[SAS_DATA_HDR_DW], uint16_t tag, uint32_t offset) {
    memset(dw, 0, sizeof(uint32_t) * SAS_DATA_HDR_DW);
    dw[0] = SAS_FRAME_B0_DATA;                                            /* 0x60 ✓ */
    dw[SAS_DATA_DW_TAG] = ((uint32_t)(tag & 0xFF) << 8) | ((uint32_t)tag >> 8);
    dw[SAS_DATA_DW_OFFSET] = sas_bswap32(offset);
}

#endif /* SAS_HBA_HW_H */
