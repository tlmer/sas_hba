/*============================================================================
 * sas_hdd_regs.h — SAS HDD 侧常量（帧级口径 + 自有 CSR 表）  [2026-10-08 建]
 *
 * ★★ 本文件 = **HDD 模型 / TB / 序列层 的共同基准** —— 每条带出处：
 *     · 帧级口径 = 本参考实现 HDD RTL 实测（含 缺陷#..缺陷# 修复 ✓）+ 与 HBA 侧
 *       `sas_hba/tlm/sw/sas_hba_regs.h` **逐字节对照**（两侧同名常量值必须相等 ✓）
 *     · 自有 CSR = `RTL 源:行号`（读译� / 写译�）
 *
 * ⚠ **两侧对拍红线**（改动前必须双侧同改 ✓）：
 *   · 帧字节 k = dword 内 bits[8k+7:8k]（byte0 = bits[7:0] ✓）
 *   · COMMAND：byte0=0x10；tag@16-17（be16）；LUN@**24-31**（非规范 8-15 ✓）；
 *     CDB@**36-51**（9 头 + 4 CDB + 1 CRC = 14 dword ✓）
 *   · DATA：6 dword 头：byte0=0x60、DW4={0,tag be16}、DW5=DataOffset(be32) ✓
 *   · RESPONSE：12 dword（含 CRC 槽）：byte0=0x40、DW4 tag(be16)、
 *     **byte35=STATUS**、byte34[1:0]=DATA PRESENT、DW10=SENSE LEN(be32，现状恒 0 ✓)
 *   · 帧型半字节（侧带 `rx_type`/`tx_type`）：1=COMMAND 6=DATA 4=RESPONSE 5=XFER_RDY ✓
 *============================================================================*/
#ifndef SAS_HDD_REGS_H
#define SAS_HDD_REGS_H

#include <stdint.h>

/*==========================================================================*/
/* §1 帧级常量（与 HBA 侧 `sas_hba_regs.h` §10 逐值一致 ✓）                   */
/*==========================================================================*/
#define SAS_FT_COMMAND   1u
#define SAS_FT_DATA      6u
#define SAS_FT_RESPONSE  4u
#define SAS_FT_XFER_RDY  5u
#define SAS_FT_TASK      2u
#define SAS_FT_OPEN      3u

#define SAS_FRAME_B0_COMMAND  0x10u   /* byte0（FT<<4 ✓）*/
#define SAS_FRAME_B0_TASK     0x20u
#define SAS_FRAME_B0_DATA     0x60u
#define SAS_FRAME_B0_RESPONSE 0x40u
#define SAS_FRAME_B0_XFER_RDY 0x50u
#define SAS_FRAME_B0_OPEN     0x30u   /* 项目专有 ✓（非 SAS 规范值）*/

#define SAS_CMD_FRAME_DW    14        /* COMMAND：9 头 + 4 CDB + 1 CRC ✓ */
#define SAS_CMD_DW_TAG      4         /* DW4：tag/tptt 各 be16 ✓ */
#define SAS_CMD_DW_LUN_LO   6         /* DW6-7 = LUN（帧字节 24-31 ✓）*/
#define SAS_CMD_DW_CDB_LO   9         /* DW9-12 = CDB（帧字节 36-51 ✓）*/
#define SAS_CMD_TAG_BYTE    16
#define SAS_CMD_CDB_BYTE    36
#define SAS_CMD_LUN_BYTE    24

#define SAS_DATA_HDR_DW     6         /* DATA 帧头 ✓ */
#define SAS_DATA_DW_TAG     4
#define SAS_DATA_DW_OFFSET  5
#define SAS_DATA_OFF_BYTE   20

#define SAS_RESP_FRAME_DW    12       /* RESPONSE 12 dword（DW11=CRC 槽 ✓）*/
#define SAS_RESP_DW_TAG      4
#define SAS_RESP_DW_STATUS   8        /* byte35 = STATUS（bits[31:24] ✓）*/
#define SAS_RESP_DW_DATA_LEN 10       /* 帧字节 40-43（be32，现状恒 0 ✓）*/
#define SAS_RESP_STATUS_BYTE 35
#define SAS_RESP_DATAPRES_BYTE 34

#define SAS_SCSI_GOOD   0x00u
#define SAS_SCSI_CHKCON 0x02u

/*==========================================================================*/
/* §2 HDD 自有 CSR（出处 `RTL 源`；⚠ 与 hisi_sas 寄存器模型
 *    **完全不同** —— 无 GLOBAL/DQ/CQ/PHY 分段、无 IPTT/CQE 概念 ✓
 *    （本模型未实现该 CSR 面；本表供宿主/扩展序列表驱动 ✓））               */
/*==========================================================================*/
#define SAS_HDD_CAP_LO        0x000   /* 能力低 32（只读 ✓）*/
#define SAS_HDD_CAP_HI        0x004
#define SAS_HDD_VS_CAP        0x008   /* 厂商能力 ✓ */
#define SAS_HDD_CC            0x00C   /* 控制/配置 ✓ */
#define SAS_HDD_STATUS        0x010
#define SAS_HDD_CTRL_RST      0x014   /* **WO，magic = 0x53415348「SASH」** ✓ */
#define SAS_HDD_CRTO          0x018   /* 命令超时 ✓ */
#define SAS_HDD_CMD_FIFO      0x020   /* 0x020-0x03C 命令 FIFO（入队/出队/状态 ✓）*/
#define SAS_HDD_SAS_CFG       0x040   /* 0x040-0x064 本地 SAS 地址/角色/速率 ✓ */
#define SAS_HDD_TIMERS        0x070   /* 0x070-0x07C 定时器组 ✓ */
#define SAS_HDD_INTC          0x080   /* 0x080-0x0AC 中断控制 ✓ */
#define SAS_HDD_INTR_STATUS   0x088   /* ★ **W1C** ✓（`RTL 源`）*/
#define SAS_HDD_DMA_CH        0x0B0   /* 0x0B0-0x1AF DMA 通道 0-7 ✓ */
#define SAS_HDD_STATS         0x100   /* 0x100-0x1EF 统计计数器 ✓ */
#define SAS_HDD_GPIO          0x1F0   /* 0x1F0-0x1FC ✓ */
#define SAS_HDD_LUN_TBL       0x200   /* 0x200-0x3FF（512B ✓）*/
#define SAS_HDD_DEV_TBL       0x400   /* 0x400-0xBFF（ITCT 类，2KB ✓）*/
#define SAS_HDD_ERR_LOG       0xC00   /* 0xC00-0xFFF ✓ */
#define SAS_HDD_CTRL_RST_MAGIC 0x53415348u   /* "SASH" ✓ */

/* 复位默认值（`RTL 源` ✓）*/
#define SAS_HDD_PHY_CFG_DEF   0x03E80302u
#define SAS_HDD_ROLE_CFG_DEF  0x0000011Fu

#endif /* SAS_HDD_REGS_H */
