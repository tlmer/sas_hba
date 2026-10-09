/*============================================================================
 * sas_hba_regs.h — SAS HBA（hisi_sas v3 兼容寄存器面）寄存器/位域/帧常量  [2026-10-08 建]
 *
 * ★★ 本文件 = **模型 / TB / sw 序列层 的共同基准** —— 每条带出处：
 *     · `RTL 源:行`  · `RTL 源:行`
 *     · 内核 `内核驱动（公开）:行` / `内核驱动（公开）:行`
 *     · 本参考实现 md 参考平台实证（波形/采样记录，见各条 Ⓜ 注）
 *    ⚠ **改这里必须同步 `md/设计方案.md`，反之亦然** ✓
 *
 * ★ 地址口径（照 `RTL 源` 段译码，RTL 实证）：
 *    · 段：GLOBAL `off<0x260` ≤ DQ `<0x4E0` ≤ CQ `<0x6D0`⚠ ≤ PHY `0x2000..0x4400`
 *          AXI_MASTER `0x5000..0x5020`、RAS `0x6000..0x6030`（off = addr − 基址）
 *    · DQ/CQ **步长 20(0x14)**，子偏移 `(off−base)%0x14`（0x260%0x14=8 ⇒ 逻辑
 *      DQ0 的 reg0 恰落子偏移 0 ✓）；PHY **步长 0x400**
 *    · 基址由顶层/TB 分配（本模型**不绑死**；参考平台 = guest 映射窗 ✓）
 *
 * ★ 帧/IU 字节口径：全线统一 **帧字节 k = dword 内 bits[8k+7:8k]**（byte0=bits[7:0]）
 *    —— COMMAND 帧 CDB@帧字节 36、tag@16-17（be16）等，见 §7 ✓
 *
 * ⚠ **建模红线**（RTL 与内核/自身注释不一致处 —— 模型按【RTL 实测】建，差异留档）：
 *    ① CQ 段上界 0x6D0 ⇒ RTL 内 CQ[25..31] 不可达（内部却支持 32 个 CQ）
 *       `RTL 源`；模型按 32 段实现（注释差异 ✓）
 *    ② **无 EOI**：`intr_ack` 顶层恒 0（`RTL 源`）⇒ 中断状态只读写
 *       无效（唯一例外 PHY CHL_INT0 bit2 sticky，`regfile）；模型默认
 *       实现**内核期望的 W1C**，可开 `cfg_rtl_no_eoi` 复现 RTL ✗
 *    ③ ENT_INT_SRC1/3 位号与内核有系统偏差（`intc vs `内核驱动（公开）`）
 *    ④ CHNL_INT_STATUS 只汇总 PHY0-7（`intc 循环 `pi<8`），PHY8 恒 0
 *    ⑤ **SGE 译码错位**：RTL `sge_engine 按 len@8-11/off@12-15 取，内核
 *       struct 是 page_ctrl@8-15；模型/序列层**按内核布局**（跑真驱动的硬前提 ✓）
 *    ⑥ DATA 帧型位 **[3:0] vs [7:4] 双约定**（HBA `data_wr_fsm vs 其余全部
 *       [7:4]）；模型 TX 一律 [7:4]（0x60），差异记入 `md/对拍报告.md`
 *    ⑦ OPEN = 0x30 为项目专有编码（非 SAS 规范）
 *    ⑧ CLOSE 三处不一致（`close_fsm:21-22` 注释 0x10 / `link_top 0x010101BC /
 *       HDD codec 0xA1）；模型只按 `link_top` dword 常量建
 *============================================================================*/
#ifndef SAS_HBA_REGS_H
#define SAS_HBA_REGS_H

#include <stdint.h>

/*==========================================================================*/
/* §1 段译码（off = addr − 基址；出处 `RTL 源`）    */
/*==========================================================================*/
#define SAS_HBA_SEG_GLOBAL_END   0x00000260u   /* GLOBAL: off < 0x260 ✓ */
#define SAS_HBA_SEG_DQ_BASE      0x00000260u   /* DQ:  0x260 ≤ off < 0x4E0 ✓ */
#define SAS_HBA_SEG_CQ_BASE      0x000004E0u   /* CQ:  0x4E0 ≤ off < 0x6D0⚠（红线①）*/
#define SAS_HBA_SEG_CQ_RTL_END   0x000006D0u   /* RTL 译码上界（0x4E0+32*0x14=0x760 ✗）*/
#define SAS_HBA_SEG_PHY_BASE     0x00002000u   /* PHY: 0x2000 ≤ off < 0x4400（9 个 ✓）*/
#define SAS_HBA_SEG_AXI_BASE     0x00005000u   /* AXI_MASTER: 0x5000..0x5020 ✓ */
#define SAS_HBA_SEG_RAS_BASE     0x00006000u   /* RAS: 0x6000..0x6030 ✓ */

#define SAS_HBA_DQ_STRIDE        0x14          /* `RTL 源` ✓ */
#define SAS_HBA_CQ_STRIDE        0x14          /* `RTL 源` ✓ */
#define SAS_HBA_PHY_STRIDE       0x400         /* `RTL 源` ✓ */
#define SAS_HBA_MAX_DQ           32            /* DQ/CQ 逻辑个数（段空间 ≥ 16 个 ✓）*/
#define SAS_HBA_MAX_PHY          9             /* PHY0..8 ✓（CHNL 汇总只到 PHY7 ⚠红线④）*/

/* 子偏移 = (off − base) % STRIDE（勿用 & (STRIDE-1)：0x14 非 2 的幂 ✓）*/
#define SAS_HBA_DQ_OFF(off)  (((off) - SAS_HBA_SEG_DQ_BASE) % SAS_HBA_DQ_STRIDE)
#define SAS_HBA_DQ_IDX(off)  (((off) - SAS_HBA_SEG_DQ_BASE) / SAS_HBA_DQ_STRIDE)
#define SAS_HBA_CQ_OFF(off)  (((off) - SAS_HBA_SEG_CQ_BASE) % SAS_HBA_CQ_STRIDE)
#define SAS_HBA_CQ_IDX(off)  (((off) - SAS_HBA_SEG_CQ_BASE) / SAS_HBA_CQ_STRIDE)
#define SAS_HBA_PHY_OFF(off) (((off) - SAS_HBA_SEG_PHY_BASE) % SAS_HBA_PHY_STRIDE)
#define SAS_HBA_PHY_IDX(off) (((off) - SAS_HBA_SEG_PHY_BASE) / SAS_HBA_PHY_STRIDE)
/* 绝对地址（给定基址 SAS_HBA_BASE ✓）*/
#define SAS_HBA_DQ_REG(i, r)  (SAS_HBA_SEG_DQ_BASE + ((i) * SAS_HBA_DQ_STRIDE) + (r))
#define SAS_HBA_CQ_REG(i, r)  (SAS_HBA_SEG_CQ_BASE + ((i) * SAS_HBA_CQ_STRIDE) + (r))
#define SAS_HBA_PHY_REG(i, r) (SAS_HBA_SEG_PHY_BASE + ((i) * SAS_HBA_PHY_STRIDE) + (r))

/*==========================================================================*/
/* §2 GLOBAL 段（出处 `RTL 源`）                         */
/*==========================================================================*/
#define SAS_HBA_CFG_DLVRY_QUEUE_ENABLE 0x000  /* bit_i = DQ_i 使能；并驱 port role ✓ */
#define SAS_HBA_CFG_IOST_BASE_LO       0x008  /* IOST 表基址（64B/条目 ✓）*/
#define SAS_HBA_CFG_IOST_BASE_HI       0x00C
#define SAS_HBA_CFG_ITCT_BASE_LO       0x010  /* ITCT 表基址 ✓ */
#define SAS_HBA_CFG_ITCT_BASE_HI       0x014
#define SAS_HBA_CFG_BROKEN_MSG_LO      0x018
#define SAS_HBA_CFG_BROKEN_MSG_HI      0x01C
#define SAS_HBA_CFG_PHY_CONTEXT        0x020
#define SAS_HBA_CFG_PHY_STATE          0x024
#define SAS_HBA_CFG_PHY_PORT_NUM_MA    0x028
#define SAS_HBA_CFG_PHY_CONN_RATE      0x030
#define SAS_HBA_CFG_ITCT_CLR           0x044
#define SAS_HBA_CFG_SATA_INIT_D2H_LO   0x060
#define SAS_HBA_CFG_SATA_INIT_D2H_HI   0x064
#define SAS_HBA_CFG_MAX_TAG            0x068  /* 最大 IPTT 数（4096 ✓）*/
#define SAS_HBA_CFG_ITY_NEXUS_LOSS_T   0x0A0  /* I_T_NEXUS_LOSS_TIME ✓ */
#define SAS_HBA_CFG_MAX_CON_TIME_LIMIT 0x0A4
#define SAS_HBA_CFG_BUS_INACTIVE_LIMIT 0x0A8
#define SAS_HBA_CFG_REJECT_TO_OPEN_T   0x0AC
#define SAS_HBA_CFG_CQ_INT_CONVERGE_EN 0x0B0
#define SAS_HBA_CFG_ABT_SET_QUERY_IPTT 0x0D4
#define SAS_HBA_CFG_ABT_SET_IPTT_DONE  0x0D8
#define SAS_HBA_INT_CHNL_STATUS        0x148  /* [4i+0..3]=PHY_i: SL_PHY_ENABLE/HOTPLUG_TOUT/
                                                 NOT_RDY/PHY_RDY（只到 PHY7 ⚠红线④）*/
#define SAS_HBA_INT_COAL_EN            0x19C
#define SAS_HBA_INT_OQ_COAL_TIME       0x1A0
#define SAS_HBA_INT_OQ_COAL_CNT        0x1A4
#define SAS_HBA_INT_ENT_COAL_TIME      0x1A8
#define SAS_HBA_INT_ENT_COAL_CNT       0x1AC
#define SAS_HBA_INT_OQ_SRC             0x1B0  /* 只读；CQ 完成位图 ✓（无清路 ⚠红线②）*/
#define SAS_HBA_INT_OQ_SRC_MSK         0x1B4  /* ⚠ 无写通路（W1C 未实现，红线②）*/
#define SAS_HBA_INT_ENT_SRC1           0x1B8  /* 只读；⚠位号与内核有偏差（红线③）*/
#define SAS_HBA_INT_ENT_SRC2           0x1BC
#define SAS_HBA_INT_ENT_SRC3           0x1C0
#define SAS_HBA_INT_ENT_SRC_MSK1       0x1C4  /* 驱动初始化写 0xfefefefe ✓ */
#define SAS_HBA_INT_ENT_SRC_MSK2       0x1C8
#define SAS_HBA_INT_ENT_SRC_MSK3       0x1CC
#define SAS_HBA_INT_PHYUPDOWN_MSK      0x1D0
#define SAS_HBA_INT_CHNL_ENT_MSK       0x1D4
#define SAS_HBA_INT_HGC_COM_MSK        0x1D8
#define SAS_HBA_INT_SAS_ECC_INTR       0x1E8
#define SAS_HBA_INT_SAS_ECC_INTR_MSK   0x1EC
#define SAS_HBA_CFG_HGC_ERR_STAT_EN    0x238
#define SAS_HBA_STS_CQE_SEND_CNT       0x248  /* 只读；= cq_engine.cqe_send_cnt ✓ */

/* 中断向量编码（`RTL 源`）*/
#define SAS_HBA_ITR_VEC_CHNL 0u   /* 00 = immediate/chnl（PHY up/down ⇒ MSI vector1 ✓）*/
#define SAS_HBA_ITR_VEC_ENT  1u   /* 01 = ENT（⇒ vector2 ✓）*/
#define SAS_HBA_ITR_VEC_OQ   2u   /* 10 = OQ/coal（CQ 完成 ⇒ vector2 ✓）*/

/*==========================================================================*/
/* §3 DQ / CQ 段内寄存器（子偏移；`RTL 源`）              */
/*==========================================================================*/
#define SAS_HBA_XX_BASE_ADDR_LO 0x00   /* 环基址低 32 ✓ */
#define SAS_HBA_XX_BASE_ADDR_HI 0x04
#define SAS_HBA_XX_DEPTH        0x08   /* 条目数 ✓ */
#define SAS_HBA_XX_WR_PTR       0x0C   /* 生产者指针（软件写 ✓）*/
#define SAS_HBA_XX_RD_PTR       0x10   /* 消费者指针（硬件推进 ✓；RTL: dq_engine）*/

/*==========================================================================*/
/* §4 PHY 段（子偏移；`RTL 源`）                          */
/*==========================================================================*/
#define SAS_HBA_PHY_CFG             0x000   /* 复位 0x03E8_0302（角色/速率 ✓）*/
#define SAS_HBA_PHY_HARD_LINKRATE   0x004
#define SAS_HBA_PHY_PROG_LINK_RATE  0x008
#define SAS_HBA_PHY_CTRL            0x014
#define SAS_HBA_PHY_SERDES_CFG      0x01C
#define SAS_HBA_PHY_SL_CFG          0x084
#define SAS_HBA_PHY_SL_CONTROL      0x094
#define SAS_HBA_PHY_TX_ID_DW0       0x09C   /* 7 DWORD（DW4/DW3 镜像 sas_addr_out ✓）*/
#define SAS_HBA_PHY_TX_ID_DW1       0x0A0
#define SAS_HBA_PHY_TX_ID_DW2       0x0A4
#define SAS_HBA_PHY_TX_ID_DW3       0x0A8
#define SAS_HBA_PHY_TX_ID_DW4       0x0AC
#define SAS_HBA_PHY_TX_ID_DW5       0x0B0
#define SAS_HBA_PHY_TX_ID_DW6       0x0B4
#define SAS_HBA_PHY_TXID_AUTO       0x0B8
#define SAS_HBA_PHY_RX_IDAF_DW0     0x0C4   /* RX 地址帧（对端 SAS 地址 ✓）*/
#define SAS_HBA_PHY_CON_CFG_DRIVER  0x130
#define SAS_HBA_PHY_SSP_CON_TIMER   0x134
#define SAS_HBA_PHY_SMP_CON_TIMER   0x138
#define SAS_HBA_PHY_STP_CON_TIMER   0x13C
#define SAS_HBA_PHY_CHL_INT0        0x1B4   /* 位定义见下 ✓ */
#define SAS_HBA_PHY_CHL_INT1        0x1B8
#define SAS_HBA_PHY_CHL_INT2        0x1BC
#define SAS_HBA_PHY_CHL_INT_MSK0    0x1C0
#define SAS_HBA_PHY_CHL_INT_MSK1    0x1C4
#define SAS_HBA_PHY_CHL_INT_MSK2    0x1C8
#define SAS_HBA_PHY_ERR_CNT_DWS_LOST 0x380
#define SAS_HBA_PHY_ERR_CNT_CODE    0x394
#define SAS_HBA_PHY_ERR_CNT_DISP    0x398

/* CHL_INT0 位（`内核驱动（公开）`）*/
#define SAS_HBA_CHL0_HOTPLUG_TOUT  (1u << 0)
#define SAS_HBA_CHL0_SL_RX_BCST    (1u << 1)
#define SAS_HBA_CHL0_SL_PHY_ENABLE (1u << 2)   /* ★ sticky；RTL 内唯一 W1C 通路
                                                  `regfile（写 1 清 ✓）*/
#define SAS_HBA_CHL0_NOT_RDY       (1u << 4)
#define SAS_HBA_CHL0_PHY_RDY       (1u << 5)

/*==========================================================================*/
/* §5 AXI_MASTER / RAS 段（绝对子偏移；`RTL 源`）         */
/*==========================================================================*/
#define SAS_HBA_AM_CTRL_GLOBAL   0x5000   /* 全局控制 ✓ */
#define SAS_HBA_AM_CFG_MAX_TRANS 0x5010   /* 最大突发拍数 ✓ */
#define SAS_HBA_RAS_INTR0        0x6000
#define SAS_HBA_RAS_INTR1        0x6004
#define SAS_HBA_RAS_INTR0_MASK   0x6008
#define SAS_HBA_RAS_INTR1_MASK   0x600C

/*==========================================================================*/
/* §6 DQ 64B 命令条目位域（dw0..dw15；`内核驱动（公开）` + RTL 实测切片）    */
/*==========================================================================*/
/* dw0（`RTL 源`）*/
#define SAS_HBA_DQ0_ABORT_FLAG    0          /* [0] ✓ */
#define SAS_HBA_DQ0_ABORT_DEV_TY  2          /* [2] ✓ */
#define SAS_HBA_DQ0_RESP_REPORT   5          /* [5] ✓ */
#define SAS_HBA_DQ0_TLR_CTRL_LO   6          /* [7:6] ✓ */
#define SAS_HBA_DQ0_PHY_ID_LO     8          /* [16:8]（9bit ✓）*/
#define SAS_HBA_DQ0_FORCE_PHY     17         /* [17] ✓ */
#define SAS_HBA_DQ0_PORT_LO       18         /* [21:18] ✓ */
#define SAS_HBA_DQ0_PRIORITY      27         /* [27] ✓ */
#define SAS_HBA_DQ0_CMD_LO        29         /* [31:29] ✓ */
/* dw0[31:29] cmd 编码（`RTL 源`）*/
#define SAS_HBA_CMD_SSP      1u
#define SAS_HBA_CMD_SMP      2u
#define SAS_HBA_CMD_STP_EXP  3u
#define SAS_HBA_CMD_STP_DIR  4u
#define SAS_HBA_CMD_ABORT    5u

/* dw1（`RTL 源`；RTL 实际切片 `cmd_parser 同 bit ✓）*/
#define SAS_HBA_DQ1_UNCON_CMD     3          /* [3] ✓ */
#define SAS_HBA_DQ1_DIR_LO        5          /* [6:5] 0=NO_DATA 1=TO_INI 2=TO_DEV ✓ */
#define SAS_HBA_DQ1_RESET         7          /* [7] ✓ */
#define SAS_HBA_DQ1_PIR           8          /* [8] ✓ */
#define SAS_HBA_DQ1_VDTL          10         /* [10] ✓ */
#define SAS_HBA_DQ1_FRAME_TY_LO   11         /* [15:11]（5bit：0x10/0x20/… 半字节 <<4 侧 ✓）*/
#define SAS_HBA_DQ1_DEV_ID_LO     16         /* [31:16] ✓ */
#define SAS_HBA_DIR_NO_DATA   0u
#define SAS_HBA_DIR_TO_INI    1u
#define SAS_HBA_DIR_TO_DEVICE 2u

/* dw2（`RTL 源`）*/
#define SAS_HBA_DQ2_CFL_LO        0          /* [8:0] 命令帧长（dword ✓）*/
#define SAS_HBA_DQ2_NCQ_TAG_LO    10         /* [14:10] ✓ */
#define SAS_HBA_DQ2_MRFL_LO       15         /* [23:15] ✓ */
#define SAS_HBA_DQ2_SG_MOD_LO     24         /* [25:24] ✓ */

/* dw3 */
#define SAS_HBA_DQ3_IPTT_LO       0          /* [15:0] = IPTT ★RTL 实测 `dq_engine
                                                （内核消费同：`v3_hw.c）✓ */
/* dw4 */
#define SAS_HBA_DQ4_DATA_XFER_LEN 0          /* [31:0] 传输字节数 ✓ */
/* dw5 */
#define SAS_HBA_DQ5_FIRST_BURST_LO 0         /* [15:0] ✓ */
/* dw6（`RTL 源`）*/
#define SAS_HBA_DQ6_DIF_SGL_LEN_LO  0        /* [15:0] ✓ */
#define SAS_HBA_DQ6_DATA_SGL_LEN_LO 16       /* [31:16] ✓ */
/* dw7（`RTL 源`）*/
#define SAS_HBA_DQ7_ADDR_MODE_SEL 15         /* [15] ✓ */
#define SAS_HBA_DQ7_ABORT_IPTT_LO 16         /* [31:16] ✓ */
/* dw8-9 / dw10-11 / dw12-13 / dw14-15（64bit 地址两 dword：LO=dwN，HI=dwN+1 ✓）*/
#define SAS_HBA_DQ_CMD_TABLE_LO_DW  8        /* 命令表地址 ★RTL `cmd_parser
                                                （[319]，高 32 位隐含 0）✓ */
#define SAS_HBA_DQ_STS_BUF_LO_DW    10       /* 状态缓冲地址 ✓ */
#define SAS_HBA_DQ_PRD_TABLE_LO_DW  12       /* SGE 表地址 ★RTL `[447]` ✓ */
#define SAS_HBA_DQ_DIF_PRD_LO_DW    14       /* DIF 表地址（RTL 未用 ✓）*/
#define SAS_HBA_DQ_ENTRY_DW         16       /* 条目 = 64B = 16 dword ✓ */
#define SAS_HBA_DQ_ENTRY_BYTES      64
#define SAS_HBA_DQ_FETCH_LEN        3        /* AXI 读 4 拍（len=3 ✓）`dq_engine */

/*==========================================================================*/
/* §7 CQE（16B = 4 dword；`RTL 源`）               */
/*==========================================================================*/
#define SAS_HBA_CQE_BYTES        16
#define SAS_HBA_CQE0_CMPLT_LO    0    /* [1:0]：0=正常、3=出错 ✓ */
#define SAS_HBA_CQE0_ERRPHASE_LO 2    /* [9:2] ✓ */
#define SAS_HBA_CQE0_RSPNS_XFRD  10   /* [10] ✓ */
#define SAS_HBA_CQE0_RSPNS_GOOD  11   /* [11] ✓ */
#define SAS_HBA_CQE0_ERX         12   /* [12] ✓ */
#define SAS_HBA_CQE0_ABORT_LO    13   /* [15:13] ✓ */
#define SAS_HBA_CQE1_IPTT_LO     0    /* dw1[15:0]（字节 4-5 小端）★内核 `v3_hw.c
                                         `iptt = dw1 & 0xffff` ✓ */
#define SAS_HBA_CQE1_DEV_ID_LO   16   /* dw1[31:16]（字节 6-7 ✓）*/
#define SAS_HBA_CQE_CMPLT_OK     0u
#define SAS_HBA_CQE_CMPLT_ERR    3u

/*==========================================================================*/
/* §8 SGE（24B；⚠ 红线⑤：模型/序列层按【内核布局】建 `内核驱动（公开）`）    */
/*==========================================================================*/
#define SAS_HBA_SGE_BYTES      24
#define SAS_HBA_SGE_ADDR_LO    0    /* 字节 0-7：DMA 地址（le64 ✓）*/
#define SAS_HBA_SGE_PGCTRL0    8    /* 字节 8-11 ✓（RTL 错位取成 len ✗ 红线⑤）*/
#define SAS_HBA_SGE_PGCTRL1    12   /* 字节 12-15 ✓ */
#define SAS_HBA_SGE_DATA_LEN   16   /* 字节 16-19 ✓ */
#define SAS_HBA_SGE_DATA_OFF   20   /* 字节 20-23 ✓ */
#define SAS_HBA_SGE_MAX_CNT    124  /* `HISI_SAS_SGE_PAGE_CNT` ✓ */

/*==========================================================================*/
/* §9 命令表（SSP；`cmd_parser）                                     */
/*==========================================================================*/
#define SAS_HBA_CTBL_HDR_OFF    0    /* [0..23]  ssp_frame_hdr ✓ */
#define SAS_HBA_CTBL_LUN_OFF    24   /* [24..31] LUN（8B ✓）*/
#define SAS_HBA_CTBL_TMF_OFF    34   /* [34]     TMF byte ✓ */
#define SAS_HBA_CTBL_CDB_OFF    36   /* [36..51] CDB[16] ★RTL `[(36*8) +: 128]` ✓ */
#define SAS_HBA_CTBL_SSP_BYTES  1024 /* CMD_TBL_SSP_SZ ✓ */
#define SAS_HBA_STS_BUF_BYTES   1040 /* 16B err_record + 1024B IU ✓ */

/*==========================================================================*/
/* §10 SSP 帧常量（帧字节 k = dword 内 bits[8k+7:8k] ✓）                       */
/*==========================================================================*/
/* 帧型半字节（★RTL 侧带 `rx_type`：1=COMMAND 6=DATA 4=RESPONSE 5=XFER_RDY ✓；
   ⚠ 线上 byte0 三套约定并存见红线⑥ —— 模型 TX 一律 [7:4] ✓）*/
#define SAS_FT_COMMAND   1u
#define SAS_FT_DATA      6u
#define SAS_FT_RESPONSE  4u
#define SAS_FT_XFER_RDY  5u
#define SAS_FT_TASK      2u
#define SAS_FT_OPEN      3u
/* byte0 线上值（[7:4] 约定：FT<<4 ✓；OPEN=0x30 专有 ⑦）*/
#define SAS_FRAME_B0_COMMAND  0x10u
#define SAS_FRAME_B0_TASK     0x20u
#define SAS_FRAME_B0_DATA     0x60u
#define SAS_FRAME_B0_RESPONSE 0x40u
#define SAS_FRAME_B0_XFER_RDY 0x50u
#define SAS_FRAME_B0_OPEN     0x30u

/* COMMAND 帧 = 14 dword：DW0..DW8 头(9) + CDB DW9..DW12(4) + CRC(1) ✓
   `RTL 源` */
#define SAS_CMD_FRAME_DW    14
#define SAS_CMD_DW_TAG      4    /* DW4：[23:16]=TPTT、[15:0]=tag（各自 be16 交换 ✓）*/
#define SAS_CMD_DW_LUN_LO   6    /* DW6-7：LUN 8 字节 ✓ */
#define SAS_CMD_DW_ATTR     8    /* DW8：byte33={first_burst,prio,attr}；TMF 时 byte34 ✓ */
#define SAS_CMD_DW_CDB_LO   9    /* DW9..DW12 = 帧字节 36-51 = CDB[16] ✓ */
#define SAS_CMD_TAG_BYTE    16   /* 帧字节 16-17 = initiator tag（**be16**：byte16=tag[15:8] ✓）*/
#define SAS_CMD_CDB_BYTE    36   /* = `RTL 源` 「IU 12-27」✓ */

/* DATA 帧 = 6 dword 头 + payload ✓（`RTL 源`）
   DW0 byte0=0x60；DW4 = {16'd0, init_tag be16}；DW5 = DataOffset(be32) ✓ */
#define SAS_DATA_HDR_DW     6
#define SAS_DATA_DW_TAG     4
#define SAS_DATA_DW_OFFSET  5
#define SAS_DATA_OFF_BYTE   20   /* 帧字节 20-23 = DW5 ✓ */

/* XFER_RDY = 4 dword（DW0 byte0=0x50；DW1={tag lo…}；DW2=burst_remaining；DW4=offset ✓）
   `RTL 源` —— ⚠ 本工程读路径未激活（HDD 直发 DATA ✓）*/
#define SAS_XRDY_DW         4
#define SAS_XRDY_DW_BURST   2
#define SAS_XRDY_DW_OFFSET  4

/* RESPONSE = 12 dword（含 CRC 头末 DW=11 ✓ `RTL 源`）
   DW0=0x40；DW4 tag（be16 ✓）；DW8：byte34[1:0]=DATA PRESENT、**byte35=STATUS**；
   DW10 = SENSE LEN（be32，帧字节 40-43 ✓）*/
#define SAS_RESP_FRAME_DW    12
#define SAS_RESP_DW_TAG      4
#define SAS_RESP_DW_STATUS   8    /* byte35 = bits[31:24] ✓ */
#define SAS_RESP_DW_DATA_LEN 10   /* 帧字节 40-43 ✓（HDD 现状恒 0 ✓）*/
#define SAS_RESP_STATUS_BYTE 35
#define SAS_RESP_DATAPRES_BYTE 34

/* OPEN 帧 = 6 dword（byte0=0x30 ✓ 专有⑦；HDD 解析：`RTL 源,109-120`）*/
#define SAS_OPEN_FRAME_DW    6

/* SCSI status ✓ */
#define SAS_SCSI_GOOD   0x00u
#define SAS_SCSI_CHKCON 0x02u

/* 链路原语 dword（照 `RTL 源`；⚠ 红线⑧ CLOSE 只按 link_top ✓）*/
#define SAS_PRIM_SOF_ID  0xBC071515u
#define SAS_PRIM_SOF_SSP 0xBC071515u   /* SOF_SSP 同 SOF_ID 值域（link_top ✓）*/
#define SAS_PRIM_EOF     0xBC151507u
#define SAS_PRIM_ACK     0xBC150715u
#define SAS_PRIM_NAK     0xBC070715u
#define SAS_PRIM_CLOSE   0xBC010101u   /* link_top（非 SAS-5 规范值 ✗ 红线⑧）*/
/* 原语 ID（RX 译码 `link_top；HDD codec `RTL 源` ✓）*/
#define SAS_PRIMID_IDLE   0x10u
#define SAS_PRIMID_ACK    0x40u
#define SAS_PRIMID_NAK    0x41u
#define SAS_PRIMID_RRDY   0x50u
#define SAS_PRIMID_OPEN_ACC 0x81u
#define SAS_PRIMID_OPEN_REJ 0x82u
#define SAS_PRIMID_EOF    0x30u
#define SAS_PRIMID_SOF_SSP 0x20u
#define SAS_PRIMID_CLOSE  0xA1u        /* ⚠ HDD codec 值；与 link_top dword 冲突（红线⑧）*/

/* ACK/NAK 监督（`RTL 源,76`；HDD 侧同 300 ✓）*/
#define SAS_AN_TIMEOUT_CYCLES 300u     /* 1 µs @300 MHz（每 2 拍重装旧版已修 ✓）*/
#define SAS_AN_MAX_RETRIES    3u       /* SAS-5 7.4.3 ✓ */

#endif /* SAS_HBA_REGS_H */
