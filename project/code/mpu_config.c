#include "fsl_device_registers.h"

/* ================================================================
 * SystemInitHook - 覆盖 SDK 弱定义，启动时自动配置 MPU 并启用 D-Cache
 * 
 * 背景说明：
 * ARMv7-M 默认背景区域映射（0x60000000-0x9FFFFFFF）为 Device/nGnRnE
 * 类型，导致 SDRAM 访问不可缓存。必须通过 MPU 显式将一个 region 配置
 * 为 Normal Memory (Write-back/Write-allocate)，D-Cache 才能对 SDRAM
 * 区域生效。本函数在 main() 之前由启动代码调用，对用户透明。
 * ================================================================ */

void SystemInitHook(void)
{
    /* ---- 第 1 步：禁能 MPU，配置期间必须关闭 ---- */
    MPU->CTRL = 0;

    /* ---- 第 2 步：逐个配置 6 个 MPU Region ---- */
    
    /* Region 0: SDRAM Cacheable (0x80000000, 32MB)
     * 属性: Normal Memory, Write-back, Write-allocate, Non-shareable
     * 目标: hash_table、pq 等 SDRAM 热数据享受 D-Cache 加速
     * TEX=0, C=1, B=1, S=0 → Outer & Inner WB/WA, 无共享 */
    MPU->RNR  = 0;
    MPU->RBAR = 0x80000000U;
    MPU->RASR = (0U  << 28)   /* XN: 不可执行（数据区） */
              | (3U  << 24)   /* AP[2:0]: 111 = 全权限读写 */
              | (0U  << 19)   /* TEX[2:0]: 000 → 配合 C/B 选择 Normal */
              | (1U  << 17)   /* C: 1 = Cacheable */
              | (1U  << 16)   /* B: 1 = Bufferable (Write-back) */
              | (0U  << 15)   /* S: 0 = Non-shareable */
              | (24U << 1)    /* SIZE: log2(32MB)-1 = 24 */
              | (1U  << 0);   /* ENABLE: 1 */

    /* Region 1: SDRAM Non-Cacheable (0x81E00000, 2MB)
     * 属性: Device/nGnRnE
     * 目标: DMA 缓冲区（帧缓冲、USB、SPI 等外设共享内存）
     * 原因: 避免 D-Cache 与 DMA 之间的数据一致性问题 */
    MPU->RNR  = 1;
    MPU->RBAR = 0x81E00000U;
    MPU->RASR = (0U  << 28)   /* XN: 不可执行 */
              | (3U  << 24)   /* AP: 全权限 */
              | (0U  << 19)   /* TEX: 0 */
              | (0U  << 17)   /* C: 0 = Non-cacheable */
              | (0U  << 16)   /* B: 0 = Non-bufferable */
              | (0U  << 15)   /* S: 0 */
              | (20U << 1)    /* SIZE: log2(2MB)-1 = 20 */
              | (1U  << 0);   /* ENABLE */

    /* Region 2: ITCM (0x00000000, 64KB)
     * 属性: Normal Memory (TCM 本身与 CPU 同速，无需 Cache)
     * 目标: path_planning.c 热函数（已加 ITCM_NonCacheable section）
     * 说明: TCM 是紧耦合内存，访问延迟 = 1 cycle，Cache 无意义且浪费 */
    MPU->RNR  = 2;
    MPU->RBAR = 0x00000000U;
    MPU->RASR = (1U  << 28)   /* XN: 0 = 可执行（指令区） */
              | (3U  << 24)   /* AP: 全权限 */
              | (0U  << 19)   /* TEX: 0 */
              | (0U  << 17)   /* C: 0 */
              | (0U  << 16)   /* B: 0 */
              | (0U  << 15)   /* S: 0 */
              | (15U << 1)    /* SIZE: log2(64KB)-1 = 15 */
              | (1U  << 0);   /* ENABLE */

    /* Region 3: DTCM (0x20000000, 512KB)
     * 属性: Normal Memory (TCM 无需 Cache)
     * 目标: g_dist_map, g_bfs_visited, g_obs_buf 等关键热数据
     * 说明: DTCM 是数据紧耦合内存，0 等待访问，D-Cache 旁路 */
    MPU->RNR  = 3;
    MPU->RBAR = 0x20000000U;
    MPU->RASR = (0U  << 28)   /* XN: 不可执行 */
              | (3U  << 24)   /* AP: 全权限 */
              | (0U  << 19)   /* TEX: 0 */
              | (0U  << 17)   /* C: 0 */
              | (0U  << 16)   /* B: 0 */
              | (0U  << 15)   /* S: 0 */
              | (18U << 1)    /* SIZE: log2(512KB)-1 = 18 */
              | (1U  << 0);   /* ENABLE */

    /* Region 4: OCRAM (0x20200000, 512KB)
     * 属性: Normal Memory (片上 RAM，同速访问)
     * 目标: 次热数据溢出区（当 DTCM 容量不足时使用）
     * 说明: OCRAM 与 CPU 同频，延迟约 2-3 cycle，无需 Cache */
    MPU->RNR  = 4;
    MPU->RBAR = 0x20200000U;
    MPU->RASR = (0U  << 28)   /* XN: 不可执行 */
              | (3U  << 24)   /* AP: 全权限 */
              | (0U  << 19)   /* TEX: 0 */
              | (0U  << 17)   /* C: 0 */
              | (0U  << 16)   /* B: 0 */
              | (0U  << 15)   /* S: 0 */
              | (18U << 1)    /* SIZE: log2(512KB)-1 = 18 */
              | (1U  << 0);   /* ENABLE */

    /* Region 5: FlexSPI (0x70000000, 4MB)
     * 属性: Device/nGnRnE, Read-Only
     * 目标: 外部 Flash（代码/只读数据）
     * 说明: I-Cache 已负责取指加速，D-Cache 无需覆盖 Flash 数据读 */
    MPU->RNR  = 5;
    MPU->RBAR = 0x70000000U;
    MPU->RASR = (0U  << 28)   /* XN: 不可执行（数据视角） */
              | (6U  << 24)   /* AP[2:0]: 110 = 特权 RW / 用户 RO */
              | (0U  << 19)   /* TEX: 0 */
              | (0U  << 17)   /* C: 0 */
              | (0U  << 16)   /* B: 0 */
              | (0U  << 15)   /* S: 0 */
              | (21U << 1)    /* SIZE: log2(4MB)-1 = 21 */
              | (1U  << 0);   /* ENABLE */

    /* ---- 第 3 步：数据同步屏障 ---- */
    __DSB();
    __ISB();

    /* ---- 第 4 步：使能 MPU（含 PRIVDEFENA 背景默认映射） ---- */
    /* PRIVDEFENA=1: 未覆盖区域使用默认内存映射（对系统安全关键） */
    MPU->CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;

    /* ---- 第 5 步：启用 D-Cache ---- */
    /* I-Cache 已在 SCB_EnableICache() 中启用（位于 SystemInit() 内），
     * 此处仅负责启用 D-Cache。使能后 SDRAM Region 0 的数据访问自动缓存 */
    SCB_EnableDCache();
}
