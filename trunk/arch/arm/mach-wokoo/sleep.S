/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2020 Codefair Semiconductor Technology Co., Ltd.
 *		http://www.codefairsemi.com
 *
 * wokoo Sleep Code
 */

#include <linux/linkage.h>
#include <linux/arm-smccc.h>
#include <linux/init.h>
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/hwcap.h>
#include <asm/pgtable-hwdef.h>
#include <asm/pgtable.h>
#include <asm/memory.h>

	.align 3

 /*
  * wokoo_cpu_suspend
  *
  * suspend code entry for bootloader to call
  */

ENTRY(wokoo_cpu_suspend)
 MOV      r10, lr
 LDR      r6,[r0,#0x14]
 LDR      r7,[r0,#0x1c]
 LDR      r8,[r0,#0x24]
 LDR      r9,[r0,#0x2c]

 // Enable DFILPCFG0.dfi_lp_en_sr
 // Set DFILPCFG0.dfi_lp_wakeup_sr£¬wake
 // time >= 2048 cycle, dll and io all pown down. 
 LDR      r3,[r7,#0x198]
 ORR      r3,r3,#0x7100
 STR      r3,[r7,#0x198]

 // Set DSGCR.LPIOPD and DSGCR.LPDLLPD ot 0x1
 // Set PHY to response DFI lower power request,
 // shut down dll and io.  
 LDR      r3,[r8,#0x2c]
 ORR      r3,r3,#0x18
 STR      r3,[r8,#0x2c]

 // 6 steps ddr enter self_refresh mode
 // Step 1: disable all AXI ports
 MOV      r3,#0
 STR      r3,[r7,#0x490]
 STR      r3,[r7,#0x540]
 STR      r3,[r7,#0x5f0]

 // Step 2: waitting all AXI ports to idle
 MOV      r1,#7
 MOVT     r1,#7
 ADD      r2,r7,#0x3fc
loop1: 
 LDR      r3,[r2,#0]
 TST      r3,r1
 BNE      loop1

 // Step 3: disable SBRCTL.scrub_en
 ADD      r3,r7,#0xf20
 ADD      r3,r3,#4
 LDR      r1,[r3,#0]
 BIC      r1,r1,#1
 STR      r1,[r3,#0]

 // Step 4: waitting SBRCTL.scrub_busy to 0x0
 ADD      r1,r7,#0xf20
 ADD      r1,r1,#8
loop2:
 LDR      r3,[r1,#0]
 TST      r3,#1
 BNE      loop2

 // Step 5: enable PWRCTL.selfref_sw
 LDR      r3,[r7,#0x30]
 ORR      r3,r3,#0x20
 STR      r3,[r7,#0x30]
 // Step 6: inqure self_refresh status
loop3:
 ADD      r2,r7,#04
 LDR      r3,[r2,#0]
 TST      r3,#30
 BEQ      loop3

 // Let all chip to deep sleep
 //DSB      ST
 //MOV      r4,#0x1
 //STR      r4,[r9,#0xc]

 // Waitting for gpio interupt to wake up 

 WFI
 NOP
 NOP
 NOP
 NOP

 // Enable SBRCTL.scrub_en
 LDR      r3,[r7,#0x30]
 ORR      r3,r3,#0x01
 STR      r3,[r7,#0x30]

 // Enable all AXI ports
 MOV      r3,#1
 STR      r3,[r7,#0x490]
 STR      r3,[r7,#0x540]
 STR      r3,[r7,#0x5f0]

 // Rest ddr 
 MOV      r2,#0x300
 MOVT     r2,#0x300
 STR      r2,[r6,#0x38]
 NOP
 NOP
 NOP
 NOP
 NOP
 NOP
 NOP
 NOP
 NOP
 NOP
 NOP
 NOP
 MOV      r2,#0x3000000
 STR      r2,[r6,#0x38]

 // Disable DDR lower power 
 MOV      r3,#1
 STR      r3,[r7,#0x30]
 MOV      r3,#0
 STR      r3,[r7,#0x30]

 // Config DDR PHY
 DMB
 MOV      r3,#0x2e02
 MOVT     r3,#0x18c
 STR      r3,[r8,#8]
 DMB
 MOV      r3,#4
 STR      r3,[r8,#0x30]

 DMB
 MOV      r3,#0xc40
 STR      r3,[r8,#0x28]
 MOV      r3,#0x25f
 MOVT     r3,#0xf800
 STR      r3,[r8,#0x2c]
loop4:
 ADD      r2,r8,#0xc
 LDR      r3,[r2,#0]
 TST      r3,#1
 BEQ      loop4

 // Start PHY init
 MOV      r3,#1
 MOVT     r3,#4
 STR      r3,[r8,#4]

 // Waitting DDR PHY init   
loop5:
 ADD      r2,r8,#0xc
 LDR      r3,[r2,#0]
 TST      r3,#0xf
 BEQ      loop5

 BX       r10
ENDPROC(wokoo_cpu_suspend)

 /*
  * wokoo_cpu_resume
  *
  * resume code entry for bootloader to call
  */

ENTRY(wokoo_cpu_resume)
 bl v7_invalidate_l1
 b  cpu_resume
ENDPROC(wokoo_cpu_resume)
