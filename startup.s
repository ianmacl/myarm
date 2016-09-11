/* ***************************************************************************************************************

	crt.s						STARTUP  ASSEMBLY  CODE 
								-----------------------
	Module includes the interrupt vectors and start-up code.

  *************************************************************************************************************** */

/* Stack Size Allocation */
.set  UND_STACK_SIZE, 0x00000004		/* stack for "undefined instruction" interrupts     */
.set  ABT_STACK_SIZE, 0x00000004		/* stack for "abort" interrupts	                    */
.set  FIQ_STACK_SIZE, 0x00000004		/* stack for "FIQ" fast interrupts mode        		*/
.set  IRQ_STACK_SIZE, 0X00000080		/* stack for "IRQ" normal interrupts mode  			*/
.set  SVC_STACK_SIZE, 0x00000080		/* stack for "SVC" supervisor mode  			    */
/*.set  SWI_STACK_SIZE, 0x00000080		/* stack for "SWI" software interrupt mode 			*/

/* Standard definitions of Mode bits and Interrupt (I & F) flags in PSRs (program status registers) */
.set  MODE_USR, 0x10            		/* Normal User Mode 										*/
.set  MODE_FIQ, 0x11            		/* FIQ Processing Fast Interrupts Mode 						*/
.set  MODE_IRQ, 0x12            		/* IRQ Processing Standard Interrupts Mode 					*/
.set  MODE_SVC, 0x13            		/* Supervisor Processing Software Interrupts Mode 			*/
.set  MODE_ABT, 0x17            		/* Abort Processing memory Faults Mode 						*/
.set  MODE_UND, 0x1B            		/* Undefined Processing Undefined Instructions Mode 		*/
.set  MODE_SYS, 0x1F            		/* System Running Priviledged Operating System Tasks  Mode	*/
.set  I_BIT, 0x80               		/* when I bit is set, IRQ is disabled (program status registers) */
.set  F_BIT, 0x40               		/* when F bit is set, FIQ is disabled (program status registers) */

.text
.arm

.global	Reset_Handler
.global _startup
.func   _startup

_startup:
_vectors:       ldr     PC, Reset_Addr    	/* 0x0000 */     
                ldr     PC, Undef_Addr		/* 0x0004 */
                ldr     PC, SWI_Addr		/* 0x0008 */
                ldr     PC, PAbt_Addr		/* 0x000C */
                ldr     PC, DAbt_Addr		/* 0x0010 */
                nop							/* 0x0014 -> loads checksum */
                ldr     PC, [PC,#-0xFF0]	/* 0x0018 -> goes to 0xFFFFF020 (Vector Address Register)  */
                ldr     PC, FIQ_Addr		/* 0x001C */

Reset_Addr:     .word   Reset_Handler		/* defined in this module below  */
Undef_Addr:     .word   UNDEF_Routine		/* defined in main.c  */
SWI_Addr:       .word   SWI_Routine			/* defined in main.c  */
PAbt_Addr:      .word   UNDEF_Routine		/* defined in main.c  */
DAbt_Addr:      .word   UNDEF_Routine		/* defined in main.c  */
IRQ_Addr:       .word   IRQ_Routine			/* defined in main.c  */
FIQ_Addr:       .word   FIQ_Routine			/* defined in main.c  */
                .word   0					/* rounds the vectors and ISR addresses to 64 bytes total  */

Reset_Handler:  
				/* Setup stacks with size indicated above; disable IRQ and FIQ */    			  
    			ldr   r0, =_stack_end
    			msr   CPSR_c, #MODE_UND|I_BIT|F_BIT 	/* Undefined Instruction Mode  */
    			mov   sp, r0
    			sub   r0, r0, #UND_STACK_SIZE
    			msr   CPSR_c, #MODE_ABT|I_BIT|F_BIT 	/* Abort Mode */
    			mov   sp, r0
    			sub   r0, r0, #ABT_STACK_SIZE
    			msr   CPSR_c, #MODE_FIQ|I_BIT|F_BIT 	/* FIQ Mode */
    			mov   sp, r0	
   				sub   r0, r0, #FIQ_STACK_SIZE
    			msr   CPSR_c, #MODE_IRQ|I_BIT|F_BIT 	/* IRQ Mode */
    			mov   sp, r0
    			sub   r0, r0, #IRQ_STACK_SIZE
    			msr   CPSR_c, #MODE_SVC|I_BIT|F_BIT 	/* Supervisor Mode */
    			mov   sp, r0
    			sub   r0, r0, #SVC_STACK_SIZE
    			msr   CPSR_c, #MODE_SYS|I_BIT|F_BIT 	/* User Mode */
    			mov   sp, r0

				/* copy .data section (Copy from ROM to RAM) */
                ldr     R1, =_etext
                ldr     R2, =_data
                ldr     R3, =_edata
1:        		cmp     R2, R3
                ldrlo   R0, [R1], #4
                strlo   R0, [R2], #4
                blo     1b

				/* Clear .bss section (Zero init)  */
                mov     R0, #0
                ldr     R1, =_bss_start
                ldr     R2, =_bss_end
2:				cmp     R1, R2
                strlo   R0, [R1], #4
                blo     2b

				/* Enter the C code  */
                b       main

.endfunc
.end




