#include <linux/linkage.h>
#include <asm/memory.h>
#include <asm/glue.h>
#include <asm/vfpmacros.h>
#include <asm/arch/entry-macro.S>
#include <asm/thread_notify.h>
#include <asm/arch/hardware.h>
#include <asm/ptrace.h>
#include <asm/assembler.h>
#include <asm/asm-offsets.h>

	.align	5
	.globl __scu_call_wrap
#__scu_call_wrap:
#
# call to the defined funcion.
# r0: func argument array , max = 6
# r1: argument num.
# r2: the function number.

#;	EXPORT __scu_call_wrap
#;	
#;	CODE32
#;	AREA   ||funwrap||, CODE, READONLY	
__scu_call_wrap:
	STMFD    r13!,{r4,r5,r6,lr}
	mov		r6,r2
	ldmia	r0, {r0-r5}
	STMFD    r13!,{r4,r5}
	mov	lr , pc
	mov pc , r6
	add     r13 , r13 ,#8
	LDMFD    r13!,{r4,r5,r6,pc}
#	END

/**
 * 20091126,HSL@RK,change to get syscall struct pt_regs *.
 * way: search stack frame for lr = ret_fast_syscall.
 * code from bachtrace.S -- __backtrace.
 */
#define frame	r4
#define sv_fp	r5
#define sv_pc	r6
#define mask	r7
#define offset	r8
        
ENTRY(__scu_get_regs)
		mov	r1, #0x10
		mov	r0, fp
#if !defined(CONFIG_FRAME_POINTER) || !defined(CONFIG_PRINTK)
    mov r0, #0
		mov	pc, lr
#else
		stmfd	sp!, {r4 - r8, lr}	@ Save an extra register so we have a location...
		movs	frame, r0		@ if frame pointer is zero
		beq	no_frame		@ we have no stack frames

		tst	r1, #0x10		@ 26 or 32-bit mode?
		moveq	mask, #0xfc000003	@ mask for 26-bit
		movne	mask, #0		@ mask for 32-bit

1:		stmfd	sp!, {pc}		@ calculate offset of PC stored
		ldr	r0, [sp], #4		@ by stmfd for this CPU
		adr	r1, 1b
		sub	offset, r0, r1

/*
 * Stack frame layout:
 *             optionally saved caller registers (r4 - r10)
 *             saved fp
 *             saved sp
 *             saved lr
 *    frame => saved pc
 *             optionally saved arguments (r0 - r3)
 * saved sp => <next word>
 *
 * Functions start with the following code sequence:
 *                  mov   ip, sp
 *                  stmfd sp!, {r0 - r3} (optional)
 * corrected pc =>  stmfd sp!, {..., fp, ip, lr, pc}
 */
for_each_frame:	tst	frame, mask		@ Check for address exceptions
		bne	no_frame

1001:		ldr	sv_pc, [frame, #0]	@ get saved pc
1002:		ldr	sv_fp, [frame, #-12]	@ get saved fp

		sub	sv_pc, sv_pc, offset	@ Correct PC for prefetching
		bic	sv_pc, sv_pc, mask	@ mask PC/LR for the mode

1003:		ldr	r2, [sv_pc, #-4]	@ if stmfd sp!, {args} exists,
		ldr	r3, .Ldsi+4		@ adjust saved 'pc' back one
		teq	r3, r2, lsr #10		@ instruction
		subne	r0, sv_pc, #4		@ allow for mov
		subeq	r0, sv_pc, #8		@ allow for mov + stmia

		ldr	r1, [frame, #-4]	@ get saved lr
		ldr	r2, .Ldsi+8
		bic	r1, r1, mask		@ mask PC/LR for the mode
		bic	r2, r2, mask		@ mask PC/LR for the mode
		cmp r1, r2
		beq found_frame
		
		teq	sv_fp, #0		@ zero saved fp means
		beq	no_frame		@ no further frames

		cmp	sv_fp, frame		@ next frame must be
		mov	frame, sv_fp		@ above the current frame
		bhi	for_each_frame
    b no_frame

found_frame:
    add r0, frame , #0x0c 	
    b return_r
no_frame:	
     mov r0, #0
return_r:
    ldmfd	sp!, {r4 - r8, pc}
		
		.section __ex_table,"a"
		.align	3
		.long	1001b, no_frame
		.long	1002b, no_frame
		.long	1003b, no_frame
		.previous
.Ldsi:		
    .word	0xe92dd800 >> 10	@ stmfd sp!, {... fp, ip, lr, pc}
		.word	0xe92d0000 >> 10	@ stmfd sp!, {}
		.word ret_fast_syscall
#endif	

/*
 * 20091215,continue running from break point.
 * r0 is the struct pt_regs *.
 */
ENTRY(__scu_bk_continue)
    mov r12, r0	 
    ldr	r0, [r12, #S_PSR]		
	  msr	spsr_cxsf, r0
	  ldmia	r12, {r0 - pc}^			@ load r0 - pc, cpsr
	
/******************TCM CODE AND DATA **********************************************/	
#define TCM_FUNC	".tcm.text"	
#define SDRC_OFFSET       	    0X10
#define SCU_OFFSET       	      (SDRC_OFFSET+0X4)  
#define GATE0_OFFSET            (SCU_OFFSET+0x4)
#define SCU_GAT1_REG_OFFSET     0X20


/*
 * 20090924,HSL,take care of soft reset reg have no usage.
 * gate 0 have to much use clk,so only close gate 1,2.
*/
  .section TCM_FUNC, "ax"
	.align	3
	
ENTRY(__rk28_halt_here)
	mov	r0, #0
	mrc	p15, 0, r1, c1, c0, 0		@ Read control register
	mcr	p15, 0, r0, c7, c10, 4		@ Drain write buffer
	bic	r2, r1, #1 << 12
	mrs	r3, cpsr			@ Disable FIQs while Icache
	orr	ip, r3, #PSR_F_BIT		@ is disabled
	msr	cpsr_c, ip
	mcr	p15, 0, r2, c1, c0, 0		@ Disable I cache
	mcr	p15, 0, r0, c7, c0, 4		@ Wait for interrupt
	nop             @20091221,flush cpu line
	nop
	nop
	nop
	nop
	mcr	p15, 0, r1, c1, c0, 0		@ Restore ICache enable
	msr	cpsr_c, r3			@ Restore FIQ state
	mov	pc, lr
                .global rk28_debugs
rk28_debugs:                
	.long 	0 
	.global save_sp
save_sp:                
	.long 	0 
	.previous

#if FIQ_ENABLE
        .align
ENTRY(rk28_fiq_handle)
        mrs r8, spsr
        adr r9,__fiq_save
        stmia r9 , {r0-r8,lr}
        mov   r0,r9     @save addr.
        mov r5,#(SVC_MODE|PSR_I_BIT|PSR_F_BIT)
        msr  cpsr_cxsf,r5       @ to svc mod.disable irq,fiq.
        sub r1, sp, #(S_FRAME_SIZE)

        add     r2 , r1 , #32   @ r0--r7.
        stmia  r2,{r8-lr}       @ the svc sp not change here.
        ldmia  r0,{r3-r10}
        stmia  r1,{r3-r10}
        ldr      r9,[r0,#36]     @get fiq lr.
        sub    r9,r9,#4
        str      r9 , [r1,#S_PC]
        ldr      r10,[r0,#32]     @get fiq spsr.
        str      r10, [r1,#S_PSR]
        mov    r5,r1
        mov    sp,r5    @stack frame 
        mov    r0,r5
        ldr      r11,1f
        blx      r11
        b        3f
1:
                .long           rk28_debug_fiq
3:                
        msr  spsr_cxsf,r10
        ldmia r5, {r0 - pc}^
__fiq_save:                @for save fiq spsr r0-r7,spsr,lr.
                .long     0                
                .previous
#endif

