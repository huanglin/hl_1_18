#include <linux/linkage.h>

#include <asm/assembler.h>


ENTRY(RGB565_2_Luma_0_s)
	stmfd				sp!,{r4, r5, r6, r7, r8, r9, r10, r11, lr}
	
	mov					r3, #77
	mov					r12, #150
	mov					r14, #29
	ldr					r4, [r1], #4
1:	
	and					r6, r4, #0xf800
	and					r7, r4, #0x07e0						
	and					r8, r4, #0x1f						
	lsr					r9, r6, #8
	mul					r6, r9, r3
	lsr					r10, r7, #3
	mla					r7, r10, r12, r6
	lsl					r11, r8, #3
	mla					r8, r11, r14, r7
	lsr					r4, r4, #16
	and					r11, r8, #0xf000
	lsr					r5, r11, #12
	and					r6, r4, #0xf800
	and					r7, r4, #0x07e0
	and					r8, r4, #0x1f
	lsr					r6, r6, #8
	mul					r9, r6, r3
	lsr					r7, r7, #3
	mla					r10, r7, r12, r9
	lsl					r8, r8, #3
	mla					r11, r8, r14, r10
	ldr 				r4, [r1], #4
	and					r11, r11, #0xf000
	lsr					r11, r11, #8
	orr					r5, r5, r11
	
	and					r6, r4, #0xf800	
	and					r7, r4, #0x07e0
	and					r8, r4, #0x1f
	lsr					r6, r6, #8
	mul					r9, r6, r3
	lsr					r7, r7, #3
	mla					r10, r7, r12, r9
	lsl					r8, r8, #3
	mla					r11, r8, r14, r10
	lsr					r4, r4, #16
	and					r11, r11, #0xf000
	lsr					r11, r11, #4
	orr					r5, r5, r11
	and					r6, r4, #0xf800
	and					r7, r4, #0x07e0
	and					r8, r4, #0x1f
	lsr					r6, r6, #8
	mul					r9, r6, r3
	lsr					r7, r7, #3
	mla					r10, r7, r12, r9
	lsl					r8, r8, #3
	mla					r11, r8, r14, r10
	subs				r2, r2, #1
	and					r11, r11, #0xf000
	ldrne				r4, [r1], #4
	orr					r5, r5, r11
	strh					r5, [r0], #2
	bne					1b
	
9:
	ldmfd				sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}
ENDPROC(RGB565_2_Luma_0_s)

//			rgb_buf += (h*w - w) ;
//	   RGB565_2_Luma_270_s(gray, rgb_buf, w, h);
ENTRY(RGB565_2_Luma_270_s)
	stmfd				sp!,{r4, r5, r6, r7, r8, r9, r10, r11, lr}
	mov					lr, r2, lsl #1
	mov					r12, r1
	mov					r5, #77
	lsl					r5, r5, #16
	orr					r5, r5, #150
	mov					r6, #29
	orr					r6, r6, r3, lsl #16
1:
	ldrh				r4, [r1], -lr
	and					r7, r4, #0xf800			
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0				
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f					
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	
	ldrh				r4, [r1], -lr
	and					r9, r8, #0xf000
	lsr					r10, r9, #12
	
	and					r7, r4, #0xf800				
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0				
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f					
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	
	ldrh				r4, [r1], -lr
	and					r9, r8, #0xf000
	lsr					r9, r9, #8
	orr					r10, r9, r10
	
	and					r7, r4, #0xf800				
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0				
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f					
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	
	ldrh				r4, [r1], -lr
	and					r9, r8, #0xf000
	lsr					r9, r9, #4
	orr					r10, r9, r10
	
	and					r7, r4, #0xf800
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	and					r9, r8, #0xf000
	orr					r10, r9, r10
	strh				r10, [r0], #2
	subs				r3, r3, #4
	bne					1b
	lsr					r3, r6, #16
	subs				r2, r2, #1
	addne				r12, r12, #2
	movne				r1, r12
	bne					1b
9:
	ldmfd				sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}
ENDPROC(RGB565_2_Luma_270_s)


//rgb_buf += (w - 1);
//RGB565_2_Luma_90_s(gray, rgb_buf, w, h);
ENTRY(RGB565_2_Luma_90_s)
	stmfd				sp!,{r4, r5, r6, r7, r8, r9, r10, r11, lr}
	mov					lr, r2,lsl #1
	mov					r12, r1
	mov					r5, #77
	lsl					r5, r5, #16
	orr					r5, r5, #150
	mov					r6, #29
	orr					r6, r6, r3, lsl #16
1:
	ldrh				r4, [r1], lr
	and					r7, r4, #0xf800		
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0			
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f				
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	and					r9, r8, #0xf000
	lsr					r10, r9, #12
	ldrh				r4, [r1], lr
	and					r7, r4, #0xf800	
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0	
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f	
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	and					r9, r8, #0xf000
	lsr					r9, r9, #8
	orr					r10, r9, r10
	ldrh				r4, [r1], lr
	and					r7, r4, #0xf800		
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0	
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f	
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	and					r9, r8, #0xf000
	lsr					r9, r9, #4
	orr					r10, r9, r10
	ldrh				r4, [r1], lr
	and					r7, r4, #0xf800	
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0	
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f	
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	and					r9, r8, #0xf000
	orr					r10, r9, r10
	strh				r10, [r0], #2
	subs				r3, r3, #4
	bne					1b
	lsr					r3, r6, #16
	subs				r2, r2, #1
	subne				r12, r12, #2
	movne				r1, r12
	bne					1b
9:
	ldmfd				sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}
ENDPROC(RGB565_2_Luma_90_s)

ENDPROC(RGB565_2_Luma_180_s)
//rgb_buf += (h*w - 1) ;
//for (j=0;j<h;j++)
//			{
//				RGB565_2_Luma_180_s(gray, rgb_buf, w>>2);
//        rgb_buf -= w;
//        gray += (w>>2);
//			}
ENTRY(RGB565_2_Luma_180_s)
	stmfd				sp!,{r4, r5, r6, r7, r8, r9, r10, r11, lr}
	
	mov					r3, #77
	mov					r12, #150
	mov					r14, #29
	ldr					r4, [r1], #-4
1:	
	and					r6, r4, #0xf800
	and					r7, r4, #0x07e0						
	and					r8, r4, #0x1f						
	lsr					r9, r6, #8
	mul					r6, r9, r3
	lsr					r10, r7, #3
	mla					r7, r10, r12, r6
	lsl					r11, r8, #3
	mla					r8, r11, r14, r7
	lsr					r4, r4, #16
	and					r11, r8, #0xf000
	lsr					r5, r11, #12
	and					r6, r4, #0xf800
	and					r7, r4, #0x07e0
	and					r8, r4, #0x1f
	lsr					r6, r6, #8
	mul					r9, r6, r3
	lsr					r7, r7, #3
	mla					r10, r7, r12, r9
	lsl					r8, r8, #3
	mla					r11, r8, r14, r10
	ldr 				r4, [r1], #-4
	and					r11, r11, #0xf000
	lsr					r11, r11, #8
	orr					r5, r5, r11
	
	and					r6, r4, #0xf800	
	and					r7, r4, #0x07e0
	and					r8, r4, #0x1f
	lsr					r6, r6, #8
	mul					r9, r6, r3
	lsr					r7, r7, #3
	mla					r10, r7, r12, r9
	lsl					r8, r8, #3
	mla					r11, r8, r14, r10
	lsr					r4, r4, #16
	and					r11, r11, #0xf000
	lsr					r11, r11, #4
	orr					r5, r5, r11
	and					r6, r4, #0xf800
	and					r7, r4, #0x07e0
	and					r8, r4, #0x1f
	lsr					r6, r6, #8
	mul					r9, r6, r3
	lsr					r7, r7, #3
	mla					r10, r7, r12, r9
	lsl					r8, r8, #3
	mla					r11, r8, r14, r10
	subs				r2, r2, #1
	and					r11, r11, #0xf000
	ldrne				r4, [r1], #-4
	orr					r5, r5, r11
	strh					r5, [r0], #2
	bne					1b
	
9:
	ldmfd				sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}
ENDPROC(RGB565_2_Luma_180_s)

//			rgb_buf += (h*w - w) ;
//	   RGB565_2_Luma_270_s(gray, rgb_buf, w, h, panel_w);
ENTRY(RGB565_2_Luma_270_s_n1)
	ldr					r12, [sp]
	stmfd				sp!,{r4, r5, r6, r7, r8, r9, r10, r11, lr}
	mov					r11, r12, lsr #1
	mov					lr, r2, lsl #1
	mov					r12, r1
	mov					r5, #77
	lsl					r5, r5, #16
	orr					r5, r5, #150
	mov					r6, #29
	orr					r6, r6, r3, lsl #16
1:
	ldrh				r4, [r1], -lr
	and					r7, r4, #0xf800			
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0				
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f					
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	
	ldrh				r4, [r1], -lr
	and					r9, r8, #0xf000
	lsr					r10, r9, #12
	
	and					r7, r4, #0xf800				
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0				
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f					
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	
	ldrh				r4, [r1], -lr
	and					r9, r8, #0xf000
	lsr					r9, r9, #8
	orr					r10, r9, r10
	
	and					r7, r4, #0xf800				
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0				
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f					
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	
	ldrh				r4, [r1], -lr
	and					r9, r8, #0xf000
	lsr					r9, r9, #4
	orr					r10, r9, r10
	
	and					r7, r4, #0xf800
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	and					r9, r8, #0xf000
	orr					r10, r9, r10
	strh				r10, [r0], #2
	subs				r3, r3, #4
	bne					1b
	lsr					r3, r6, #16
	sub					r0, r0, r3, lsr #1
	add					r0, r0, r11
	subs				r2, r2, #1
	addne				r12, r12, #2
	movne				r1, r12
	bne					1b
9:
	ldmfd				sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}
ENDPROC(RGB565_2_Luma_270_s_n1)





//rgb_buf += (w - 1);
//RGB565_2_Luma_90_s(gray, rgb_buf, w, h);
ENTRY(RGB565_2_Luma_90_s_n1)
	ldr					r12, [sp]
	stmfd				sp!,{r4, r5, r6, r7, r8, r9, r10, r11, lr}
	mov					r11, r12, lsr #1
	mov					lr, r2,lsl #1
	mov					r12, r1
	mov					r5, #77
	lsl					r5, r5, #16
	orr					r5, r5, #150
	mov					r6, #29
	orr					r6, r6, r3, lsl #16
1:
	ldrh				r4, [r1], lr
	and					r7, r4, #0xf800		
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0			
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f				
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	and					r9, r8, #0xf000
	lsr					r10, r9, #12
	ldrh				r4, [r1], lr
	and					r7, r4, #0xf800	
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0	
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f	
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	and					r9, r8, #0xf000
	lsr					r9, r9, #8
	orr					r10, r9, r10
	ldrh				r4, [r1], lr
	and					r7, r4, #0xf800		
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0	
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f	
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	and					r9, r8, #0xf000
	lsr					r9, r9, #4
	orr					r10, r9, r10
	ldrh				r4, [r1], lr
	and					r7, r4, #0xf800	
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0	
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f	
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	and					r9, r8, #0xf000
	orr					r10, r9, r10
	strh				r10, [r0], #2
	subs				r3, r3, #4
	bne					1b
	lsr					r3, r6, #16
	sub					r0, r0, r3, lsr #1
	add					r0, r0, r11
	subs				r2, r2, #1
	subne				r12, r12, #2
	movne				r1, r12
	bne					1b
9:
	ldmfd				sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}
ENDPROC(RGB565_2_Luma_90_s_n1)

//RGB565_2_Luma_0_s_n1(gray, rgb_buf, w, h, panel_w);
ENTRY(RGB565_2_Luma_0_s_n1)
	ldr					r12, [sp]
	stmfd				sp!,{r4, r5, r6, r7, r8, r9, r10, r11, lr}
	mov					r11, r12, lsr #1
	//mov					lr, r2, lsl #1
	//mov					r12, r1
	mov					r5, #77
	lsl					r5, r5, #16
	orr					r5, r5, #150
	mov					r6, #29
	orr					r6, r6, r2, lsl #16
1:
	ldr 				r4, [r1], #4
	and					r7, r4, #0xf800
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f					
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	
//	ldrh				r4, [r1], -lr
	lsr					r4, r4, #16
	and					r9, r8, #0xf000
	lsr					r10, r9, #12
	
	and					r7, r4, #0xf800				
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0				
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f					
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	
//	ldrh				r4, [r1], -lr
	ldr					r4, [r1], #4
	and					r9, r8, #0xf000
	lsr					r9, r9, #8
	orr					r10, r9, r10
	
	and					r7, r4, #0xf800				
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0				
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f					
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	
//	ldrh				r4, [r1], -lr
	lsr					r4, r4, #16
	and					r9, r8, #0xf000
	lsr					r9, r9, #4
	orr					r10, r9, r10
	
	and					r7, r4, #0xf800
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	and					r9, r8, #0xf000
	orr					r10, r9, r10
	strh				r10, [r0], #2
	subs				r2, r2, #4
	bne					1b
	lsr					r2, r6, #16
	sub					r0, r0, r2, lsr #1
	add					r0, r0, r11
	subs				r3, r3, #1
//	addne				r12, r12, #2
//	movne				r1, r12
	bne					1b
9:
	ldmfd				sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}
ENDPROC(RGB565_2_Luma_0_s_n1)

//RGB565_2_Luma_180_s_n1(gray, rgb_buf, w, h, panel_w);
ENTRY(RGB565_2_Luma_180_s_n1)
	ldr					r12, [sp]
	stmfd				sp!,{r4, r5, r6, r7, r8, r9, r10, r11, lr}
	mov					r11, r12, lsr #1
	//mov					lr, r2, lsl #1
	//mov					r12, r1
	mov					r5, #77
	lsl					r5, r5, #16
	orr					r5, r5, #150
	mov					r6, #29
	orr					r6, r6, r2, lsl #16
1:
	ldr 				r4, [r1], #-4
	lsl					r7, r4, #16
	orr					r4, r7, r4, lsr #16
	and					r7, r4, #0xf800
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f					
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	
//	ldrh				r4, [r1], -lr
	lsr					r4, r4, #16
	and					r9, r8, #0xf000
	lsr					r10, r9, #12
	
	and					r7, r4, #0xf800
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0				
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f					
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	
//	ldrh				r4, [r1], -lr
	ldr					r4, [r1], #-4
	and					r9, r8, #0xf000
	lsr					r9, r9, #8
	orr					r10, r9, r10
	
	lsl					r7, r4, #16
	orr					r4, r7, r4, lsr #16
	
	and					r7, r4, #0xf800				
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0				
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f					
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	
//	ldrh				r4, [r1], -lr
	lsr					r4, r4, #16
	and					r9, r8, #0xf000
	lsr					r9, r9, #4
	orr					r10, r9, r10
	
	and					r7, r4, #0xf800
	lsr					r7, r7, #8
	smulbt			r8, r7, r5
	and					r7, r4,	#0x07e0
	lsr					r7, r7, #3
	smlabb			r9, r7, r5, r8
	and					r7, r4, #0x1f
	lsl					r7, r7, #3
	smlabb			r8, r7, r6, r9
	and					r9, r8, #0xf000
	orr					r10, r9, r10
	strh				r10, [r0], #2
	subs				r2, r2, #4
	bne					1b
	lsr					r2, r6, #16
	sub					r0, r0, r2, lsr #1
	add					r0, r0, r11
	subs				r3, r3, #1
//	addne				r12, r12, #2
//	movne				r1, r12
	bne					1b
9:
	ldmfd				sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}
ENDPROC(RGB565_2_Luma_180_s_n1)

//Gray8bit_to_4bit_0_s_n1(graynew, gray8bit, w, h, panel_w);
ENTRY(Gray8bit_to_4bit_0_s_n1)
	ldr					r12, [sp]
	stmfd				sp!,{r4, r5, r6, r7, r8, r9, r10, r11, lr}
	mov					r11, r12, lsr #1
	//mov					lr, r2, lsl #1
	//mov					r12, r1
	mov					r5, #0xf
	ldmia				r1!, {r4, r10}
	mov					r6, r2

1:
	
	and					r7, r5, r4, lsr #4
	and					r8, r5, r4, lsr #12
	orr					r7, r7, r8, lsl #4
	and					r8, r5, r4, lsr #20
	orr					r7, r7, r8, lsl #8
	and					r8, r5, r4, lsr #28
	orr					r7, r7, r8, lsl #12
	
	and					r8, r5, r10, lsr #4
	orr					r7, r7, r8, lsl #16
	and					r8, r5, r10, lsr #12
	orr					r7, r7, r8, lsl #20
	and					r8, r5, r10, lsr #20
	orr					r7, r7, r8, lsl #24
	and					r8, r5, r10, lsr #28
	orr					r7, r7, r8, lsl #28
	
	str 				r7, [r0], #4
	ldmia				r1!, {r4, r10}
	subs				r2, r2, #8
	bne					1b
	lsl					r2, r6, #16
	lsr					r2, r2, #16
	sub					r0, r0, r2, lsr #1
	add					r0, r0, r11
	subs				r3, r3, #1
	bne					1b
	ldmfd				sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}
	
ENDPROC(Gray8bit_to_4bit_0_s_n1)

//Gray8bit_to_4bit_180_s_n1(graynew, gray8bit, w, h, panel_w);
ENTRY(Gray8bit_to_4bit_180_s_n1)
		ldr					r12, [sp]
	stmfd				sp!,{r4, r5, r6, r7, r8, r9, r10, r11, lr}
	mov					r11, r12, lsr #1
	//mov					lr, r2, lsl #1
	//mov					r12, r1
	mov					r5, #0xf

	mov					r6, r2

1:
	ldr					r4, [r1], #-4
	and					r7, r5, r4, lsr #28
	and					r8, r5, r4, lsr #20
	orr					r7, r7, r8, lsl #4
	and					r8, r5, r4, lsr #12
	orr					r7, r7, r8, lsl #8
	and					r8, r5, r4, lsr #4
	orr					r7, r7, r8, lsl #12
	strh				r7, [r0], #2
	subs				r2, r2, #4
	bne					1b
	lsl					r2, r6, #16
	lsr					r2, r2, #16
	sub					r0, r0, r2, lsr #1
	add					r0, r0, r11
	subs				r3, r3, #1
	bne					1b
	ldmfd				sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}
	
ENDPROC(Gray8bit_to_4bit_180_s_n1)

//Gray8bit_to_4bit_90_s_n1(graynew, gray8bit, w, h, panel_w);
ENTRY(Gray8bit_to_4bit_90_s_n1)
	ldr					r12, [sp]
	stmfd				sp!,{r4, r5, r6, r7, r8, r9, r10, r11, lr}
	mov					r11, r12, lsr #1
	mov					lr, r2
	mov					r12, r1
	mov					r5, #0xf
	mov					r6, r3

1:
	ldrb				r4, [r1], lr
	and					r7, r5, r4, lsr #4
	ldrb				r4, [r1], lr
	and					r8, r5, r4, lsr #4
	ldrb				r4, [r1], lr
	orr					r7, r7, r8, lsl #4
	and					r8, r5, r4, lsr #4
	ldrb				r4, [r1], lr
	orr					r7, r7, r8, lsl #8
	and					r8, r5, r4, lsr #4
	orr					r7, r7, r8, lsl #12
	strh				r7, [r0], #2
	subs				r3, r3, #4
	bne					1b
	lsl					r3, r6, #16
	lsr					r3, r3, #16
	sub					r0, r0, r3, lsr #1
	add					r0, r0, r11
	sub					r12, r12, #1
	mov					r1, r12
	subs				r2, r2, #1
	bne					1b
	ldmfd				sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}
	
ENDPROC(Gray8bit_to_4bit_90_s_n1)

//Gray8bit_to_4bit_270_s_n1(graynew, gray8bit, w, h, panel_w);
ENTRY(Gray8bit_to_4bit_270_s_n1)
	ldr					r12, [sp]
	stmfd				sp!,{r4, r5, r6, r7, r8, r9, r10, r11, lr}
	mov					r11, r12, lsr #1
	mov					lr, r2
	mov					r12, r1
	mov					r5, #0xf
	mov					r6, r3

1:
	ldrb				r4, [r1], -lr
	and					r7, r5, r4, lsr #4
	ldrb				r4, [r1], -lr
	and					r8, r5, r4, lsr #4
	ldrb				r4, [r1], -lr
	orr					r7, r7, r8, lsl #4
	and					r8, r5, r4, lsr #4
	ldrb				r4, [r1], -lr
	orr					r7, r7, r8, lsl #8
	and					r8, r5, r4, lsr #4
	orr					r7, r7, r8, lsl #12
	strh				r7, [r0], #2
	subs				r3, r3, #4
	bne					1b
	lsl					r3, r6, #16
	lsr					r3, r3, #16
	sub					r0, r0, r3, lsr #1
	add					r0, r0, r11
	add					r12, r12, #1
	mov					r1, r12
	subs				r2, r2, #1
	bne					1b
	ldmfd				sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}
	
ENDPROC(Gray8bit_to_4bit_270_s_n1)