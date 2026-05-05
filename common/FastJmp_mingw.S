/* SPDX-FileCopyrightText: 2002-2024 PCSX2 Dev Team
 * SPDX-License-Identifier: LGPL-3.0+
 *
 * MinGW-w64 GAS port of FastJmp.asm (Win64 ABI).
 *
 * Layout of fastjmp_buf (Win64), matches FastJmp.asm and FastJmp.h's
 * BUF_SIZE = 240:
 *   +  0  rip
 *   +  8  rbx
 *   + 16  rsp
 *   + 24  rbp
 *   + 32  rsi
 *   + 40  rdi
 *   + 48  r12
 *   + 56  r13
 *   + 64  r14
 *   + 72  r15
 *   + 80  xmm6  (16-byte aligned)
 *   + 96  xmm7
 *   +112  xmm8
 *   +128  xmm9
 *   +144  xmm10
 *   +160  xmm11
 *   +176  xmm12
 *   +192  xmm13
 *   +208  xmm14
 *   +224  xmm15
 *
 * Win64 calling convention: 1st arg = rcx, 2nd arg = rdx, return in rax.
 */

	.intel_syntax noprefix
	.text

	.globl	fastjmp_set
	.def	fastjmp_set; .scl 2; .type 32; .endef
	.seh_proc fastjmp_set
fastjmp_set:
	.seh_endprologue
	mov	rax, qword ptr [rsp]
	mov	rdx, rsp
	add	rdx, 8
	mov	qword ptr [rcx + 0], rax
	mov	qword ptr [rcx + 8], rbx
	mov	qword ptr [rcx + 16], rdx
	mov	qword ptr [rcx + 24], rbp
	mov	qword ptr [rcx + 32], rsi
	mov	qword ptr [rcx + 40], rdi
	mov	qword ptr [rcx + 48], r12
	mov	qword ptr [rcx + 56], r13
	mov	qword ptr [rcx + 64], r14
	mov	qword ptr [rcx + 72], r15
	movaps	xmmword ptr [rcx + 80], xmm6
	movaps	xmmword ptr [rcx + 96], xmm7
	movaps	xmmword ptr [rcx + 112], xmm8
	add	rcx, 112
	movaps	xmmword ptr [rcx + 16], xmm9
	movaps	xmmword ptr [rcx + 32], xmm10
	movaps	xmmword ptr [rcx + 48], xmm11
	movaps	xmmword ptr [rcx + 64], xmm12
	movaps	xmmword ptr [rcx + 80], xmm13
	movaps	xmmword ptr [rcx + 96], xmm14
	movaps	xmmword ptr [rcx + 112], xmm15
	xor	eax, eax
	ret
	.seh_endproc

	.globl	fastjmp_jmp
	.def	fastjmp_jmp; .scl 2; .type 32; .endef
	.seh_proc fastjmp_jmp
fastjmp_jmp:
	.seh_endprologue
	mov	eax, edx
	mov	rdx, qword ptr [rcx + 0]
	mov	rbx, qword ptr [rcx + 8]
	mov	rsp, qword ptr [rcx + 16]
	mov	rbp, qword ptr [rcx + 24]
	mov	rsi, qword ptr [rcx + 32]
	mov	rdi, qword ptr [rcx + 40]
	mov	r12, qword ptr [rcx + 48]
	mov	r13, qword ptr [rcx + 56]
	mov	r14, qword ptr [rcx + 64]
	mov	r15, qword ptr [rcx + 72]
	movaps	xmm6, xmmword ptr [rcx + 80]
	movaps	xmm7, xmmword ptr [rcx + 96]
	movaps	xmm8, xmmword ptr [rcx + 112]
	add	rcx, 112
	movaps	xmm9, xmmword ptr [rcx + 16]
	movaps	xmm10, xmmword ptr [rcx + 32]
	movaps	xmm11, xmmword ptr [rcx + 48]
	movaps	xmm12, xmmword ptr [rcx + 64]
	movaps	xmm13, xmmword ptr [rcx + 80]
	movaps	xmm14, xmmword ptr [rcx + 96]
	movaps	xmm15, xmmword ptr [rcx + 112]
	jmp	rdx
	.seh_endproc
