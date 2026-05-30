// Byte-equivalence harness for the x86 emitter.
//
// Drives the REAL emitter (linked from common/emitter/*.o) over a broad corpus
// of instruction shapes and dumps the emitted bytes as a hex digest. This run
// produces the GOLDEN reference. After the cursor-passing refactor, rebuild the
// emitter objects, re-run this harness, and diff the output: any difference is
// a refactor bug. Identical output across all shapes is the merge gate.
//
// Build:
//   g++ -std=c++17 -O2 -I common -I common/emitter -I. \
//       harness.cpp <all emitter .o> -o harness

#include "x86emitter.h"
#include <cstdio>
#include <cstdint>

using namespace x86Emitter;

static u8 buf[4096];

// Emit one instruction, print "name: <hex bytes>".
#define DUMP(label, ...)                                   \
	do {                                                   \
		xSetPtr(buf);                                      \
		__VA_ARGS__;                                       \
		u8* end = xGetPtr();                               \
		printf("%-28s", label);                            \
		for (u8* p = buf; p < end; p++) printf("%02x", *p);\
		printf("\n");                                      \
	} while (0)

int main()
{
	// A spread of registers exercising low/high IDs (REX.R/B/X paths),
	// 8/16/32/64-bit operand sizes, and every memory-addressing shape.
	const xRegister32 r32a(0), r32b(7), r32c(8), r32d(12);   // eax, edi, r8d, r12d
	const xRegister64 r64a(0), r64b(8);                       // rax, r8
	const xRegister16 r16a(0), r16b(9);                       // ax, r9w
	const xRegister8  r8a(0),  r8b(10);                       // al, r10b
	const xRegisterSSE x0(0), x1(7), x2(8), x3(13);           // xmm0, xmm7, xmm8, xmm13
	const xAddressReg  a0(0), a1(8);                          // rax, r8 as address regs

	// ---- ALU reg-reg, all sizes / extended regs ----
	DUMP("add r32,r32",      xADD(r32a, r32b));
	DUMP("add r32,r8d",      xADD(r32a, r32c));
	DUMP("add r8d,r12d",     xADD(r32c, r32d));
	DUMP("add r64,r8",       xADD(r64a, r64b));
	DUMP("add r16,r9w",      xADD(r16a, r16b));
	DUMP("add al,r10b",      xADD(r8a,  r8b));
	DUMP("sub r32,r32",      xSUB(r32a, r32b));
	DUMP("and r32,r32",      xAND(r32a, r32b));
	DUMP("or  r64,r8",       xOR (r64a, r64b));
	DUMP("xor r8d,r32",      xXOR(r32c, r32a));
	DUMP("cmp r32,r32",      xCMP(r32a, r32b));
	DUMP("mov r32,r32",      xMOV(r32a, r32b));
	DUMP("mov r64,r8",       xMOV(r64a, r64b));

	// ---- ALU reg-imm (imm8 and imm32 forms) ----
	DUMP("add r32,imm8",     xADD(r32a, 5));
	DUMP("add r32,imm32",    xADD(r32a, 0x12345678));
	DUMP("add r8d,imm32",    xADD(r32c, 0x12345678));
	DUMP("mov r32,imm32",    xMOV(r32a, 0xdeadbeef));
	DUMP("cmp r32,imm8",     xCMP(r32a, 1));

	// ---- memory operands: every SIB / disp / RIP form ----
	DUMP("mov r32,[base]",        xMOV(r32a, ptr32[a0]));
	DUMP("mov r32,[base+d8]",     xMOV(r32a, ptr32[a0 + 8]));
	DUMP("mov r32,[base+d32]",    xMOV(r32a, ptr32[a0 + 0x1000]));
	DUMP("mov r32,[base+idx]",    xMOV(r32a, ptr32[a0 + a1]));
	DUMP("mov r32,[base+idx*4]",  xMOV(r32a, ptr32[a1*4 + a0]));
	DUMP("mov r32,[idx*8+d32]",   xMOV(r32a, ptr32[a1*8 + 0x40]));
	DUMP("mov r8d,[r8+d8]",       xMOV(r32c, ptr32[a1 + 8]));
	DUMP("mov [base],r32",        xMOV(ptr32[a0], r32a));
	DUMP("mov [base+idx*2],r32",  xMOV(ptr32[a1*2 + a0], r32a));

	// ---- SSE reg-reg / reg-mem, extended xmm (REX paths) ----
	DUMP("movaps x,x",       xMOVAPS(x0, x1));
	DUMP("movaps x8,x13",    xMOVAPS(x2, x3));
	DUMP("movaps x,[mem]",   xMOVAPS(x0, ptr[a0]));
	DUMP("movaps x8,[r8]",   xMOVAPS(x2, ptr[a1]));
	DUMP("movdqa x,x",       xMOVDQA(x0, x1));
	DUMP("movdqa x13,x8",    xMOVDQA(x3, x2));
	DUMP("pxor x,x",         xPXOR(x0, x1));
	DUMP("pxor x8,x13",      xPXOR(x2, x3));

	// ---- 16-bit (0x66 prefix) and shift group ----
	DUMP("add r16,r16",      xADD(r16a, r16b));
	DUMP("mov r16,[mem]",    xMOV(r16a, ptr16[a0]));

	return 0;
}
