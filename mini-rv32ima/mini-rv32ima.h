// Copyright 2022 Charles Lohr, you may use this file or any portions herein under any of the BSD, MIT, or CC0 licenses.

#ifndef _MINI_RV32IMAH_H
#define _MINI_RV32IMAH_H

/**
    To use mini-rv32ima.h for the bare minimum, the following:

	#define MINI_RV32_RAM_SIZE ram_amt
	#define MINIRV32_IMPLEMENTATION

	#include "mini-rv32ima.h"

	Though, that's not _that_ interesting. You probably want I/O!


	Notes:
		* There is a dedicated CLNT at 0x10000000.
		* There is free MMIO from there to 0x12000000.
		* You can put things like a UART, or whatever there.
		* Feel free to override any of the functionality with macros.
*/

#ifndef MINIRV32WARN
	#define MINIRV32WARN( x... );
#endif

#ifndef MINIRV32_DECORATE
	#define MINIRV32_DECORATE static
#endif

#ifndef MINIRV32_RAM_IMAGE_OFFSET
	#define MINIRV32_RAM_IMAGE_OFFSET  0x80000000UL
#endif

#ifndef MINIRV32_POSTEXEC
	#define MINIRV32_POSTEXEC(...);
#endif

#ifndef MINIRV32_HANDLE_MEM_STORE_CONTROL
	#define MINIRV32_HANDLE_MEM_STORE_CONTROL(...);
#endif

#ifndef MINIRV32_HANDLE_MEM_LOAD_CONTROL
	#define MINIRV32_HANDLE_MEM_LOAD_CONTROL(...);
#endif

#ifndef MINIRV32_OTHERCSR_WRITE
	#define MINIRV32_OTHERCSR_WRITE(...);
#endif

#ifndef MINIRV32_OTHERCSR_READ
	#define MINIRV32_OTHERCSR_READ(...);
#endif

#ifndef MINIRV32_CUSTOM_MEMORY_BUSz
	#define MINIRV32_STORE8( ofs, val ) *(uint64_t*)(image + ofs) = val
	#define MINIRV32_STORE4( ofs, val ) *(uint32_t*)(image + ofs) = val
	#define MINIRV32_STORE2( ofs, val ) *(uint16_t*)(image + ofs) = val
	#define MINIRV32_STORE1( ofs, val ) *(uint8_t*)(image + ofs) = val
	#define MINIRV32_LOAD8( ofs ) *(uint64_t*)(image + ofs)
	#define MINIRV32_LOAD4( ofs ) *(uint32_t*)(image + ofs)
	#define MINIRV32_LOAD2( ofs ) *(uint16_t*)(image + ofs)
	#define MINIRV32_LOAD1( ofs ) *(uint8_t*)(image + ofs)
	#define MINIRV32_LOAD4_SIGNED( ofs ) *(int32_t*)(image + ofs)
	#define MINIRV32_LOAD2_SIGNED( ofs ) *(int16_t*)(image + ofs)
	#define MINIRV32_LOAD1_SIGNED( ofs ) *(int8_t*)(image + ofs)
#endif

// As a note: We quouple-ify these, because in HLSL, we will be operating with
// uint4's.  We are going to uint4 data to/from system RAM.
//
// We're going to try to keep the full processor state to 12 x uint4.
struct MiniRV32IMAState
{
	uint64_t regs[32];

	uint64_t pc;
	uint64_t mstatus;
	// uint32_t cyclel;
	// uint32_t cycleh;
	uint64_t cycle;

	// uint32_t timerl;
	// uint32_t timerh;
	uint64_t timer;
	// uint32_t timermatchl;
	// uint32_t timermatchh;
	uint64_t timermatch;

	uint64_t mscratch;
	uint64_t mtvec;
	uint64_t mie;
	uint64_t mip;

	uint64_t mepc;
	uint64_t mtval;
	uint64_t mcause;

	// Note: only a few bits are used.  (Machine = 3, User = 0)
	// Bits 0..1 = privilege.
	// Bit 2 = WFI (Wait for interrupt)
	// Bit 3+ = Load/Store reservation LSBs.
	uint32_t extraflags;
};

/**
   * Executes mulh(riscv instruction) on two operands.
  */
  uint64_t mulh(uint64_t rs1, uint64_t rs2) {
    uint64_t partial_products[4];
    // rs1[low] * rs2[low]
    partial_products[0] = (rs1 & (~((~0UL) << 32))) * (rs2 & (~((~0UL) << 32)));
    // rs1[low] * rs2[high]
    partial_products[1] = (uint64_t) (((int64_t) (rs1 & (~((~0UL) << 32)))) * (((int64_t) rs2) >> 32));
    // rs1[high] * rs2[low]
    partial_products[2] = (uint64_t) (((int64_t) (rs2 & (~((~0UL) << 32)))) * (((int64_t) rs1) >> 32));
    // rs1[high] * rs2[high]
    partial_products[3] = (uint64_t) (((int64_t) rs1) >> 32) * (((int64_t) rs2) >> 32);
    partial_products[3] =  partial_products[3] + (uint64_t)(((int64_t)  partial_products[2]) >> 32) + (uint64_t)(((int64_t)  partial_products[1]) >> 32); 
    partial_products[0] = (partial_products[0] >> 32) + (partial_products[1] & (~((~0UL) << 32))) + (partial_products[2] & (~((~0UL) << 32)));
    return partial_products[3] + (partial_products[0] >> 32);
  }

  /**
   * Executes mulhu(riscv instruction) on two operands.
  */
  uint64_t mulhu(uint64_t rs1, uint64_t rs2) {
    uint64_t partial_products[4];
    // rs1[low] * rs2[low]
    partial_products[0] = (rs1 & (~((~0UL) << 32))) * (rs2 & (~((~0UL) << 32)));
    // rs1[low] * rs2[high]
    partial_products[1] = (uint64_t) (((rs1 & (~((~0UL) << 32)))) * ((rs2) >> 32));
    // rs1[high] * rs2[low]
    partial_products[2] = (uint64_t) (((rs2 & (~((~0UL) << 32)))) * ((rs1) >> 32));
    // rs1[high] * rs2[high]
    partial_products[3] = (uint64_t) ((rs1) >> 32) * ((rs2) >> 32);
    partial_products[3] =  partial_products[3] + (uint64_t)(( partial_products[2]) >> 32) + (uint64_t)(( partial_products[1]) >> 32); 
    partial_products[0] = (partial_products[0] >> 32) + (partial_products[1] & (~((~0UL) << 32))) + (partial_products[2] & (~((~0UL) << 32)));
    return partial_products[3] + (partial_products[0] >> 32);
  }

  /**
   * Executes mulhsu(riscv instruction) on two operands.
  */
  uint64_t mulhsu(uint64_t rs1, uint64_t rs2) {
    uint64_t partial_products[4];
    // rs1[low] * rs2[low]
    partial_products[0] = (rs1 & (~((~0UL) << 32))) * (rs2 & (~((~0UL) << 32)));
    // rs1[low] * rs2[high]
    partial_products[1] = (uint64_t) (((rs1 & (~((~0UL) << 32)))) * ((rs2) >> 32));
    // rs1[high] * rs2[low]
    partial_products[2] = (uint64_t) (((int64_t) (rs2 & (~((~0UL) << 32)))) * (((int64_t) rs1) >> 32));
    // rs1[high] * rs2[high]
    partial_products[3] = (uint64_t) (((int64_t) rs1) >> 32) * ((int64_t)((rs2) >> 32));
    partial_products[3] =  partial_products[3] + (uint64_t)(((int64_t)  partial_products[2]) >> 32) + (uint64_t)((partial_products[1]) >> 32); 
    partial_products[0] = (partial_products[0] >> 32) + (partial_products[1] & (~((~0UL) << 32))) + (partial_products[2] & (~((~0UL) << 32)));
    return partial_products[3] + (partial_products[0] >> 32);
  }

  /**
   * Executes div(riscv instruction) on two operands.
  */
  /* uint64_t div(uint64_t rs1, uint64_t rs2) {
    //return 0UL;
    uint64_t absolute_rs1 = (rs1 >> 63) ? ((~rs1) + 1UL) : rs1;
    uint64_t absolute_rs2 = (rs2 >> 63) ? ((~rs2) + 1UL) : rs2;

    // handling zero division
    if (rs2 == 0UL)
      return ~0UL;
    
    switch ((rs1 >> 63) + (rs2 >> 63)){
    case 0:
      return (absolute_rs1/absolute_rs2);

    case 1:
      return 0UL - (absolute_rs1/absolute_rs2);

    case 2:
      return (absolute_rs1/absolute_rs2);
    } 
    return 0UL;
  } */

  /**
   * Executes rem(riscv instruction) on two operands.
  */
  uint64_t rem(uint64_t rs1, uint64_t rs2) {
    //return 0UL;
    uint64_t absolute_rs1 = (rs1 >> 63) ? ((~rs1) + 1UL) : rs1;
    uint64_t absolute_rs2 = (rs2 >> 63) ? ((~rs2) + 1UL) : rs2;

    if (rs2 == 0UL)
      return rs1;
    
    switch ((rs1 >> 63)){
    case 0:
      return (absolute_rs1%absolute_rs2);

    case 1:
      return 0UL - (absolute_rs1%absolute_rs2);
    } 

    return 0UL;
  }

  /**
   * Executes divw(riscv instruction) on two operands.
  */
  uint32_t divw(uint32_t rs1, uint32_t rs2) {
    //return 0UL;
    uint64_t absolute_rs1 = (rs1 >> 31) ? ((~rs1) + 1U) : rs1;
    uint64_t absolute_rs2 = (rs2 >> 31) ? ((~rs2) + 1U) : rs2;

    // handling zero division
    if (rs2 == 0U)
      return ~0U;
    
    switch ((rs1 >> 31) + (rs2 >> 31)){
    case 0:
      return (absolute_rs1/absolute_rs2);

    case 1:
      return 0U - (absolute_rs1/absolute_rs2);

    case 2:
      return (absolute_rs1/absolute_rs2);
    } 
    return 0U;
  }

  /**
   * Executes rem(riscv instruction) on two operands.
  */
  uint32_t remw(uint32_t rs1, uint32_t rs2) {
    //return 0UL;
    uint32_t absolute_rs1 = (rs1 >> 31) ? ((~rs1) + 1UL) : rs1;
    uint32_t absolute_rs2 = (rs2 >> 31) ? ((~rs2) + 1UL) : rs2;

    if (rs2 == 0U)
      return rs1;
    
    switch ((rs1 >> 31)){
    case 0:
      return (absolute_rs1%absolute_rs2);

    case 1:
      return 0U - (absolute_rs1%absolute_rs2);
    } 

    return 0U;
  }

// don't really know what this does - Kaveesha
#ifndef MINIRV32_STEPPROTO
MINIRV32_DECORATE int32_t MiniRV32IMAStep( struct MiniRV32IMAState * state, uint8_t * image, uint32_t vProcAddress, uint32_t elapsedUs, int count );
#endif

#ifdef MINIRV32_IMPLEMENTATION

#ifndef MINIRV32_CUSTOM_INTERNALS
#define CSR( x ) state->x
#define SETCSR( x, val ) { state->x = val; }
#define REG( x ) state->regs[x]
#define REGSET( x, val ) { state->regs[x] = val; }
#endif

#ifndef MINIRV32_STEPPROTO
MINIRV32_DECORATE int32_t MiniRV32IMAStep( struct MiniRV32IMAState * state, uint8_t * image, uint32_t vProcAddress, uint32_t elapsedUs, int count )
#else
MINIRV32_STEPPROTO
#endif
{
	uint64_t new_timer = CSR( timer ) + elapsedUs;
	// if( new_timer < CSR( timerl ) ) CSR( timerh )++;
	CSR( timer ) = new_timer;

	// Handle Timer interrupt.
	// if( ( CSR( timerh ) > CSR( timermatchh ) || ( CSR( timerh ) == CSR( timermatchh ) && CSR( timerl ) > CSR( timermatchl ) ) ) && ( CSR( timermatchh ) || CSR( timermatchl ) ) )
	if( ( CSR( timer ) > CSR( timermatch ) ) && ( CSR( timermatch ) ) )
	{
		CSR( extraflags ) &= ~4; // Clear WFI
		CSR( mip ) |= 1<<7; //MTIP of MIP // https://stackoverflow.com/a/61916199/2926815  Fire interrupt.
	}
	else
		CSR( mip ) &= ~(1<<7);

	// If WFI, don't run processor.
	if( CSR( extraflags ) & 4 )
		return 1;

	uint64_t trap = 0;
	uint64_t rval = 0;
	uint64_t pc = CSR( pc );
	uint64_t cycle = CSR( cycle );

	if( ( CSR( mip ) & (1<<7) ) && ( CSR( mie ) & (1<<7) /*mtie*/ ) && ( CSR( mstatus ) & 0x8 /*mie*/) )
	{
		// Timer interrupt.
		trap = 0x7UL | (1UL << 63);
		pc -= 4;
	}
	else // No timer interrupt?  Execute a bunch of instructions.
	for( int icount = 0; icount < count; icount++ )
	{
		printf("goes here %016lx %d\n", pc, count);
		uint32_t ir = 0;
		rval = 0;
		cycle++;
		uint64_t ofs_pc = pc - MINIRV32_RAM_IMAGE_OFFSET;

		if( ofs_pc >= MINI_RV32_RAM_SIZE )
		{
			trap = 1 + 1;  // Handle access violation on instruction read.
			break;
		}
		else if( ofs_pc & 3 )
		{
			trap = 1 + 0;  //Handle PC-misaligned access
			break;
		}
		else
		{
			ir = MINIRV32_LOAD4( ofs_pc );
			uint32_t rdid = (ir >> 7) & 0x1f;

			switch( ir & 0x7f )
			{
				case 0x37: // LUI (0b0110111)
					rval = (uint64_t) ((int32_t) ( ir & 0xfffff000 ));
					break;
				case 0x17: // AUIPC (0b0010111)
					rval = pc + ( (uint64_t) ((int32_t) ( ir & 0xfffff000 )) );
					break;
				case 0x6F: // JAL (0b1101111)
				{
					int64_t reladdy = ((ir & 0x80000000)>>11) | ((ir & 0x7fe00000)>>20) | ((ir & 0x00100000)>>9) | ((ir&0x000ff000));
					if( reladdy & 0x00100000 ) reladdy |= 0xffffffffffe00000L; // Sign extension.
					rval = pc + 4;
					pc = pc + reladdy - 4;
					break;
				}
				case 0x67: // JALR (0b1100111)
				{
					uint32_t imm = ir >> 20;
					int64_t imm_se = imm | (( imm & 0x800 )?0xfffffffffffff000L:0L);
					rval = pc + 4;
					pc = ( (REG( (ir >> 15) & 0x1f ) + imm_se) & ~1) - 4;
					break;
				}
				case 0x63: // Branch (0b1100011)
				{
					uint32_t immm4 = ((ir & 0xf00)>>7) | ((ir & 0x7e000000)>>20) | ((ir & 0x80) << 4) | ((ir >> 31)<<12);
					if( immm4 & 0x1000 ) immm4 |= 0xffffffffffffe000L;
					int64_t rs1 = REG((ir >> 15) & 0x1f);
					int64_t rs2 = REG((ir >> 20) & 0x1f);
					immm4 = pc + immm4 - 4;
					rdid = 0;
					switch( ( ir >> 12 ) & 0x7 )
					{
						// BEQ, BNE, BLT, BGE, BLTU, BGEU
						case 0: if( rs1 == rs2 ) pc = immm4; break;
						case 1: if( rs1 != rs2 ) pc = immm4; break;
						case 4: if( rs1 < rs2 ) pc = immm4; break;
						case 5: if( rs1 >= rs2 ) pc = immm4; break; //BGE
						case 6: if( (uint32_t)rs1 < (uint32_t)rs2 ) pc = immm4; break;   //BLTU
						case 7: if( (uint32_t)rs1 >= (uint32_t)rs2 ) pc = immm4; break;  //BGEU
						default: trap = (2+1);
					}
					break;
				}
				case 0x03: // Load (0b0000011)
				{
					uint64_t rs1 = REG((ir >> 15) & 0x1f);
					uint64_t imm = ir >> 20;
					int64_t imm_se = imm | (( imm & 0x800 )?0xfffffffffffff000UL:0UL);
					uint64_t rsval = rs1 + imm_se;

					rsval -= MINIRV32_RAM_IMAGE_OFFSET;
					if( rsval >= MINI_RV32_RAM_SIZE-3 )
					{
						rsval += MINIRV32_RAM_IMAGE_OFFSET;
						if( rsval >= 0x10000000 && rsval < 0x12000000 )  // UART, CLNT
						{
							/* if( rsval == 0x1100bffc ) // https://chromitem-soc.readthedocs.io/en/latest/clint.html
								rval = CSR( timerh );
							else  */
							if( rsval == 0x1100bff8 )
								rval = CSR( timer );
							else
								MINIRV32_HANDLE_MEM_LOAD_CONTROL( rsval, rval );
						}
						else
						{
							trap = (5+1);
							rval = rsval;
						}
					}
					else
					{
						switch( ( ir >> 12 ) & 0x7 )
						{
							//LB, LH, LW, LD, LBU, LHU, LWU
							case 0: rval = MINIRV32_LOAD1_SIGNED( rsval ); break;
							case 1: rval = MINIRV32_LOAD2_SIGNED( rsval ); break;
							case 2: rval = MINIRV32_LOAD4_SIGNED( rsval ); break;
							case 3: rval = MINIRV32_LOAD8( rsval ); break;
							case 4: rval = MINIRV32_LOAD1( rsval ); break;
							case 5: rval = MINIRV32_LOAD2( rsval ); break;
							case 6: rval = MINIRV32_LOAD4( rsval ); break;
							default: trap = (2+1);
						}
					}
					break;
				}
				case 0x23: // Store 0b0100011
				{
					uint64_t rs1 = REG((ir >> 15) & 0x1f);
					uint64_t rs2 = REG((ir >> 20) & 0x1f);
					uint64_t addy = ( ( ir >> 7 ) & 0x1f ) | ( ( ir & 0xfe000000 ) >> 20 );
					if( addy & 0x800 ) addy |= 0xfffffffffffff000L;
					addy += rs1 - MINIRV32_RAM_IMAGE_OFFSET;
					rdid = 0;

					if( addy >= MINI_RV32_RAM_SIZE-3 )
					{
						addy += MINIRV32_RAM_IMAGE_OFFSET;
						if( addy >= 0x10000000 && addy < 0x12000000 )
						{
							// Should be stuff like SYSCON, 8250, CLNT
							/* if( addy == 0x11004004 ) //CLNT
								CSR( timermatchh ) = rs2;
							else */ 
							if( addy == 0x11004000 ) //CLNT
								CSR( timermatch ) = rs2;
							else if( addy == 0x11100000 ) //SYSCON (reboot, poweroff, etc.)
							{
								SETCSR( pc, pc + 4 );
								return rs2; // NOTE: PC will be PC of Syscon.
							}
							else
								MINIRV32_HANDLE_MEM_STORE_CONTROL( addy, rs2 );
						}
						else
						{
							trap = (7+1); // Store access fault.
							rval = addy;
						}
					}
					else
					{
						switch( ( ir >> 12 ) & 0x7 )
						{
							//SB, SH, SW
							case 0: MINIRV32_STORE1( addy, rs2 ); break;
							case 1: MINIRV32_STORE2( addy, rs2 ); break;
							case 2: MINIRV32_STORE4( addy, rs2 ); break;
							case 3: MINIRV32_STORE8( addy, rs2 ); break;
							default: trap = (2+1);
						}
					}
					break;
				}
				case 0x13: // Op-immediate 0b0010011
				case 0x33: // Op           0b0110011
				{
					uint64_t imm = ir >> 20;
					imm = imm | (( imm & 0x800 )?0xfffff000:0);
					uint64_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t is_reg = !!( ir & 0x20 );
					uint64_t rs2 = is_reg ? REG(imm & 0x1f) : imm;

					if( is_reg && ( ir & 0x02000000 ) )
					{
						switch( (ir>>12)&7 ) //0x02000000 = RV32M
						{
							case 0: rval = rs1 * rs2; break; // MUL
#ifndef CUSTOM_MULH // If compiling on a system that doesn't natively, or via libgcc support 64-bit math.
							case 1: rval = mulh(rs1, rs2); break; // MULH
							case 2: rval = mulhsu(rs1, rs2); break; // MULHSU
							case 3: rval = mulhu(rs1, rs2); break; // MULHU
#else
							CUSTOM_MULH
#endif
							case 4: if( rs2 == 0 ) rval = -1; else rval = ((int64_t)rs1 == INT64_MIN && (int64_t)rs2 == -1) ? rs1 : ((int64_t)rs1 / (int64_t)rs2); break; // DIV
							case 5: if( rs2 == 0 ) rval = 0xffffffff; else rval = rs1 / rs2; break; // DIVU
							case 6: if( rs2 == 0 ) rval = rs1; else rval = ((int64_t)rs1 == INT64_MIN && (int64_t)rs2 == -1) ? 0 : ((uint64_t)((int64_t)rs1 % (int64_t)rs2)); break; // REM
							case 7: if( rs2 == 0 ) rval = rs1; else rval = rs1 % rs2; break; // REMU
						}
					}
					else
					{
						switch( (ir>>12)&7 ) // These could be either op-immediate or op commands.  Be careful.
						{
							case 0: rval = (is_reg && (ir & 0x40000000) ) ? ( rs1 - rs2 ) : ( rs1 + rs2 ); break; 
							case 1: rval = rs1 << (rs2 & 0x3F); break;
							case 2: rval = (int64_t)rs1 < (int64_t)rs2; break;
							case 3: rval = rs1 < rs2; break;
							case 4: rval = rs1 ^ rs2; break;
							case 5: rval = (ir & 0x40000000 ) ? ( ((int64_t)rs1) >> (rs2 & 0x3F) ) : ( rs1 >> (rs2 & 0x3F) ); break;
							case 6: rval = rs1 | rs2; break;
							case 7: rval = rs1 & rs2; break;
						}
					}
					break;
				}
				case 0x1B: // Op32-immediate 0b0011011
				case 0x3B: // Op32          0b0111011
				{
					uint32_t imm = ir >> 20;
					imm = imm | (( imm & 0x800 )?0xfffffffffffff000:0);
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t is_reg = !!( ir & 0x20 );
					uint32_t rs2 = is_reg ? REG(imm & 0x1f) : imm;

					if( is_reg && ( ir & 0x02000000 ) )
					{
						switch( (ir>>12)&7 ) //0x02000000 = RV32M
						{
							case 0: rval = (int32_t)(( (int32_t) rs1 ) * ( (int32_t) rs2 )); break; // MUL
							case 4: if( ((int32_t) rs2) == 0 ) rval = -1; else rval = ((int32_t)rs1 == INT32_MIN && (int32_t)rs2 == -1) ? ((int32_t)rs1) : ((int32_t)rs1 / (int32_t)rs2); break; // DIV
							case 5: if( ((uint32_t)rs2) == 0 ) rval = 0xffffffffffffffff; else rval = ((uint32_t)rs1) / ((uint32_t)rs2); break; // DIVU
							case 6: if( ((int32_t) rs2) == 0 ) rval = ((int32_t)rs1); else rval = ((int32_t)rs1 == INT32_MIN && (int32_t)rs2 == -1) ? 0 : ((int32_t)((uint32_t)((int32_t)rs1 % (int32_t)rs2))); break; // REM
							case 7: if( ((uint32_t)rs2) == 0 ) rval = ((int32_t)rs1); else rval = (int32_t)(((uint32_t)rs1) % ((uint32_t)rs2)); break; // REMU
						}
					}
					else
					{
						switch( (ir>>12)&7 ) // These could be either op-immediate or op commands.  Be careful.
						{
							case 0: rval = (int32_t)((is_reg && (ir & 0x40000000) ) ? ( rs1 - rs2 ) : ( rs1 + rs2 )); break; 
							case 1: rval = (int32_t)(rs1 << (rs2 & 0x1F)); break;
							case 5: rval = (int32_t)((ir & 0x40000000 ) ? ( ((int32_t)rs1) >> (rs2 & 0x1F) ) : ( rs1 >> (rs2 & 0x1F) )); break;
							default: trap = 3;
						}
					}
					break;
				}
				case 0x0f: // 0b0001111
					rdid = 0;   // fencetype = (ir >> 12) & 0b111; We ignore fences in this impl.
					break;
				case 0x73: // Zifencei+Zicsr  (0b1110011)
				{
					uint32_t csrno = ir >> 20;
					uint32_t microop = ( ir >> 12 ) & 0x7;
					if( (microop & 3) ) // It's a Zicsr function.
					{
						int64_t rs1imm = (ir >> 15) & 0x1f;
						uint64_t rs1 = REG(rs1imm);
						uint64_t writeval = rs1;

						// https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
						// Generally, support for Zicsr
						switch( csrno )
						{
						case 0x340: rval = CSR( mscratch ); break;
						case 0x305: rval = CSR( mtvec ); break;
						case 0x304: rval = CSR( mie ); break;
						case 0xC00: rval = cycle; break;
						case 0x344: rval = CSR( mip ); break;
						case 0x341: rval = CSR( mepc ); break;
						case 0x300: rval = CSR( mstatus ); break; //mstatus
						case 0x342: rval = CSR( mcause ); break;
						case 0x343: rval = CSR( mtval ); break;
						case 0xf11: rval = 0xff0ff0ff; break; //mvendorid
						case 0x301: rval = ((0x00401101) | (2UL << 62)); break; //misa (XLEN=32, IMA+X)
						//case 0x3B0: rval = 0; break; //pmpaddr0
						//case 0x3a0: rval = 0; break; //pmpcfg0
						//case 0xf12: rval = 0x00000000; break; //marchid
						//case 0xf13: rval = 0x00000000; break; //mimpid
						//case 0xf14: rval = 0x00000000; break; //mhartid
						default:
							MINIRV32_OTHERCSR_READ( csrno, rval );
							break;
						}

						switch( microop )
						{
							case 1: writeval = rs1; break;  			//CSRRW
							case 2: writeval = rval | rs1; break;		//CSRRS
							case 3: writeval = rval & ~rs1; break;		//CSRRC
							case 5: writeval = rs1imm; break;			//CSRRWI
							case 6: writeval = rval | rs1imm; break;	//CSRRSI
							case 7: writeval = rval & ~rs1imm; break;	//CSRRCI
						}

						switch( csrno )
						{
						case 0x340: SETCSR( mscratch, writeval ); break;
						case 0x305: SETCSR( mtvec, writeval ); break;
						case 0x304: SETCSR( mie, writeval ); break;
						case 0x344: SETCSR( mip, writeval ); break;
						case 0x341: SETCSR( mepc, writeval ); break;
						case 0x300: SETCSR( mstatus, writeval ); break; //mstatus
						case 0x342: SETCSR( mcause, writeval ); break;
						case 0x343: SETCSR( mtval, writeval ); break;
						//case 0x3a0: break; //pmpcfg0
						//case 0x3B0: break; //pmpaddr0
						//case 0xf11: break; //mvendorid
						//case 0xf12: break; //marchid
						//case 0xf13: break; //mimpid
						//case 0xf14: break; //mhartid
						//case 0x301: break; //misa
						default:
							MINIRV32_OTHERCSR_WRITE( csrno, writeval );
							break;
						}
					}
					else if( microop == 0x0 ) // "SYSTEM" 0b000
					{
						rdid = 0;
						if( csrno == 0x105 ) //WFI (Wait for interrupts)
						{
							CSR( mstatus ) |= 8;    //Enable interrupts
							CSR( extraflags ) |= 4; //Infor environment we want to go to sleep.
							SETCSR( pc, pc + 4 );
							return 1;
						}
						else if( ( ( csrno & 0xff ) == 0x02 ) )  // MRET
						{
							//https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
							//Table 7.6. MRET then in mstatus/mstatush sets MPV=0, MPP=0, MIE=MPIE, and MPIE=1. La
							// Should also update mstatus to reflect correct mode.
							uint32_t startmstatus = CSR( mstatus );
							uint32_t startextraflags = CSR( extraflags );
							SETCSR( mstatus , (( startmstatus & 0x80) >> 4) | ((startextraflags&3) << 11) | 0x80 );
							SETCSR( extraflags, (startextraflags & ~3) | ((startmstatus >> 11) & 3) );
							pc = CSR( mepc ) -4;
						}
						else
						{
							switch( csrno )
							{
							case 0: trap = ( CSR( extraflags ) & 3) ? (11+1) : (8+1); break; // ECALL; 8 = "Environment call from U-mode"; 11 = "Environment call from M-mode"
							case 1:	trap = (3+1); break; // EBREAK 3 = "Breakpoint"
							default: trap = (2+1); break; // Illegal opcode.
							}
						}
					}
					else
						trap = (2+1); 				// Note micrrop 0b100 == undefined.
					break;
				}
				case 0x2f: // RV32A (0b00101111)
				{
				if (((ir >> 12) & 3) == 2U) {
				printf("BUt it does amo %08x, %016lx, %016lx\n", (ir>>12)&3, REG((ir >> 15) & 0x1f), MINI_RV32_RAM_SIZE);
					uint64_t rs1 = REG((ir >> 15) & 0x1f);
					uint64_t rs2 = REG((ir >> 20) & 0x1f);
					uint32_t irmid = ( ir>>27 ) & 0x1f;

					rs1 -= MINIRV32_RAM_IMAGE_OFFSET;

					// We don't implement load/store from UART or CLNT with RV32A here.

					if( rs1 >= MINI_RV32_RAM_SIZE-3 )
					{
						trap = (7+1); //Store/AMO access fault
						rval = rs1 + MINIRV32_RAM_IMAGE_OFFSET;
					}
					else
					{
						rval = MINIRV32_LOAD4( rs1 );

						// Referenced a little bit of https://github.com/franzflasch/riscv_em/blob/master/src/core/core.c
						uint32_t dowrite = 1;
						switch( irmid )
						{
							case 2: //LR.W (0b00010)
								dowrite = 0;
								CSR( extraflags ) = (CSR( extraflags ) & 0x07) | (rs1<<3);
								break;
							case 3:  //SC.W (0b00011) (Make sure we have a slot, and, it's valid)
								rval = ( CSR( extraflags ) >> 3 != ( rs1 & 0x1fffffffffffffff ) );  // Validate that our reservation slot is OK.
								dowrite = !rval; // Only write if slot is valid.
								break;
							case 1: break; //AMOSWAP.W (0b00001)
							case 0: rs2 += rval; printf("here add\n"); break; //AMOADD.W (0b00000)
							case 4: rs2 ^= rval; break; //AMOXOR.W (0b00100)
							case 12: rs2 &= rval; break; //AMOAND.W (0b01100)
							case 8: rs2 |= rval; break; //AMOOR.W (0b01000)
							case 16: rs2 = ((int32_t)rs2<(int32_t)rval)?rs2:rval; break; //AMOMIN.W (0b10000)
							case 20: rs2 = ((int32_t)rs2>(int32_t)rval)?rs2:rval; break; //AMOMAX.W (0b10100)
							case 24: rs2 = (rs2<rval)?rs2:rval; break; //AMOMINU.W (0b11000)
							case 28: rs2 = (rs2>rval)?rs2:rval; break; //AMOMAXU.W (0b11100)
							default: trap = (2+1); dowrite = 0; break; //Not supported.
						}
						if( dowrite ) MINIRV32_STORE4( rs1, rs2 );
					}
					break;
				} else {
					uint64_t rs1 = REG((ir >> 15) & 0x1f);
					uint64_t rs2 = REG((ir >> 20) & 0x1f);
					uint32_t irmid = ( ir>>27 ) & 0x1f;

					rs1 -= MINIRV32_RAM_IMAGE_OFFSET;

					// We don't implement load/store from UART or CLNT with RV32A here.

					if( rs1 >= MINI_RV32_RAM_SIZE-3 )
					{
						trap = (7+1); //Store/AMO access fault
						rval = rs1 + MINIRV32_RAM_IMAGE_OFFSET;
					}
					else
					{
						rval = MINIRV32_LOAD8( rs1 );

						// Referenced a little bit of https://github.com/franzflasch/riscv_em/blob/master/src/core/core.c
						uint32_t dowrite = 1;
						switch( irmid )
						{
							case 2: //LR.W (0b00010)
								dowrite = 0;
								CSR( extraflags ) = (CSR( extraflags ) & 0x07) | (rs1<<3);
								break;
							case 3:  //SC.W (0b00011) (Make sure we have a slot, and, it's valid)
								rval = ( CSR( extraflags ) >> 3 != ( rs1 & 0x1fffffffffffffff ) );  // Validate that our reservation slot is OK.
								dowrite = !rval; // Only write if slot is valid.
								break;
							case 1: break; //AMOSWAP.W (0b00001)
							case 0: rs2 += rval; break; //AMOADD.W (0b00000)
							case 4: rs2 ^= rval; break; //AMOXOR.W (0b00100)
							case 12: rs2 &= rval; break; //AMOAND.W (0b01100)
							case 8: rs2 |= rval; break; //AMOOR.W (0b01000)
							case 16: rs2 = ((int64_t)rs2<(int64_t)rval)?rs2:rval; break; //AMOMIN.W (0b10000)
							case 20: rs2 = ((int64_t)rs2>(int64_t)rval)?rs2:rval; break; //AMOMAX.W (0b10100)
							case 24: rs2 = (rs2<rval)?rs2:rval; break; //AMOMINU.W (0b11000)
							case 28: rs2 = (rs2>rval)?rs2:rval; break; //AMOMAXU.W (0b11100)
							default: trap = (2+1); dowrite = 0; break; //Not supported.
						}
						if( dowrite ) MINIRV32_STORE8( rs1, rs2 );
					}
					break;
				} }
				
				default: trap = (2+1); // Fault: Invalid opcode.
			}

			// If there was a trap, do NOT allow register writeback.
			if( trap )
				break;

			if( rdid )
			{
				REGSET( rdid, rval ); // Write back register.
			}
		}

		MINIRV32_POSTEXEC( pc, ir, trap );

		pc += 4;
	}

	// Handle traps and interrupts.
	if( trap )
	{
		if( trap & (1UL << 63) ) // If prefixed with 1 in MSB, it's an interrupt, not a trap.
		{
			SETCSR( mcause, trap );
			SETCSR( mtval, 0 );
			pc += 4; // PC needs to point to where the PC will return to.
		}
		else
		{
			SETCSR( mcause,  trap - 1 );
			SETCSR( mtval, (trap > 5 && trap <= 8)? rval : pc );
		}
		SETCSR( mepc, pc ); //TRICKY: The kernel advances mepc automatically.
		//CSR( mstatus ) & 8 = MIE, & 0x80 = MPIE
		// On an interrupt, the system moves current MIE into MPIE
		SETCSR( mstatus, (( CSR( mstatus ) & 0x08) << 4) | (( CSR( extraflags ) & 3 ) << 11) );
		pc = (CSR( mtvec ) - 4);

		// If trapping, always enter machine mode.
		CSR( extraflags ) |= 3;

		trap = 0;
		pc += 4;
	}

	//if( CSR( cycle ) > cycle ) CSR( cycleh )++;
	SETCSR( cycle , cycle );
	SETCSR( pc, pc );
	return 0;
}

#endif

#endif


