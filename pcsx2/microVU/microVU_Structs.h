/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2010  PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 *
 *  PCSX2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with PCSX2.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "microVU_Types.h"

#include <deque>
#include <vector>
#include <cstring>

class microBlockManager;

struct microBlockLink
{
	microBlock block;
	microBlockLink* next;
};

struct microBlockLinkRef
{
	microBlock* pBlock;
	u64 quick;
};

struct microRange
{
	s32 start; // Start PC (The opcode the block starts at)
	s32 end;   // End PC   (The opcode the block ends with)
};

#define mProgSize (0x4000 / 4)
struct microProgram
{
	u32                data [mProgSize];     // Holds a copy of the VU microProgram
	microBlockManager* block[mProgSize / 2]; // Array of Block Managers
	std::deque<microRange>* ranges;          // The ranges of the microProgram that have already been recompiled
	u32 startPC; // Start PC of this program
	int idx;     // Program index
};

typedef std::deque<microProgram*> microProgramList;

struct microProgramQuick
{
	microBlockManager* block; // Quick reference to valid microBlockManager for current startPC
	microProgram*      prog;  // The microProgram who is the owner of 'block'
};

struct microProgManager
{
	microIR<mProgSize> IRinfo;             // IR information
	microProgramList*  prog [mProgSize/2]; // List of microPrograms indexed by startPC values
	microProgramQuick  quick[mProgSize/2]; // Quick reference to valid microPrograms for current execution
	microProgram*      cur;                // Pointer to currently running MicroProgram
	int                total;              // Total Number of valid MicroPrograms
	int                isSame;             // Current cached microProgram is Exact Same program as vuRegs[mVU.index].Micro (-1 = unknown, 0 = No, 1 = Yes)
	int                cleared;            // Micro Program is Indeterminate so must be searched for (and if no matches are found then recompile a new one)
	u32                curFrame;           // Frame Counter
	u8*                codePtr;            // Pointer to program's recompilation code
	u8*                codeStart;          // Start of program's rec-cache
	u8*                codeEnd;            // Limit of program's rec-cache
	microRegInfo       lpState;            // Pipeline state from where program left off (useful for continuing execution)
};

static const uint mVUdispCacheSize = __pagesize; // Dispatcher Cache Size (in bytes)
static const uint mVUcacheSafeZone =  3; // Safe-Zone for program recompilation (in megabytes)
static const uint mVUcacheReserve = 64; // mVU0, mVU1 Reserve Cache Size (in megabytes)
