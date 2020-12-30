#ifndef _SERVER_CONFIG_H_
#define _SERVER_CONFIG_H_

#include <stdint.h>

// Memory map
#ifdef __arm__
static const uint32_t SLCR_OFFSET = 0xf8000000,
	FLOCRA_REG_OFFSET = 0xffffffff, // ocra_grad_ctrl core's main offset (TODO: fill this in with Vivado value once it's ready)
	FLOCRA_MEM_OFFSET = FLOCRA_REG_OFFSET + 262144; // 256KiB offset inside the core to access the BRAMs

#else // emulate the memory in a single file on the desktop, for the purposes of debugging, emulation etc
static const unsigned EMU_PAGESIZE = 0x1000; // 4 KiB, page size on the RP and my desktop machine
static const uint32_t SLCR_OFFSET = 0, // 1 page in size
	FLOCRA_REG_OFFSET = SLCR_OFFSET + EMU_PAGESIZE,
	FLOCRA_MEM_OFFSET = FLOCRA_REG_OFFSET + 64 * EMU_PAGESIZE,
	END_OFFSET = FLOCRA_MEM_OFFSET + 64 * EMU_PAGESIZE;
#endif

// Auxiliary parameters
static const unsigned COMMS_BUFFER = 8192; // TODO expand if necessary

#endif
