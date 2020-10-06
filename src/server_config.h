#ifndef _SERVER_CONFIG_H_
#define _SERVER_CONFIG_H_

#include <stdint.h>

// Memory map
#ifdef __arm__
static const uint32_t SLCR_OFFSET = 0xf8000000,
	CFG_OFFSET = 0x40000000,
	STS_OFFSET = 0x40001000,
	RX_DATA_OFFSET = 0x40010000,
	TX_DATA_OFFSET = 0x40020000,
	MICRO_SEQ_MEMORY_OFFSET = 0x40030000,
	MICRO_SEQ_CONFIG_OFFSET = 0x40040000,
	ATTEN_OFFSET = 0x40050000,
	SPI_SEQ_OFFSET = 0x40060000,
	GRAD_CTRL_REG_OFFSET = 0x40100000, // ocra_grad_ctrl core's main offset
	GRAD_CTRL_MEM_OFFSET = GRAD_CTRL_REG_OFFSET + 32768; // 32KiB offset inside the core to access the BRAMs

#else // emulate the memory in a single file on the desktop, for the purposes of debugging, emulation etc
static const unsigned EMU_PAGESIZE = 0x1000; // 4 KiB, page size on the RP and my desktop machine
static const uint32_t SLCR_OFFSET = EMU_PAGESIZE, // 1 page in size
	CFG_OFFSET = SLCR_OFFSET + EMU_PAGESIZE, // 1 page after SLCR_OFFSET, 1 page in size
	STS_OFFSET = CFG_OFFSET + EMU_PAGESIZE, // 1 page after CFG_OFFSET, 1 page in size
	RX_DATA_OFFSET = STS_OFFSET + 16 * EMU_PAGESIZE,
	TX_DATA_OFFSET = RX_DATA_OFFSET + 16 * EMU_PAGESIZE,
	MICRO_SEQ_MEMORY_OFFSET = TX_DATA_OFFSET + 16 * EMU_PAGESIZE,
	MICRO_SEQ_CONFIG_OFFSET = MICRO_SEQ_MEMORY_OFFSET + 16 * EMU_PAGESIZE,
	ATTEN_OFFSET = MICRO_SEQ_CONFIG_OFFSET + 16 * EMU_PAGESIZE,
	SPI_SEQ_OFFSET = ATTEN_OFFSET + 16 * EMU_PAGESIZE,
	GRAD_CTRL_REG_OFFSET = SPI_SEQ_OFFSET + EMU_PAGESIZE,
	GRAD_CTRL_MEM_OFFSET = GRAD_CTRL_REG_OFFSET + EMU_PAGESIZE,
	END_OFFSET = GRAD_CTRL_MEM_OFFSET + 64 * EMU_PAGESIZE;
#endif

// FPGA clock frequency (HZ)
// todo make this remotely set (based on RP version)
static const double FPGA_CLK_FREQ_HZ = 125e6;

// Auxiliary parameters
static const unsigned PULSE_MEM_SIZE = 32768, COMMS_BUFFER = 8192;

#endif
