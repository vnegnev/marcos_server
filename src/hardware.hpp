/** @file hardware.hpp
    @brief Hardware management and interface/data transfer to the Zynq PL.
*/

#ifndef _HARDWARE_HPP_
#define _HARDWARE_HPP_

#include <inttypes.h> // TODO is this the right include?
#include <unistd.h>

// Memory-mapped device sizes
static const unsigned PAGESIZE = sysconf(_SC_PAGESIZE); // should be 4096 (4KiB) on both x86_64 and ARM
static const unsigned SLCR_SIZE = PAGESIZE,
	CFG_SIZE = PAGESIZE,
	STS_SIZE = PAGESIZE,
	RX_DATA_SIZE = 16 * PAGESIZE,
	TX_DATA_SIZE = 16 * PAGESIZE,
	MICRO_SEQ_MEMORY_SIZE = 16 * PAGESIZE,
	MICRO_SEQ_CONFIG_SIZE = PAGESIZE,
	GRAD_CONFIG_SIZE = PAGESIZE,
	GRAD_MEM_SIZE = 8 * PAGESIZE;

struct mpack_node_t;

class server_action;

class hardware {
public:
	hardware();
	~hardware();

	int run_request(server_action &sa);

        /// @brief Set up shared memory, control registers etc; these
        /// aspects are not client-configurable. If compiled on x86,
        /// just mimics the shared memory.
	void init_mem();

	// OBSOLETE
	// unsigned configure_hw(mpack_node_t &cfg, server_action &sa); // set up control registers
private:
	// Peripherals in PL
	// VNTODO: why are some peripherals declared as voids, some as volatile ints?
	char *_cfg, *_sts, *_tx_data;
	volatile uint32_t *_slcr, *_lo_freq, *_rx_rate, *_micro_seq_config, *_micro_seq_memory, *_tx_divider;
	volatile uint32_t *_grad_config, *_grad_mem, *_grad_update_divider, *_grad_spi_divider;
	volatile uint16_t *_rx_cntr, *_tx_size;
	volatile uint64_t *_rx_data;
};

#endif
