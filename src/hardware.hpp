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
	FLOCRA_REG_SIZE = 64 * PAGESIZE,
	FLOCRA_MEM_SIZE = 64 * PAGESIZE;

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

	/// @brief Halt and reset all outputs to default values, even
	/// if the cores are currently running. Activated when an
	/// emergency stop command arrives.
	void halt_and_reset();
private:
	// Config variables
	unsigned _read_tries_limit = 100000; // retry attempts for each data sample
	unsigned _halt_tries_limit = 100000; // retry attemps for HALT state at the end of the sequence
	unsigned _samples_per_halt_check = 2; // how often to check halt status (in read samples) during normal readout
	
	// Peripheral register addresses in PL
	volatile uint32_t *_slcr, *_flo_regs, *_flo_mem, *_ctrl, *_direct, *_exec, *_status,
		*_status_latch, *_err, *_buf_full, *_rx_locs, *_rx0_data, *_rx1_data;

	// methods to support simulation; most efficient to inline them
	inline void wr32(volatile uint32_t *addr, uint32_t data);
	inline uint32_t rd32(volatile uint32_t *addr);
	inline size_t hw_mpack_node_copy_data(mpack_node_t node, volatile char *buffer, size_t bufsize);
};

#endif
