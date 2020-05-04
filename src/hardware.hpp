/** @file hardware.hpp
    @brief Hardware management and interface/data transfer to the Zynq PL.
*/

#ifndef _HARDWARE_HPP_
#define _HARDWARE_HPP_

#include <inttypes.h> // TODO is this the right include?

struct mpack_node_t;

// typedef struct {
// 	float grad_x, grad_y, grad_z;
// } grad_offset_t;

typedef enum {
	GRAD_ZERO_DISABLED_OUTPUT = 0,
	GRAD_ZERO_ENABLED_OUTPUT,
	GRAD_OFFSET_ENABLED_OUTPUT
} grad_state_t;

typedef enum {
	GRAD_OFFSET_X,
	GRAD_OFFSET_Y,
	GRAD_OFFSET_Z,
	GRAD_OFFSET_Z2
} grad_offset_t;

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
	/// @brief Compute and write the default pulses to TX
	/// memory. Uses the member variables _rf_amp and _rf_tx_samples.
	void compute_pulses();

	// OBSOLETE
	// unsigned configure_hw(mpack_node_t &cfg, server_action &sa); // set up control registers
private:
	// Peripherals in PL
	// VNTODO: why are some peripherals declared as voids, some as volatile ints?
	void *_cfg, *_sts;
	volatile uint32_t *_slcr, *_rx_freq, *_rx_rate, *_seq_config, *_pulseq_memory, *_tx_divider;
	volatile uint32_t *_grad_mem_x, *_grad_mem_y, *_grad_mem_z;
	grad_offset_t _grad_offset;
	volatile uint16_t *_rx_cntr, *_tx_size;

	volatile uint64_t *_rx_data;
	void *_tx_data;

	uint16_t _rf_amp;
	uint32_t _tx_samples;

	// Set the gradient offsets. idx corresponds to 0 = x, 1 = y, 2 = z, 3 = z2.
	// 
	// enable_output is to match the old server's functionality;
	// unclear what it would be used for in practice (perhaps
	// emergency stops?).
	int set_gradient_offset(int32_t offset, int idx, bool clear_mem=true, bool enable_output=true);
};

#endif
