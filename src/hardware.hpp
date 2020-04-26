/** @file hardware.hpp
    @brief Hardware management and interface/data transfer to the Zynq PL.
*/

#ifndef _HARDWARE_HPP_
#define _HARDWARE_HPP_

#include <inttypes.h> // TODO is this the right include?

struct mpack_node_t;

typedef struct {
	float grad_x, grad_y, grad_z;
} grad_offset_t;

typedef enum {
	GRAD_ZERO_DISABLED_OUTPUT = 0,
	GRAD_ZERO_ENABLED_OUTPUT,
	GRAD_OFFSET_ENABLED_OUTPUT
} grad_state_t;

class server_action;

class hardware {
public:
	hardware();
	~hardware();

	void run_request(server_action &sa); //

        /// @brief Set up shared memory, control registers etc; these
        /// aspects are not client-configurable. If compiled on x86,
        /// just mimics the shared memory.
	void init_mem();
	/// @brief Compute and write the default pulses to TX
	/// memory. Duration in microseconds, RF amplitude in percent
	/// of full-scale.  Optional pointer to server_action for info
	/// messages.
	void compute_pulses(double duration=10.0, double amp=50.0, server_action *sa=nullptr);
	unsigned configure_hw(mpack_node_t &cfg, server_action &sa); // set up control registers
private:
	// Peripherals in PL
	// VNTODO: why are some peripherals declared as voids, some as volatile ints?
	void *cfg, *sts;
	volatile uint32_t *slcr, *rx_freq, *rx_rate, *seq_config, *pulseq_memory, *tx_divider;
	volatile uint32_t *grad_mem_x, *grad_mem_y, *grad_mem_z;
	grad_offset_t grad_offset;
	volatile uint16_t *rx_cntr, *tx_size;

	volatile uint64_t *rx_data;
	void *tx_data;	
};

#endif
