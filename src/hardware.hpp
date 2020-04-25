/* Hardware management and interface/data transfer to the Zynq PL */

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

	int run_request(server_action &sa); // 
	void init(); // set up shared memory, control registers etc; these aspects are not client-configurable
	void configure_hw(mpack_node_t &cfg); // set up control registers
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
