/* Hardware management and interface/data transfer to the Zynq PL */

#ifndef _HARDWARE_HPP_
#define _HARDWARE_HPP_

#include <inttypes.h> // TODO is this the right include?

typedef struct {
	float gradient_x, gradient_y, gradient_z;
} gradient_offset_t;

typedef enum {
	GRAD_ZERO_DISABLED_OUTPUT = 0,
	GRAD_ZERO_ENABLED_OUTPUT,
	GRAD_OFFSET_ENABLED_OUTPUT
} gradient_state_t;

class hardware {
public:
	hardware();
	~hardware();
private:
	int fd; // file for /dev/mem

	// Peripherals in PL
	// VNTODO: why are some peripherals declared as voids, some as volatile ints?
	void *cfg, *sts;
	volatile uint32_t *slcr, *rx_freq, *rx_rate, *seq_config, *pulseq_memory, *tx_divider;
	volatile uint32_t *gradient_memory_x, *gradient_memory_y, *gradient_memory_z;
	gradient_offset_t gradient_offset;
	volatile uint16_t *rx_cntr, *tx_size;

	volatile uint64_t *rx_data;
	void *tx_data;	
};

#endif
