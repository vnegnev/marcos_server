#include "version.h"
#include "hardware.hpp"
#include "iface.hpp"

#include <iostream>
#include <unistd.h>

// Linux-related
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

hardware *hw;
iface *ifa;

int main(int argc, char *argv[]) {
	std::cout << "MaRCoS server, " << __DATE__ << " " << __TIME__ << std::endl;
	std::cout << "Version " << VERSION_MAJOR << "." << VERSION_MINOR << std::endl;

	hw = new hardware();
	ifa = new iface();

	int sock_server, sock_client;

	// Peripherals in PL
	// VNTODO: why are some peripherals declared as voids, some as volatile ints?
	void *cfg, *sts;
	volatile uint32_t *slcr, *rx_freq, *rx_rate, *seq_config, *pulseq_memory, *tx_divider;
	volatile uint16_t *rx_cntr, *tx_size;

	volatile uint64_t *rx_data;
	void *tx_data;

	struct sockaddr_in addr;
	uint32_t command;

	// Cleanup
	delete hw;
	delete ifa;
}
