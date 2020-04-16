extern "C" {	
// Linux-related
#include <unistd.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
}

#include "version.hpp"
#include "hardware.hpp"
#include "iface.hpp"

#include <iostream>
#include <sstream>

unsigned SERVER_VERSION_UINT;
std::string SERVER_VERSION_STR;

hardware *hw;
iface *ifa;

int main(int argc, char *argv[]) {
	std::cout << "MaRCoS server, " << __DATE__ << " " << __TIME__ << std::endl;

	// Global version string creation
	std::stringstream sv;
	sv << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_DEBUG;
	SERVER_VERSION_UINT = ((VERSION_MAJOR << 16) & 0xff0000) | ((VERSION_MINOR << 8) & 0xff00) | (VERSION_DEBUG & 0xff);
	SERVER_VERSION_STR = sv.str();
	
	std::cout << "Version " << SERVER_VERSION_STR << std::endl;

	hw = new hardware();
	ifa = new iface();
	// while (true) ifa->run_stream();
	ifa->run_stream();

	// Peripherals in PL
	// VNTODO: why are some peripherals declared as voids, some as volatile ints?
	void *cfg, *sts;
	volatile uint32_t *slcr, *rx_freq, *rx_rate, *seq_config, *pulseq_memory, *tx_divider;
	volatile uint16_t *rx_cntr, *tx_size;

	volatile uint64_t *rx_data;
	void *tx_data;

	uint32_t command;

	// Cleanup
	delete hw;
	delete ifa;
}
