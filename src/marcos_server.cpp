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
	std::cout << "FPGA clock: " << FPGA_CLK_FREQ_HZ/1e6 << " MHz" << std::endl;
	if (FPGA_CLK_FREQ_HZ == 125e6) std::cout << "Red Pitaya 125-14" << std::endl;
	else if (FPGA_CLK_FREQ_HZ == 122.88e6) std::cout << "StemLAB 122.88-16" << std::endl;
	else std::cout << "UNKNOWN DEVICE" << std::endl;

	// Global version string creation
	std::stringstream sv;
	sv << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_DEBUG;
	SERVER_VERSION_UINT = ((VERSION_MAJOR << 16) & 0xff0000) | ((VERSION_MINOR << 8) & 0xff00) | (VERSION_DEBUG & 0xff);
	SERVER_VERSION_STR = sv.str();
	
	std::cout << "Server version " << SERVER_VERSION_STR << std::endl;

	hw = new hardware();
	ifa = new iface();
	// while (true) ifa->run_stream();
	ifa->run_stream();

	// Cleanup
	delete hw;
	delete ifa;
}
