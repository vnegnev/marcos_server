#include "hardware.hpp"
#include "iface.hpp"
#include <fcntl.h>
#include <sys/mman.h>

hardware::hardware() {
	grad_offset.grad_x = 0;
	grad_offset.grad_y = 0;
	grad_offset.grad_z = 0;

#ifdef __arm__	
	init();
#endif
}

hardware::~hardware() {
}

int hardware::run_request(server_action &sa) {
	// See whether HW needs to be reconfigured
	int commands = sa.command_count();
	auto wr = sa.get_writer();
	
	mpack_start_map(wr, commands); // each command should fill in an element of the map
	auto cfg = sa.get_command("configure_hw");
	if (not mpack_node_is_missing(cfg)) {
		mpack_write_cstr(wr, "configure_hw");
		configure_hw(cfg);
		commands -= 1;		
		mpack_write(wr, 0); // status of configure_hw: 0
	}

	// Test client-server throughput after the main hardware stuff is done
	auto tln = sa.get_command("test_throughput");
	if (not mpack_node_is_missing(tln)) {
		mpack_write_cstr(wr, "test_throughput");
		
		unsigned data_size = mpack_node_uint(tln);

		mpack_start_map(wr, 2); // Two elements in map
		mpack_write_cstr(wr, "array1");
		mpack_start_array(wr, data_size);
		for (int k{0}; k<data_size; ++k) mpack_write(wr, 1.01*k); // generic, needs C11
		mpack_finish_array(wr);

		mpack_write_cstr(wr, "array2");
		mpack_start_array(wr, data_size);
		for (int k{0}; k<data_size; ++k) mpack_write(wr, 1.01*(k+10)); // generic, needs C11
		mpack_finish_array(wr);
		
		mpack_finish_map(wr);
		commands--;
	}
	mpack_finish_map(wr);
	
	if (commands != 0) {
		// Fill in remaining elements of the response map (TODO: maybe make this more sophisticated?)
		while (commands != 0) {
			char t[100];
			sprintf(t, "UNKNOWN%d", commands);
			mpack_write_cstr(wr, t);
			mpack_write(wr, 0);
			commands--;
			
		}
		sa.add_error("not all client commands were understood");
	}
}

void hardware::init() {
	int fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		throw hw_error("failed to access memory device - check sudo permissions and/or platform (must be run on RP)");
	}

	// set up shared memory (please refer to the memory offset table)

	// VN: I'm not sure why in the original server, different data
	// types were used for some of these. Perhaps to allow
	// different access widths?
	slcr = (uint32_t *) mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, SLCR_OFFSET);
	cfg = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, CFG_OFFSET);
	sts = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, STS_OFFSET);
	rx_data = (uint64_t *) mmap(NULL, 16*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, RX_DATA_OFFSET);
	tx_data = mmap(NULL, 16*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, TX_DATA_OFFSET);
	pulseq_memory = (uint32_t *) mmap(NULL, 16*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, PULSEQ_MEMORY_OFFSET);
	seq_config = (uint32_t *) mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, SEQ_CONFIG_OFFSET);  

	/*
	  NOTE: The block RAM can only be addressed with 32 bit transactions, so gradient_memory needs to
	  be of type uint32_t. The HDL would have to be changed to an 8-bit interface to support per
	  byte transactions
	*/
	grad_mem_x = (uint32_t *) mmap(NULL, 2*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, GRADIENT_MEMORY_X_OFFSET);
	grad_mem_y = (uint32_t *) mmap(NULL, 2*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, GRADIENT_MEMORY_Y_OFFSET);
	grad_mem_z = (uint32_t *) mmap(NULL, 2*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, GRADIENT_MEMORY_Z_OFFSET);

	// Map the control registers
	//rx_rst = ((uint8_t *)(cfg + 0));	
	tx_divider = ((uint32_t *)(cfg + 0));
	rx_freq = ((uint32_t *)(cfg + 4));

	// DEBUGGING: TEST OF VOID POINTER ARITHMETIC
	auto rx_freq2 = (uint32_t *)cfg + 1;
	printf("rx freq, rx_freq2: 0x%x, 0x%x;", rx_freq, rx_freq2);
	
	rx_rate = ((uint32_t *)(cfg + 8));
	rx_cntr = ((uint16_t *)(sts + 0));
	//tx_rst = ((uint8_t *)(cfg + 1));
	tx_size = ((uint16_t *)(cfg + 12));
}

void hardware::configure_hw(mpack_node_t &cfg) {
	// Old comment: set FPGA clock to 143 MHz (VN: not sure how this works - probably 122 MHz in RP-122?)
	auto cw1 = mpack_node_map_cstr_optional(cfg, "fpga_clk_word1");
	uint32_t fw1 = mpack_node_is_missing(cw1) ? 0xDF0D : mpack_node_uint(cw1);

	auto cw2 = mpack_node_map_cstr_optional(cfg, "fpga_clk_word2");
	uint32_t fw2 = mpack_node_is_missing(cw2) ? 0x03f03f30 : mpack_node_uint(cw2);

	auto cw3 = mpack_node_map_cstr_optional(cfg, "fpga_clk_word3");
	uint32_t fw3 = mpack_node_is_missing(cw3) ? 0x00100700 : mpack_node_uint(cw3);
		
	slcr[2] = fw1;
	slcr[92] = (slcr[92] & ~fw2) | fw3;

	// Old comment: erase pulse sequence memory
	for (int i=0; i<32; ++i) pulseq_memory[i] = 0x0;

	// Old comment: halt the microsequencer
	seq_config[0] = 0x00;

	// Old comment: set the NCO to 15.67 MHz
}
