#include "hardware.hpp"
#include "iface.hpp"
#include <fcntl.h>
#include <cmath>
#include <sys/mman.h>

hardware::hardware() {
	grad_offset.grad_x = 0;
	grad_offset.grad_y = 0;
	grad_offset.grad_z = 0;
	
	init_mem();
	init_pulses();
}

hardware::~hardware() {
}

void hardware::run_request(server_action &sa) {
	// See whether HW needs to be reconfigured
	int commands = sa.command_count();
	auto wr = sa.get_writer();
	
	mpack_start_map(wr, commands); // each command should fill in an element of the map
	auto cfg = sa.get_command("configure_hw");
	if (not mpack_node_is_missing(cfg)) {
		mpack_write_cstr(wr, "configure_hw");
		configure_hw(cfg, sa);
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
		sa.add_error("not all client commands were understood");
		
		// Fill in remaining elements of the response map (TODO: maybe make this more sophisticated?)
		while (commands != 0) {
			char t[100];
			sprintf(t, "UNKNOWN%d", commands);
			mpack_write_cstr(wr, t);
			mpack_write(wr, 0);
			commands--;
			
		}
	}
}

void hardware::init_mem() {
#ifdef __arm__
	int fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		throw hw_error("failed to access memory device - check sudo permissions and/or platform");
	}
#else
	char tempfile[100] = "/tmp/marcos_server_mem";
	
	int fd = open(tempfile, O_RDWR);
	if (fd < 0) {
		char errstr[1024];
		size_t filesize_KiB = 4 * (GRADIENT_MEMORY_Z_OFFSET + 2 * EMU_PAGESIZE) / EMU_PAGESIZE;
		sprintf(errstr, "Failed to open simulated memory device.\n"\
		        "Check whether %s exists, and if not create it using:\n"\
		        "fallocate -l %dKiB %s", tempfile, filesize_KiB, tempfile);
		throw hw_error(errstr);
	}
#endif

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
	tx_divider = (uint32_t *)cfg + 0;
	rx_freq = (uint32_t *)cfg + 1;
	
	rx_rate = (uint32_t *)cfg + 2;
	rx_cntr = (uint16_t *)sts + 0;
	//tx_rst = ((uint8_t *)(cfg + 1));
	tx_size = (uint16_t *)cfg + 3;

	// Fill in some default values (can be altered later by calling configure_hw() )
	// Old comment: set FPGA clock to 143 MHz (VN: not sure how this works - probably 122 MHz in RP-122?)
	slcr[2] = 0xDF0D;
	slcr[92] = (slcr[92] & ~0x03F03F30) | 0x00100700;

	// Old comment: erase pulse sequence memory
	for (int i=0; i<32; ++i) pulseq_memory[i] = 0x0;

	// Old comment: halt the microsequencer
	seq_config[0] = 0x00;

	// Old comment: set the NCO to 15.67 MHz
	*rx_freq = (uint32_t) floor(15670000 / FPGA_CLK_FREQ_HZ * (1<<30) + 0.5);

	// Old comment: set default rx sample rate
	*rx_rate = 250;

	// Old comment: fill tx buffer with zeros [DO WE EVEN NEED A TX BUFFER?]
	memset(tx_data, 0, 16 * sysconf(_SC_PAGESIZE) ); // 65536B

	// Old comment: this divider makes the sample duration a convenient 1us (VN: adjusted to be close to the tx clock freq)
	*tx_divider = (uint32_t) round(FPGA_CLK_FREQ_HZ/1e6);
}

void hardware::init_pulses(double duration, double rf_amp, server_action *sa) {
	// Old comment: local oscillator for the excitation pulse
	
	// VN: I think this could be done more easily without copying
	// and just directly modifying the memory-mapped data, but
	// I'll see whether there's a difference

	int16_t pulse[32768];
	for (int i=0; i < 32768; ++i) pulse[i] = 0;

	uint32_t offset_gap = 1000, memory_gap = 2*offset_gap;

	// compute relevant pulse properties based on desired duration
	unsigned tx_samples = duration * FPGA_CLK_FREQ_HZ / *tx_divider;
	if (sa != nullptr) {
		char info[100];
		sprintf(info, "a %f us pulse will need %d samples", duration, tx_samples);
		sa->add_info(info);
	}
}

void hardware::configure_hw(mpack_node_t &cfg, server_action &sa) {
	// Enforce that all three words are present for FPGA clock configuration
	auto cw1 = mpack_node_map_cstr_optional(cfg, "fpga_clk_word1");
	auto cw2 = mpack_node_map_cstr_optional(cfg, "fpga_clk_word2");
	auto cw3 = mpack_node_map_cstr_optional(cfg, "fpga_clk_word3");
	
	if (not mpack_node_is_missing(cw1) and not mpack_node_is_missing(cw2) and not mpack_node_is_missing(cw3) ) {
		slcr[2] = mpack_node_uint(cw1);
		slcr[92] = (slcr[92] & ~mpack_node_uint(cw2)) | mpack_node_uint(cw3);
	} else {
		sa.add_error("you only provided some FPGA clock control words; check you're providing fpga_clk_word1,"
		             "fpga_clk_word2 and fpga_clk_word3");
	}

	// RX frequency (please pre-calculate the 32-bit int on the client)
	auto rxfn = mpack_node_map_cstr_optional(cfg, "rx_freq");
	if (not mpack_node_is_missing(rxfn)) {
		*rx_freq = mpack_node_uint(rxfn);

		double true_freq = ( static_cast<double>(*rx_freq) * FPGA_CLK_FREQ_HZ / (1 << 30) / 1e6);
		char t[100];
		sprintf(t, "true RX freq: %f MHz", true_freq);
		sa.add_info(t);
	}

	// TX divider
	auto txdn = mpack_node_map_cstr_optional(cfg, "tx_divider");
	if (not mpack_node_is_missing(txdn)) {
		*tx_divider = mpack_node_uint(txdn);
		if (*tx_divider < 1 or *tx_divider > 1000) sa.add_warning("TX divider outside the range [1, 1000]; make sure this is what you want");
		char t[100];
		sprintf(t, "TX sample duration: %f us", *tx_divider / FPGA_CLK_FREQ_HZ);		
		sa.add_info(t);
	}
}
