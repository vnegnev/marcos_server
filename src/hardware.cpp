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
	compute_pulses();
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
		size_t hw_commands_executed = 0;
		hw_commands_executed = configure_hw(cfg, sa);
		mpack_write(wr, hw_commands_executed);
		if (mpack_node_map_count(cfg) > hw_commands_executed) {
			sa.add_error("not all configure_hw commands were successfully executed");
		}
		commands -= 1;
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
		if (commands > 0) sa.add_error("not all client commands were understood");
		else {
			sa.add_error("check the request message format");
			commands = 0;
		}
		
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

void hardware::compute_pulses(double duration, double rf_amp, server_action *sa) {
	// Old comment: local oscillator for the excitation pulse
	
	// VN: I think this could be done more easily without copying
	// and just directly modifying the memory-mapped data, but
	// it's not worth investigating right now. TODO: optimise the
	// code a bit by doing something like
	// 
	// int16_t pulse = tx_data;
	// 
	// and then avoiding any need for copying etc. I suspect
	// originally this was a quick way to fix some kind of
	// endianness bug.

	if (rf_amp > 100) throw data_error("rf amplitude over 100.0");
	if (rf_amp < 0) throw data_error("rf amplitude below 0.0");

	int16_t pulse[32768];
	for (int i=0; i < 32768; ++i) pulse[i] = 0;

	uint32_t offset_gap = 1000, memory_gap = 2*offset_gap;

	// compute relevant pulse properties based on desired duration
	unsigned tx_samples = duration * FPGA_CLK_FREQ_HZ / (*tx_divider * 1e6);
	if (sa != nullptr) {
		char s[100];
		sprintf(s, "a %f us pulse will need %d samples", duration, tx_samples);
		sa->add_info(s);
	}

	if (tx_samples > 249) throw data_error("TX samples required is too high; reduce the pulse duration or increase the TX divider");
	if (tx_samples < 1) throw data_error("less than 1 TX sample required; check your settings");

	uint16_t rf_amp_u = (uint16_t) round(rf_amp / 100.0 * 65535);
	
	/* Below block is copied and pasted from the original server code. */
	// RF Pulse 0: RF:90x+ offset 0
	for(int i = 0; i <= 2*tx_samples; i=i+2) {
		pulse[i] = rf_amp_u;
	}

	// RF Pulse 1: RF:180x+ offset 1000 in 32 bit space
	// this is a hard pulse with double the duration of the hard 90
	for(int i = 1*memory_gap; i <= 1*memory_gap+(2*tx_samples)*2; i=i+2) {
		pulse[i] = rf_amp_u;
	}

	// RF Pulse 2: RF:180y+ offset 2000 in 32 bit space
	for(int i = 2*memory_gap; i <= 2*memory_gap+(2*tx_samples)*2; i=i+2) {
		pulse[i+1] = rf_amp_u;
	}

	// RF Pulse 3: RF:180y- offset 3000 in 32 bit space
	for(int i = 3*memory_gap; i <= 3*memory_gap+(2*tx_samples)*2; i=i+2) {
		pulse[i+1] = -rf_amp_u;
	}

	// RF Pulse 4: RF:180x+ offset 4000 in 32 bit space
	// this is a hard 180 created by doubling the amplitude of the hard 90
	for(int i = 4*memory_gap; i <= 4*memory_gap+(2*tx_samples); i=i+2) {
		pulse[i] = 2*rf_amp_u;
	}

	// RF Pulse 5: SINC PULSE
	for(int i = 5*memory_gap; i <= 5*memory_gap+512; i=i+2) {
		int j = (int)((i - (5*memory_gap+64)) / 2) - 128;
		pulse[i] = (int16_t) floor(48*rf_amp_u*(0.54 + 0.46*(cos((M_PI*j)/(2*48)))) * sin((M_PI*j)/(48))/(M_PI*j)); 
	}
	pulse[5*memory_gap+64+256] = rf_amp_u;

	// RF Pulse 6: SIN PULSE
	for(int i = 6*memory_gap; i <= 6*memory_gap+512; i=i+2) {
		pulse[i] = (int16_t) floor(rf_amp_u * sin((M_PI*i)/(128)));
	}

	auto size = 32768-1;
	*tx_size = size;
	memset(tx_data, 0, 65536);
	memcpy(tx_data, pulse, 2 * size);	
}

unsigned hardware::configure_hw(mpack_node_t &cfg, server_action &sa) {
	unsigned commands_executed = 0;
	// Enforce that all three words are present for FPGA clock configuration
	auto fcwa1 = mpack_node_map_cstr_optional(cfg, "fpga_clk");
	
	if (not mpack_node_is_missing(fcwa1)) {
		if (mpack_node_array_length(fcwa1) != 3) {
			sa.add_error("you only provided some FPGA clock control words; check you're providing all 3");
		} else {
			slcr[2] = mpack_node_uint(mpack_node_array_at(fcwa1, 0));
			slcr[92] = (slcr[92] & ~mpack_node_uint(mpack_node_array_at(fcwa1, 1)) )
				| mpack_node_uint(mpack_node_array_at(fcwa1, 2));
			++commands_executed;
		}
	}

	// RX frequency (please pre-calculate the 32-bit int on the client)
	auto rxfn = mpack_node_map_cstr_optional(cfg, "rx_freq");
	if (not mpack_node_is_missing(rxfn)) {
		auto freq = mpack_node_uint(rxfn);

		double true_freq = ( static_cast<double>(freq) * FPGA_CLK_FREQ_HZ / (1 << 30) / 1e6);
		if (true_freq < 0.000001 or true_freq > 100) {
			sa.add_error("RX frequency outside the range [0.000001, 100] MHz");
		} else {
			*rx_freq = freq;
		}
		char t[100];
		sprintf(t, "true RX freq: %f MHz", true_freq);
		sa.add_info(t);
		++commands_executed;
	}

	// TX divider
	auto txdn = mpack_node_map_cstr_optional(cfg, "tx_divider");
	if (not mpack_node_is_missing(txdn)) {
		*tx_divider = mpack_node_uint(txdn);
		if (*tx_divider < 1 or *tx_divider > 1000) {
			sa.add_warning("TX divider outside the range [1, 1000]; make sure this is what you want");
		}
		char t[100];
		sprintf(t, "TX sample duration: %f us", *tx_divider * 1e6 / FPGA_CLK_FREQ_HZ);		
		sa.add_info(t);
		++commands_executed;
	}

	// Recompute pulses
	auto rpdn = mpack_node_map_cstr_optional(cfg, "recompute_pulses");
	if (not mpack_node_is_missing(rpdn)) {
		// Mandatory pulse properties
		if (mpack_node_map_count(rpdn) != 2) {
			sa.add_error("improper number of recompute_pulses arguments");
		} else {
			double duration = mpack_node_double(mpack_node_map_cstr(rpdn, "duration"));
			double rf_amp = mpack_node_double(mpack_node_map_cstr(rpdn, "rf_amp"));
			try {
				compute_pulses(duration, rf_amp, &sa);
				++commands_executed;
			} catch (marcos_error &e) {
				sa.add_error(e.what());
			}
		}
	}

	return commands_executed;
}
