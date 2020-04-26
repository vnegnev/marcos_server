#include "hardware.hpp"
#include "iface.hpp"
#include <fcntl.h>
#include <cmath>
#include <cassert>
#include <sys/mman.h>

hardware::hardware() {
	_grad_offset.grad_x = 0;
	_grad_offset.grad_y = 0;
	_grad_offset.grad_z = 0;
	
	init_mem();
	compute_pulses();
}

hardware::~hardware() {
}

int hardware::run_request(server_action &sa) {
	// See whether HW needs to be reconfigured
	size_t commands_present = sa.command_count();
	size_t commands_understood = 0;	
	auto wr = sa.get_writer();
	int problems = 0;
	int status;
	mpack_start_map(wr, commands_present); // each command should fill in an element of the map

	if (commands_present == 0) {
		++problems;
		sa.add_error("no commands present or incorrectly formatted request");
	}

	// FPGA clock config [TODO: understand this better]
	auto fcwa1 = sa.get_command_and_start_reply("fpga_clk", status);
	if (status == 1) {
		++commands_understood;
		// Enforce that all three words are present for FPGA clock configuration
		if (mpack_node_array_length(fcwa1) != 3) {
			sa.add_error("you only provided some FPGA clock control words; check you're providing all 3");
			mpack_write(wr, c_err); // error
		} else {
			_slcr[2] = mpack_node_u32(mpack_node_array_at(fcwa1, 0));
			_slcr[92] = (_slcr[92] & ~mpack_node_u32(mpack_node_array_at(fcwa1, 1)) )
				| mpack_node_u32(mpack_node_array_at(fcwa1, 2));
			mpack_write(wr, c_ok); // okay
		}
	} else if (status == -1) {
		// sa.add_error("unknown MPack error");
		// TODO: callback or similar
	}

	// RX frequency (please pre-calculate the 32-bit int on the client)
	auto rxfn = sa.get_command_and_start_reply("rx_freq", status);
	if (status == 1) {
		++commands_understood;
		auto freq = mpack_node_u32(rxfn);

		double true_freq = ( static_cast<double>(freq) * FPGA_CLK_FREQ_HZ / (1 << 30) / 1e6);
		if (true_freq < 0.000001 or true_freq > 60) {
			sa.add_error("RX frequency outside the range [0.000001, 60] MHz");
			mpack_write(wr, c_err);
		} else {
			*_rx_freq = freq;
			mpack_write(wr, c_ok);
		}
		char t[100];
		sprintf(t, "true RX freq: %f MHz", true_freq);
		sa.add_info(t);
	} // else if (status == -1) do some error handling

	// TX divider
	auto txdn = sa.get_command_and_start_reply("tx_div", status);
	if (status == 1) {
		++commands_understood;
		*_tx_divider = mpack_node_u32(txdn);
		if (*_tx_divider < 1 or *_tx_divider > 1000) {
			sa.add_warning("TX divider outside the range [1, 1000]; make sure this is what you want");
			mpack_write(wr, c_warn);
		} else mpack_write(wr, c_ok);
		char t[100];
		sprintf(t, "TX sample duration: %f us", *_tx_divider * 1e6 / FPGA_CLK_FREQ_HZ);		
		sa.add_info(t);
	} // else if (status == -1) do some error handling

	// RF amplitude (please pre-calculate the 16-bit int on the client)
	auto rfan = sa.get_command_and_start_reply("rf_amp", status);
	if (status == 1) {
		++commands_understood;
		auto rf_amp = mpack_node_u16(rfan);
		// TODO: handle the possibility of rf_amp being incorrectly read, e.g. if it's bigger than a u16
		        // if (not sa.reader_err()) {
		_rf_amp = rf_amp;
		mpack_write(wr, c_ok);
		double true_amp = _rf_amp * 100.0 / 65535;
		char t[100];
		sprintf(t, "true RF amp: %f\%", true_amp);
		sa.add_info(t);			
		// if (rf_amp_f > 100 or rf_amp_f < 0) sa.add_error("RF amplitude outside the range [0, ]")
			// } else mpack_write(wr, c_err);
	}

	// Duration of a pulse, in TX samples
	auto txsn = sa.get_command_and_start_reply("tx_samples", status);
	if (status == 1) {
		++commands_understood;
		uint32_t tx_samples = mpack_node_u32(txsn);
		if (tx_samples < 1 or tx_samples > 249) {
			sa.add_error("TX samples per pulse outside the range [1, 249]; check your settings");
			mpack_write(wr, c_err);
		} else {
			_tx_samples = tx_samples;
			mpack_write(wr, c_ok);
		}
	}

	// Recompute pulses
	auto rpn = sa.get_command_and_start_reply("recomp_pul", status);
	if (status == 1) {
		++commands_understood;
		if (mpack_node_bool(rpn)) {
			compute_pulses();
			mpack_write(wr, c_ok);
		} else {
			sa.add_warning("recomp_pul requested but set to false; doing nothing");
			mpack_write(wr, c_warn);
		}
	}

	// Fill in pulse memory directly from a binary blob
	auto rtxd = sa.get_command_and_start_reply("raw_tx_data", status);
	if (status == 1) {
		++commands_understood;
		if (mpack_node_bin_size(rtxd) <= 16 * sysconf(_SC_PAGESIZE)) {
			size_t bytes_copied = mpack_node_copy_data(rtxd, (char *)_tx_data, 16 * sysconf(_SC_PAGESIZE));
			char t[100];
			sprintf(t, "tx data bytes copied: %d", bytes_copied);
			sa.add_info(t);
			mpack_write(wr, c_ok);
		} else {
			sa.add_error("too much raw TX data");
			mpack_write(wr, c_err);
		}
	}

	

	// Test client-server throughput
	auto tln = sa.get_command_and_start_reply("test_throughput", status);
	if (status == 1) {
		++commands_understood;
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
	} else if (status == -1) {
		// TODO: callback or similar
	}

	// Final housekeeping
	mpack_finish_map(wr);
	
	if (commands_understood != commands_present) {
		assert(commands_understood <= commands_present && "Serious bug in logic");
		sa.add_error("not all client commands were understood");
		
		// Fill in remaining elements of the response map (TODO: maybe make this more sophisticated?)
		while (commands_present != 0) {
			char t[100];
			sprintf(t, "UNKNOWN%d", commands_present);
			mpack_write_kv(wr, t, -1);
			commands_present--;
		}
	}
	return problems;
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
	_slcr = (uint32_t *) mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, SLCR_OFFSET);
	_cfg = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, CFG_OFFSET);
	_sts = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, STS_OFFSET);
	_rx_data = (uint64_t *) mmap(NULL, 16*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, RX_DATA_OFFSET);
	_tx_data = mmap(NULL, 16*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, TX_DATA_OFFSET);
	_pulseq_memory = (uint32_t *) mmap(NULL, 16*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, PULSEQ_MEMORY_OFFSET);
	_seq_config = (uint32_t *) mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, SEQ_CONFIG_OFFSET);  

	/*
	  NOTE: The block RAM can only be addressed with 32 bit transactions, so gradient_memory needs to
	  be of type uint32_t. The HDL would have to be changed to an 8-bit interface to support per
	  byte transactions
	*/
	_grad_mem_x = (uint32_t *) mmap(NULL, 2*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, GRADIENT_MEMORY_X_OFFSET);
	_grad_mem_y = (uint32_t *) mmap(NULL, 2*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, GRADIENT_MEMORY_Y_OFFSET);
	_grad_mem_z = (uint32_t *) mmap(NULL, 2*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, GRADIENT_MEMORY_Z_OFFSET);
	
	// Map the control registers
	//rx_rst = ((uint8_t *)(cfg + 0));	
	_tx_divider = (uint32_t *)_cfg + 0;
	_rx_freq = (uint32_t *)_cfg + 1;
	
	_rx_rate = (uint32_t *)_cfg + 2;
	_rx_cntr = (uint16_t *)_sts + 0;
	//tx_rst = ((uint8_t *)(cfg + 1));
	_tx_size = (uint16_t *)_cfg + 3;
	
	// Fill in some default values (can be altered later by calling configure_hw() )
	// Old comment: set FPGA clock to 143 MHz (VN: not sure how this works - probably 122 MHz in RP-122?)
	_slcr[2] = 0xDF0D;
	_slcr[92] = (_slcr[92] & ~0x03F03F30) | 0x00100700;
	
	// Old comment: erase pulse sequence memory
	for (int i=0; i<32; ++i) _pulseq_memory[i] = 0x0;
	
	// Old comment: halt the microsequencer
	_seq_config[0] = 0x00;
	
	// Old comment: set the NCO to 15.67 MHz
	*_rx_freq = (uint32_t) floor(15670000 / FPGA_CLK_FREQ_HZ * (1<<30) + 0.5);
	
	// Old comment: set default rx sample rate
	*_rx_rate = 250;
	
	// Old comment: fill tx buffer with zeros [DO WE EVEN NEED A TX BUFFER?]
	memset(_tx_data, 0, 16 * sysconf(_SC_PAGESIZE) ); // 65536B
	
	// Old comment: this divider makes the sample duration a convenient 1us (VN: adjusted to be close to the tx clock freq)
	*_tx_divider = (uint32_t) round(FPGA_CLK_FREQ_HZ/1e6);
}

void hardware::compute_pulses() {
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
	
	/* Below block is copied and pasted from the original server code. */
	// RF Pulse 0: RF:90x+ offset 0
	int16_t pulse[32768];
	for (int i=0; i < 32768; ++i) pulse[i] = 0;
	
	uint32_t offset_gap = 1000, memory_gap = 2*offset_gap;
	
	for(int i = 0; i <= 2*_tx_samples; i=i+2) {
		pulse[i] = _rf_amp;
	}

	// RF Pulse 1: RF:180x+ offset 1000 in 32 bit space
	// this is a hard pulse with double the duration of the hard 90
	for(int i = 1*memory_gap; i <= 1*memory_gap+(2*_tx_samples)*2; i=i+2) {
		pulse[i] = _rf_amp;
	}

	// RF Pulse 2: RF:180y+ offset 2000 in 32 bit space
	for(int i = 2*memory_gap; i <= 2*memory_gap+(2*_tx_samples)*2; i=i+2) {
		pulse[i+1] = _rf_amp;
	}

	// RF Pulse 3: RF:180y- offset 3000 in 32 bit space
	for(int i = 3*memory_gap; i <= 3*memory_gap+(2*_tx_samples)*2; i=i+2) {
		pulse[i+1] = -_rf_amp;
	}

	// RF Pulse 4: RF:180x+ offset 4000 in 32 bit space
	// this is a hard 180 created by doubling the amplitude of the hard 90
	for(int i = 4*memory_gap; i <= 4*memory_gap+(2*_tx_samples); i=i+2) {
		pulse[i] = 2*_rf_amp;
	}

	// RF Pulse 5: SINC PULSE
	for(int i = 5*memory_gap; i <= 5*memory_gap+512; i=i+2) {
		int j = (int)((i - (5*memory_gap+64)) / 2) - 128;
		pulse[i] = (int16_t) floor(48*_rf_amp*(0.54 + 0.46*(cos((M_PI*j)/(2*48)))) * sin((M_PI*j)/(48))/(M_PI*j)); 
	}
	pulse[5*memory_gap+64+256] = _rf_amp;

	// RF Pulse 6: SIN PULSE
	for(int i = 6*memory_gap; i <= 6*memory_gap+512; i=i+2) {
		pulse[i] = (int16_t) floor(_rf_amp * sin((M_PI*i)/(128)));
	}

	auto size = 32768-1;
	*_tx_size = size;
	memset(_tx_data, 0, 65536);
	memcpy(_tx_data, pulse, 2 * size);	
}

unsigned hardware::configure_hw(mpack_node_t &cfg, server_action &sa) {
	unsigned commands_executed = 0;

	// FPGA clock config [TODO: understand this better]
	auto fcwa1 = mpack_node_map_cstr_optional(cfg, "fpga_clk");	
	if (not mpack_node_is_missing(fcwa1)) {
		// Enforce that all three words are present for FPGA clock configuration
		if (mpack_node_array_length(fcwa1) != 3) {
			sa.add_error("you only provided some FPGA clock control words; check you're providing all 3");
		} else {
			_slcr[2] = mpack_node_uint(mpack_node_array_at(fcwa1, 0));
			_slcr[92] = (_slcr[92] & ~mpack_node_uint(mpack_node_array_at(fcwa1, 1)) )
				| mpack_node_uint(mpack_node_array_at(fcwa1, 2));
			++commands_executed;
		}
	}

	// RX frequency (please pre-calculate the 32-bit int on the client)
	auto rxfn = mpack_node_map_cstr_optional(cfg, "rx_freq");
	if (not mpack_node_is_missing(rxfn)) {
		auto freq = mpack_node_u32(rxfn);

		double true_freq = ( static_cast<double>(freq) * FPGA_CLK_FREQ_HZ / (1 << 30) / 1e6);
		if (true_freq < 0.000001 or true_freq > 60) {
			sa.add_error("RX frequency outside the range [0.000001, 60] MHz");
		} else {
			*_rx_freq = freq;
		}
		char t[100];
		sprintf(t, "true RX freq: %f MHz", true_freq);
		sa.add_info(t);
		++commands_executed;
	}

	// TX divider
	auto txdn = mpack_node_map_cstr_optional(cfg, "tx_div");
	if (not mpack_node_is_missing(txdn)) {
		*_tx_divider = mpack_node_uint(txdn);
		if (*_tx_divider < 1 or *_tx_divider > 1000) {
			sa.add_warning("TX divider outside the range [1, 1000]; make sure this is what you want");
		}
		char t[100];
		sprintf(t, "TX sample duration: %f us", *_tx_divider * 1e6 / FPGA_CLK_FREQ_HZ);		
		sa.add_info(t);
		++commands_executed;
	}

	// RF amplitude (please pre-calculate the 16-bit int on the client)
	auto rfan = mpack_node_map_cstr_optional(cfg, "rf_amp");
	if (not mpack_node_is_missing(rfan)) {
		_rf_amp = mpack_node_u16(rfan);
		// if (rf_amp_f > 100 or rf_amp_f < 0) sa.add_error("RF amplitude outside the range [0, ]")
		double true_amp = _rf_amp * 100.0 / 65535;
		char t[100];
		sprintf(t, "true RF amp: %f\%", true_amp);
		sa.add_info(t);
		++commands_executed;
	}

	// Duration of a pulse, in TX samples
	auto txsn = mpack_node_map_cstr_optional(cfg, "tx_samples");
	if (not mpack_node_is_missing(txsn)) {
		uint32_t tx_samples = mpack_node_u32(txsn);
		if (tx_samples < 1 or tx_samples > 249) {
			sa.add_error("TX samples per pulse outside the range [1, 249]; check your settings");
		} else {
			_tx_samples = tx_samples;
		}
		++commands_executed;
	}

	// Recompute pulses
	auto rpn = mpack_node_map_cstr_optional(cfg, "recomp_pul");
	if (not mpack_node_is_missing(rpn)) {
		if (mpack_node_bool(rpn)) {
			compute_pulses();
			++commands_executed;
		} else {
			sa.add_warning("recomp_pul requested but set to false; doing nothing");
		}
	}

	// Fill in pulse memory directly from a binary blob
	auto rtxd = mpack_node_map_cstr_optional(cfg, "raw_tx_data");
	if (not mpack_node_is_missing(rtxd)) {
		if (mpack_node_bin_size(rtxd) <= 16 * sysconf(_SC_PAGESIZE)) {
			size_t bytes_copied = mpack_node_copy_data(rtxd, (char *)_tx_data, 16 * sysconf(_SC_PAGESIZE));
			char t[100];
			sprintf(t, "tx data bytes copied: %d", bytes_copied);
			sa.add_info(t);
			++commands_executed;
		} else {
			sa.add_error("too much raw TX data");
		}
	}
	

	// // compute relevant pulse properties based on desired duration
	// unsigned tx_samples = duration * FPGA_CLK_FREQ_HZ / (*tx_divider * 1e6);
	// if (sa != nullptr) {
	// 	char s[100];
	// 	sprintf(s, "a %f us pulse will need %d samples", duration, tx_samples);
	// 	sa->add_info(s);
	// }

	// if (tx_samples > 249) throw data_error("TX samples required is too high; reduce the pulse duration or increase the TX divider");
	// if (tx_samples < 1) throw data_error("less than 1 TX sample required; check your settings");

	// uint16_t rf_amp_u = (uint16_t) round(rf_amp / 100.0 * 65535);

	// //////////////

	// // Recompute pulses
	// auto rpdn = mpack_node_map_cstr_optional(cfg, "recompute_pulses");
	// if (not mpack_node_is_missing(rpdn)) {
	// 	// Mandatory pulse properties
	// 	if (mpack_node_map_count(rpdn) != 2) {
	// 		sa.add_error("improper number of recompute_pulses arguments");
	// 	} else {
	// 		double duration = mpack_node_double(mpack_node_map_cstr(rpdn, "duration"));
	// 		double rf_amp = mpack_node_double(mpack_node_map_cstr(rpdn, "rf_amp"));
	// 		try {
	// 			compute_pulses(duration, rf_amp, &sa);
	// 			++commands_executed;
	// 		} catch (marcos_error &e) {
	// 			sa.add_error(e.what());
	// 		}
	// 	}
	// }

	return commands_executed;
}
