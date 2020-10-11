#include "hardware.hpp"
#include "iface.hpp"
#include <fcntl.h>
#include <cmath>
#include <cassert>
#include <sys/mman.h>

hardware::hardware() {	
	init_mem();
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
		// sa.add_error("Unknown MPack error from fpga_clk");
		// TODO: callback or similar
	}

	// LO frequency (please pre-calculate the 32-bit int on the client)
	auto lofn = sa.get_command_and_start_reply("lo_freq", status);
	if (status == 1) {
		++commands_understood;
		auto freq = mpack_node_u32(lofn);

		double true_freq = ( static_cast<double>(freq) * FPGA_CLK_FREQ_HZ / (1 << 30) / 1e6);
		if (true_freq < 0.0000001 or true_freq > 60) {
			sa.add_error("RX frequency outside the range [0.0000001, 60] MHz");
			mpack_write(wr, c_err);
		} else {
			// NOTE: the data write is purely to avoid a
			// hardware bug that seems to clear _tx_size
			// when _lo_freq is written to -- TODO IS THIS
			// COMMENT STILL RELEVANT?

			// printf("cfg regs before: 0x%x, 0x%x, 0x%x, 0x%x\n", *_tx_divider, *_lo_freq, *_rx_rate, *_tx_size);
			
			*_lo_freq = freq;			
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
		uint32_t tx_divider = mpack_node_u32(txdn);
		if (tx_divider < 1 or tx_divider > 10000) {
			sa.add_warning("TX divider outside the range [1, 10000]; make sure this is what you want");
			mpack_write(wr, c_warn);
		} else {
			*_tx_divider = tx_divider;
			mpack_write(wr, c_ok);
		}
		char t[100];
		sprintf(t, "TX sample duration: %f us", *_tx_divider * 1e6 / FPGA_CLK_FREQ_HZ);		
		sa.add_info(t);
	} // else if (status == -1) do some error handling

	// RX sampling rate, in clock cycles
	auto rxr = sa.get_command_and_start_reply("rx_div", status);
	if (status == 1) {
		++commands_understood;
		uint32_t rx_div = mpack_node_u32(rxr);
		if (rx_div < 25 or rx_div > 8192) { // these settings are hardcoded in the 'CIC compiler' parameters of Vivado cores
			sa.add_error("RX divider outside the range [25, 8192]; check your settings");
			mpack_write(wr, c_err);
		} else {
			*_rx_divider = rx_div;
			mpack_write(wr, c_ok);
		}
	}

	// TX size, in samples
	auto txs = sa.get_command_and_start_reply("tx_size", status);
	if (status == 1) {
		++commands_understood;
		uint16_t tx_size = mpack_node_u16(txs);
		if (tx_size < 1 or tx_size > 32767) { // TODO: figure out why these limits are in place
			sa.add_error("TX size outside the range [1, 32767]; check your settings");
			mpack_write(wr, c_err);
		} else {
			*_tx_size = tx_size;
			mpack_write(wr, c_ok);
		}
	}	

	// Fill in pulse memory directly from a binary blob
	auto rtxd = sa.get_command_and_start_reply("raw_tx_data", status);
	if (status == 1) {
		++commands_understood;
		if (mpack_node_bin_size(rtxd) <= TX_DATA_SIZE) {
			size_t bytes_copied = mpack_node_copy_data(rtxd, (char *)_tx_data, TX_DATA_SIZE);
			char t[100];
			sprintf(t, "tx data bytes copied: %d", bytes_copied);
			sa.add_info(t);
			mpack_write(wr, c_ok);
		} else {
			sa.add_error("too much raw TX data");
			mpack_write(wr, c_err);
		}
	}

	// Fill in pulse sequence memory directly from a binary blob (i.e. SEQUENCE instruction memory, not pulse memory)
	auto sd = sa.get_command_and_start_reply("seq_data", status);
	if (status == 1) {
		++commands_understood;
		if (mpack_node_bin_size(sd) <= MICRO_SEQ_MEMORY_SIZE) {
			size_t bytes_copied;
			
			if (false) { // Copy via a temp int array (only useful for debugging)
				char temp_buf[MICRO_SEQ_MEMORY_SIZE];
				bytes_copied = mpack_node_copy_data(sd, temp_buf, MICRO_SEQ_MEMORY_SIZE);

				// Copy via ints
				for (size_t k=0; k<bytes_copied/4; ++k) {
					_micro_seq_memory[k] = ((uint32_t *)temp_buf)[k];
				}
			} else { // Copy directly
				bytes_copied = mpack_node_copy_data(sd, (char *)_micro_seq_memory, MICRO_SEQ_MEMORY_SIZE);
			}
			
			char t[100];
			sprintf(t, "sequence data bytes copied: %d", bytes_copied);
			sa.add_info(t);
			mpack_write(wr, c_ok);

			// NOTE: do not read from _micro_seq_memory, it will crash the RP since the AXI bus hangs!
			// uint32_t *a = (uint32_t *)temp_buf;
			// for (int k=0; k<bytes_copied/4; ++k) printf("_micro_seq_memory[%d] = 0x%08x\n", k, a[k]);
			
		} else {
			sa.add_error("too much pulse sequence data");
			mpack_write(wr, c_err);
		}
	}

	// First element of array: gradient update divider D, in clock
	// periods with an offset of +4. Worked example: for a 100 MHz
	// ocra_grad_ctrl main clock (10ns period), a value of D = 121
	// would lead to updates to the serialiser core every (D + 4)
	// = 125 ticks or 1.25us. If each channel of a 4-channel DAC
	// like that used in the ocra1 were updated once every 4
	// updates, then a broadcast every 4 updates would lead to an
	// update period of 1.25us * 4 = 5us.  For a clock rate of
	// 122.88 MHz (8.138ns), a divider of 303 gives a broadcast
	// interval (effective DAC sample rate) of 9.9934895833333
	// us. Note that to achieve very slow sampling rates, D should
	// just be set to its max value of 1023, and the time interval
	// of each sample should be lengthened in the sample data
	// itself.
	// 
	// Second element of array: SPI clock divider S, in clock
	// periods with an offset of 1. Worked example: for a 100 MHz
	// ocra_grad_ctrl main clock (10ns period), a value of S = 1
	// would lead to an SPI clock period of (1 + S) * 10ns, and a
	// minimum total interval between updates on a single SPI
	// interface of 24 * (1 + S) + 2 = 26 + 24 * S ticks, or 50
	// ticks (satisfied by D = 46). To be clear, the ocra1 can use
	// lower values of D, as long as updates to a single DAC
	// happen no more frequently than 26 + 24 * S ticks apart
	// (i.e. multiple channels are written in parallel). The
	// gpa-fhdo is subject to the D >= 26 + 24 * S restriction,
	// however.
	auto gd = sa.get_command_and_start_reply("grad_div", status);
	if (status == 1) {
		++commands_understood;
		// Enforce that two 32-bit ints are present
		if (mpack_node_array_length(gd) != 2) {
			sa.add_error("Wrong number of grad_div arguments; check you're providing two 32-bit ints.");
			mpack_write(wr, c_err); // error
		} else {
			uint32_t D = mpack_node_u32(mpack_node_array_at(gd, 0));
			uint32_t S = mpack_node_u32(mpack_node_array_at(gd, 1));			

			if (S < 1 or S > 63) {
				sa.add_error("grad SPI clock divider outside the range [1, 63]; check your settings");
				mpack_write(wr, c_err);
			} else if (D < 13 or D > 1023) {
				sa.add_error("grad update interval divider outside the range [13, 1023]; check your settings");
				mpack_write(wr, c_err);
			} else {
				*_grad_update_divider = D;
				*_grad_spi_divider = S;
				mpack_write(wr, c_ok);
			}
		}
	}

	// Gradient serialiser control
	auto gs = sa.get_command_and_start_reply("grad_ser", status);
	if (status == 1) {
		++commands_understood;
		uint32_t grad_ser_ctrl = mpack_node_u32(gs);
		if (grad_ser_ctrl > 0xf) { // more than lower 4 bits filled
			sa.add_error("serialiser enables outside the range [0, 0xf], check your settings");
			mpack_write(wr, c_err);
		} else {
			*_grad_serialiser_ctrl = grad_ser_ctrl;
			if (grad_ser_ctrl == 0) sa.add_warning("No gradient serialisers enabled");
			mpack_write(wr, c_ok);
		}
	}
	
	// Fill in gradient memory
	// TODO: add an input offset too, to avoid having to overwrite everything every time
	auto gm = sa.get_command_and_start_reply("grad_mem", status);
	if (status == 1) {
		++commands_understood;
		char t[100];

		// uint32_t ro = *(uint32_t *)(GRAD_CTRL_REG_OFFSET);
		if ( mpack_node_bin_size(gm) <= GRAD_MEM_SIZE ) {
			size_t bytes_copied = mpack_node_copy_data(gm, (char *)_grad_mem, GRAD_MEM_SIZE);
			sprintf(t, "gradient mem data bytes copied: %d", bytes_copied);
			sa.add_info(t);
			mpack_write(wr, c_ok);
		} else {
			sprintf(t, "too much grad mem data: %d bytes > %d", mpack_node_bin_size(gm), GRAD_MEM_SIZE);
			sa.add_error(t);
			mpack_write(wr, c_err);
		}
	}

	// Configure 

	// Acquire data
	auto acq = sa.get_command_and_start_reply("acq", status);
	if (status == 1) {
		++commands_understood;
		uint32_t samples = mpack_node_u32(acq);
		if (samples != 0) {
			printf("rx cnt before wait: %d\n", *_rx_cntr);

			_micro_seq_config[0] = 0x00000007; // start running the sequence
			// usleep(100000); // sleep for 10ms to allow some data to arrive?
			mpack_start_bin(wr, samples*8); // two 32b floats per sample
			unsigned tries_tally = 0;
			for (unsigned k=0; k<samples; ++k) {
				unsigned tries = 0;
				unsigned tries_limit = 10000;
				bool success = false;

				// NOTE: the logic below won't work, because _rx_cntr only monotonically increases;
				// it never decreases in response to too many reads.
				// Could easily have a 'FIFO fullness' line, but would need to tweak the HDL for that.
				// Keep the logic for now, assuming that at some point, _rx_cntr will actually reflect
				// the amount of data present in the FIFO.
				while (not success and (tries < tries_limit)) {
					if (*_rx_cntr > 0) {
						// temp uint64_t for rx_data storage
						// (could read direct to buffer, but want to check stuff like endianness and throughput first)
						uint64_t sample = *_rx_data; // perform the hardware read, perhaps even two reads
						// uint64_t sample = *_rx_cntr; // DEBUG ONLY: save current FIFO count (NOTE: not as a float!)
						mpack_write_bytes(wr, (char *)&sample, 8);
						success = true;
					} else ++tries;						
				}
				if (tries == tries_limit) {
					printf("Couldn't read, writing empty byte\n");
					char empty[8] = {0,0,0,0,0,0,0,0};
					mpack_write_bytes(wr, empty, 8);
				}
				tries_tally += tries;
			}
			mpack_finish_bin(wr);
			_micro_seq_config[0] = 0x00000000;
			printf("rx cnt after end: %d, total read tries: %d\n", *_rx_cntr, tries_tally);
			usleep(10000);
			printf("rx cnt after end, wait 10ms: %d\n", *_rx_cntr);
			usleep(10000);
			printf("rx cnt after end, wait 10ms: %d\n", *_rx_cntr);
			usleep(10000);
			printf("rx cnt after end, wait 10ms: %d\n", *_rx_cntr);
			
			// printf("rx cnt: %d\n", _rx_cntr);
			// maybe do a usleep here?
		} else {
			sa.add_error("zero samples requested");
			mpack_write(wr, c_err);
		}
	}

	// Test client-server network throughput
	auto tln = sa.get_command_and_start_reply("test_net", status);
	if (status == 1) {
		++commands_understood;
		unsigned data_size = mpack_node_uint(tln);

		mpack_start_map(wr, 2); // Two elements in map
		mpack_write_cstr(wr, "array1");
		mpack_start_array(wr, data_size);
		for (unsigned k{0}; k < data_size; ++k) mpack_write(wr, 1.01*k); // generic, needs C11
		mpack_finish_array(wr);

		mpack_write_cstr(wr, "array2");
		mpack_start_array(wr, data_size);
		for (unsigned k{0}; k < data_size; ++k) mpack_write(wr, 1.01*(k+10)); // generic, needs C11
		mpack_finish_array(wr);
		
		mpack_finish_map(wr);
	} else if (status == -1) {
		// TODO: callback or similar
	}

	// Test various bus properties including throughput [TODO]
	auto tbt = sa.get_command_and_start_reply("test_bus", status);

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
		size_t filesize_KiB =  4 * END_OFFSET / EMU_PAGESIZE; // 4 because 4 KiB / page
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
	_slcr = (uint32_t *) mmap(NULL, SLCR_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, SLCR_OFFSET);
	_cfg = (char *) mmap(NULL, CFG_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, CFG_OFFSET);
	_sts = (char *) mmap(NULL, STS_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, STS_OFFSET);
	_rx_data = (uint64_t *) mmap(NULL, RX_DATA_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RX_DATA_OFFSET);
	_tx_data = (char *) mmap(NULL, TX_DATA_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, TX_DATA_OFFSET);
	_micro_seq_memory = (uint32_t *) mmap(NULL, MICRO_SEQ_MEMORY_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MICRO_SEQ_MEMORY_OFFSET);
	_micro_seq_config = (uint32_t *) mmap(NULL, MICRO_SEQ_CONFIG_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, MICRO_SEQ_CONFIG_OFFSET);

	/*
	  NOTE: The block RAM can only be addressed with 32 bit transactions, so gradient_memory needs to
	  be of type uint32_t. The HDL would have to be changed to an 8-bit interface to support per
	  byte transactions
	*/
	_grad_config = (uint32_t *) mmap(NULL, GRAD_CONFIG_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GRAD_CTRL_REG_OFFSET);
	_grad_mem = (uint32_t *) mmap(NULL, GRAD_MEM_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GRAD_CTRL_MEM_OFFSET);
	
	// Map the control registers
	//rx_rst = ((uint8_t *)(cfg + 0));	

	_tx_divider = (uint32_t *)(_cfg + 0);
	_lo_freq = (uint32_t *)(_cfg + 4);
	_rx_divider = (uint32_t *)(_cfg + 8);
	_tx_size = (uint16_t *)(_cfg + 12); // note that this is a uint16_t; be careful if you modify the code
	
	_rx_cntr = (uint16_t *)(_sts + 0);
	
	_grad_update_divider = _grad_config + 0; // register 0 in ocra_grad_ctrl
	_grad_spi_divider = _grad_config + 1; // register 1 in ocra_grad_ctrl
	_grad_serialiser_ctrl = _grad_config + 2;
	_grad_status = _grad_config + 4;

	//tx_rst = ((uint8_t *)(cfg + 1));
	
	// Fill in some default values (can be altered later by calling configure_hw() )
	// Old comment: set FPGA clock to 143 MHz (VN: not sure how this works - probably 122 MHz in RP-122?)
	_slcr[2] = 0xDF0D;
	_slcr[92] = (_slcr[92] & ~0x03F03F30) | 0x00100700;
	
	// Old comment: erase pulse sequence memory
	for (int i=0; i<32; ++i) _micro_seq_memory[i] = 0x0;
	
	// Old comment: halt the microsequencer
	_micro_seq_config[0] = 0x00;
	
	// Old comment: set the NCO to 15.67 MHz
	*_lo_freq = (uint32_t) floor(15670000 / FPGA_CLK_FREQ_HZ * (1<<30) + 0.5);
	
	// Old comment: set default rx sample rate
	*_rx_divider = 250;

	// Old comment: this divider makes the sample duration a convenient 1us (VN: adjusted to be close to the tx clock freq)
	*_tx_divider = (uint32_t) round(FPGA_CLK_FREQ_HZ/1e6);	
	
	// fill tx and grad memories with zeros
	// memset(_tx_data, 0, TX_DATA_SIZE);
	memset((void *)_grad_mem, 0, GRAD_MEM_SIZE);
}
