#include "hardware.hpp"
#include "iface.hpp"
#include <fcntl.h>
#include <cmath>
#include <cassert>
#include <chrono>
#include <sys/mman.h>

#ifdef VERILATOR_BUILD
#include "flocra_model.hpp"
extern flocra_model *fm;
#endif

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

	// Read directly from memory
	auto rm = sa.get_command_and_start_reply("read_mem", status);
	if (status == 1) {
		++commands_understood;
		
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

	// Alter main control register
	auto ctrl = sa.get_command_and_start_reply("ctrl", status);
	if (status == 1) {
		++commands_understood;
		wr32(_ctrl, mpack_node_u32(ctrl));
		mpack_write(wr, c_ok);
	}	

	// Command directly to the buffers
	auto dir = sa.get_command_and_start_reply("direct", status);
	if (status == 1) {
		++commands_understood;
		wr32(_direct, mpack_node_u32(dir));
		// TODO: add sanity check for instruction/data type
		// TODO: implement higher-level direct commands, like 32b writes etc
		mpack_write(wr, c_ok);
	}

	// Read all registers
	sa.get_command_and_start_reply("regstatus", status);
	if (status == 1) {
		++commands_understood;
		mpack_start_array(wr, 7);
		
		mpack_write(wr, rd32(_exec));
		mpack_write(wr, rd32(_status));
		mpack_write(wr, rd32(_status_latch));
		mpack_write(wr, rd32(_buf_err));
		mpack_write(wr, rd32(_buf_full));
		mpack_write(wr, rd32(_buf_empty));
		mpack_write(wr, rd32(_rx_locs));
		
		mpack_finish_array(wr);
	}

	// Fill in flocra execution memory
	// TODO: add an input offset too, to avoid having to overwrite everything every time
	auto fm = sa.get_command_and_start_reply("flo_mem", status);
	if (status == 1) {
		++commands_understood;
		char t[100];

		// uint32_t ro = *(uint32_t *)(GRAD_CTRL_REG_OFFSET);
		if ( mpack_node_bin_size(fm) <= FLOCRA_MEM_SIZE ) {
			size_t bytes_copied = hw_mpack_node_copy_data(fm, reinterpret_cast<volatile char*>(_flo_mem), FLOCRA_MEM_SIZE);
			sprintf(t, "flo mem data bytes copied: %ld", bytes_copied);
			sa.add_info(t);
			mpack_write(wr, c_ok);
		} else {
			sprintf(t, "too much flo mem data: %ld bytes > %d -- streaming not yet implemented", mpack_node_bin_size(fm), FLOCRA_MEM_SIZE);
			sa.add_error(t);
			mpack_write(wr, c_err);
		}
	}

	// Configure acquisition retry limit (how many times the server will poll the RX FIFO when waiting for data)
	auto arl = sa.get_command_and_start_reply("acq_rlim", status);
	if (status == 1) {
		++commands_understood;
		uint32_t rl = mpack_node_u32(arl);
		if (rl < 1000 or rl > 10000000) {
			sa.add_error("acquisition retry limit outside the range [1000, 10,000,000]; check your settings");
			mpack_write(wr, c_err);
		} else {
			_read_tries_limit = rl;
			mpack_write(wr, c_ok);
		}
	}

	// Acquire data
	// auto acq = sa.get_command_and_start_reply("acq", status);
	// if (status == 1) {
	// 	++commands_understood;
	// 	uint32_t samples = mpack_node_u32(acq);
	// 	if (samples != 0) {
	// 		if (*_rx_cntr != 16386 && false) { // don't do this check for now
	// 			char t[100];
	// 			sprintf(t, "rx fifo not full before start: %d [will be solved with new firmware]", *_rx_cntr);
	// 			sa.add_warning(t);
	// 		}

	// 		_micro_seq_config[0] = 0x00000007; // start running the sequence
	// 		// usleep(100000); // sleep for 10ms to allow some data to arrive?
	// 		mpack_start_bin(wr, samples*8); // two 32b floats per sample
	// 		unsigned tries_tally = 0;
	// 		unsigned failed_reads = 0;
	// 		unsigned samples_since_last_halt_check = 0;
			
	// 		for (unsigned k = 0; k < samples; ++k) {
	// 			unsigned tries = 0;
	// 			bool success = false;

	// 			// NOTE: the logic below won't work, because _rx_cntr only monotonically increases;
	// 			// it never decreases in response to too many reads.
	// 			// Could easily have a 'FIFO fullness' line, but would need to tweak the HDL for that.
	// 			// Keep the logic for now, assuming that at some point, _rx_cntr will actually reflect
	// 			// the amount of data present in the FIFO.
	// 			while (not success and (tries < _read_tries_limit)) {
	// 				if (*_rx_cntr > 0) {
	// 					// temp uint64_t for rx_data storage
	// 					// (could read direct to buffer, but want to check stuff like endianness and throughput first)
	// 					uint64_t sample = *_rx_data; // perform the hardware read, perhaps even two reads
	// 					// uint64_t sample = *_rx_cntr; // DEBUG ONLY: save current FIFO count (NOTE: not as a float!)
	// 					mpack_write_bytes(wr, (char *)&sample, 8);
	// 					success = true;
	// 				} else ++tries;					
	// 			}
				
	// 			if (tries == _read_tries_limit) {
	// 				++failed_reads;
	// 				char empty[8] = {0,0,0,0,0,0,0,0};
	// 				mpack_write_bytes(wr, empty, 8);
	// 			}
				
	// 			tries_tally += tries;

	// 			if (samples_since_last_halt_check == _samples_per_halt_check) {
	// 				samples_since_last_halt_check = 0;
	// 				if ( (_micro_seq_config[3] & 0x1e) == 0x0e) {
	// 					// micro_sequencer FSM state is halted
	// 					// no more samples will arrive, so fill the remaining buffer with zeros
	// 					unsigned lost_samples = samples - k - 1;
	// 					k = samples; // halt outer loop
	// 					for (unsigned m = 0; m < lost_samples; ++m) {
	// 						++failed_reads;
	// 						char empty[8] = {0,0,0,0,0,0,0,0};
	// 						mpack_write_bytes(wr, empty, 8);
	// 					}
	// 					char t[100];
	// 					sprintf(t, "sequence halted; %d samples were never acquired", lost_samples);
	// 					sa.add_warning(t);
	// 				}
	// 			} else ++samples_since_last_halt_check;
	// 		}
	// 		mpack_finish_bin(wr);
			
	// 		// char yz[100];sprintf(yz, "grad status 0x%08x", *_grad_status);sa.add_info(yz);

	// 		// Final pause to see when the HALT instruction is executed
	// 		bool reached_halt = false;
	// 		unsigned halt_tries = 0;
	// 		while ( (halt_tries < _halt_tries_limit) and not reached_halt) {
	// 			// check micro_sequencer FSM state and compare against Halted
	// 			if ( (_micro_seq_config[3] & 0x1e) == 0x0e) reached_halt = true;
	// 			else ++halt_tries;
	// 		}
			
	// 		_micro_seq_config[0] = 0x00000000;

	// 		char t[100];			
	// 		sprintf(t, "rx cnt after end: %d, total read tries: %d\n", *_rx_cntr, tries_tally);
	// 		sa.add_info(t);

	// 		if (failed_reads) {
	// 			sprintf(t, "Encountered %d acquisition failures, wrote empty data", failed_reads);
	// 			sa.add_warning(t);
	// 		}

	// 		if (not reached_halt) sa.add_warning("micro_sequencer did not halt; timeout occurred");
			
	// 		uint32_t gs = *_grad_status; // single read, to avoid clearing the error bits
	// 		if (gs & 0x10000) sa.add_error("ocra1 core: gradient data was lost during sequence");
	// 		if (gs & 0x20000) sa.add_error("gpa-fhdo core: gradient data was lost during sequence");
	// 		// sprintf(t, "grad status 0x%08x", gs);
	// 		// sa.add_info(t);
			
	// 		// printf("rx cnt after end: %d, total read tries: %d\n", *_rx_cntr, tries_tally);
	// 		// usleep(10000);
	// 		// printf("rx cnt after end, wait 10ms: %d\n", *_rx_cntr);
	// 		// usleep(10000);
	// 		// printf("rx cnt after end, wait 10ms: %d\n", *_rx_cntr);
	// 		// usleep(10000);
	// 		// printf("rx cnt after end, wait 10ms: %d\n", *_rx_cntr);
			
	// 		// printf("rx cnt: %d\n", _rx_cntr);
	// 		// maybe do a usleep here?
	// 	} else {
	// 		sa.add_error("zero samples requested");
	// 		mpack_write(wr, c_err);
	// 	}
	// }

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

	// Test bus throughput
	auto tbt = sa.get_command_and_start_reply("test_bus", status);
	if (status == 1) {
		++commands_understood;
		auto n_tests = mpack_node_u32(tbt);

		auto start_t = std::chrono::system_clock::now();
		unsigned m = 0;
		for (unsigned k = 0; k < n_tests; ++k) {
			m += k;
		}
		auto null_t = std::chrono::system_clock::now();
		
		for (unsigned k = 0; k < n_tests; ++k) {
			m += rd32(_status); // repeatedly read the register
		}
		auto read_t = std::chrono::system_clock::now();

		for (unsigned k = 0; k < n_tests; ++k) {
			wr32(_ctrl, k & 0xfffffffc); // repeatedly write the register, but avoid setting the lower 2 bits
		}

		wr32(_ctrl, m & 0xfffffffc); // avoid m getting optimised out of the first loop
		
		auto write_t = std::chrono::system_clock::now();

		// reply will contain the three differences
		int64_t null_ti = std::chrono::duration_cast<std::chrono::microseconds>(null_t - start_t).count(),
			read_ti = std::chrono::duration_cast<std::chrono::microseconds>(read_t - null_t).count(),
			write_ti = std::chrono::duration_cast<std::chrono::microseconds>(write_t - read_t).count();
		
		mpack_start_array(wr, 3);
		mpack_write(wr, null_ti);
		mpack_write(wr, read_ti);
		mpack_write(wr, write_ti);
		mpack_finish_array(wr);
	}

	// Print out system state information, taking the FPGA clock
	// frequency as input (122.88 for RP-122, 125 for RP-125).
	// This command should always be run last, in case the other
	// commands set some of the relevant parameters already.
	// auto stat = sa.get_command_and_start_reply("state", status);
	// if (status == 1) {
	// 	++commands_understood;
	// 	auto nco_clk_freq_hz = mpack_node_double_strict(stat);
	// 	double tx_and_grad_clk_period_us = 0.007;
		
	// 	{
	// 		char t[100];
	// 		sprintf(t, "LO frequency [CHECK]: %f MHz", *_lo_freq * nco_clk_freq_hz / (1 << 30) / 1e6);
	// 		sa.add_info(t);
	// 		sprintf(t, "TX sample duration [CHECK]: %f us", *_tx_divider * tx_and_grad_clk_period_us);
	// 		sa.add_info(t);
	// 		sprintf(t, "RX sample duration [CHECK]: %f us", *_rx_divider * 1.0e6 / nco_clk_freq_hz);
	// 		sa.add_info(t);			
			
	// 		sprintf(t, "gradient sample duration (*not* DAC sampling rate): %f us",
	// 		        (*_grad_update_divider + 4) * tx_and_grad_clk_period_us);
	// 		sa.add_info(t);
	// 		sprintf(t, "gradient SPI transmission duration: %f us",
	// 		        (*_grad_spi_divider * 24 + 26) * tx_and_grad_clk_period_us);
	// 		sa.add_info(t);
			
	// 		mpack_write(wr, c_ok);
	// 	}
	// }

	// Final housekeeping
	mpack_finish_map(wr);
	
	if (commands_understood != commands_present) {
		assert(commands_understood <= commands_present && "Serious bug in logic");
		sa.add_error("not all client commands were understood");
		
		// Fill in remaining elements of the response map (TODO: maybe make this more sophisticated?)
		while (commands_present != 0) {
			char t[100];
			sprintf(t, "UNKNOWN%ld", commands_present);
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
		        "fallocate -l %ldKiB %s", tempfile, filesize_KiB, tempfile);
		throw hw_error(errstr);
	}
#endif

	// set up shared memory (please refer to the memory offset table)

	// VN: I'm not sure why in the original server, different data
	// types were used for some of these. Perhaps to allow
	// different access widths?
	_slcr = (uint32_t *) mmap(NULL, SLCR_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, SLCR_OFFSET);
	_flo_base = (uint32_t *) mmap(NULL, FLOCRA_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, FLOCRA_OFFSET);
	
	// Map the control and status registers
	_ctrl = _flo_base + 0;
	// slv_reg1 for extension in the future
	_direct = _flo_base + 2;
	// slv_reg3 for extension in the future
	_exec = _flo_base + 4; // execution information
	_status = _flo_base + 5; // external status, ADC etc
	_status_latch = _flo_base + 6; // latched external status
	_buf_err = _flo_base + 7; // latched buffer errors
	_buf_full = _flo_base + 8; // latched full buffers
	_buf_empty = _flo_base + 9; // empty buffers	
	_rx_locs = _flo_base + 10; // RX data available
	_rx0_data = _flo_base + 11; // RX0 data
	_rx1_data = _flo_base + 12; // RX1 data

	// /2 since mem is halfway in address space, /4 to convert to 32-bit instead of byte addressing	
	_flo_mem = _flo_base + FLOCRA_SIZE/2/4;

	halt_and_reset();
}

void hardware::halt_and_reset() {		
	// Old OCRA server comment: set FPGA clock to 143 MHz (VN: not
	// sure how this works - probably not needed any more?)
	_slcr[2] = 0xDF0D;
	_slcr[92] = (_slcr[92] & ~0x03F03F30) | 0x00100700;

	// TODO: wait a while for all the buffers to empty

	// TODO: write some immediate defaults to every buffer after
	// it has emptied, in order of priority (i.e. first TX, next
	// gradients)

	// TODO: handle gradient reset in a clever way: set SPI
	// divider to max, configure DAC boards, write a clear
	// command. Should be independent of any previously-configured
	// settings (i.e. do it for both potential GPA boards etc).

	// TODO: empty RX FIFOs (do this last)
}

void hardware::wr32(volatile uint32_t *addr, uint32_t data) {
#ifdef VERILATOR_BUILD
	if (addr >= _flo_base && addr < _flo_base + FLOCRA_SIZE) {
		// do byte-address arithmetic
		auto offs_addr = reinterpret_cast<volatile char *>(addr)
			- reinterpret_cast<volatile char *>(_flo_base);
		// printf("addresses 0x%0lx, 0x%0lx\n", addr, _flo_base);
		// printf("write flo 0x%0lx, 0x%08x\n", offs_addr, data);
		fm->wr32(offs_addr, data); // convert to byte addressing
	} else {
		printf("write addr 0x%0lx, 0x%08x NOT SIMULATED\n", (size_t) addr, data);
	}
#else
	// printf("write flo addr 0x%0x data 0x%0x\n", addr, data);
	// printf("_slcr: 0x%0x, _flo_base: 0x%0x\n", _slcr, _flo_base);
	*addr = data;
#endif
}

uint32_t hardware::rd32(volatile uint32_t *addr) {
#ifdef VERILATOR_BUILD
	if (addr >= _flo_base && addr < _flo_base + FLOCRA_SIZE) {
		// do byte-address arithmetic
		auto offs_addr = reinterpret_cast<volatile char *>(addr)
			- reinterpret_cast<volatile char *>(_flo_base);		
		// printf("read flo 0x%0lx\n", offs_addr);
		return fm->rd32(offs_addr); // convert to byte addressing
	} else {
		printf("read addr 0x%0lx NOT SIMULATED\n", (size_t) addr);
		return 0;
	}
#else
	// printf("ctrl: 0x%0x, value 0x%0x\n", _ctrl, *_ctrl);
	// printf("read flo addr 0x%0x\n", addr);
	return *addr;
	// return 0;
#endif
}

size_t hardware::hw_mpack_node_copy_data(mpack_node_t node, volatile char *buffer, size_t bufsize) {
#ifdef VERILATOR_BUILD
	// Copy the data via individual 32b bus writes
	char *tmp = reinterpret_cast<char *>(malloc(bufsize));
	size_t bytes_copied = mpack_node_copy_data(node, tmp, bufsize);

	// Inefficient, but won't be a major delay in the simulation anyway
	// TODO: check pointer arithmetic!
	auto tmp_u32 = reinterpret_cast<uint32_t *>(tmp);
	size_t offset = 0;
	for (size_t k = 0; k < bytes_copied/4; ++k) {
		wr32(reinterpret_cast<volatile uint32_t *>(buffer + offset), tmp_u32[k]);
		offset += 4;
	}

	free(tmp);
	return bytes_copied;
#else
	return mpack_node_copy_data(node, (char *)buffer, bufsize); // discard volatile qualifier
#endif
}
