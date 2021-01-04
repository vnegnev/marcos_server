#include "hardware.hpp"
#include "iface.hpp"
#include <fcntl.h>
#include <cmath>
#include <cassert>
#include <chrono>
#include <vector>
#include <sys/mman.h>

#ifdef VERILATOR_BUILD
#include "flocra_model.hpp"
extern flocra_model *fm;
#endif

// variadic macro for debugging enable/disable
#define debug_printf(...) printf(__VA_ARGS__)

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
			size_t bytes_copied = hw_mpack_node_copy_data(fm, _flo_mem, FLOCRA_MEM_SIZE);
			sprintf(t, "flo mem data bytes copied: %zu", bytes_copied);
			sa.add_info(t);
			mpack_write(wr, c_ok);
		} else {
			sprintf(t, "too much flo mem data: %zu bytes > %d -- streaming not yet implemented", mpack_node_bin_size(fm), FLOCRA_MEM_SIZE);
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

	// read all outstanding data from RX FIFOs
	auto fr = sa.get_command_and_start_reply("flush_rx", status);
	if (status == 1) {
		++commands_understood;
		char t[100];
		std::vector<uint32_t> rx0_i, rx0_q, rx1_i, rx1_q;
		read_rx(rx0_i, rx0_q, rx1_i, rx1_q);

		// encode the RX replies
		unsigned rx0_elem = rx0_i.size(), rx1_elem = rx1_i.size();
		if (!rx0_elem and !rx1_elem) {
			mpack_write(wr, c_ok);
			sprintf(t, "no RX data received");
			sa.add_warning(t);
		} else {
			mpack_start_map(wr, (rx0_elem ? 2:0) + (rx1_elem ? 2:0));
			if (rx0_elem) {
				mpack_write_cstr(wr, "rx0_i");
				mpack_start_array(wr, rx0_elem);
				for (unsigned k = 0; k < rx0_elem; ++k) mpack_write_int(wr, rx0_i[k]);
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx0_q");
				mpack_start_array(wr, rx0_elem);
				for (unsigned k = 0; k < rx0_elem; ++k) mpack_write_int(wr, rx0_q[k]);
				mpack_finish_array(wr);				
			}
			
			if (rx1_elem) {
				mpack_write_cstr(wr, "rx1_i");
				mpack_start_array(wr, rx1_elem);
				for (unsigned k = 0; k < rx1_elem; ++k) mpack_write_int(wr, rx1_i[k]);
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx1_q");
				mpack_start_array(wr, rx1_elem);
				for (unsigned k = 0; k < rx1_elem; ++k) mpack_write_int(wr, rx1_q[k]);
				mpack_finish_array(wr);				
			}

			mpack_finish_map(wr);
		}
	}

	// Run a sequence
	auto runs = sa.get_command_and_start_reply("run_seq", status);
	if (status == 1) {
		++commands_understood;
		char t[100];

		// ensure that nothing is currently running; halt if it is
		auto exec = rd32(_exec);
		if ( (exec >> 24) != FLO_STATE_IDLE) {
			sprintf(t, "flo FSM was not idle when the run began");
			sa.add_warning(t);

			// halt FSM
			wr32(_ctrl, 0x2); // set bit 1 to halt
		}

		const size_t total_bytes_to_copy = mpack_node_bin_size(runs);
		const char *rundata = (char *)mpack_node_bin_data(runs);
		size_t mem_offset = 0;
		
		// initially fill the flocra memory
		if (total_bytes_to_copy <= FLOCRA_MEM_SIZE) {
			hw_memcpy(_flo_mem, rundata, total_bytes_to_copy);
			mem_offset += total_bytes_to_copy;
		} else {
			// first time, wrap around back to the start again,
			// so don't need to update flo memory ptr
			hw_memcpy(_flo_mem, rundata, FLOCRA_MEM_SIZE);
			mem_offset += FLOCRA_MEM_SIZE;
		}

		// prepare main FSM control loop

		// max bytes to copy into _flo_mem at a time (would block for this long)
		const unsigned max_bytes_to_copy = 128;

		// check execution state for issues periodically
		const unsigned execution_check_interval = 20;

		// monitor buffer-low and buffer-underrun events
		unsigned mem_buffer_low = 0;
		bool mem_buffer_underrun = false;
		bool finished = false;

		// monitor RX-nearly-full and RX-full events
		unsigned rx_nearly_full = 0;
		bool rx_full = false;
		size_t rx_full_mem_loc = 0;

		// monitor output buffers
		uint32_t buf_full = 0, buf_err = 0;

		// RX data
		std::vector<uint32_t> rx0_i, rx0_q, rx1_i, rx1_q;
		unsigned rx_reads_per_loop = _min_rx_reads_per_loop;
				
		// start the FSM
		wr32(_ctrl, 0x1);

		// main copying and reading loop
		unsigned execution_loops = 0;
		// track program counter execution
		size_t pc_offset = 0, old_pc_hw = 0, old_pc = 0;
		while (not finished) {
			uint32_t exec = rd32(_exec);
			uint32_t state = exec >> 24;
			size_t pc_hw = (exec & 0xffffff) << 2; // convert to mem offset

			if (pc_hw + 16 < old_pc_hw) {
				// extra 16 since sometimes the PC can
				// go back a few instructions when
				// waiting/pausing
				debug_printf("PC wrapped\n");
				pc_offset += FLOCRA_MEM_SIZE;
			}
			old_pc_hw = pc_hw;
			
			size_t pc = pc_hw + pc_offset;
			// unwrap pc
			debug_printf("pc_hw %zu, old_pc_hw %zu, pc %zu\n", pc_hw, old_pc_hw, pc);
			
			size_t total_bytes_remaining = total_bytes_to_copy - mem_offset;
			int bytes_to_copy = 0;
			if (total_bytes_remaining != 0) {			       
				// check whether data needs copying in this round
				// 
				// -16 to keep a buffer zone of 4 un-copied
				// instructions before PC location
				bytes_to_copy = pc - mem_offset + FLOCRA_MEM_SIZE - 16;
				if (bytes_to_copy > (int)total_bytes_remaining) bytes_to_copy = total_bytes_remaining;
			}
			
			if (bytes_to_copy > 0) {
				// Monitor memory reserve during streaming
				if (pc > mem_offset) {
					// memory reserve has run dry
					bytes_to_copy = 0;
					mem_buffer_underrun = true;
					debug_printf("mem buf underrun\n");
					break;
				} else if (mem_offset - pc < FLOCRA_MEM_SIZE/4) {
					// memory reserve only 1/4 full
					++mem_buffer_low;
					debug_printf("mem buf low\n");
				}

				if (bytes_to_copy > (int)max_bytes_to_copy) {
					// avoid starving the RX for time
					bytes_to_copy = max_bytes_to_copy;
				}
				
				size_t local_mem_offset = mem_offset & FLOCRA_MEM_MASK;

				// check whether this copy will wrap
				if ( local_mem_offset + bytes_to_copy > FLOCRA_MEM_SIZE) { // wrapping: copy twice
					int first_bytes = FLOCRA_MEM_SIZE - local_mem_offset;
					int second_bytes = bytes_to_copy - first_bytes;
					hw_memcpy(_flo_mem + local_mem_offset,
					          rundata + mem_offset, first_bytes);
					mem_offset += first_bytes;
					hw_memcpy(_flo_mem, rundata + mem_offset, second_bytes);
					mem_offset += second_bytes;
				} else { // no wrapping
					debug_printf("hw_memcpy: %zu, %zu, %u\n", local_mem_offset, mem_offset, bytes_to_copy);
					auto k = (char *)hw_memcpy(_flo_mem + local_mem_offset,
					          rundata + mem_offset, bytes_to_copy);
					debug_printf("bytes copied: %zu\n", k - _flo_mem - local_mem_offset);
					mem_offset += bytes_to_copy;
				}
			}

			// Read out RX
			unsigned rx_locs = read_rx(rx0_i, rx0_q, rx1_i, rx1_q, rx_reads_per_loop);
			// Simple dynamic FIFO read-out speed governor
			if (rx_locs > FLOCRA_RX_FIFO_SPACE - 2*_max_rx_reads_per_loop) {
				rx_full = true;
				rx_full_mem_loc = old_pc;
				// desperately try to clear the FIFOs
				rx_reads_per_loop = _max_rx_reads_per_loop;
			} else if (rx_locs > 2 * FLOCRA_RX_FIFO_SPACE / 3) {
				++rx_nearly_full;
				// try hard to clear the FIFOs; scale up quickly
				if (rx_reads_per_loop < _max_rx_reads_per_loop) rx_reads_per_loop = rx_reads_per_loop * 4;
			} else if (rx_locs > FLOCRA_RX_FIFO_SPACE / 3) {
				if (rx_reads_per_loop < _max_rx_reads_per_loop) rx_reads_per_loop = rx_reads_per_loop * 2; // scale up 
			} else {
				if (rx_reads_per_loop > _min_rx_reads_per_loop) rx_reads_per_loop = rx_reads_per_loop / 2;
			}

			// periodic buffer status checking
			if (execution_loops % execution_check_interval == 0) {
				buf_full = buf_full | rd32(_buf_full);
				buf_err = rd32(_buf_err);
				if (buf_err) break;
			}			
			
			if (state == FLO_STATE_HALT) {
				finished = true;			
			}
			old_pc = pc;
			++execution_loops;
		}
		
		// gracefully reset the FSM if no errors occurred
		if (finished) {
			wr32(_ctrl, 0x0);			
		// emergency halt the FSM and reset the hardware
		} else {
			wr32(_ctrl, 0x20);
			halt_and_reset();
		}

		// post-mortem reporting
		if (mem_buffer_underrun) {
			sprintf(t, "memory buffer underrun around byte address 0x%0zx", old_pc);
			sa.add_error(t);
		} else if (mem_buffer_low) {
			sprintf(t, "memory buffer low for %u inner loop cycles", mem_buffer_low);
			sa.add_warning(t);
		} else if (mem_offset != total_bytes_to_copy) {
			sprintf(t, "didn't copy %zu bytes before FSM halted", total_bytes_to_copy - mem_offset);
			sa.add_error(t);
		}

		if (rx_full) {
			sprintf(t, "RX FIFO/s full during sequence around byte address 0x%0zx", rx_full_mem_loc);
			sa.add_error(t);
		} else if (rx_nearly_full) {
			sprintf(t, "RX FIFO/s almost filled during sequence %u times", rx_nearly_full);
			sa.add_warning(t);
		}

		if (buf_err) {
			sprintf(t, "output buffers overflowed during sequence: 0x%08x", buf_err);
			sa.add_error(t);			
		} else if (buf_full) {
			sprintf(t, "output buffers were full during sequence: 0x%08x", buf_full);
			sa.add_warning(t);
		}

		// readout of any final data remaining at the end
		unsigned read_tries = 0;
		unsigned final_rx_read = 0;
		// while (read_tries < _halt_tries_limit) {
		// 	final_rx_read += read_rx(rx0_i, rx0_q, rx1_i, rx1_q);
		// }
		while (read_tries < 100) {
			final_rx_read += read_rx(rx0_i, rx0_q, rx1_i, rx1_q, 100);
			++read_tries;
			// TODO why does the RX
		}		
		// delete after debugging is over
		printf("Final RX read: %d\n", final_rx_read);

		// TODO make sure all the buffers and RX FIFOs are empty

		// encode the RX replies
		unsigned rx0_elem = rx0_i.size(), rx1_elem = rx1_i.size();
		if (!rx0_elem and !rx1_elem) {
			mpack_write(wr, c_ok);
			sprintf(t, "no RX data received");
			sa.add_warning(t);
		} else {
			mpack_start_map(wr, (rx0_elem ? 2:0) + (rx1_elem ? 2:0) );
			if (rx0_elem) {
				mpack_write_cstr(wr, "rx0_i");
				mpack_start_array(wr, rx0_elem);
				for (unsigned k = 0; k < rx0_elem; ++k) mpack_write_int(wr, rx0_i[k]);
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx0_q");
				mpack_start_array(wr, rx0_elem);
				for (unsigned k = 0; k < rx0_elem; ++k) mpack_write_int(wr, rx0_q[k]);
				mpack_finish_array(wr);				
			}
			
			if (rx1_elem) {
				mpack_write_cstr(wr, "rx1_i");
				mpack_start_array(wr, rx1_elem);
				for (unsigned k = 0; k < rx1_elem; ++k) mpack_write_int(wr, rx1_i[k]);
				mpack_finish_array(wr);
				mpack_write_cstr(wr, "rx1_q");
				mpack_start_array(wr, rx1_elem);
				for (unsigned k = 0; k < rx1_elem; ++k) mpack_write_int(wr, rx1_q[k]);
				mpack_finish_array(wr);				
			}

			mpack_finish_map(wr);
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

	// Determine if the server is running on hardware or just an emulation
	sa.get_command_and_start_reply("are_you_real", status);
	if (status == 1) {
		++commands_understood;
#ifdef __arm__
		mpack_write(wr, "hardware");
#else
		
#ifdef VERILATOR_BUILD
		mpack_write(wr, "simulation");
#else
		mpack_write(wr, "software");
#endif
		
#endif
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
			sprintf(t, "UNKNOWN%zu", commands_present);
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
	_rx0_i_data = _flo_base + 11; // RX0 i data
	_rx1_i_data = _flo_base + 12; // RX1 i data
	_rx0_q_data = _flo_base + 13; // RX0 q data
	_rx1_q_data = _flo_base + 14; // RX1 q data

	// /2 since mem is halfway in address space, /4 to convert to 32-bit instead of byte addressing	
	_flo_mem = reinterpret_cast<volatile char *>(_flo_base + FLOCRA_SIZE/2/4);

	halt_and_reset();
}

void hardware::halt_and_reset() {		
	// Old OCRA server comment: set FPGA clock to 143 MHz (VN: not
	// sure how this works - probably not needed any more?)
	_slcr[2] = 0xDF0D;
	_slcr[92] = (_slcr[92] & ~0x03F03F30) | 0x00100700;

	// Wait a while for all the output buffers to empty
	unsigned k = 0;
	while (k < _halt_tries_limit) {
		if ( rd32(_buf_empty) == 0x00ffffff ) break;
	}

	// TODO: write some immediate defaults to every buffer after
	// it has emptied, in order of priority (i.e. first TX, next
	// gradients)

	// TODO: handle gradient reset in a clever way: set SPI
	// divider to max, configure DAC boards, write a clear
	// command. Should be independent of any previously-configured
	// settings (i.e. do it for both potential GPA boards etc).

	// TODO: empty RX FIFOs (do this last)
	printf("TODO: finish halt_and_reset()\n");
}

unsigned hardware::read_rx(std::vector<uint32_t> &rx0_i, std::vector<uint32_t> &rx0_q,
                           std::vector<uint32_t> &rx1_i, std::vector<uint32_t> &rx1_q,
                           const unsigned max_reads) {
	uint32_t rxlocs = rd32(_rx_locs);
	int fifo0_locs = rxlocs & 0xffff, fifo1_locs = rxlocs >> 16;

	unsigned reads = 0;

	debug_printf("initial fifo locs: %d, %d\n", fifo0_locs, fifo1_locs);

	while (fifo0_locs > 0 or fifo1_locs > 0) {
		if (reads >= max_reads) break;

		bool read_fifo0 = false, read_fifo1 = false;
		if (fifo1_locs > 2*fifo0_locs) {
			// too much data in fifo1, read it exclusively
			read_fifo1 = true;
		} else if (fifo0_locs > 2*fifo1_locs) {
			// too much data in fifo0, read it exclusively
			read_fifo0 = true;
		} else { // read both
			read_fifo0 = true;
			read_fifo1 = true;
		}

		if (read_fifo0) { // read pair of samples
			rx0_q.push_back(rd32(_rx0_q_data)); // read q first
			rx0_i.push_back(rd32(_rx0_i_data)); // pop fifo
			++reads;
			--fifo0_locs;
		}
		if (read_fifo1) { // read pair of samples
			rx1_q.push_back(rd32(_rx1_q_data)); // read q first
			rx1_i.push_back(rd32(_rx1_i_data)); // pop fifo
			++reads;
			--fifo1_locs;
		}
	}

	// Let some time pass so that the FIFO-fullness register can update
	// (this is purely to make the simulation behaviour more realistic)
#ifdef VERILATOR_BUILD
	rd32(_rx1_q_data);
#endif
	
	// return the FIFO closest to filling
	if (fifo0_locs > fifo1_locs) return fifo0_locs;
	else return fifo1_locs;
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
	return *addr;
#endif
}

void* hardware::hw_memcpy(volatile void *s1, const void *s2, size_t n) {
#ifdef VERILATOR_BUILD
	// copy the data via individual 32b bus writes
	auto *s1u = reinterpret_cast<volatile uint32_t *>(s1);
	auto *s2u = reinterpret_cast<const volatile uint32_t *>(s2);
	size_t nu = n / 4;

	// algorithm copied verbatim from mpack, just acting on uint32
	while (nu-- != 0) wr32(s1u++, *s2u++);
	return (void *)s1u;
#else
	return memcpy((void *)s1, s2, n);
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
