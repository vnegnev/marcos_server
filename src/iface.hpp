/**@file iface.hpp
   @brief Communications interface to the host via Ethernet 

*/

#ifndef _IFACE_HPP_
#define _IFACE_HPP_

extern "C" {
#include "server_config.h"
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
}

#include "mpack/mpack.h"
#include <string>
#include <vector>

/// @brief handle properties of the stream, like file descriptor
/// (TODO: add more if necessary)
struct stream_t {
	int fd;
};

struct mpack_tree_t;
struct mpack_node_t;

// stream read and write functions
size_t read_stream(mpack_tree_t* tree, char* buffer, size_t count);
size_t write_stream(stream_t *stream, char* buffer, size_t count);

/// @brief Server action class, encapsulating the logic for telling
/// the hardware what to do and internally constructing a reply
/// containing relevant data etc returned from the hardware. Also
/// includes errors, warnings and/or info from each stage of the process.
class server_action {
public:
	/// @brief Interpret the 
	server_action(mpack_node_t request_root, char *reply_buffer);//, reply writer, reply buffer, hardware object);
	~server_action();
	/// @brief Run the request on the hardware
	void run();
	/// @brief Fill in the buffer to reply
	ssize_t construct_reply();
	void add_error(std::string s);	
	void add_warning(std::string s);
	void add_info(std::string s);
private:
	unsigned _request_type, _reply_index, _request_version;

	// temporarily public for dbg
public:
	char *_reply_buffer;
	ssize_t _reply_size;
private:
	mpack_writer_t _writer;
	std::vector<std::string> _infos, _warnings, _errors;

	/// @brief Verify the version of the protocol used by the client.
	/// If the major version differs, throw a runtime error.
	/// If the minor version differs, throw a warning.
	/// If the debug version differs, send back info.
	void check_version();	
};
	
///@brief Interface manager class, encapsulating the interface logic
class iface {
public:
	iface(unsigned port=11111);

	/// @brief Set up socket
	void init(unsigned port=11111);
	/// @brief Run request-response loop
	void run_stream(); // main
	/// @brief Unpack and act on each received packet, calling
	/// other methods and passing in nodes of the MPack tree where
	/// necessary. Return the size used in reply_buf.
	int process_message(mpack_node_t root, char *reply_buf);
	~iface();
private:
	struct sockaddr_in _address;
	size_t _addrlen;

	int _server_fd;
	stream_t _stream_fd;
	
	int16_t _pulse[PULSE_MEM_SIZE];
	uint64_t _buffer[COMMS_BUFFER];
};

#endif
