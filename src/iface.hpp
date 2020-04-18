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

/// @brief stream read and write functions
size_t read_stream(mpack_tree_t* tree, char* buffer, size_t count);
void write_stream(mpack_writer_t* writer, const char* buffer, size_t count);

/// @brief MaRCoS msgpack packet types
enum marcos_packet {
	marcos_request=0,
	marcos_emergency_stop=1,
	marcos_close_server=2,
	marcos_reply=128,
	marcos_reply_error=129
};

/// @brief Server action class, encapsulating the logic for telling
/// the hardware what to do and internally constructing a reply
/// containing relevant data etc returned from the hardware. Also
/// includes errors, warnings and/or info from each stage of the process.
class server_action {
public:
	/// @brief Interpret the incoming request and start preparing the reply in advance
	server_action(mpack_node_t request_root, mpack_writer_t* writer); // TODO: add hardware object
	~server_action();
	/// @brief Run the request on the hardware, returning a basic error status
	int run_request();
	/// @brief Finish filling the buffer to reply: include the status messages and anything else left over
	ssize_t finish_reply();
	/// @brief Flush reply buffer to the stream
	void send_reply();
	void add_error(std::string s);	
	void add_warning(std::string s);
	void add_info(std::string s);
private:
	mpack_node_t _rd; /// @brief Short for request data; payload containing request data from client
	mpack_writer_t* _wr;
	unsigned _request_type, _reply_index, _request_version;
	std::vector<std::string> _errors, _warnings, _infos;

	/// @brief Encode the vectors of strings containing messages
	/// (errors, warnings and infos) into msgpack
	void encode_messages();

	/// @brief Verify the version of the protocol used by the client.
	/// If the major version differs, throw a runtime error.
	/// If the minor version differs, throw a warning.
	/// If the debug version differs, send back info.
	void check_version();

	/// @brief Carry out emergency stop: zero the RF and DACs, halt sequence, etc...
	void emergency_stop();
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
	bool _run_iface = true;
	
	struct sockaddr_in _address;
	size_t _addrlen;

	int _server_fd;
	stream_t _stream_fd;
	
	int16_t _pulse[PULSE_MEM_SIZE];
	uint64_t _buffer[COMMS_BUFFER];
};

#endif
