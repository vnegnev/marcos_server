/* Communications interface to the host via Ethernet */

#ifndef _IFACE_HPP_
#define _IFACE_HPP_

#include "server_config.h"
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// handle properties of the stream
typedef struct stream_t {
	int fd;
} stream_t;

struct mpack_tree_t;
struct mpack_node_t;

// reader function
size_t read_stream(mpack_tree_t* tree, char* buffer, size_t count);

struct iface {
public:
	iface();
	void init(unsigned port=11111);
	// void parse_stream(stream_t *stream_context);
	void run_stream();
	int process_message(mpack_node_t root);
	~iface();
private:
	struct sockaddr_in address;
	size_t addrlen;

	int server_fd;
	stream_t stream_fd;
	
	int16_t pulse[PULSE_MEM_SIZE];
	uint64_t buffer[COMMS_BUFFER];

	int yes = 1; // don't understand, TODO read setsockopt manpage
};

#endif
