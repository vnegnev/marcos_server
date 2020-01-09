/* Communications interface to the host via Ethernet */

#ifndef _IFACE_HPP_
#define _IFACE_HPP_

#include "server_config.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class iface {
public:
	iface();
	~iface();
private:
	struct sockaddr_in addr;

	int16_t pulse[PULSE_MEM_SIZE];
	uint64_t buffer[COMMS_BUFFER];

	int yes = 1; // don't understand, TODO read setsockopt manpage
};

#endif
