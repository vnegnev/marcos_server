#include "mpack/mpack.h"
#include "iface.hpp"

size_t read_stream(mpack_tree_t* tree, char* buffer, size_t count) {
	stream_t* stream = (stream_t *)mpack_tree_context(tree);
	ssize_t step = read(stream->fd, buffer, count);
	if (step <= 0) mpack_tree_flag_error(tree, mpack_error_io);
	return step;
}

iface::iface() {
	// stream_fd = new stream_t();
}

void iface::init(unsigned port) {
	if ( (stream_fd.fd = socket(AF_INET, SOCK_STREAM, 0) ) < 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}
	
	int reuseaddr = 1; // whether to reuse the address
	if (setsockopt(stream_fd.fd, SOL_SOCKET, SO_REUSEADDR,
	               (void *)&reuseaddr , sizeof(reuseaddr)) ) {
		perror("setsockopt failed");
		exit(EXIT_FAILURE);
	}

	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = port; 

	if (bind(stream_fd.fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	
	if ( listen(stream_fd.fd, 1024) ) { // backlog of 1024
		perror("listen failed");
		exit(EXIT_FAILURE);
	}

	if((my_socket = accept(stream_fd.fd, NULL, NULL)) < 0) {
		perror("accept failed");
		exit(EXIT_FAILURE);
	}	
}

void iface::parse_stream() {
	const unsigned max_size = 1024*1024;
	const unsigned max_nodes = 1024;
	mpack_tree_t tree;
	mpack_tree_init_stream(&tree, &read_stream, &stream_fd, max_size, max_nodes);
	while (true) {
		mpack_tree_parse(&tree);
		if ( mpack_tree_error(&tree) != mpack_ok ) break;
		if ( process_message(mpack_tree_root(&tree)) ) break;
	}

	if (mpack_tree_destroy(&tree) != mpack_ok) {
		perror("An error occurred tearing down the tree!\n");
		exit(EXIT_FAILURE);
	}
}

int iface::process_message(mpack_node_t tree) {
	printf("Received a message!");
	
	return 0;
}

iface::~iface() {
	// delete stream_fd;
}
