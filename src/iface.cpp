#include "mpack/mpack.h"
#include "iface.hpp"

size_t read_stream(mpack_tree_t* tree, char* buffer, size_t count) {
	stream_t* stream = (stream_t *)mpack_tree_context(tree);
	ssize_t step = read(stream->fd, buffer, count);
	if (step <= 0) mpack_tree_flag_error(tree, mpack_error_io);
	return step;
}

iface::iface() {
	init();
}

void iface::init(unsigned port) {
	if ( (server_fd = socket(AF_INET, SOCK_STREAM, 0) ) < 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}
	
	int reuseaddr = 1; // whether to reuse the address
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
	               (void *)&reuseaddr , sizeof(reuseaddr)) ) {
		perror("setsockopt failed");
		exit(EXIT_FAILURE);
	}

	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(port);
	addrlen = sizeof(address);

	if (bind(server_fd, (struct sockaddr *)&address, addrlen) < 0) {
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	
	if ( listen(server_fd, 3) ) { // backlog of 3
		perror("listen failed");
		exit(EXIT_FAILURE);
	}
}

void iface::run_stream() {
	if((stream_fd.fd = accept(server_fd, NULL, NULL)) < 0) {
	// if((my_socket = accept(stream_fd.fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0) {
		perror("accept failed");
		exit(EXIT_FAILURE);
	}

	const unsigned max_size = 1024*1024;
	const unsigned max_nodes = 1024;

	// TODO: add an outer loop here
	mpack_tree_t tree;
	mpack_tree_init_stream(&tree, &read_stream, &stream_fd, max_size, max_nodes);
	while (true) {
		mpack_tree_parse(&tree);
		//mpack_tree_try_parse(&tree);
		if ( mpack_tree_error(&tree) != mpack_ok ) {
			fprintf(stderr, "MPack tree error %d; see enumerator at https://ludocode.github.io/mpack/group__common.html for info\n", mpack_tree_error(&tree));
			break;
		}
		if ( process_message(mpack_tree_root(&tree)) ) break;
	}

	if (mpack_tree_destroy(&tree) != mpack_ok) {
		perror("Error occurred tearing down the MPack tree\n");
		exit(EXIT_FAILURE);
	}
}

int iface::process_message(mpack_node_t root) {
	// printf("%s : %d, %s: %d",
	//        mpack_node_str(mpack_node_map_key_at(root, 0)),
	//        mpack_node_int(mpack_node_map_int(root, 0)),
	//        mpack_node_str(mpack_node_map_key_at(root, 1)),
	//        mpack_node_int(mpack_node_map_int(root, 1)));

	printf("%s, %s, %d\n",
	       mpack_node_cstr_alloc(mpack_node_array_at(root, 0), 10),
	       mpack_node_cstr_alloc(mpack_node_map_key_at(mpack_node_array_at(root, 1), 0), 10),
	       mpack_node_int(mpack_node_map_str(mpack_node_array_at(root, 1), "asdf", 4))
		);
	return 0;
}

iface::~iface() {
}
