#include "mpack/mpack.h"
#include "version.hpp"
#include "iface.hpp"
#include <sstream>

std::string version_str(unsigned ver) {
	std::stringstream v;
	v << ((ver & 0xff0000) >> 16) << "." << ((ver & 0xff00) >> 8) << "." << (ver & 0xff);
	return v.str();
}

size_t read_stream(mpack_tree_t* tree, char* buffer, size_t count) {
	stream_t* stream = (stream_t *)mpack_tree_context(tree);
	ssize_t step = read(stream->fd, buffer, count);
	if (step <= 0) mpack_tree_flag_error(tree, mpack_error_io);
	return step;
}

size_t write_stream(stream_t *stream, char* buffer, size_t count) {
	ssize_t amount = write(stream->fd, buffer, count);
	if (amount <= 0) return 0;// MPACK FLAG ERROR;
	return amount;
}

server_action::server_action(mpack_node_t request_root, char *reply_buffer):
	_reply_buffer(reply_buffer)
{
	auto r = request_root;
	_request_type = mpack_node_uint(mpack_node_array_at(r, 0));
	_reply_index = mpack_node_uint(mpack_node_array_at(r, 1));
	_request_version = mpack_node_uint(mpack_node_array_at(r, 3));

	mpack_node_t payload = mpack_node_array_at(r, 4);
	check_version();

}

server_action::~server_action() {
	// if (mpack_tree_destroy(&_tree) != mpack_ok) {
	// 	perror("Error occurred tearing down the reply MPack tree\n");
	// 	// throw std::runtime_error("Error occurred tearing down an MPack tree"); // SHOULD NOT THROW EXCEPTIONS IN DESTRUCTORS
	// }
}

ssize_t server_action::construct_reply() {
	// TODO: use the mpack writer for this
	strncpy(_reply_buffer, "Hello world!", 100);
	_reply_size = strlen(_reply_buffer);
	return _reply_size;
}

void server_action::add_error(std::string s) { _errors.push_back(s); }
void server_action::add_warning(std::string s) { _warnings.push_back(s); }
void server_action::add_info(std::string s) { _infos.push_back(s); }

iface::iface(unsigned port) {
	init(port);
}

void iface::init(unsigned port) {
	if ( (_server_fd = socket(AF_INET, SOCK_STREAM, 0) ) < 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}
	
	int reuseaddr = 1; // whether to reuse the address
	if (setsockopt(_server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
	               (void *)&reuseaddr , sizeof(reuseaddr)) ) {
		perror("setsockopt failed");
		exit(EXIT_FAILURE);
	}

	_address.sin_family = AF_INET;
	_address.sin_addr.s_addr = INADDR_ANY;
	_address.sin_port = htons(port);
	_addrlen = sizeof(_address);

	if (bind(_server_fd, (struct sockaddr *)&_address, _addrlen) < 0) {
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	
	if ( listen(_server_fd, 3) ) { // backlog of 3
		perror("listen failed");
		exit(EXIT_FAILURE);
	}
}

void iface::run_stream() {
	if((_stream_fd.fd = accept(_server_fd, NULL, NULL)) < 0) {
	// if((my_socket = accept(stream_fd.fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0) {
		perror("accept failed");
		exit(EXIT_FAILURE);
	}

	const unsigned max_size = 1024*1024;
	const unsigned max_nodes = 1024;
	
	char reply_buf[max_size]; // maybe should go on heap?

	// TODO: add an outer loop here

	// mpack reader mechanism
	mpack_tree_t tree;
	mpack_tree_init_stream(&tree, &read_stream, &_stream_fd, max_size, max_nodes);

	// mpack writer mechanism
	// CONTINUE HERE; use mpack_writer_init(), mpack_writer_set_context(), mpack_writer_set_flush() and maybe mpack_writer_set_error_handler(); then when ready to send stuff, mpack_writer_flush_message(). First though, test basic string replies.
	while (true) {
		mpack_tree_parse(&tree);
		//mpack_tree_try_parse(&tree);
		if ( mpack_tree_error(&tree) != mpack_ok ) {
			fprintf(stderr, "MPack tree error %d; see enumerator at https://ludocode.github.io/mpack/group__common.html for info\n", mpack_tree_error(&tree));
			break;
		}

		// Reply: use a constant buffer
		server_action sa(mpack_tree_root(&tree), reply_buf);
		
		// ssize_t reply_size = process_message(mpack_tree_root(&tree), reply_buf);
		// if (reply_size < 0) break; // DO MORE
		sa.construct_reply();
		write(_stream_fd.fd, sa._reply_buffer, sa._reply_size); // DO THIS WITH MPACK INSTEAD; SEE ABOVE COMMENTS
	}

	if (mpack_tree_destroy(&tree) != mpack_ok) {
		perror("Error occurred tearing down the MPack tree\n");
		exit(EXIT_FAILURE);
	}
}

void server_action::check_version() {	
	char client_version_major = (_request_version & 0xff0000) >> 16;
	char client_version_minor = (_request_version & 0xff00) >> 8;
	char client_version_debug = _request_version & 0xff;	

	std::stringstream e;
	
	if (client_version_major != VERSION_MAJOR) { // major version mismatch
		e << "Client version " << version_str(_request_version) << " significantly different from server version " << SERVER_VERSION << std::endl;
		add_error(e.str());
		// throw std::runtime_error(e.str());
	} else if (client_version_minor != VERSION_MINOR) {
		e << "Client version " << version_str(_request_version) << " different from server version " << SERVER_VERSION << std::endl;		
		add_warning(e.str());
	} else if (client_version_debug != VERSION_DEBUG) {
		e << "Client version " << version_str(_request_version) << " differs slightly from server version " << SERVER_VERSION << std::endl;		
		add_info(e.str());
	}
}

iface::~iface() {
}
