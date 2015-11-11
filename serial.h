
#ifndef SERIAL_H_
#define SERIAL_H_

#ifdef WIN32
#include <Windows.h>
typedef HANDLE serial_handle_t;
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
typedef int serial_handle_t;
#define INVALID_HANDLE_VALUE (-1)
#endif

#include <string>

struct SerialOption {
	std::string port;
	int speed;
#ifndef WIN32
	struct termios saveattr;
#endif
};

bool serial_open(SerialOption const *option, serial_handle_t *handle_p);
void serial_close(serial_handle_t fd, SerialOption *option);
size_t serial_write(serial_handle_t fd, void const *ptr, size_t len);
size_t serial_read(serial_handle_t fd, char *ptr, size_t len, unsigned int timeout_ms);
void serial_flush_input(serial_handle_t handle, unsigned int timeout_ms);

#endif
