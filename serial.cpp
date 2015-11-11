
#include "serial.h"


#ifdef WIN32
bool serial_open(SerialOption const *option, serial_handle_t *handle_p)
{
	serial_handle_t fd;
	DCB dcb;
	fd = CreateFile(option->port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (fd == INVALID_HANDLE_VALUE) {
		return false;
	}

	GetCommState(fd, &dcb);
	dcb.BaudRate = option->speed;
	dcb.fBinary = TRUE;
	dcb.fParity = FALSE;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = FALSE;
	dcb.fDtrControl = DTR_CONTROL_ENABLE;
	dcb.fDsrSensitivity = FALSE;
	dcb.fTXContinueOnXoff = FALSE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	dcb.fErrorChar = FALSE;
	dcb.fNull = FALSE;
	dcb.fRtsControl = RTS_CONTROL_ENABLE;
	dcb.fAbortOnError = FALSE;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	SetCommState(fd, &dcb);

	*handle_p = fd;
	return true;
}
#else
bool serial_open(SerialOption const *option, serial_handle_t *handle_p)
{
	int speed;
	serial_handle_t fd;
	struct termios attr;

	fd = open(option->port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		return false;
	}

	tcgetattr(fd, &attr);
	option->saveattr = attr;

	switch (option->speed) {
//	case      50:	speed =      B50;		break;
//	case      75:	speed =      B75;		break;
//	case     110:	speed =     B110;		break;
//	case     134:	speed =     B134;		break;
//	case     150:	speed =     B150;		break;
// 	case     200:	speed =     B200;		break;
	case     300:	speed = B300;		break;
	case     600:	speed = B600;		break;
	case    1200:	speed = B1200;		break;
//	case    1800:	speed =    B1800;		break;
	case    2400:	speed = B2400;		break;
	case    4800:	speed = B4800;		break;
	case    9600:	speed = B9600;		break;
	case   19200:	speed = B19200;		break;
	case   38400:	speed = B38400;		break;
	case   57600:	speed = B57600;		break;
	case  115200:	speed = B115200;		break;
	case  230400:	speed = B230400;		break;
//	case  460800:	speed =  B460800;		break;
//	case  500000:	speed =  B500000;		break;
//	case  576000:	speed =  B576000;		break;
//	case  921600:	speed =  B921600;		break;
//	case 1000000:	speed = B1000000;		break;
//	case 1152000:	speed = B1152000;		break;
//	case 1500000:	speed = B1500000;		break;
//	case 2000000:	speed = B2000000;		break;
//	case 2500000:	speed = B2500000;		break;
//	case 3000000:	speed = B3000000;		break;
//	case 3500000:	speed = B3500000;		break;
//	case 4000000:	speed = B4000000;		break;
	default:
		speed = B38400;
		break;
	}

	cfsetispeed(&attr, speed);
	cfsetospeed(&attr, speed);
	cfmakeraw(&attr);

	attr.c_cflag &= ~CSIZE;
	attr.c_cflag |= CS8 | CLOCAL | CREAD;
	attr.c_iflag = 0;
	attr.c_oflag = 0;
	attr.c_lflag = 0;
	attr.c_cc[VMIN] = 1;
	attr.c_cc[VTIME] = 0;

	tcsetattr(fd, TCSANOW, &attr);

	*handle_p = fd;
	return true;
}
#endif

#ifdef WIN32
void serial_close(serial_handle_t fd, SerialOption *option)
{
	CloseHandle(fd);
}
#else
void serial_close(serial_handle_t fd, SerialOption *option)
{
	tcsetattr(fd, TCSANOW, &option->saveattr);
	close(fd);
}
#endif

size_t serial_write(serial_handle_t fd, void const *ptr, size_t len)
{
#ifdef WIN32
	DWORD bytes = 0;
	if (WriteFile(fd, ptr, len, &bytes, nullptr)) {
		return bytes;
	}
	DWORD e = GetLastError();
	return 0;
#else
	return write(fd, ptr, len);
#endif
}

size_t serial_read(serial_handle_t fd, char *ptr, size_t len, unsigned int timeout_ms)
{
	unsigned long bytes;
#ifdef WIN32
	COMMTIMEOUTS to;
	GetCommTimeouts(fd, &to);
	to.ReadIntervalTimeout = 0;
	to.ReadTotalTimeoutMultiplier = 0;
	to.ReadTotalTimeoutConstant = timeout_ms < 1 ? 1 : timeout_ms;
	to.WriteTotalTimeoutMultiplier = 0;
	to.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts(fd, &to);
	bytes = 0;
	if (ReadFile(fd, ptr, len, &bytes, nullptr)) {
		return bytes;
	}
	return 0;
#else
	fd_set fds;
	struct timeval to;
	unsigned long n;
	unsigned char c;

	bytes = 0;
	while (len > 0) {
		FD_ZERO(&fds);
		FD_SET(fd, &fds);
		to.tv_sec = timeout_ms / 1000;
		to.tv_usec = (timeout_ms % 1000) * 1000;
		n = select(fd + 1, &fds, nullptr, nullptr, &to);
		if (!FD_ISSET(fd, &fds)) break;
		n = read(fd, ptr, len);
		bytes += n;
		len -= n;
		ptr += n;
	}
	return bytes;
#endif
}

void serial_flush_input(serial_handle_t handle, unsigned int timeout_ms)
{
	char tmp[256];
	while (serial_read(handle, tmp, sizeof(tmp), timeout_ms) > 0);
}

