
#include "serial.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <string>

#if 1
#include "mywifi.h"
#else
#define WIFI_SSID "SSID"
#define WIFI_PASS "PassPhrase"
#endif

#ifdef WIN32
#pragma warning(disable:4996)
#define COMPORT "\\\\.\\COM1"
#else
//#define COMPORT "/dev/ttyS0"
#define COMPORT "/dev/ttyUSB0"
#endif

#ifdef WIN32
unsigned int get_tick_count()
{
	return GetTickCount();
}
#else
unsigned int get_tick_count()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}
#endif



class ESP8266 {
public:

	static const int DEFAULT_TIMEOUT = 100;

	SerialOption opt;
	serial_handle_t handle;

	void config(std::string const &port, int speed)
	{
		opt.port = port;
		opt.speed = speed;
	}

	bool open()
	{
		return serial_open(&opt, &handle);
	}

	void close()
	{
		serial_close(handle, &opt);
	}

	size_t send(std::string const &str)
	{
		return serial_write(handle, str.c_str(), str.size());
	}

	void send_command(std::string str)
	{
		str += "\r\n";
		serial_write(handle, str.c_str(), str.size());
	}

	void echo(std::string const &str)
	{
		puts(str.c_str());
	}

	enum class ReceiveMode {
		AT,
		HTTP,
		CIPSEND,
	};


	void recv_response(ReceiveMode mode, std::vector<std::string> *lines, unsigned int timeout_ms = 0)
	{
		if (timeout_ms == 0) {
			timeout_ms = DEFAULT_TIMEOUT;
		}
		lines->clear();
		std::vector<char> vec;
		char tmp[1024];
		bool cr = false;
		int i = 0;
		int offset = 0;
		int length = 0;
		vec.reserve(1024);
		unsigned int starttick = get_tick_count();
		while (1) {
			if (offset >= length) {
				if (mode != ReceiveMode::HTTP) {
					if (get_tick_count() - starttick >= timeout_ms) break;
				}
				length = serial_read(handle, tmp, sizeof(tmp), 3);
				if (length < 0 || length > sizeof(tmp)) break;
				if (length == 0) continue;
				offset = 0;
			}
			int c = tmp[offset] & 0xff;
			offset++;
			if (c == 0x0d) {
				cr = true;
			} else {
				if (cr && c == 0x0a) {
					std::string s;
					if (!vec.empty()) {
						char const *p = &vec[0];
						int n = vec.size();
						while (n > 0 && isspace(vec[n - 1] & 0xff)) n--;
						s.assign(p, p + n);
					}
					lines->push_back(s);
					echo(s);
					if (mode == ReceiveMode::AT && s == "OK") {
						break;
					}
					if (mode == ReceiveMode::HTTP) {
						if (lines->size() > 2 && s.empty()) break;
						if (strstr(s.c_str(), ",CLOSED")) break;
						if (strstr(s.c_str(), ",CONNECT FAIL")) break;
					}
					if (mode == ReceiveMode::CIPSEND && s == "SEND OK") {
						break;
					}
					vec.clear();
				} else {
					vec.push_back(c);
				}
				cr = false;
			}
		}
	}

	bool http_accept(unsigned int *id, std::vector<char> *out)
	{
		out->clear();
		std::vector<char> vec;
		char tmp[1024];
		bool connected = false;
		unsigned int connected_id = 0;
		bool cr = false;
		int offset = 0;
		int length = 0;
		vec.reserve(1024);
		unsigned int starttick = get_tick_count();
		while (1) {
			if (offset >= length) {
				length = serial_read(handle, tmp, sizeof(tmp), 3);
				if (length < 0 || length > sizeof(tmp)) break;
				if (length == 0) continue;
				offset = 0;
			}
			if (connected) {
				if (vec.size() > 5 && memcmp(&vec[0], "+IPD,", 5) == 0) {
					for (size_t i = 5; i < vec.size(); i++) {
						if (vec[i] == ':') {
							char const *p = &vec[0];
							std::string s(p + 5, p + i);
							unsigned int len = 0;
							if (sscanf(s.c_str(), "%u,%u", id, &len) == 2 && *id == connected_id) {
								if (len > 100000) len = 0;
								out->resize(len);
								if (len > 0) {
									size_t pos = 0;
									if (offset < length) {
										pos = length - offset;
										if (pos > len) pos = len;
										memcpy(&out->at(0), tmp + offset, pos);
									}
									while (pos < len) {
										int n = serial_read(handle, &out->at(pos), len - pos, 100);
										if (n < 1) break;
										pos += n;
									}
								}
								return true;
							}
							connected = false;
						}
					}
				}
			}
			int c = tmp[offset] & 0xff;
			offset++;
			if (c == 0x0d) {
				cr = true;
			} else {
				if (cr && c == 0x0a) {
					std::string s;
					if (!vec.empty()) {
						char const *p = &vec[0];
						int n = vec.size();
						while (n > 0 && isspace(vec[n - 1] & 0xff)) n--;
						s.assign(p, p + n);
					}
					if (sscanf(s.c_str(), "%u,CONNECT", id) == 1) {
						connected = true;
						connected_id = *id;
					} else if (sscanf(s.c_str(), "%u,CLOSED", id) == 1) {
						break;
					} else if (sscanf(s.c_str(), "%u,CONNECT FAIL", id) == 1) {
						break;
					}
					vec.clear();
				} else {
					vec.push_back(c);
				}
				cr = false;
			}
		}
		return false;
	}

	bool containsOK(std::vector<std::string> const *lines)
	{
		for (std::string const &line : *lines) {
			if (line == "OK") {
				return true;
			}
		}
		return false;
	}

	bool command_(std::string const &command, std::vector<std::string> *lines, int timeout_ms = 0)
	{
		lines->clear();
		if (!command.empty()) send_command(command);
		recv_response(ReceiveMode::AT, lines, timeout_ms);
		return containsOK(lines);
	}

	bool run_command(std::string const &command, std::vector<std::string> *lines, int retry = 3, int timeout_ms = 0)
	{
		while (retry > 0) {
			if (command_(command, lines, timeout_ms)) {
				return true;
			}
			serial_flush_input(handle, 100);
			retry--;
		}
		return false;
	}

	bool run_command(std::string const &command)
	{
		std::vector<std::string> lines;
		if (run_command(command, &lines, 3)) {
			return true;
		}
		return false;
	}

	static std::string trim(char const **left, char const **right)
	{
		char const *l = *left;
		char const *r = *right;
		while (l < r && isspace(*l & 0xff)) l++;
		while (l < r && isspace(r[-1] & 0xff)) r--;
		return std::string(l, r);
	}

	static std::string trim_quot(std::string const &str)
	{
		char const *left = str.c_str();
		char const *right = left + str.size();
		char const *l = left;
		char const *r = right;
		if (l + 1 < r && *l == '\"' && r[-1] == '\"') {
			l++;
			r--;
		}
		if (l == left && r == right) return str;
		return std::string(l, r);
	}

	static bool lookup(std::vector<std::string> const *lines, std::string const &key, std::string *out, char sep = ':')
	{
		for (std::string const &line : *lines) {
			char const *begin = line.c_str();
			char const *end = begin + line.size();
			char const *p = strchr(begin, sep);
			if (p) {
				char const *left;
				char const *right;;
				left = begin;
				right = p;
				std::string k = trim(&left, &right);
				left = p + 1;
				right = end;
				std::string v = trim(&left, &right);
				if (k == key) {
					*out = v;
					return true;
				}
			}
		}
		out->clear();
		return false;
	}

	bool connect(std::string const &ssid, std::string const &pw)
	{
		std::string cmd = "AT+CWJAP=\"" + ssid + "\",\"" + pw + "\"";
		std::vector<std::string> alllines;
		for (int i = 0; i < 600; i++) {
			std::vector<std::string> lines;
			bool ok = command_(cmd, &lines, 100);
			alllines.insert(alllines.begin(), lines.begin(), lines.end());
			if (ok) return true;
			cmd.clear();
		}
		return false;
	}

	std::string get_ip_address()
	{
		std::string ipaddr;
		std::vector<std::string> lines;
		run_command("AT+CIFSR", &lines);
		lookup(&lines, "+CIFSR:STAIP", &ipaddr, ',');
		ipaddr = trim_quot(ipaddr);
		return ipaddr;
	}

	struct mac_address_t {
		uint8_t a = 0;
		uint8_t b = 0;
		uint8_t c = 0;
		uint8_t d = 0;
		uint8_t e = 0;
		uint8_t f = 0;
	};

	static std::string to_s(mac_address_t const &mac)
	{
		char tmp[100];
		sprintf(tmp, "%02X:%02X:%02X:%02X:%02X:%02X", mac.a, mac.b, mac.c, mac.d, mac.e, mac.f);
		return tmp;
	}

	bool get_mac_address_(std::string const &cmd, std::string const &key, mac_address_t *out)
	{
		*out = mac_address_t();
		std::vector<std::string> lines;
		if (run_command(cmd, &lines)) {
			std::string t;
			if (lookup(&lines, key, &t)) {
				t = trim_quot(t);
				int a, b, c, d, e, f;
				if (sscanf(t.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x", &a, &b, &c, &d, &e, &f) == 6) {
					out->a = a;
					out->b = b;
					out->c = c;
					out->d = d;
					out->e = e;
					out->f = f;
				}
				return true;
			}
		}
		return false;
	}

	bool get_st_mac_address(mac_address_t *out)
	{
		return get_mac_address_("AT+CIPSTAMAC?", "+CIPSTAMAC", out);
	}

	bool get_ap_mac_address(mac_address_t *out)
	{
		return get_mac_address_("AT+CIPAPMAC?", "+CIPAPMAC", out);
	}
};

std::string to_s(size_t n)
{
	char tmp[100];
	sprintf(tmp, "%u", n);
	return tmp;
}

std::string parse_http_request(std::string const &str)
{
	char const *begin = str.c_str();
	char const *end = begin + str.size();
	if (strncmp(begin, "GET ", 4) == 0) {
		char const *left = begin + 4;
		char const *right = left;
		while (right < end) {
			if (isspace(*right & 0xff)) {
				break;
			}
			right++;
		}
		if (left < right) {
			return std::string(left, right);
		}
	}
	return std::string();
}

std::string get_body(std::string const &path)
{
	time_t t = time(0);
	char *p = ctime(&t);
	return p ? std::string(p) : std::string();
}

size_t parse_http_head(char const *begin, char const *end, std::vector<std::string> *lines)
{
	lines->clear();
	char const *left = begin;
	char const *ptr = begin;
	while (ptr < end) {
		int c = -1;
		if (ptr < end) {
			c = *ptr & 0xff;
		}
		if (c == '\r' || c == '\n' || c == -1) {
			char const *right = ptr;
			std::string s(left, right);
			if (!s.empty()) lines->push_back(s);
			if (c == -1) break;
			ptr++;
			if (c == '\r') {
				if (ptr < end && *ptr == '\n') {
					ptr++;
				}
			}
			if (s.empty()) {
				break;
			}
			left = ptr;
		} else {
			ptr++;
		}
	}
	return ptr - begin;
}

int main(int argc, char **argv)
{
	std::string ssid = WIFI_SSID;
	std::string pass = WIFI_PASS;

	ESP8266 esp;
	esp.config(COMPORT, 115200);
	if (esp.open()) {
		if (esp.run_command("AT")) {
			bool ok;
			ok = esp.run_command("AT+CWMODE=1");
			ok = esp.connect(ssid, pass);
			if (ok) {
				std::string addr = esp.get_ip_address();
				ok = esp.run_command("AT+CIPMUX=1");
				ok = esp.run_command("AT+CIPSERVER=1,80");
				if (ok) {
					while (1) {
						std::vector<std::string> reqhead;
						std::vector<char> reqbody;
						unsigned int id = 0;
						ok = esp.http_accept(&id, &reqbody);
						if (ok && !reqbody.empty()) {
							char const *p = &reqbody[0];
							size_t offset = parse_http_head(p, p + reqbody.size(), &reqhead);
							std::string str;
							if (reqhead.size() > 0) {
								std::string path = parse_http_request(reqhead[0]);
								if (!path.empty()) {
									std::string head;
									std::string body = get_body(path);
									head += "HTTP/1.0 OK\r\n";
									head += "body-Type: text/plain\r\n";
									head += "\r\n";
									unsigned int len = head.size() + body.size();
									str = "AT+CIPSEND=" + to_s(id) + "," + to_s(len) + "\r\n";
									std::vector<std::string> lines;
									ok = esp.run_command(str, &lines, 1, 500);
									if (ok) {
										esp.send(head);
										esp.send(body);
										esp.recv_response(ESP8266::ReceiveMode::CIPSEND, &lines, 1000);
									}
								}
							}
							str = "AT+CIPCLOSE=" + to_s(id);
							ok = esp.run_command(str);
						}
					}
				}
			}
		}

		esp.close();
	}

	return 0;
}

