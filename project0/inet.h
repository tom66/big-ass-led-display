/*
 * Large display controller
 *
 * (C) 2017 Thomas Oldbury, released under MIT licence
 */

#ifndef ___INET_H___
#define ___INET_H___

#include "utils/lwiplib.h"
#include "utils/ustdlib.h"

// Choose any reasonable port above 1024 that isn't reserved for command interface
#define		INET_PORT_LISTEN					3333

// Choose any reasonable port above 1024 that isn't reserved for data interface
#define		INET_PORT_DATA						3337

// Size of command buffer; this needs to be large enough to accept a display write (up to 19.2KB)
#define		CMD_BUFFER_SIZE						32768

// Max arguments for simple commands
#define		CMD_ARGS_MAX							16
#define		SIMPLE_CMD_MAX_ARG_LEN		10

// Simple command max size, up to 64 bytes
#define		SIMPLE_CMD_SIZE						64
#define		SIMPLE_CMD_MAX_NAME_LEN		16
#define		SIMPLE_CMD_MAX_DESC_LEN		64

// Formatted reply max buffer size
#define		REPLY_PRINTF_SIZE					256

// Flags that can be used within a command_t struct
#define 	CMD_FLAG_SIMPLE						0x00000001
#define 	CMD_FLAG_END							0x80000000

// Hello response for the data channel
#define		DATA_MAGIC_RESP_HELLO			"\x55\xaa HELLO"

// Size of header for data packets
#define		DATA_HEADER_SIZE					7

// Command struct with command name, description and callbacks
struct command_t {
	const char *name;
	const char *desc;
	int32_t max_args;
	int32_t flags;
	void (*callback)(void);
};

// Data status structure, keep common variables in one common struct
struct data_state_t {
	uint32_t format;
	uint32_t data_mode;
	uint32_t x0, y0, x1, y1;
	uint32_t len_to_read;
};

// Callbacks
void tcpip_connected_callback(void *arg, struct tcp_pcb *tpcb, err_t err);
void tcpip_accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err);

// Interface
void tcpip_reply(struct tcp_pcb *newpcb, char *msg);
void tcpip_reply_formatted(struct tcp_pcb *newpcb, char *msg, ...);
void tcpip_init();
void tcpip_start_listen();

// Command processing
void cmd_recv_callback();
void cmd_simple_parse(char *cmd, uint32_t size);

// Actual commands
void cmd_help();
void cmd_red();
void cmd_green();
void cmd_blue();
void cmd_white();
void cmd_clear();
void cmd_setpix();
void cmd_setsubpix();
void cmd_commit();
void cmd_dispon();
void cmd_dispoff();
void cmd_brightness();
void cmd_status();
void cmd_reset();

#endif // ___INET_H___