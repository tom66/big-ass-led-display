/*
 * Large display controller
 *
 * (C) 2017 Thomas Oldbury, released under MIT licence
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>

#include "inet.h"
#include "hal.h"
#include "dispctl.h"

#include "tm4c129encpdt.h"

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_udma.h"
#include "inc/hw_ssi.h"

#include "driverlib/debug.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/hibernate.h"
#include "driverlib/pwm.h"
#include "driverlib/flash.h"
#include "driverlib/rom_map.h"

#include "utils/lwiplib.h"
#include "utils/ustdlib.h"

// Buffer for received commands
char cmd_buffer[CMD_BUFFER_SIZE];
uint32_t cmd_buff_len = 0;
uint32_t cmd_ready = 0;

struct data_state_t data_state;

char reply_buffer_resp[REPLY_PRINTF_SIZE];

// pcb for reply messages (global - we only support one client)
struct tcp_pcb *reply_pcb;

// Command argument stack and number of commands read. Args read beyond count may be invalid.
uint32_t command_arg_count;
uint32_t command_args_int[CMD_ARGS_MAX];

// Defined commands
const struct command_t commands[] = {
	{
		"help",
		"Show this help message",
		0,
		CMD_FLAG_SIMPLE,
		&cmd_help
	}, {
		"red",
		"Set screen to red (test)",
		0,
		CMD_FLAG_SIMPLE,
		&cmd_red
	}, {
		"green",
		"Set screen to green (test)",
		0,
		CMD_FLAG_SIMPLE,
		&cmd_green
	}, {
		"blue",
		"Set screen to blue (test)",
		0,
		CMD_FLAG_SIMPLE,
		&cmd_blue
	}, {
		"white",
		"Set screen to white (test)",
		0,
		CMD_FLAG_SIMPLE,
		&cmd_white
	}, {
		"clear",
		"Clear screen",
		0,
		CMD_FLAG_SIMPLE,
		&cmd_clear
	}, {
		"setpix",
		"Set pixel, arguments: x, y, r, g, b",
		5,
		CMD_FLAG_SIMPLE,
		&cmd_setpix
	}, {
		"setsubpix",
		"Set subpixel, arguments: x, y, r, g, b",
		5,
		CMD_FLAG_SIMPLE,
		&cmd_setsubpix
	}, {
		"commit",
		"Commit display to output",
		0,
		CMD_FLAG_SIMPLE,
		&cmd_commit
	}, {
		"dispoff",
		"Stop displaying stuff",
		0,
		CMD_FLAG_SIMPLE,
		&cmd_dispoff
	}, {
		"dispon",
		"Start displaying stuff (default)",
		0,
		CMD_FLAG_SIMPLE,
		&cmd_dispon
	}, {
		"brightness",
		"Set brightness from 0-255 (0 = darkest, 255 = brightest)",
		1,
		CMD_FLAG_SIMPLE,
		&cmd_brightness
	}, {
		"status",
		"Get some display status parameters",
		0,
		CMD_FLAG_SIMPLE,
		&cmd_status
	}, {
		"reset",
		"Reset CPU.  This will break the connection",
		0,
		CMD_FLAG_SIMPLE,
		&cmd_reset
	}, {
		"", 
		"", 
		0, 
		CMD_FLAG_END,
		0
	}
};

/**
 * SysTickIntHandler:  Interrupt for the 100Hz system timer, used for LwIP tasks.
 */
void SysTickIntHandler(void)
{
	lwIPTimer(SYSTICK_TIME_MS);
}
	
/**
 * lwIPHostTimerHandler:  Host timer handler - currently a stub.
 */
void lwIPHostTimerHandler()
{
	return;
}

/** 
 * tcpip_recv_callback: TCP received data callback; handles processing commands.
 */
err_t tcpip_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
	uint32_t n, got_newline = 0, xx, yy;
	char c;
	char *data;
	
	//UARTprintf("tcpip_recv_callback() err_t=%d pbuf=0x%08x (port=%d)\r\n", err, &p, tpcb->local_port);
	
	reply_pcb = tpcb;
	
	// For the command port treat data as commands and look for new lines to bound command inputs.
	if(tpcb->local_port == INET_PORT_LISTEN) {
		// Stuff data into the command buffer if there is space.
		if((cmd_buff_len + p->len) < CMD_BUFFER_SIZE) {
			// Copy bytes until we get a newline, then run the parser
			for(n = 0; n < p->len; n++) {
				c = *(((char*)p->payload) + n);
				if(c == '\n' || c == '\r') {
					got_newline = 1;
					break;
				}
				
				cmd_buffer[cmd_buff_len++] = c;
			}
			
			if(got_newline) {
				cmd_buffer[cmd_buff_len++] = 0;
				
				// Notify our loop function that data is available.
				cmd_ready = 1;
			}
		}
	} else if(tpcb->local_port == INET_PORT_DATA) {
		/*
		 * For the data interface, we accept limited types of command that allow the display to be
		 * written quickly.
		 *
		 * The first byte is the type of command (0x01 = RGBG write, 0x02 = RGB write)
		 * The first two bytes indicate the start pixels x and y
		 * The next two bytes indicate the end pixels x and y
		 * The next two bytes indicate the number of bytes to expect to follow.
		 * Data follows in either RGB format or RGBG format.
		 */
		
		// Store data if there is space available.  This software isn't watertight but avoid obvious memory
		// corruption issues.
		if((p->len + cmd_buff_len) < CMD_BUFFER_SIZE) {
			memcpy(cmd_buffer + cmd_buff_len, p->payload, p->len);
			cmd_buff_len += p->len;
			
			if(cmd_buff_len >= DATA_HEADER_SIZE && data_state.data_mode == 0) {
				// Read header parameters
				data_state.format = cmd_buffer[0];
				data_state.x0 = cmd_buffer[1];
				data_state.y0 = cmd_buffer[2];
				data_state.x1 = cmd_buffer[3];
				data_state.y1 = cmd_buffer[4];
				data_state.len_to_read = (cmd_buffer[5] << 8) | cmd_buffer[6];
				
				UARTprintf("params fmt=%d s=%d,%d e=%d,%d len=%d (p->len=%d, cmd_buff_len=%d)\r\n", \
					data_state.format, data_state.x0, data_state.y0, data_state.x1, data_state.y1, data_state.len_to_read, p->len, cmd_buff_len);
				
				// Indicate that we're in the second mode
				data_state.data_mode = 1;
			}
			
			// When in mode 1 see if there's enough data yet.
			if(data_state.data_mode == 1) {
				// Have we read enough data?
				if(cmd_buff_len >= (data_state.len_to_read - DATA_HEADER_SIZE)) {
					// OK, commit the data..
					UARTprintf("got enough data (p->len=%d, cmd_buff_len=%d, reqd=%d), committing\r\n", p->len, cmd_buff_len, data_state.len_to_read);
					
					// We only support format 0x02 at the moment
					if(data_state.format == 0x02) {
						for(n = 0, yy = data_state.y0; yy < data_state.y1; yy++) {
							for(xx = data_state.x0; xx < data_state.x1; xx++, n += 3) {
								disp_write_pixel_rgb(xx, yy, cmd_buffer[DATA_HEADER_SIZE + n], cmd_buffer[DATA_HEADER_SIZE + n + 1], cmd_buffer[DATA_HEADER_SIZE + n + 2]);
							}
						}
					}
					
					disp_flush_display(0);
					
					// ready to accept the next packet
					data_state.data_mode = 0;
					cmd_buff_len = 0;
				}
			}
		}
	}
	
	// Signal to LwIP we are ready for more data
	tcp_recved(tpcb, p->len);
	
	// Free the pbuf; we're done here
	pbuf_free(p);
	
	return ERR_OK;
}

/**
 * tcpip_connected_callback: TCP connected callback; handles establishing connection.
 */
void tcpip_connected_callback(void *arg, struct tcp_pcb *tpcb, err_t err)
{
	UARTprintf("tcpip_connected_callback() err_t=%d reached\r\n", err);
	
	// Set up the receive callback
	tcp_recv(tpcb, tcpip_recv_callback);
	
	// What port did the request come from?
	if(tpcb->local_port == INET_PORT_LISTEN) {
		// Enqueue a "hello" message for the simple command interface.
		tcpip_reply(tpcb, "HELLO THIS IS DISPLAY.\r\n");
		tcpip_reply(tpcb, "v0.1, type `!help' for commands, or not, I'm not your mum.\r\n");
	} else if(tpcb->local_port == INET_PORT_DATA) {
		// Reply with some magic bytes for the data interface
		tcpip_reply(tpcb, DATA_MAGIC_RESP_HELLO);
	}
}

/** 
 * tcpip_accept_callback:  Callback for incoming connections to our port. 
 */
void tcpip_accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
	tcp_accepted(newpcb);
	tcp_connect(newpcb, IP_ADDR_ANY, INET_PORT_LISTEN, tcpip_connected_callback);
	
	UARTprintf("tcpip_accept_callback() err_t=%d reached, stub\r\n", err);
}

/**
 * tcpip_sent_callback: Sent callback - just prints debug message.
 */
void tcpip_sent_callback()
{
	UARTprintf("tcpip_sent_callback() stub\r\n");
}

/**
 * tcpip_reply: simple string reply to packets.
 */
void tcpip_reply(struct tcp_pcb *newpcb, char *msg)
{
	tcp_write(newpcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);
	tcp_output(newpcb);
	
	// Delay to allow write out
	//SysCtlDelay(100000);
	
	//UARTprintf("reply sent %s\r\n", msg);
}

/**
 * tcpip_reply_formatted: simple string reply to packets.
 */
void tcpip_reply_formatted(struct tcp_pcb *newpcb, char *msg, ...)
{
	char buffer[REPLY_PRINTF_SIZE];
	
  va_list args;
  va_start(args, msg);
	
  vsnprintf(buffer, REPLY_PRINTF_SIZE, msg, args);
  tcpip_reply(newpcb, buffer);
	
  va_end(args);
}

/**
 * tcpip_init:  Initialise ethernet/TCPIP stack
 */
void tcpip_init()
{
	uint32_t mac_flash_0, mac_flash_1, cycles;
	uint32_t ip_addr, netmask, gateway;
	uint8_t mac_bytes[8];
	
	// ** Setup Ethernet controller **
	// Configure SysTick for a periodic interrupt.
	MAP_SysTickPeriodSet(CPU_CLOCK / SYSTICK_RATE);
	MAP_SysTickEnable();
	MAP_SysTickIntEnable();
	
	// Read MAC, abort if unprogramed
	MAP_FlashUserGet(&mac_flash_0, &mac_flash_1);
	
	if((mac_flash_0 == 0xffffffff) || (mac_flash_1 == 0xffffffff)) {
			UARTprintf("Error: No MAC programmed! Aborting.\r\n");
			while(1) ;
	}
	
	// Show status
	UARTprintf("Eth: Starting LwIP controller...\r\n");

	// Setup LwIP with our MAC
	mac_bytes[0] = ((mac_flash_0 >>  0) & 0xff);
	mac_bytes[1] = ((mac_flash_0 >>  8) & 0xff);
	mac_bytes[2] = ((mac_flash_0 >> 16) & 0xff);
	mac_bytes[3] = ((mac_flash_1 >>  0) & 0xff);
	mac_bytes[4] = ((mac_flash_1 >>  8) & 0xff);
	mac_bytes[5] = ((mac_flash_1 >> 16) & 0xff);
	UARTprintf("Eth: Starting EMAC with MAC address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", \
		mac_bytes[0], mac_bytes[1], mac_bytes[2], mac_bytes[3], mac_bytes[4], mac_bytes[5]);
	lwIPInit(CPU_CLOCK, mac_bytes, 0, 0, 0, IPADDR_USE_DHCP);

	// Wait for DHCP to complete IP assignment (need to get IP)
	// Wait a number of cycles before aborting.
	UARTprintf("Eth: Waiting for IP (DHCP)...\r\n");
	cycles = 20000000;
	while(lwIPLocalIPAddrGet() == 0xffffffff || lwIPLocalIPAddrGet() == 0x00000000) {
		cycles--;
		if(cycles == 0) {
			UARTprintf("Eth: Error - unable to obtain link or IP\r\n");
			cpu_reset();
		}
	}
	
	UARTprintf("Eth: LwIP ready. Got IP.\r\n");
	
	ip_addr = lwIPLocalIPAddrGet();
	netmask = lwIPLocalNetMaskGet();
	gateway = lwIPLocalGWAddrGet();
	UARTprintf("Eth: Local   %03d.%03d.%03d.%03d (0x%08x)\r\n", \
		ip_addr & 0xff, (ip_addr >> 8) & 0xff, (ip_addr >> 16) & 0xff, (ip_addr >> 24) & 0xff, ip_addr);
	UARTprintf("Eth: Netmask %03d.%03d.%03d.%03d (0x%08x)\r\n", \
		netmask & 0xff, (netmask >> 8) & 0xff, (netmask >> 16) & 0xff, (netmask >> 24) & 0xff, netmask);
	UARTprintf("Eth: Gateway %03d.%03d.%03d.%03d (0x%08x)\r\n", \
		gateway & 0xff, (gateway >> 8) & 0xff, (gateway >> 16) & 0xff, (gateway >> 24) & 0xff, gateway);
	
	// Setup interrupt priorities
	MAP_IntPrioritySet(INT_EMAC0, 0xc0);			// Mid-Low priority
	MAP_IntPrioritySet(FAULT_SYSTICK, 0x80);	// Mid priority (Less than LED display)
}

/**
 * tcpip_start_listen: Start listening on ports for incoming data.
 */
void tcpip_start_listen()
{
	struct tcp_pcb *tpcb_simple, *tpcb_data;
	uint32_t err;
	
	UARTprintf("Inet: Initialising port %d as simple command interface\r\n", INET_PORT_LISTEN);
	
	// Create the pcb and bind the port for the simple command interface
	tpcb_simple = tcp_new();
	
	if(!tpcb_simple) {
		UARTprintf("Inet: Unable to allocate memory for tpcb in tcp_new() for simple command port\r\n");
		cpu_reset();
	}
	
	err = tcp_bind(tpcb_simple, IP_ADDR_ANY, INET_PORT_LISTEN);

	if(err != ERR_OK) {
		UARTprintf("Inet: Unable to listen on port %d, error=%d (for simple command port)\r\n", INET_PORT_LISTEN, err);
		cpu_reset();
	}
	
	// Start listening
	tpcb_simple = tcp_listen(tpcb_simple);
	
	if(!tpcb_simple) {
		UARTprintf("Inet: Unable to start listening, tcp_listen() returns NULL (for simple command port)\r\n");
		cpu_reset();
	}
	
	tcp_accept(tpcb_simple, tcpip_connected_callback);
	
	// Setup sent data callback.  We don't do anything with this, other than debug.
	tcp_sent(tpcb_simple, tcpip_sent_callback);
	UARTprintf("Inet: Ready to receive commands on port %d\r\n", INET_PORT_LISTEN);
	
	// Create the pcb and bind the port for the data interface
	UARTprintf("Inet: Initialising port %d as fixed data interface\r\n", INET_PORT_DATA);
	
	tpcb_data = tcp_new();
	
	if(!tpcb_data) {
		UARTprintf("Inet: Unable to allocate memory for tpcb in tcp_new() for simple command port\r\n");
		cpu_reset();
	}
	
	err = tcp_bind(tpcb_data, IP_ADDR_ANY, INET_PORT_DATA);

	if(err != ERR_OK) {
		UARTprintf("Inet: Unable to listen on port %d, error=%d (for simple command port)\r\n", INET_PORT_DATA, err);
		cpu_reset();
	}
	
	// Start listening
	tpcb_data = tcp_listen(tpcb_data);
	
	if(!tpcb_simple) {
		UARTprintf("Inet: Unable to start listening, tcp_listen() returns NULL (for simple command port)\r\n");
		cpu_reset();
	}
	
	tcp_accept(tpcb_data, tcpip_connected_callback);
	
	// Listen continuously for commands.  Data operations are handled inside the receive callback.
	while(1) {
		if(cmd_ready) {
			//UARTprintf("Got data - responding\r\n");
			cmd_recv_callback();
			
			// Clear pointer, erase buffer
			memset(cmd_buffer, 0, cmd_buff_len);
			cmd_buff_len = 0;
			cmd_ready = 0;
		}
	}
}
	
/**
 * cmd_recv_callback:  Callback for processing received commands.
 */
void cmd_recv_callback()
{
	/*
	 * There are two main types of commands.
	 * 
	 * Commands that begin with ! are simple commands that are terminated
	 * by a newline (\n). e.g. `!help', `!reset`.
	 *
	 * Commands that begin with # are long commands that specify a payload
	 * length. The command begins with #, followed by the length in bytes (hexadecimal),
	 * up to 32KB (padded to four digits), followed by the command name (padded to eight
	 * bytes with nulls), followed by the payload data, followed by a XOR checksum in hex.  
	 * The length includes all the data after the payload.
	 */
	char simple_command[SIMPLE_CMD_SIZE + 1];
	uint32_t n, parse = 0;
	
	//UARTprintf("data: %s %d\r\n", cmd_buffer, cmd_buff_len);
	
	// Look for the ! as the start of the command. This signifies a simple command.
	if(cmd_buffer[0] == '!') {
		/*
		// Clear our buffer
		memset(simple_command, 0, SIMPLE_CMD_SIZE + 1);
		
		// Read the command, pass it to the simple command parser, and then be done.
		// Only parse and execute the command if we see a newline; otherwise, it's not ready yet.
		for(n = 0; n < SIMPLE_CMD_SIZE; n++) {
			if(n > cmd_buff_len) 
				break;
			if(cmd_buffer[n] == '\n') {
				parse = 1;
				break;
			}
			simple_command[n] = cmd_buffer[n];
		}
		
		simple_command[n + 1] = 0;
		*/
		
		// Parse command if valid (has newline, is at least two bytes long)
		cmd_simple_parse((char *)&cmd_buffer, cmd_buff_len);
	}
}

/**
 * cmd_simple_parse: Parse a simple command.
 */
void cmd_simple_parse(char *cmdbuf, uint32_t len)
{
	uint32_t n, found = 0, arg_int = 0, arg_len = 0;
	char command_name[SIMPLE_CMD_MAX_NAME_LEN + 1];
	char arg_buff[SIMPLE_CMD_MAX_ARG_LEN + 1];
	
	memset(command_name, 0, SIMPLE_CMD_MAX_NAME_LEN);
	memset(arg_buff, 0, SIMPLE_CMD_MAX_ARG_LEN);
	
	command_arg_count = 0;
	
	// Copy command name up to first comma or newline; skip first byte
	for(n = 1; n < len; n++) {
		if(cmdbuf[n] == '\n' || cmdbuf[n] == ',')
			break;
		command_name[n - 1] = tolower(cmdbuf[n]);
	}
	
	// Copy arguments onto argument stack; arguments are assumed
	// to be int. In future we can assume a quotation mark indicates a string,
	// but that isn't implemented yet.
	n++;
	for(; n < len; n++) {
		if(cmdbuf[n] == ',' || n == (len - 1)) {
			// too many arguments? avoid overflow
			if(command_arg_count > (CMD_ARGS_MAX - 1)) 
				break;
			
			// pop new argument onto stack
			arg_int = strtol(&arg_buff, 0, 10);
			command_args_int[command_arg_count] = arg_int;
			command_arg_count++;
			
			UARTprintf("arg=%s val=%d\r\n", arg_buff, arg_int);
			
			memset(arg_buff, 0, SIMPLE_CMD_MAX_ARG_LEN);
			arg_len = 0;
		} else {
			arg_buff[arg_len] = cmdbuf[n];
			
			// arguments too long? avoid overflow
			if(arg_len > SIMPLE_CMD_MAX_ARG_LEN) 
				break;
			
			arg_len++;
		}
	}
	
	// Try to find a matching command
	for(n = 0; !(commands[n].flags & CMD_FLAG_END); n++) {
		if(strcmp(commands[n].name, command_name) == 0) { 
			commands[n].callback();
			found = 1;
			break;
		}
	}
	
	if(!found) {
		tcpip_reply_formatted(reply_pcb, "Invalid command `%s'\r\n", command_name);
	}
}

/**********
 * Commands that may be executed are implemented below.
 */

// Help, no arguments
void cmd_help()
{
	uint32_t n;
	char prefix;
	
	tcpip_reply_formatted(reply_pcb, "\r\n");
	tcpip_reply_formatted(reply_pcb, "HELP: Commands begin with `!' or '#'.\r\n\r\n");

	// Print name and description for each command.
	for(n = 0; !(commands[n].flags & CMD_FLAG_END); n++) {
		// Command prefix depends on type
		if(commands[n].flags & CMD_FLAG_SIMPLE) 
			prefix = '!';
		else
			prefix = '#';
		
		tcpip_reply_formatted(reply_pcb, "  Command:     %c%s\r\n", prefix, commands[n].name);
		tcpip_reply_formatted(reply_pcb, "  Arguments:   %d (max)\r\n", commands[n].max_args);
		tcpip_reply_formatted(reply_pcb, "\r\n");
		tcpip_reply_formatted(reply_pcb, "  %s\r\n\r\n\r\n", commands[n].desc);
	}
}

// Set whole screen red/green/blue/white
void cmd_red()
{
	disp_fill_rgb(255, 0, 0);
	disp_flush_display(1);
	
	tcpip_reply_formatted(reply_pcb, "It is said to be red\r\n");
}

void cmd_green()
{
	disp_fill_rgb(0, 255, 0);
	disp_flush_display(1);
	
	tcpip_reply_formatted(reply_pcb, "Let there be green\r\n");
}

void cmd_blue()
{
	disp_fill_rgb(0, 0, 255);
	disp_flush_display(1);
	
	tcpip_reply_formatted(reply_pcb, "As blue as a penguin that's been dipped in blue paint\r\n");
}

void cmd_white()
{
	disp_fill_rgb(255, 255, 255);
	disp_flush_display(1);
	
	tcpip_reply_formatted(reply_pcb, "It's very white\r\n");
}

void cmd_clear()
{
	disp_fill_rgb(0, 0, 0);
	disp_flush_display(1);
}

void cmd_setpix()
{
}

void cmd_setsubpix()
{
}

void cmd_commit()
{
	disp_flush_display(1);
}

void cmd_dispon()
{
	disp_run();
	
	tcpip_reply_formatted(reply_pcb, "Display active\r\n");
}

void cmd_dispoff()
{
	disp_stop();
	
	tcpip_reply_formatted(reply_pcb, "Display disabled\r\n");
}

void cmd_brightness()
{
	uint32_t bri = command_args_int[0];
	
	if(bri > 255)
		bri = 255;
	
	disp_set_brightness(bri);
	
	tcpip_reply_formatted(reply_pcb, "Display brightness set to %d (OE width = %d)\r\n", bri, oe_ints_global);
}

void cmd_status()
{
	tcpip_reply_formatted(reply_pcb, "Giant display controlled by TMC4C129ENCPDT\r\n");
	tcpip_reply_formatted(reply_pcb, "\r\n");
	tcpip_reply_formatted(reply_pcb, "COMPILE TIME PARAMETERS: \r\n");
	tcpip_reply_formatted(reply_pcb, "\r\n");
	tcpip_reply_formatted(reply_pcb, "Number of configured superblocks    = %d\r\n", DISP_NUM_SUPERBLOCKS);
	tcpip_reply_formatted(reply_pcb, "Number of configured inner modules  = %d\r\n", DISP_NUM_INNER_MODULES);
	tcpip_reply_formatted(reply_pcb, "Configured subfield count           = %d\r\n", DISP_SUBFIELDS);
	tcpip_reply_formatted(reply_pcb, "\r\n");
	tcpip_reply_formatted(reply_pcb, "POWER USAGE: \r\n");
	tcpip_reply_formatted(reply_pcb, "\r\n");
	tcpip_reply_formatted(reply_pcb, "Curr. power consumption before ABL  = %6d mW (%3d W)\r\n", calc_cur_power, calc_cur_power / 1000);
	tcpip_reply_formatted(reply_pcb, "Avg. power consumption before ABL   = %6d mW (%3d W)\r\n", calc_avg_power, calc_avg_power / 1000);
	tcpip_reply_formatted(reply_pcb, "ABL limit (peak)                    = %6d mW (%3d W)\r\n", abl_peak_power, abl_peak_power / 1000);
	tcpip_reply_formatted(reply_pcb, "ABL limit (averaged)                = %6d mW (%3d W)\r\n", abl_avg_power, abl_avg_power / 1000);
	tcpip_reply_formatted(reply_pcb, "ABL limit applied (0-1023)          = %d\r\n", abl_limit_oe);
	tcpip_reply_formatted(reply_pcb, "\r\n");
}

void cmd_reset()
{
	tcpip_reply_formatted(reply_pcb, "Goin' down\r\n");
	
	SysCtlReset();
}

