/*
 * Large display controller
 *
 * (C) 2017 Thomas Oldbury, released under MIT licence
 */
 
#include <stdint.h>
#include <stdbool.h>

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
#include "httpserver_raw/httpd.h"

#include "hal.h"
#include "inet.h"

/**
 * part_init: Setup core part parameters. Display not initialised.
 */
void part_init()
{
	// ** Setup core clock **
	SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
	SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), CPU_CLOCK);
	
	// ** Initialise GPIO **
	// Enable all port clocks
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

	// Set IO pad configuration
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);			// PN0 / LED2
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);			// PN1 / LED1
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);	// LED2 on during init
	
	// ** Setup debug UART **
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0, 115200, CPU_CLOCK);
}

/**
 * boot_init:  Run the boot process. Status messages shown on the display.
 */
void boot_init()
{
	// Setup the basic part settings
	part_init();
	UARTprintf("\r\n");
	UARTprintf("\r\n");
	UARTprintf("TM4C129ENCPDT Huge Display Controller\r\n");
	UARTprintf("(C) 2017 Thomas Oldbury. Released under MIT licence.\r\n");
	UARTprintf("\r\n");
	UARTprintf("Part resources initialised.\r\n");
	
	// Setup display stuff and show test pattern
	UARTprintf("HAL: Initialising display...\r\n");
	disp_setup();
	disp_run();
	disp_test_pattern_boot();
	UARTprintf("HAL: Display test complete.\r\n");
	
	// Setup ethernet stuff
	UARTprintf("HAL: Initialising TCP/IP stack...\r\n");
	tcpip_init();
	
	// Start listening for packets
	UARTprintf("HAL: Initialising TCP/IP port listening...\r\n");
	tcpip_start_listen();
}

/**
 * cpu_reset: Reset CPU after shutting down display resources.
 */
void cpu_reset()
{
	disp_stop();
	
	// Wait a bit for UART flush, then reset
	SysCtlDelay(10000000);
	SysCtlReset();
}
