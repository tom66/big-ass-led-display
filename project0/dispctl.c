/*
 * Display control for Huge RGB Display
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

#include "hal.h"
#include "dispctl.h"

// OE scale factors for calculated pulse widths
// Red/green/blue colour balance altered here; global brightness setting affects whole panel
// Subfield and whole-frame ABL is calculated as an additional factor
uint32_t oe_ints_global = 1024;		// 1024 = max intensity
uint32_t oe_ints_r	 		= 950;		// ^
uint32_t oe_ints_g		 	= 500;		// ^
uint32_t oe_ints_b		 	= 1024;		// ^

// OE pulse widths - as driven to panel
uint32_t oe_width_r, oe_width_g, oe_width_b, oe_width_all, oe_base;

// OE states
uint32_t oe_r_state, oe_g_state, oe_b_state;

// Generate OE?  If cleared, no OE pulses are generated for this subfield,
// which reduces quiescent power
uint32_t oe_gen;

// Pulse widths of each subfield as a base figure
const uint16_t subfield_widths[DISP_SUBFIELDS] = {
		DISPCTL_OE_MAX_PULSE / 32,											// 7
		DISPCTL_OE_MAX_PULSE / 16,											// 8
		DISPCTL_OE_MAX_PULSE / 8,												// 9
		DISPCTL_OE_MAX_PULSE / 4,												// 10
		DISPCTL_OE_MAX_PULSE / 2,												// 11
		DISPCTL_OE_MAX_PULSE,														// 12
		DISPCTL_OE_MAX_PULSE / 64,											// 6
		DISPCTL_OE_MAX_PULSE / 128,											// 5
		DISPCTL_OE_MAX_PULSE / 256,											// 4
		DISPCTL_OE_MAX_PULSE / 512,											// 3
		DISPCTL_OE_MAX_PULSE / 1024,										// 2
		DISPCTL_OE_MAX_PULSE / 2048,										// 1
};

// Dividers for said subfields (must be a power of two)
const uint16_t subfield_divider[DISP_SUBFIELDS] = {
	/*
		1, 	4, 	16, 	64, 	256, 	1024,
		2,	8,	32,	 128,	 	512,	2048
	*/
	/*
		1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048
	*/
		64, 128, 256, 512, 1024, 2048, 32, 16, 8, 4, 2, 1
};

// Current subfield from 0 - (DISP_SUBFIELDS-1)
uint32_t cur_subfield = 0;

// Frame counter (32 bit, overflowing)
uint32_t disp_frame = 0;

// LED divider for frame sync output.
uint32_t disp_led_count = 0;
uint32_t disp_led_state = 0;

// Latch port lookup
const struct disp_latch_lut_t disp_latch_lut[DISP_NUM_LATCHES] = {
	// PORT BASE,  			PIN MASK,   	PIN INVERSE MASK
	{ GPIO_PORTD_BASE,	GPIO_PIN_1,	  ~GPIO_PIN_1	},
	
	// Others blank for now
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
	{ GPIO_PORTD_BASE,	0,						0 },
};

// Address bitmask lookup
const struct disp_addr_lut_t disp_addr_lut[DISP_NUM_INNER_MODULES] = {
	{ 0 },
	{                                               DISPCTL_ADDR_BIT2MSK },
	{                        DISPCTL_ADDR_BIT1MSK                        },
	{                        DISPCTL_ADDR_BIT1MSK | DISPCTL_ADDR_BIT2MSK },
	{ DISPCTL_ADDR_BIT0MSK                                               },
	{ DISPCTL_ADDR_BIT0MSK                        | DISPCTL_ADDR_BIT2MSK },
	{ DISPCTL_ADDR_BIT0MSK | DISPCTL_ADDR_BIT1MSK                        },
	{ DISPCTL_ADDR_BIT0MSK | DISPCTL_ADDR_BIT1MSK | DISPCTL_ADDR_BIT2MSK },
};

// Current display refresh rate in Hz.  Currently fixed; future feature should allow for dynamic refresh
// rates.
uint32_t disp_refresh_rate = DISP_REFRESH_RATE_DEFAULT;

// Display buffers
struct pix_rgbg_t disp_1[DISP_SUBPIX_WIDTH * DISP_SUBPIX_HEIGHT];
struct pix_rgbg_t disp_2[DISP_SUBPIX_WIDTH * DISP_SUBPIX_HEIGHT];

// Current display pointer and working (scratchpad) display pointer,
// these are flipped after the display is flushed
struct pix_rgbg_t *disp_current_ptr;
struct pix_rgbg_t *disp_working_ptr;

// Current calculated subfield in byte packets
// Data is ordered in each byte:  Rx/G1/Bx/G2 Rx/G1/Bx/G2  - with each Rx/G1/G2/Bx being one subpixel 
uint8_t disp_cur_subfield[DISP_SUBFIELD_SIZE_BYTES];

// Gamma lookup table, map of 0-255 to gamma map
const uint16_t gamma_lookup_12b[256] = { 
	0x0000, 0x0002, 0x0004, 0x0005, 0x0007, 0x0009, 0x000b, 0x000c, 0x000e, 0x0010, 0x0012, 0x0014, 
	0x0015, 0x0017, 0x0019, 0x001b, 0x001c, 0x001e, 0x0020, 0x0022, 0x0024, 0x0025, 0x0027, 0x0029, 
	0x002b, 0x002d, 0x002f, 0x0031, 0x0034, 0x0036, 0x0038, 0x003b, 0x003d, 0x0040, 0x0042, 0x0045, 
	0x0048, 0x004b, 0x004d, 0x0050, 0x0053, 0x0057, 0x005a, 0x005d, 0x0061, 0x0064, 0x0067, 0x006b, 
	0x006f, 0x0073, 0x0076, 0x007a, 0x007e, 0x0083, 0x0087, 0x008b, 0x0090, 0x0094, 0x0099, 0x009d, 
	0x00a2, 0x00a7, 0x00ac, 0x00b1, 0x00b6, 0x00bb, 0x00c1, 0x00c6, 0x00cc, 0x00d1, 0x00d7, 0x00dd, 
	0x00e3, 0x00e9, 0x00ef, 0x00f6, 0x00fc, 0x0103, 0x0109, 0x0110, 0x0117, 0x011e, 0x0125, 0x012c, 
	0x0134, 0x013b, 0x0143, 0x014a, 0x0152, 0x015a, 0x0162, 0x016a, 0x0173, 0x017b, 0x0184, 0x018c, 
	0x0195, 0x019e, 0x01a7, 0x01b0, 0x01ba, 0x01c3, 0x01cd, 0x01d7, 0x01e0, 0x01ea, 0x01f5, 0x01ff, 
	0x0209, 0x0214, 0x021f, 0x022a, 0x0235, 0x0240, 0x024b, 0x0257, 0x0262, 0x026e, 0x027a, 0x0286, 
	0x0292, 0x029e, 0x02ab, 0x02b8, 0x02c4, 0x02d1, 0x02de, 0x02ec, 0x02f9, 0x0307, 0x0315, 0x0322, 
	0x0331, 0x033f, 0x034d, 0x035c, 0x036b, 0x037a, 0x0389, 0x0398, 0x03a7, 0x03b7, 0x03c7, 0x03d7, 
	0x03e7, 0x03f7, 0x0408, 0x0418, 0x0429, 0x043a, 0x044b, 0x045d, 0x046e, 0x0480, 0x0492, 0x04a4, 
	0x04b6, 0x04c9, 0x04db, 0x04ee, 0x0501, 0x0515, 0x0528, 0x053c, 0x054f, 0x0563, 0x0578, 0x058c, 
	0x05a1, 0x05b5, 0x05ca, 0x05e0, 0x05f5, 0x060b, 0x0620, 0x0636, 0x064d, 0x0663, 0x067a, 0x0691, 
	0x06a8, 0x06bf, 0x06d6, 0x06ee, 0x0706, 0x071e, 0x0736, 0x074f, 0x0768, 0x0781, 0x079a, 0x07b3, 
	0x07cd, 0x07e7, 0x0801, 0x081b, 0x0835, 0x0850, 0x086b, 0x0886, 0x08a2, 0x08bd, 0x08d9, 0x08f5, 
	0x0912, 0x092e, 0x094b, 0x0968, 0x0985, 0x09a3, 0x09c1, 0x09df, 0x09fd, 0x0a1b, 0x0a3a, 0x0a59, 
	0x0a78, 0x0a98, 0x0ab7, 0x0ad7, 0x0af7, 0x0b18, 0x0b38, 0x0b59, 0x0b7a, 0x0b9c, 0x0bbe, 0x0bdf, 
	0x0c02, 0x0c24, 0x0c47, 0x0c6a, 0x0c8d, 0x0cb0, 0x0cd4, 0x0cf8, 0x0d1c, 0x0d41, 0x0d66, 0x0d8b, 
	0x0db0, 0x0dd6, 0x0dfb, 0x0e21, 0x0e48, 0x0e6e, 0x0e95, 0x0ebd, 0x0ee4, 0x0f0c, 0x0f34, 0x0f5c, 
	0x0f85, 0x0fad, 0x0fd7, 0x0fff
};

// Frame wait and sync, set by runtime software, cleared by interrupts.
// Must be volatile to avoid compiler optimisation.
volatile uint32_t disp_frame_wait = 0;
volatile uint32_t disp_frame_sync = 0;
volatile uint32_t disp_ptr_order = 0;

// ABL peak and average limits
uint32_t abl_peak_power = 24000;			// 24 watts peak
uint32_t abl_avg_power = 12000;				// 12 watts average
uint32_t calc_cur_power = 0;
uint32_t calc_avg_power = 0;
uint32_t abl_limit_oe = 0;

/*
 * Timer0AIntHandler:  OE timeout for Red OE
 */
void Timer0AIntHandler()
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	
	if(oe_r_state == OE_STATE_GOLOW) {
		oe_r_state = OE_STATE_GOHIGH;
		
		// load timer
		TimerLoadSet(TIMER0_BASE, TIMER_A, oe_width_r);
		TimerEnable(TIMER0_BASE, TIMER_A);
		
		if(oe_gen) {
			DISPCTL_R_OE_ON();
		}
	} else if(oe_r_state == OE_STATE_GOHIGH) { 
		DISPCTL_R_OE_OFF();
	}
}
	
/*
 * Timer0BIntHandler:  OE timeout for Green OE
 */
void Timer0BIntHandler()
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
	
	if(oe_g_state == OE_STATE_GOLOW) {
		oe_g_state = OE_STATE_GOHIGH;
		
		// load timer
		TimerLoadSet(TIMER0_BASE, TIMER_B, oe_width_g);
		TimerEnable(TIMER0_BASE, TIMER_B);
		
		if(oe_gen) {
			DISPCTL_G_OE_ON();
		}
	} else if(oe_g_state == OE_STATE_GOHIGH) { 
		DISPCTL_G_OE_OFF();
	}
}
	
/*
 * Timer1AIntHandler:  OE timeout for Blue OE
 */
void Timer1AIntHandler()
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	
	if(oe_b_state == OE_STATE_GOLOW) {
		oe_b_state = OE_STATE_GOHIGH;
		
		// load timer
		TimerLoadSet(TIMER1_BASE, TIMER_A, oe_width_b);
		TimerEnable(TIMER1_BASE, TIMER_A);
		
		if(oe_gen) {
			DISPCTL_B_OE_ON();
		}
	} else if(oe_b_state == OE_STATE_GOHIGH) { 
		DISPCTL_B_OE_OFF();
	}
}

/*
 * Timer1BIntHandler:  Overall OE timeout for subfield generation.
 */
void Timer1BIntHandler()
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
	disp_drive_subfield();
}

/*
 * Timer2AIntHandler:  Frame timer.
 */
void Timer2AIntHandler()
{
	uint32_t fr_power = 0, n;
	uint32_t abl_limit_peak = 1023, abl_limit_avg = 1023; 
	struct pix_rgbg_t *temp;
	
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	
	// Sync frame?
	if(disp_frame_sync) {
		/*
		if(disp_ptr_order) {
			disp_current_ptr = disp_1;
			disp_working_ptr = disp_2;
		} else {
			disp_current_ptr = disp_2;
			disp_working_ptr = disp_1;
		}
		*/
		disp_ptr_order ^= 1;
		temp = disp_current_ptr;
		disp_current_ptr = disp_working_ptr;
		disp_working_ptr = temp;
		disp_frame_sync = 0;
	}
	
	// Run the ABL engine for this frame.  This works out the peak power of the frame,
	// based on the sum of pixels, and the average power over time.
	//
	// It will reduce the global brightness if either peak or average limits are exceeded.
	//
	// A separate ABL limit applies to individual subfields based on peak current within
	// a given subfield.
	for(n = 0; n < DISP_PIXELS; n++) {
		// TODO: account for relative brightness of R/G/B?
		fr_power += disp_current_ptr[n].r + disp_current_ptr[n].g1 + disp_current_ptr[n].b + disp_current_ptr[n].g2; 
	}

	fr_power /= DISP_PIXELS * 4;
	fr_power *= ABL_POW_MULT;
	fr_power >>= ABL_POW_SHIFT;
	calc_cur_power = fr_power;
	
	// Calculate average power using an IIR
	calc_avg_power = ((calc_cur_power * ABL_AVG_PWR_MULT_NEW) + (calc_avg_power * ABL_AVG_PWR_MULT_CUR)) >> ABL_AVG_PWR_SHIFT;
	
	// Calculate OE for ABL limit; peak overrides average, because peak always 
	// triggers at the highest level (lowest OE result)
	if(calc_avg_power > abl_avg_power) {
		abl_limit_avg = (abl_avg_power << DISP_INT_SHIFT) / calc_avg_power;
	} 
	
	if(calc_cur_power > abl_peak_power) {
		abl_limit_peak = (abl_peak_power << DISP_INT_SHIFT) / calc_cur_power;
	}
	
	if(abl_limit_peak < abl_limit_avg) {
		abl_limit_oe = abl_limit_peak;
	} else {
		abl_limit_oe = abl_limit_avg;
	}
	
	// Increase divider count, if the count is reached toggle the LED.
	disp_led_count++;
	if(disp_led_count == LED_DIVIDE_FSYNC) {
		disp_led_count = 0;
		disp_led_state = 1 - disp_led_state;
		
		if(disp_led_state) 
			GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
		else
			GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
	}
	
	// Drive the frame.
	disp_drive_frame();
}
	
/*
 * disp_setup: Initialise I/O and SSI modules.
 */
void disp_setup()
{
	uint32_t block, x, y, n;
	
	// ** Setup quad SSI **
	// Enable SSI0 clocks after resetting module
	SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralReset(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0)));
	
	// Init GPIO
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
	GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);
	GPIOPinConfigure(GPIO_PA6_SSI0XDAT2);
	GPIOPinConfigure(GPIO_PA7_SSI0XDAT3);
  GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	
	// Set pin drive current
	// Clock has highest drive, data lines are lower
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
	//HWREG(GPIO_PORTA_BASE + GPIO_O_DR8R) |= (GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
		
	// Init SSI0 module at LED clock rate
	SSIConfigSetExpClk(SSI0_BASE, CPU_CLOCK, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, LED_CLOCK, 8);
	
	// Enable advanced SSI0 mode (quad SSI)
	SSIAdvModeSet(SSI0_BASE, SSI_ADV_MODE_WRITE);
	SSIAdvFrameHoldEnable(SSI0_BASE);
	HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_HSCLKEN;
	SSIEnable(SSI0_BASE);
	
	// ** Setup GPIO outputs **
	// Global RGB OE.  We go for separate control over R/G/B which gives us fine colour balance control.
	// These are driven by fast interrupts.
	GPIOPinTypeGPIOOutput(DISPCTL_R_OE_PORT, DISPCTL_R_OE_BIT);
	GPIOPinTypeGPIOOutput(DISPCTL_G_OE_PORT, DISPCTL_G_OE_BIT);
	GPIOPinTypeGPIOOutput(DISPCTL_B_OE_PORT, DISPCTL_B_OE_BIT);
	
	// Disable global RGB OE signals so panel is blank (active LOW enable)
	GPIOPinWrite(DISPCTL_R_OE_PORT, DISPCTL_R_OE_BIT, DISPCTL_R_OE_BIT);
	GPIOPinWrite(DISPCTL_G_OE_PORT, DISPCTL_G_OE_BIT, DISPCTL_G_OE_BIT);
	GPIOPinWrite(DISPCTL_B_OE_PORT, DISPCTL_B_OE_BIT, DISPCTL_B_OE_BIT);
	
	// Row select and address
	GPIOPinTypeGPIOOutput(DISPCTL_ODDEVEN_PORT, DISPCTL_ODDEVEN_BIT);
	GPIOPinTypeGPIOOutput(DISPCTL_ADDR_PORT, DISPCTL_ADDR_BIT0MSK);
	GPIOPinTypeGPIOOutput(DISPCTL_ADDR_PORT, DISPCTL_ADDR_BIT1MSK);
	GPIOPinTypeGPIOOutput(DISPCTL_ADDR_PORT, DISPCTL_ADDR_BIT2MSK);
	
	// Setup latch signals as outputs for each superblock
	for(block = 0; block < DISP_NUM_LATCHES; block++) {
		if(disp_latch_lut[block].port_mask != 0) {
			GPIOPinTypeGPIOOutput(disp_latch_lut[block].port_base, disp_latch_lut[block].port_mask);
		}
	}
	
	// Turn off LED2 after init
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ~GPIO_PIN_0);	
	
	// ** Setup OE timers **
	// Four timers are used, Timer0A, 0B and 1A, for R, G and B respectively,
	// and Timer1B for global subfield timing.
	// Timers are disabled until required.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	
	// Timer0
	TimerDisable(TIMER0_BASE, TIMER_A);
	TimerDisable(TIMER0_BASE, TIMER_B);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT);
	TimerPrescaleSet(TIMER0_BASE, TIMER_A, DISPCTL_OE_PRESCALER);
	TimerPrescaleSet(TIMER0_BASE, TIMER_B, DISPCTL_OE_PRESCALER);
	TimerLoadSet(TIMER0_BASE, TIMER_A, DISPCTL_OE_MAX_PULSE);
	TimerLoadSet(TIMER0_BASE, TIMER_B, DISPCTL_OE_MAX_PULSE);
	TimerMatchSet(TIMER0_BASE, TIMER_A, 0);
	TimerMatchSet(TIMER0_BASE, TIMER_B, 0);
	IntPrioritySet(INT_TIMER0A, 0x20); // High priority
	IntPrioritySet(INT_TIMER0B, 0x40); // 
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
	IntEnable(INT_TIMER0A);
	IntEnable(INT_TIMER0B);
	
	// Timer1
	TimerDisable(TIMER1_BASE, TIMER_A);
	TimerDisable(TIMER1_BASE, TIMER_B);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT);
	TimerPrescaleSet(TIMER1_BASE, TIMER_A, DISPCTL_OE_PRESCALER);
	TimerPrescaleSet(TIMER1_BASE, TIMER_B, DISPCTL_OE_PRESCALER);
	TimerLoadSet(TIMER1_BASE, TIMER_A, DISPCTL_OE_MAX_PULSE);
	TimerLoadSet(TIMER1_BASE, TIMER_B, DISPCTL_OE_MAX_PULSE);
	TimerMatchSet(TIMER1_BASE, TIMER_A, 0);
	TimerMatchSet(TIMER1_BASE, TIMER_B, 0);
	IntPrioritySet(INT_TIMER1A, 0x60);
	IntPrioritySet(INT_TIMER1B, 0x80); // Lower priority for global SF gen to ensure it fires after SFs have been generated
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
	IntEnable(INT_TIMER1A);
	IntEnable(INT_TIMER1B);
	
	// ** Setup frame timer **
	// This is a general purpose timer (Timer2A) that operates in the 50 - 150Hz range
	// It fires periodically outputting the appropriate subfields
	// Timer is enabled after display starts running
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	TimerDisable(TIMER2_BASE, TIMER_A);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
	TimerPrescaleSet(TIMER2_BASE, TIMER_A, DISPCTL_FRGEN_PRESCALE);
	IntPrioritySet(INT_TIMER2A, 0xa0);	// Lower priority than all other tasks; avoids immediate OE collision risk
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	IntEnable(INT_TIMER2A);
	
	// ** Setup pointers **
	disp_current_ptr = (struct pix_rgbg_t *)&disp_1;
	disp_working_ptr = (struct pix_rgbg_t *)&disp_2;
}

/*
 * disp_output_subfield:  Output a subfield.
 *
 * @param		sf		The subfield to display
 */
void disp_output_subfield(uint8_t *sf)
{
	static uint32_t frame = 0;
	uint8_t *sf_module = sf;	// Pointer to the current module subfield. Set to top left for now.
	uint32_t block, module, total_module = 0;
	volatile uint32_t dly;
	
	// Process SUPERBLOCKS, each of 8 modules, which are addressed from left to right, top to bottom
	// An x,y coordinate calculating the first pixel of sub
	// Each SUPERBLOCK is addressed visually, top to bottom
	for(block = 0; block < DISP_NUM_SUPERBLOCKS; block++) {
		for(module = 0; module < DISP_NUM_INNER_MODULES; module++, total_module++) {
			// If we exceed our total module limit then exit.
			if(total_module > DISP_NUM_MODULES)
				break;
			
			// Set the address for this module.  This masks the module to the latch signal
			// that we are sending.
			DISPCTL_SB_ADDR_WRITE(disp_addr_lut[module]);
			
			// Address upper half by setting ODD/EVEN high
			DISPCTL_ODDEVEN_HIGH();
			
			// For each display module, drive two sets of 64 bit data
			// SSI lines are configured in quad SSI module:
			//   bit 0 = R   -  1st pix
			//   bit 1 = G1  ^
			//   bit 2 = B   ^
			//   bit 3 = G2  ^
			//   bit 4 = R   - 2nd pix
			//   bit 5 = G1  ^
			//   bit 6 = B   ^
			//   bit 7 = G2  ^
			
			// Each byte write drives two groups of pixels (4 bits)
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			
			// Wait for the FIFO to empty to proceed
			while(!(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_TFE)) ;
			
			// Latch this module's data. NOPs are added to delay and extend the latch timing slightly.
			DISPCTL_LATCH(disp_latch_lut[0]);
			
			// Address lower half by setting ODD/EVEN low
			DISPCTL_ODDEVEN_LOW();
			
			// Clock next set of 64 bits of data
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			HWREG(SSI0_BASE + SSI_O_DR) = *sf_module++;
			
			// Wait for the FIFO to empty to proceed
			while(!(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_TFE)) ;
			
			// Latch this module's data
			DISPCTL_LATCH(disp_latch_lut[0]);
		}
	}
}

/*
 * disp_run:  Start displaying stuff.
 */
void disp_run()
{
	SSIAdvModeSet(SSI0_BASE, SSI_ADV_MODE_QUAD_WRITE);
	
	UARTprintf("xsz=%6d, ysz=%6d, bytes=%d\r\n", DISP_PIX_WIDTH, DISP_PIX_HEIGHT, DISP_SUBFIELD_SIZE_BYTES);
	
	// Load timer period and enable the timer
	TimerLoadSet(TIMER2_BASE, TIMER_A, DISPCTL_FRGEN_CALC_FREQ(disp_refresh_rate));
	TimerEnable(TIMER2_BASE, TIMER_A);
}

/*
 * disp_stop:  Stop displaying stuff.
 */
void disp_stop()
{
	// Turn off the timer.  The subfields stop shortly afterwards.
	TimerDisable(TIMER2_BASE, TIMER_A);
}

/*
 * disp_output_frame:  Output one frame with desired subfield timing.
 */
void disp_drive_frame()
{
	// Clear framewait flag
	disp_frame_wait = 0;

	// Drive the first subfield - the subfield driver handles it from there.
	cur_subfield = 0;
	disp_drive_subfield();

	// Increase frame counter
	disp_frame++;
}

/*
 * disp_drive_subfield:  Compute and output one subfield.  Triggered by blue OE rising (off)
 * edge, this will output a subfield, or stop the loop if all subfields have been
 * output for the given intensity.
 */
void disp_drive_subfield()
{
	uint32_t x, y, n = 0, n0 = 0, n1 = 0, j = 0, k = 0, field = 1, oe_max = 0, valid = 1;
	uint8_t byte;
	volatile uint32_t dly;
	
	uint32_t base_width, base_scaled, bitmask = 0, v = 0;
	uint32_t oe_offs_r, oe_offs_g, oe_offs_b;
	
	// OE must be off
	DISPCTL_R_OE_OFF();
	DISPCTL_G_OE_OFF();
	DISPCTL_B_OE_OFF();
	
	// Abort if at max subfield, we need to do another frame
	if(cur_subfield == DISP_SUBFIELDS) {
		cur_subfield = 0;
		return;
	}
		
	//bitmask = 1 << cur_subfield;
	bitmask = subfield_divider[cur_subfield];
	
	for(n = 0; n < DISP_SUBFIELD_SIZE_BYTES; n++) {
		n0 = n << 1;
		n1 = n0 + 1;
		
		// Compute the pixels set in this subfield
		byte = ((!!(disp_current_ptr[n1].r & bitmask)) << 0) | ((!!(disp_current_ptr[n1].g2 & bitmask)) << 1) | \
					 ((!!(disp_current_ptr[n1].b & bitmask)) << 2) | ((!!(disp_current_ptr[n1].g1 & bitmask)) << 3) | \
					 ((!!(disp_current_ptr[n0].r & bitmask)) << 4) | ((!!(disp_current_ptr[n0].g2 & bitmask)) << 5) | \
					 ((!!(disp_current_ptr[n0].b & bitmask)) << 6) | ((!!(disp_current_ptr[n0].g1 & bitmask)) << 7);
	
		/*
		if(byte != 0x00)
			valid = 1;
		*/
		disp_cur_subfield[n] = byte;
	}
	
	disp_output_subfield((uint8_t*)&disp_cur_subfield);
	
	// Determine maximum OE to use: commanded brightness, or brightness (ABL) limit
	if(oe_ints_global < abl_limit_oe) { 
		oe_max = oe_ints_global;
	} else {
		oe_max = abl_limit_oe;
	}
	
	// Compute field widths; these are used by the ISRs to set up the second OE pulse interrupt
	base_width = subfield_widths[cur_subfield];
	base_scaled = (base_width * oe_max) 								>> DISP_INT_SHIFT;
	oe_width_r = (base_scaled * oe_ints_r) 							>> DISP_INT_SHIFT;
	oe_width_g = (base_scaled * oe_ints_g) 							>> DISP_INT_SHIFT;
	oe_width_b = (base_scaled * oe_ints_b) 							>> DISP_INT_SHIFT;
	oe_width_r = (oe_width_r * DISPCTL_OE_SFTDRV_SCALE) >> DISP_INT_SHIFT;
	oe_width_g = (oe_width_g * DISPCTL_OE_SFTDRV_SCALE) >> DISP_INT_SHIFT;
	oe_width_b = (oe_width_b * DISPCTL_OE_SFTDRV_SCALE) >> DISP_INT_SHIFT;
	oe_offs_r = (oe_width_r * DISPCTL_OE_SFTDRV_R_OFFS) >> DISP_INT_SHIFT;
	oe_offs_g = (oe_width_g * DISPCTL_OE_SFTDRV_G_OFFS) >> DISP_INT_SHIFT;
	oe_offs_b = (oe_width_b * DISPCTL_OE_SFTDRV_B_OFFS) >> DISP_INT_SHIFT;
	oe_width_all = base_width + ((base_width * DISPCTL_OE_OFFSET_ALL) >> DISP_INT_SHIFT);
	oe_base = oe_max;
	
	// Load timers for OE initial pulse (Don't do pulses if field is empty)
	oe_gen = valid;
	oe_r_state = OE_STATE_GOLOW;
	oe_g_state = OE_STATE_GOLOW;
	oe_b_state = OE_STATE_GOLOW;
	TimerLoadSet(TIMER0_BASE, TIMER_A, oe_offs_r);
	TimerLoadSet(TIMER0_BASE, TIMER_B, oe_offs_g);
	TimerLoadSet(TIMER1_BASE, TIMER_A, oe_offs_b);
	TimerLoadSet(TIMER1_BASE, TIMER_B, oe_width_all);
	TimerEnable(TIMER0_BASE, TIMER_A);
	TimerEnable(TIMER0_BASE, TIMER_B);
	TimerEnable(TIMER1_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_B);
	
	// Move to next subfield.
	cur_subfield++;
}

/* 
 * disp_write_pixel:  Write to the pixel array at a coordinate.
 */
void disp_write_pixel(uint32_t x, uint32_t y, struct pix_rgbg_t *pix) 
{
	uint32_t addr;
	uint32_t rr, gg1, gg2, bb;
	
	// ignore pixels out of boundaries (<0 not possible)
	if(x >= DISP_PIX_WIDTH)
		return;
	if(y >= DISP_PIX_HEIGHT)
		return;
	
	// clip values
	rr  = pix->r;
	gg1 = pix->g1;
	bb  = pix->b;
	gg2 = pix->g2;
	
	if(rr > 255)
		rr = 255;
	if(gg1 > 255)
		gg1 = 255;
	if(gg2 > 255)
		gg2 = 255;
	if(bb > 255)
		bb = 255;
	
	addr = x + (y * DISP_PIX_WIDTH);
	
	disp_working_ptr[addr].r  = gamma_lookup_12b[pix->r];
	disp_working_ptr[addr].g1 = gamma_lookup_12b[pix->g1];
	disp_working_ptr[addr].b  = gamma_lookup_12b[pix->b];
	disp_working_ptr[addr].g2 = gamma_lookup_12b[pix->g2];
}

/* 
 * disp_write_pixel_rgb:  Write to the pixel array at a coordinate quickly (direct arg variant)
 */
void disp_write_pixel_rgb(uint32_t x, uint32_t y, uint32_t r, uint32_t g, uint32_t b) 
{
	uint32_t addr;
	
	// ignore pixels out of boundaries (<0 not possible)
	if(x >= DISP_PIX_WIDTH)
		return;
	if(y >= DISP_PIX_HEIGHT)
		return;
	
	// clip values
	if(r > 255)
		r = 255;
	if(g > 255)
		g = 255;
	if(b > 255)
		b = 255;
	
	addr = x + (y * DISP_PIX_WIDTH);
	
	disp_working_ptr[addr].r  = gamma_lookup_12b[r];
	disp_working_ptr[addr].g1 = gamma_lookup_12b[g];
	disp_working_ptr[addr].b  = gamma_lookup_12b[b];
	disp_working_ptr[addr].g2 = gamma_lookup_12b[g];
}

/* 
 * disp_fill_rgb:  Fill panel with colour.
 */
void disp_fill_rgb(uint32_t r, uint32_t g, uint32_t b)
{
	uint32_t x, y;
	
	for(x = 0; x < DISP_PIX_WIDTH; x++) {
		for(y = 0; y < DISP_PIX_HEIGHT; y++) {
			disp_write_pixel_rgb(x, y, r, g, b);
		}
	}
}

/*
 * disp_test_pattern_boot:  Boot test pattern.
 */
void disp_test_pattern_boot()
{
	uint32_t i;
	
	disp_fill_rgb(255, 0, 0);
	disp_flush_display(1);
	
	for(i = 20; i > 0; i--) 
		disp_wait_for_frame();
	
	disp_fill_rgb(0, 255, 0);
	disp_flush_display(1);
	
	for(i = 20; i > 0; i--) 
		disp_wait_for_frame();
	
	disp_fill_rgb(0, 0, 255);
	disp_flush_display(1);
	
	for(i = 20; i > 0; i--) 
		disp_wait_for_frame();
	
	disp_fill_rgb(255, 255, 255);
	disp_flush_display(1);
	
	for(i = 20; i > 0; i--) 
		disp_wait_for_frame();
		
	disp_fill_rgb(0, 0, 0);
	disp_flush_display(1);
}

/*
 * disp_debug: Dump some useful info
 */
void disp_debug()
{
	UARTprintf("Frame = %7d   Power = %6d mW peak, %6d mW avg.   ABL = %5d   OEW_scale = %5d\r\n", \
		disp_frame, calc_cur_power, calc_avg_power, abl_limit_oe, oe_base);
}

/*
 * disp_wait_for_frame: Wait for frame end. Blocks until frame committed. 
 * Simple way of syncing to display.
 */
void disp_wait_for_frame()
{
	disp_frame_wait = 1;
	
	// Now wait for ISR to clear this flag
	while(disp_frame_wait) ;
}

/*
 * disp_flush_display:  Flushes current display to output on next frame
 * boundary. (Sets flag triggering flush.)
 *
 * @param		block		if set will delay until frame sync actually happens
 */
void disp_flush_display(uint32_t block)
{
	disp_frame_sync = 1;
	
	if(block) 
		disp_wait_for_frame();
}

/*
 * disp_set_brightness:  Set global brightness (0-255)
 *
 * @param		bright		New brightness figure from 0-255, will be gamma corrected to 0-1023
 */
void disp_set_brightness(uint32_t bri)
{
	if(bri > 255)
		bri = 255;
	
	oe_ints_global = gamma_lookup_12b[bri] >> 2;
}
