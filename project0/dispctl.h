/*
 * Display control for Huge RGB Display
 */
 
#ifndef ___DISPCTL_H___
#define ___DISPCTL_H___

#include <stdint.h>

// LED bit clock: as high as the display will support with the cabling used
#define LED_CLOCK											10000000 // 12500000

// Display core parameters
#define DISP_SUBPIX_WIDTH							32 // 160
#define DISP_SUBPIX_HEIGHT						32 // 120

#define DISP_PIX_WIDTH								16 // 80
#define DISP_PIX_HEIGHT								16 // 60

#define DISP_SUBFIELD_SIZE_BYTES			(DISP_PIX_WIDTH * DISP_PIX_HEIGHT)
#define DISP_PIXELS										(DISP_PIX_WIDTH * DISP_PIX_HEIGHT)

// Drive configuration:
//   One SSI port driving WHOLE panel (SSI0) with many buffer ICs to reduce loading issues
//   20 MHz clock 
// 
// Each group of eight modules has a separate latch signal (19 latch signals)
// All latches fall low to latch data.  A latch staying high means the module will ignore
// the data (it will not be committed to the output of the module.)
//
// Data is committed to each group of 8 displays in turn, then latched on each latch signal
#define DISP_NUM_LATCHES							19

// Display is treated as 20 blocks of 8 modules each with 128 subpixels
// Note that some blocks are omitted to keep the 80x60 effective resolution. 
#define DISP_NUM_SUPERBLOCKS				  1 // 20
#define DISP_NUM_INNER_MODULES				8
#define DISP_NUM_MODULES							150

// 12-bit subfield drive, 120Hz (target) global refresh rate
#define DISP_SUBFIELDS								12
#define DISP_REFRESH_RATE_DEFAULT			80

// Latch/address change delays
#define DISPCTL_CHG_DLY1()						__asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); \
																			__asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); \
																			__asm("nop"); __asm("nop"); 
#define DISPCTL_CHG_DLY2()						__asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); \
																			__asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); \
																			__asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); 
#define DISPCTL_CHG_DLY3()						__asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); \
																			__asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); \
																			__asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); \
																			__asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); \
																			__asm("nop"); __asm("nop"); __asm("nop"); __asm("nop"); 

// Global odd/even for all display modules: PD0
#define DISPCTL_ODDEVEN_PORT					GPIO_PORTD_BASE
#define DISPCTL_ODDEVEN_BIT						GPIO_PIN_0

#define DISPCTL_ODDEVEN_HIGH()				DISPCTL_CHG_DLY1(); \
																			HWREG(DISPCTL_ODDEVEN_PORT + (GPIO_O_DATA + (DISPCTL_ODDEVEN_BIT << 2))) = DISPCTL_ODDEVEN_BIT;\
																			DISPCTL_CHG_DLY2(); 
																			
#define DISPCTL_ODDEVEN_LOW()					DISPCTL_CHG_DLY1(); \
																			HWREG(DISPCTL_ODDEVEN_PORT + (GPIO_O_DATA + (DISPCTL_ODDEVEN_BIT << 2))) = ~DISPCTL_ODDEVEN_BIT;\
																			DISPCTL_CHG_DLY3(); 


// Drives latch signal low then high causing a latch
// Latch pulse minimum 50ns set up after last clock, 100ns pulse width.
#define DISPCTL_LATCH(lut)						DISPCTL_CHG_DLY3(); \
																			HWREG(lut.port_base + (GPIO_O_DATA + (lut.port_mask << 2))) = lut.port_invmask; \
																			DISPCTL_CHG_DLY3(); \
																			HWREG(lut.port_base + (GPIO_O_DATA + (lut.port_mask << 2))) = lut.port_mask;

// Display address (in-superblock) mux pins
#define DISPCTL_ADDR_PORT							GPIO_PORTL_BASE
#define DISPCTL_ADDR_BIT0MSK					GPIO_PIN_2
#define DISPCTL_ADDR_BIT1MSK					GPIO_PIN_1
#define DISPCTL_ADDR_BIT2MSK					GPIO_PIN_0
#define DISPCTL_ADDR_BITALLMSK				(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2)

// Display superblock address write
#define DISPCTL_SB_ADDR_WRITE(lut)		DISPCTL_CHG_DLY1(); \
																			HWREG(DISPCTL_ADDR_PORT + (GPIO_O_DATA + (DISPCTL_ADDR_BITALLMSK << 2))) = lut.write_mask; \
																			DISPCTL_CHG_DLY2(); 

// Display output enable ports
// These need to be connectable to timer peripherals, which generate the respective red/green/blue low pulses
#define DISPCTL_R_OE_PORT							GPIO_PORTC_BASE
#define DISPCTL_R_OE_BIT							GPIO_PIN_7
#define DISPCTL_G_OE_PORT							GPIO_PORTB_BASE
#define DISPCTL_G_OE_BIT							GPIO_PIN_2
#define DISPCTL_B_OE_PORT							GPIO_PORTB_BASE
#define DISPCTL_B_OE_BIT							GPIO_PIN_3

#define DISPCTL_R_OE_OFF()						HWREG(DISPCTL_R_OE_PORT + (GPIO_O_DATA + (DISPCTL_R_OE_BIT << 2))) =  DISPCTL_R_OE_BIT;
#define DISPCTL_G_OE_OFF()						HWREG(DISPCTL_G_OE_PORT + (GPIO_O_DATA + (DISPCTL_G_OE_BIT << 2))) =  DISPCTL_G_OE_BIT;
#define DISPCTL_B_OE_OFF()						HWREG(DISPCTL_B_OE_PORT + (GPIO_O_DATA + (DISPCTL_B_OE_BIT << 2))) =  DISPCTL_B_OE_BIT;
#define DISPCTL_R_OE_ON()							HWREG(DISPCTL_R_OE_PORT + (GPIO_O_DATA + (DISPCTL_R_OE_BIT << 2))) = ~DISPCTL_R_OE_BIT;
#define DISPCTL_G_OE_ON()							HWREG(DISPCTL_G_OE_PORT + (GPIO_O_DATA + (DISPCTL_G_OE_BIT << 2))) = ~DISPCTL_G_OE_BIT;
#define DISPCTL_B_OE_ON()							HWREG(DISPCTL_B_OE_PORT + (GPIO_O_DATA + (DISPCTL_B_OE_BIT << 2))) = ~DISPCTL_B_OE_BIT;

// Display output enable timing
#define DISPCTL_OE_PRESCALER					12			// 100ns min pulse, 6.55ms max pulse

// Maximum possible OE pulse width (in counts) at 120Hz timing
// This forms the base of other pulse width calculations
#define DISPCTL_OE_MAX_PULSE					41600

// OE scaling to allow for a shifted R-G-B drive.  This reduces OE pulse widths 
// so red fires first, followed by green, followed by blue. 
// The effect is to reduce ripple on the power supply and allow the blue subfield to 
// trigger the next load of data into the panel.
// There is a calculated offset for each channel based on the channel pulse width.
#define DISPCTL_OE_SFTDRV_R_OFFS			100 
#define DISPCTL_OE_SFTDRV_G_OFFS			200
#define DISPCTL_OE_SFTDRV_B_OFFS			300 
#define DISPCTL_OE_SFTDRV_SCALE				700 

// Offset from OE all timing, which is the delay beyond the maximum OE width that will be
// used before the next subfield
#define DISPCTL_OE_OFFSET_ALL					150			// relative to width of field * 1024

// Display intensity scaling
#define DISP_INT_SCALE								1024		// = max intensity
#define DISP_INT_SHIFT								10			// 2^N = DISP_INT_SCALE

// Display frame generator prescaler
// The frame generator operates from around 50 - 200Hz, depending on configuration
// It is set up to allow a minimum of 25Hz (40ms max time)... cinematic! 
#define DISPCTL_FRGEN_PRESCALE				74
#define DISPCTL_FRGEN_CALC_FREQ(hz)		(1.0f / (hz * (1.0f / (CPU_CLOCK / DISPCTL_FRGEN_PRESCALE))))

// OE states
#define OE_STATE_GOLOW								1				// Go low in ISR
#define OE_STATE_GOHIGH								2				// Go high in ISR

// ABL scale factor: conversion of luminance (0-4095) to milliwatts (0-36000)
#define ABL_POW_MULT									9002
#define ABL_POW_SHIFT									10

// ABL averaging coefficients (IIR)
#define ABL_AVG_PWR_MULT_NEW					1				// New value
#define ABL_AVG_PWR_MULT_CUR					1023		// Current value
#define ABL_AVG_PWR_SHIFT							10			// Shift for divider

// LED divider for frame sync indication
#define LED_DIVIDE_FSYNC							16

extern uint32_t abl_peak_power;
extern uint32_t abl_avg_power;
extern uint32_t calc_cur_power;
extern uint32_t calc_avg_power;
extern uint32_t abl_limit_oe;
	
extern uint32_t disp_frame;
extern uint32_t oe_ints_global;

struct pix_rgbg_t {
	// Gamma corrected values
	uint16_t r, b;
	uint16_t g1, g2;
};

struct disp_latch_lut_t {
	uint32_t port_base;
	uint32_t port_mask;
	uint32_t port_invmask;
};

struct disp_addr_lut_t {
	uint32_t write_mask;
};

void Timer0AIntHandler();
void Timer0BIntHandler();
void Timer1AIntHandler();
void disp_io_setup();
void disp_output_subfield(uint8_t *sf);
void disp_run();
void disp_drive_frame();
void disp_drive_subfield();
void disp_write_pixel(uint32_t x, uint32_t y, struct pix_rgbg_t *pix) ;
void disp_write_pixel_rgb(uint32_t x, uint32_t y, uint32_t r, uint32_t g, uint32_t b) ;
void disp_wait_for_frame();
void disp_flush_display(uint32_t block);
void disp_debug();
void disp_set_brightness(uint32_t bri);

#endif // ___DISPCTL_H___