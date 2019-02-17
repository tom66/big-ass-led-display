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

#include "dispctl.h"

#define 	SYSTICKHZ       100
#define 	SYSTICKMS       (1000 / SYSTICKHZ)

struct HSVTriple {
	float h;
	float s;
	float v;
};

struct RGBTriple {
	float r;
	float g;
	float b;
};

struct RGBTripleInt {
	uint8_t r, g, b;
};

static const struct {
  unsigned int 	 width;
  unsigned int 	 height;
  unsigned int 	 bytes_per_pixel; /* 2:RGB16, 3:RGB, 4:RGBA */ 
  unsigned char	 pixel_data[16 * 16 * 3 + 1];
} gimp_image = {
  16, 16, 3,
  "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\212\0&\245\0""8\263\0""8\263\0""8\263"
  "\0\204\342\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\5\216\0\21\226"
  "\0\34\235\0&\245\0\5\216\0\5\216\0\5\216\0U\305\0Y\307\0""4\253\0\0\0\0\0"
  "\0\0\0\0\0\0\0\0\0\0\0&\245\0""8\263\0\0\200\0\0\200\0\0\200\0\0\200\0\0"
  "\200\0\0\200\0\0\200\0\0\200\0""8\263\0\0\200\0\0\0\0\0\0\0\0\0\0O\301\0"
  "&\245\0\0\200\0\0\200\0\0\200\0\0\200\377\377\377\377\377\377\0\0\200\0\0"
  "\200\0\0\200\0&\245\0&\245\0\0\200\0\0\0\0\0\0\0K\305\0\0\200\0\0\200\0\0"
  "\200\0\0\200\377\377\377\377\377\377\377\377\377\377\377\377\0\0\200\0\0"
  "\200\0\0\200\0""8\263\0\5\216\0\0\0\0z\336\0&\245\0\0\200\0\0\200\0\0\200"
  "\0\0\200\0\0\200\377\377\377\377\377\377\377\377\377\377\377\377\0\0\200"
  "\0\0\200\0&\245\0&\245\0\0\200\0\200\342\0\5\216\0\0\200\0\0\200\377\377"
  "\377\0\0\200\0\0\200\0\0\200\377\377\377\377\377\377\377\377\377\377\377"
  "\377\0\0\200\0\0\200\0""8\263\0\5\216\0""8\263\0\5\216\0\0\200\377\377\377"
  "\377\377\377\377\377\377\0\0\200\377\377\377\377\377\377\377\377\377\377"
  "\377\377\377\377\377\377\377\377\0\0\200\0\0\200\0C\271\0O\275\0\0\212\0"
  "\0\200\377\377\377\377\377\377\377\377\377\377\377\377\377\377\377\377\377"
  "\377\0\0\200\377\377\377\377\377\377\377\377\377\0\0\200\0\0\210\0]\314\0"
  "O\275\0\0\210\0\0\200\0\0\200\377\377\377\377\377\377\377\377\377\377\377"
  "\377\0\0\200\0\0\200\0\0\200\377\377\377\0\0\200\0\0\200\0\0\212\0_\314\0"
  "\204\344\0\40\241\0\0\200\0\0\200\0\0\200\377\377\377\377\377\377\377\377"
  "\377\377\377\377\0\0\200\0\0\200\0\0\200\0\0\200\0\0\200\0C\271\0&\245\0"
  "\0\0\0""8\263\0&\245\0\5\216\0\0\200\0\0\200\377\377\377\377\377\377\377"
  "\377\377\377\377\377\0\0\200\0\0\200\0\0\200\0&\245\0&\245\0\0\0\0\0\0\0"
  "&\245\0c\320\0C\271\0""0\253\0\0\200\0\0\200\377\377\377\377\377\377\0\0"
  "\200\0\0\200\0\0\200\0C\271\0&\245\0\0\200\0\0\0\0\0\0\0\0\0\0C\271\0C\271"
  "\0""8\263\0""8\263\0\0\200\0\0\200\0\0\200\0\0\200\0\0\200\0&\245\0&\245"
  "\0\0\200\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0""0\253\0C\271\0k\324\0O\301\0\0"
  "\200\0\0\200\0\5\216\0""8\263\0&\245\0\0\200\0\0\0\0\0\0\0\0\0\0\0\0\0\0"
  "\0\0\0\0\0\0\0\0\0\0\0C\271\0\204\342\0z\332\0m\326\0O\275\0\5\216\0\0\0"
  "\0\0\0\0\0\0\0\0\0\0\0\0",
};


int main()
{
	volatile uint32_t dly = 0;
	uint32_t x, y, n, i, j, c, patt;
	struct RGBTripleInt col;
	struct HSVTriple hsv;
	
	boot_init();
	
	UARTprintf("Ready.\r\n");
	
	while(1) ;
	
	n = 0;
	
	patt = 5;
	
	while(1) {
		UARTprintf("Test pattern %d\r\n", patt);
		
		switch(patt) {
			case 0:
				// Logo test
				for(n = 0; n < 20; n++) {
					for(x = 0; x < 16; x++) {
						for(y = 0; y < 16; y++) {
							disp_write_pixel_rgb(x, y, \
								(gimp_image.pixel_data[(x * 3) + (y * 48)]), \
								(gimp_image.pixel_data[(x * 3) + (y * 48) + 1]), \
								(gimp_image.pixel_data[(x * 3) + (y * 48) + 2]));
						}
					}
					
					for(dly = 1300000; dly > 0; dly--) ;
					disp_flush_display(1);
					disp_debug();
				}
				patt++;
				break;
			
			case 1:
				// Gradient test #1
				for(n = 0; n < 150; n++) {
					for(x = 0; x < 16; x++) {
						for(y = 0; y < 16; y++) {
							j = ((x * 16) + (disp_frame / 2)) & 0xff;
							disp_write_pixel_rgb(x, y, j, 0, 0);
						}
					}
			
					for(dly = 1300000; dly > 0; dly--) ;
					disp_flush_display(1);
					disp_debug();
				}
				patt++;
				break;
			
			case 2:
				// Gradient test #2
				for(n = 0; n < 150; n++) {
					for(x = 0; x < 16; x++) {
						for(y = 0; y < 16; y++) {
							j = ((y * 16) + (disp_frame / 2)) & 0xff;
							disp_write_pixel_rgb(x, y, 0, j, 0);
						}
					}
			
					for(dly = 1300000; dly > 0; dly--) ;
					disp_flush_display(1);
					disp_debug();
				}
				patt++;
				break;
			
			case 3:
				// Gradient test #3
				for(n = 0; n < 150; n++) {
					for(x = 0; x < 16; x++) {
						for(y = 0; y < 16; y++) {
							j = ((-x * 16) + (disp_frame / 2)) & 0xff;
							disp_write_pixel_rgb(x, y, 0, 0, j);
						}
					}
			
					for(dly = 1300000; dly > 0; dly--) ;
					disp_flush_display(1);
					disp_debug();
				}
				patt++;
				break;
			
			case 4:
				// Gradient test #4
				for(n = 0; n < 150; n++) {
					for(x = 0; x < 16; x++) {
						for(y = 0; y < 16; y++) {
							j = (((x + y) * 16) + (disp_frame / 2)) & 0xff;
							disp_write_pixel_rgb(x, y, j, j, j);
						}
					}
			
					for(dly = 1300000; dly > 0; dly--) ;
					disp_flush_display(1);
					disp_debug();
				}
				patt++;
				break;
			
			case 5:
				// Ramp bright test
				for(n = 0; n < 255; n++) {
					for(x = 0; x < 16; x++) {
						for(y = 0; y < 16; y++) {
							disp_write_pixel_rgb(x, y, n, n, n);
						}
					}
			
					for(dly = 1300000; dly > 0; dly--) ;
					disp_flush_display(1);
					disp_debug();
				}
				patt++;
				break;
				
			case 6:
				// Seven colour test
				for(c = 1; c < 8; c++) {
					for(n = 0; n < 255; n += 2) {
						for(x = 0; x < 16; x++) {
							for(y = 0; y < 16; y++) {
								disp_write_pixel_rgb(x, y, n * !!(c & 4), n * !!(c & 2), n * !!(c & 1));
							}
						}
				
						for(dly = 1300000; dly > 0; dly--) ;
						disp_flush_display(1);
						disp_debug();
					}
				}
				patt++;
				break;
			
			default:
				patt = 0;
				break;
		}
	}
	
	/*
	while(1) {
		for(x = 0; x < 16; x++) {
			for(y = 0; y < 16; y++) {
				disp_write_pixel_rgb(x, y, 255, 255, 255);
			}
		}
			
		for(dly = 1300000; dly > 0; dly--) ;
		disp_debug();
	}
	*/
	
	/*
	while(1) {
		for(x = 0; x < 16; x++) {
			for(y = 0; y < 16; y++) {
				disp_write_pixel_rgb(x, y, 0, 0, 0);
			}
		}
			
		for(n = 0; n < 16; n++) {
			for(x = 0; x < n; x++) {
				for(y = 0; y < 16; y++) {
					//disp_write_pixel_rgb(x, y, ((16 - x) * 16) + disp_frame, ((16 - y) * 16) + disp_frame, ((x + y) * 16) + disp_frame);
					//j = ((x * 15) + (disp_frame / 4)) & 0xff;
					//disp_write_pixel_rgb(x, y, j, 255 - j, 0);
					//j = (disp_frame / 16) & 0xff;
					j = 255;
					disp_write_pixel_rgb(x, y, j, j, j);
				}
			}
			
			for(dly = 13000000; dly > 0; dly--) ;
			
			disp_debug();
		}
	}
	*/
	
	
	
	while(1) {
		// draw some stuff...
		for(x = 0; x < 32; x++) {
			for(y = 0; y < 32; y++) {
				disp_write_pixel_rgb(x, y, x * y, x * y, x * y);
				for(dly = 300000; dly > 0; dly--) ;
			}
		}
		
		for(x = 0; x < 32; x++) {
			for(y = 0; y < 32; y++) {
				disp_write_pixel_rgb(x, y, 4095, 4095, 4095);
				for(dly = 300000; dly > 0; dly--) ;
			}
		}
	}
	
	return 0;
}