/*
 * Large display controller
 *
 * (C) 2017 Thomas Oldbury, released under MIT licence
 */

#ifndef ___HAL_H___
#define ___HAL_H___

#define		MIN(a,b)					(((a)<(b))?(a):(b))
#define		MAX(a,b)					(((a)>(b))?(a):(b))

#define		CLAMP(x,mn,mx)		MAX(MIN((x),(mx)),(mn))
#define		CLAMPH(x)					CLAMP((x),0.0f,360.0f)
#define		CLAMPSV(x)				CLAMP((x),0.0f,1.0f)

// Desired CPU clock
#define 	CPU_CLOCK					120000000

// Systick rate for ethernet task
#define 	SYSTICK_RATE    	1000		// in Hz
#define 	SYSTICK_TIME_MS		(1000.f / SYSTICK_RATE)

void part_init();
void tcpip_init();
void cpu_reset();

#endif // ___HAL_H___