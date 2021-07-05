//*****************************************************************************
//
// Functions for ADC, Clock, SysTick, Soft reset and Watchdog Timer
// fripmgroup2
// Authors:  Lei Li, Junwei Liang, Michael Redepenning
//
//*****************************************************************************

#ifndef SYSTEM_H
#define SYSTEM_H

#include <pwmControl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "circBufT.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "inc/hw_memmap.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/watchdog.h"


//*****************************************************************************
// Constants
//*****************************************************************************

#define SAMPLE_RATE_HZ 100  // at least signal_frequency * 2 * buffer size = 80
#define DISPLAY_RATE 10
#define YAW_NOISE_MARGIN 2

//*****************************************************************************
// Global Values
//*****************************************************************************

extern circBuf_t g_inBuffer;
extern uint32_t g_ulSampCnt;
extern uint32_t displayFlag;
extern uint32_t pwmFlag;
extern int32_t desired_altitude;
extern int32_t heliMode;
extern int32_t desired_yaw;


// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
void ADCIntHandler(void);

// Function to initialize sequence 3 step 0 sample channel 9
// and enable interrupts.
void initADC(void);

// The system clock
void initClocks(void);

// SysTick interrupt
void initSysTick(void);

// The interrupt handler for SysTick interrupt.
void SysTickIntHandler(void);

// Function to toggle the buttonFlag
uint32_t getButtonFlag(void);

// The soft reset interrupt
void initSoftReset(void);

// The interrupt handler for the soft reset interrupt.
void softResetIntHandler(void);

// The initialization of the watchdog timer
void initWatchDogTimer (void);

// Function to reload the watchdog timer
void reloadWatchDogTimer (void);


#endif
