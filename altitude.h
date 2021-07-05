//*****************************************************************************
//
// Header for helicopter altitude module
// fripmgroup2
// Authors:  Lei Li, Junwei Liang, Michael Redepenning
//
//*****************************************************************************

#ifndef ALTITUDE_H
#define ALTITUDE_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "circBufT.h"
#include "buttons4.h"

//*****************************************************************************
// Constants for averaging buffer size, input scaling and altitude span
//*****************************************************************************
#define BUF_SIZE 10
#define MAX_REDUCE 8
#define ALT_RANGE 800

//*****************************************************************************
// Global variables to provide access to ADC average and input altitude
//*****************************************************************************

extern circBuf_t g_inBuffer;
extern int32_t desired_altitude;


// Function to calculate the mean ADC value.
void calculateMeanVal(void);

// Function to convert the helicopter altitude into a percentage value of the helicopter height.
void calculatePercentage(void);

// Initialization of the landed altitude and the maximum altitude. Both of them will be a main ADC value.
void initLandedAltitude(void);

// Change the value of desired altitude when the up or down button has been pushed.
void updateDesiredAltitude(void);

#endif
