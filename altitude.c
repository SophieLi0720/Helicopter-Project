//*****************************************************************************
//
// Functions for the calculation of the helicopter altitude
// fripmgroup2
// Authors:  Lei Li, Junwei Liang, Michael Redepenning
//
//*****************************************************************************

#include "altitude.h"

//*****************************************************************************
// Global variables
//*****************************************************************************
/* landing height reference. */
uint32_t landed_altitude = 0;

/* A variable to convert the ADC value to a percentage of helicopter height. */
int32_t altitude_in_percentage = 0;
int32_t desired_altitude_percentage = 0;

/* Averaged ADC value. */
int32_t meanVal = 0;

/* Reference ADC value when the height is 100%. */
int32_t max_altitude = 0;

/* The up and down button states.
 * These can be RELEASED, PUSHED, and NO_CHANGE.*/
static uint8_t upButState = NO_CHANGE;
static uint8_t downButState = NO_CHANGE;


// Function to calculate the mean ADC value.
void calculateMeanVal(void) {
    uint16_t i;
    int32_t sum = 0;
    for (i = 0; i < BUF_SIZE; i++)
        sum = sum + readCircBuf(&g_inBuffer);
    meanVal = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
}


// Function to convert the helicopter altitude into a percentage value of the helicopter height.
void calculatePercentage(void ) {
    if (meanVal <= landed_altitude) {
        altitude_in_percentage = ((landed_altitude - meanVal) / MAX_REDUCE);
    } else {
        altitude_in_percentage = -((meanVal - landed_altitude) / MAX_REDUCE);
    }

    if (desired_altitude <= landed_altitude) {
        desired_altitude_percentage = ((landed_altitude - desired_altitude) / MAX_REDUCE);
    } else {
        desired_altitude_percentage = -((desired_altitude - landed_altitude) / MAX_REDUCE);
    }
}

// Initialization of the landed altitude and the maximum altitude. Both of them will be a main ADC value.
void initLandedAltitude(void) {
    calculateMeanVal();
    landed_altitude = meanVal;
    desired_altitude = meanVal;
    max_altitude = landed_altitude - ALT_RANGE;
}


// Change the value of desired altitude when the up or down button has been pushed.
void updateDesiredAltitude(void) {

    // Check state of UP button
    upButState = checkButton(UP);
    switch (upButState) {
        case PUSHED:
            // Increase desired altitude by 10%.
            desired_altitude -= MAX_REDUCE * 10;
            // Limit input to maximum altitude.
            if (desired_altitude <= max_altitude) {
                desired_altitude = max_altitude;
            }
            break;
        case RELEASED:
            break;
    }
    // Check state of DOWN button
    downButState = checkButton(DOWN);
    switch (downButState) {
        case PUSHED:
            // Decrease desired altitude by 10%.
            desired_altitude += MAX_REDUCE * 10;
            // Limit input to landed altitude.
            if (desired_altitude >= landed_altitude) {
                desired_altitude = landed_altitude;
            }
            break;
        case RELEASED:
            break;
    }
}

