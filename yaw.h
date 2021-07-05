//*****************************************************************************
//
// Functions for the calculation of the helicopter yaw angle
// fripmgroup2
// Authors:  Lei Li, Junwei Liang, Michael Redepenning
//
//*****************************************************************************

#ifndef YAW_H
#define YAW_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "buttons4.h"
#include "pwmControl.h"

//*****************************************************************************
// Constants
//*****************************************************************************

#define MAX_DEGREE 360

// The rotation direction of the helicopter
enum direction {
    ANTICLOCKWISE = -1, STILL, CLOCKWISE
};

// The Hardware Details of the GPIO output for the quadratic decoder
#define CHANNEL_A GPIO_PIN_0
#define CHANNEL_B GPIO_PIN_1
#define FIRST_BIT 0b00000001
#define SECOND_BIT 0b00000010

// The Hardware Details of the yaw reference
#define YAW_REF_PERIPH  SYSCTL_PERIPH_GPIOC
#define YAW_REF_PORT_BASE  GPIO_PORTC_BASE
#define YAW_REF_PIN  GPIO_PIN_4

//*****************************************************************************
// Global Values
//*****************************************************************************
extern uint32_t yawFlag;
extern int32_t degree;
extern int32_t desired_degree;
extern uint32_t find_yaw_ref_enable;
extern int32_t desired_yaw;


// Function to initialize the reference of orientation direction.
void orientationInitial(void);

// Function to read the input of PB0 and PB1 and find the rotation direction of the helicopter.
void checkDirection(void);

// Function to calculate the helicopter yaw angle according to the rotation direction.
void calculateYaw(void);

// Function to convert the value of yaw into degree.
void calculateDegree(void);

// The GPIO interrupt for yaw angle.
void initYawGPIO(void);

// Function responses to the GPIO interrupt for yaw angle.
void GPIOYawIntHandler(void);

// The GPIO interrupt for the reference orientation.
void getYawRef(void);

// Function responses to the GPIO interrupt for the reference orientation.
void yawRefIntHandler(void);

// Update the value of desired yaw when the LEFT button or the RIGHT button has been pushed.
void updateDesiredYaw(void);

#endif
