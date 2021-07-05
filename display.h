//*****************************************************************************
//
// Functions for the OLED display and the UART serial transmission
// fripmgroup2
// Authors:  Lei Li, Junwei Liang, Michael Redepenning
//
//*****************************************************************************

#ifndef DISPLAY_HEADER_H
#define DISPLAY_HEADER_H

#include <stdint.h>
#include <stdbool.h>
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "driverlib/uart.h"
#include "utils/ustdlib.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"

//*****************************************************************************
// Constants
//*****************************************************************************

#define MAX_STRING_LEN 17

// The Hardware Details of USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

//*****************************************************************************
// Global Values
//*****************************************************************************
extern uint32_t control;
extern int32_t yaw;
extern int32_t heliMode;
extern int32_t altitude_in_percentage;
extern int32_t desired_altitude_percentage;
extern int32_t desired_altitude;
extern int32_t desired_yaw;
extern uint32_t pwmDuty;
extern uint32_t tailDuty;
extern int32_t degree;
extern int32_t desired_degree;


// Function to initialize the Orbit OLED display.
void initDisplay(void);

// Function to display the duty cycle of the main motor, the percentage altitude,
// the yaw degree and the mode of the helicopter.
void displayValue(void);

// Initialise the serial communicator.
void initialiseUSB_UART(void);

// Function to write the characters into the UART buffer.
void UARTSend(char *pucBuffer);

// Form and send the message to the console.
void serialTransmission(void);


#endif





