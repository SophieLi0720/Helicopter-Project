//*****************************************************************************
//
// Functions for the OLED display and the UART serial transmission
// fripmgroup2
// Authors:  Lei Li, Junwei Liang, Michael Redepenning
//
//*****************************************************************************

#include "display.h"

//*****************************************************************************
// Global variables
//*****************************************************************************
// An array of strings for display helicopter mode
static char *mode_str[3] = {"Landed", "Flying", "Landing"};


// Function to initialize the Orbit OLED display.
void initDisplay(void) {
    OLEDInitialise();
}


// Function to display the duty cycle of the main motor, the percentage altitude,
// the yaw degree and the mode of the helicopter.
void displayValue(void) {
    char string[MAX_STRING_LEN];  // 16 characters across the display
    // Update line on OLED display.
    usnprintf(string, sizeof(string), "Duty(%%) = %4d", pwmDuty);
    OLEDStringDraw(string, 0, 0);
    usnprintf(string, sizeof(string), "Alt(%%) = %4d", altitude_in_percentage);
    OLEDStringDraw(string, 0, 1);
    usnprintf(string, sizeof(string), "Yaw(Deg) = %4d", degree);
    OLEDStringDraw(string, 0, 2);
    usnprintf(string, sizeof(string), "Mode # %s", mode_str[heliMode + 1]);
    OLEDStringDraw(string, 0, 3);
}


// Initialise the serial communicator.
void
initialiseUSB_UART(void) {
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);
    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}


// Function to write the characters into the UART buffer.
void
UARTSend(char *pucBuffer) {
    // Loop while there are more characters to send.
    while (*pucBuffer) {
        // Write the next character to the UART Tx FIFO.
        UARTCharPut(UART_USB_BASE, *pucBuffer);
        pucBuffer++;
    }
}


// Form and send the message to the console.
void serialTransmission(void) {
    char string[MAX_STRING_LEN];
    usprintf(string, "Alt: %d [%d]\n", altitude_in_percentage, desired_altitude_percentage);
    UARTSend(string);
    usprintf(string, "Yaw: %d [%d]\n", degree, desired_degree);
    UARTSend(string);
    usprintf(string, "Main: %d Tail: %d\n", pwmDuty, tailDuty);
    UARTSend(string);
    usprintf(string, "Mode # %s\n", mode_str[heliMode + 1]);
    UARTSend(string);
    usprintf(string, "------------\n");
    UARTSend(string);
}
