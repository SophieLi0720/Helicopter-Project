//*****************************************************************************
//
// Main function of ENCE361 Helicopter Project
// fripmgroup2
// Authors:  Lei Li, Junwei Liang, Michael Redepenning
//
//*****************************************************************************

#include <pwmControl.h>
#include "altitude.h"
#include "buttons4.h"
#include "driverlib/interrupt.h"
#include "yaw.h"
#include "system.h"
#include "display.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 10
#define PWM_START_RATE_HZ  250
#define PWM_START_DUTY 30

//*****************************************************************************
// Global variables
//*****************************************************************************
/* a global instance of the circular buffer */
circBuf_t g_inBuffer;

/* counter for the number of ADC sample */
uint32_t g_ulSampCnt;

/* flags to get events synchronous */
uint32_t yawFlag = 0;
uint32_t displayFlag = 0;
uint32_t pwmFlag = 0;

/* global variables for pi-controller */
uint32_t control = 0;
uint32_t pwmDuty = PWM_START_DUTY;
uint32_t tailDuty = PWM_TAIL_SPIN_DUTY;
uint32_t find_yaw_ref_enable = 1;

/* variable to convert the value of yaw into degree */
int32_t degree = 0;
int32_t desired_degree = 0;

// The main function.
int main(void) {

    initClocks();
    initSoftReset();

    // As a precaution, make sure that the peripherals used are reset
    SysCtlPeripheralReset(PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset(PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM
    SysCtlPeripheralReset(PWM_TAIL_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset(PWM_TAIL_PERIPH_PWM);  // Tail Rotor PWM

    //Initialise all modules
    initButtons();
    InitPIMainController();
    initialiseMainPWM();
    initialiseTailPWM();
    initSysTick();
    initCircBuf(&g_inBuffer, BUF_SIZE);
    initADC();
    initYawGPIO();
    initSW1();
    initialiseUSB_UART();
    initDisplay();
    IntMasterEnable();
    initLandedAltitude();
    orientationInitial();
    initWatchDogTimer();

    //main loop
    while (1) {
        //buttons can change altitude and yaw
        if (getButtonFlag() && !find_yaw_ref_enable) {
            updateButtons();
            updateDesiredAltitude();
            updateDesiredYaw();
        }

        if (yawFlag) {
            calculateDegree();
            yawFlag = 0;
        }

        if (displayFlag) {
            calculateMeanVal();
            calculatePercentage();
            serialTransmission();
            displayValue();
            displayFlag = 0;
        }

        //normal operation after finding reference slot
        if (pwmFlag && find_yaw_ref_enable) {
            ref_piTailMotorUpdate();
            setTailPWM(PWM_START_RATE_HZ, tailDuty);
        }

        //rotate at low speed until reference slot in the quadrature encoder is found
        if (pwmFlag && !find_yaw_ref_enable) {
            calculateMeanVal();
            piMainMotorUpdate();
            setMainPWM(PWM_START_RATE_HZ, pwmDuty);
            piTailMotorUpdate();
            setTailPWM(PWM_START_RATE_HZ, tailDuty);
            pwmFlag = 0;
        }

        reloadWatchDogTimer();
    }
}

