//*****************************************************************************
//
// Functions for controlling the PWM output
// fripmgroup2
// Authors:  Lei Li, Junwei Liang, Michael Redepenning
//
//*****************************************************************************

#include "pwmControl.h"

//*****************************************************************************
// Global variables
//*****************************************************************************

/* The mode of the helicopter to display.
 * This can be LANDED, FLYING or LANDING.
 */
int32_t heliMode = LANDED;

/* Variables for error calculation Pi-controller. */
int32_t desired_altitude = 0; //this is max altitude so must be set to a safe level before taking off.
int32_t desired_yaw = START_ERROR; // An offset to start moving to find the reference

/* Define three unique global instances of the PI controller.
 * The PiMainController and the PiTailController will be used after the helicopter finds the reference orientation.
 * The PiRefController will be used to find the reference orientation.
 */
PIDController_s PiMainController;
PIDController_s PiTailController;
PIDController_s PiRefController;

/*********************************************************
 * initialisePWM
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 *********************************************************/

void
initialiseMainPWM(void) {
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);

    // Wait for the PWM0 module to be ready.
    //
    while (!SysCtlPeripheralReady(PWM_MAIN_PERIPH_PWM)) {
    }
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    setMainPWM(PWM_START_RATE_HZ, PWM_START_DUTY);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
}

/*********************************************************
 * initialisePWM
 * M1PWM5 (J3-10, PF1) is used for the tail rotor motor
 *********************************************************/

void
initialiseTailPWM(void) {
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);

    // Wait for the PWM0 module to be ready.
    //
    while (!SysCtlPeripheralReady(PWM_TAIL_PERIPH_PWM)) {
    }
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    setTailPWM(PWM_START_RATE_HZ, PWM_TAIL_SPIN_DUTY);

    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}

/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
void
setMainPWM(uint32_t ui32Freq, uint32_t ui32MainDuty) {
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
            SysCtlClockGet() / PWM_DIVIDER / ui32Freq;
    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
                     ui32Period * ui32MainDuty / 100);
}

/********************************************************
 * Function to set the freq, duty cycle of M1PWM5
 ********************************************************/

void
setTailPWM(uint32_t ui32Freq, uint32_t ui32TailDuty) {
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
            SysCtlClockGet() / PWM_DIVIDER / ui32Freq;
    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
                     ui32Period * ui32TailDuty / 100);
}

/********************************************************
 * The initialization function for a Pi-controller
 ********************************************************/

void initPiController(PIDController_s *controller, double Kp, double Ki,
                      double outMax, double outMin, double samp_interval) {
    /* Clear controllers */
    controller->integrator = 0.0f;
    controller->out = 0.0f;
    /* Assign value to Kp and Ki */
    controller->Kp = Kp;
    controller->Ki = Ki;
    /* Assign value to Output limits*/
    controller->limMin = outMin;
    controller->limMax = outMax;
    /* Assign sample interval(in second) to T */
    controller->T = samp_interval;
}

/********************************************************
 * The specific set up for three distinct Pi-controllers
 ********************************************************/

void InitPIMainController(void) {
    initPiController(&PiMainController, MAIN_KP, MAIN_KI,
                     MAIN_MAX_OUT, MAIN_MIN_OUT, SAMPLE_INTERVAL);
}

void InitPITailController(void) {
    initPiController(&PiTailController, TAIL_KP, TAIL_KI,
                     TAIL_MAX_OUT, TAIL_MIN_OUT, SAMPLE_INTERVAL);
}

void ref_InitPITailController(void) {
    initPiController(&PiRefController, REF_KP, REF_KI,
                     REF_MAX_OUT, REF_MIN_OUT, SAMPLE_INTERVAL);
}

/********************************************************
 * The implementation of a Pi-controller
 ********************************************************/

double piUpdate(PIDController_s *controller, int32_t desired_value,
                int32_t actual_value, int32_t offset) {
    /* Error signal */
    double error = actual_value - desired_value;
    /* Proportional */
    double proportional = controller->Kp * error;
    /* Anti-wind-up dynamic integrator clamping */
    double limMinInt, limMaxInt;
    /* Compute integrator Limits */
    if (controller->limMax > proportional) {
        limMaxInt = controller->limMax - proportional;
    } else {
        limMaxInt = 0.0f;
    }
    if (controller->limMin < proportional) {
        limMinInt = controller->limMin - proportional;
    } else {
        limMinInt = 0.0f;
    }
    /* Clamp integrator */
    if (controller->integrator > limMaxInt) {
        controller->integrator = limMaxInt;
    } else if (controller->integrator < limMinInt) {
        controller->integrator = limMinInt;
    }
    /* Sum them up and compute the output and apply limits*/
    controller->out = proportional + controller->integrator;
    if (controller->out > controller->limMax) {
        controller->out = controller->limMax;
    } else if (controller->out < controller->limMin) {
        controller->out = controller->limMin;
        controller->integrator += offset;
    }
    /* Integral */
    controller->integrator += error * controller->T * controller->Ki;
    /* Apply the output */
    return controller->out;
}

/**********************************************************************
 * Functions to update the duty cycle of the main and the tail motor
 **********************************************************************/

void piMainMotorUpdate(void) {
    pwmDuty = piUpdate(&PiMainController, desired_altitude, meanVal, OFFSET_MAIN);
}

void piTailMotorUpdate(void) {
    tailDuty = piUpdate(&PiTailController, yaw, desired_yaw, OFFSET_DEFAULT);
}

void ref_piTailMotorUpdate(void) {
    tailDuty = piUpdate(&PiRefController, yaw, desired_yaw, OFFSET_DEFAULT);
}

/**********************************************************************
 * Function initialise the GPIO pin and register the GPIO interrupt
 * for the slide button SW1.
 **********************************************************************/

void initSW1(void) {
    SysCtlPeripheralEnable(SW1_BUT_PERIPH);
    GPIOPinTypeGPIOInput(SW1_BUT_PORT_BASE, SW1_BUT_PIN);
    GPIOPadConfigSet(SW1_BUT_PORT_BASE, SW1_BUT_PIN, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOIntTypeSet(SW1_BUT_PORT_BASE, SW1_BUT_PIN, GPIO_BOTH_EDGES);
    GPIOIntRegister(SW1_BUT_PORT_BASE, SW1IntHandler);
    GPIOIntEnable(SW1_BUT_PORT_BASE, GPIO_INT_PIN_7);
}

/**********************************************************************
 * Function responses to the GPIO interrupt when the slide button
 * SW1 has been pushed.
 **********************************************************************/

void SW1IntHandler(void) {
    heliMode = (heliMode + 1) % 2;

    switch (heliMode) {
        case FLYING:
            // Turn on the main motor and the tail motor
            PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);
            PWMGenEnable(PWM_MAIN_BASE, PWM_TAIL_GEN);
            PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
            PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
            // Enable the interrupt for the reference orientation
            getYawRef();
            // Start the Pi-controller to find the reference orientation
            ref_InitPITailController();
            break;
        case LANDING:
            break;
        case LANDED:
            break;
    }
    // Clear the interrupt
    GPIOIntClear(SW1_BUT_PORT_BASE, GPIO_INT_PIN_7);
}
