//*****************************************************************************
//
// Functions for ADC, Clock, SysTick, Soft reset and Watchdog Timer
// fripmgroup2
// Authors:  Lei Li, Junwei Liang, Michael Redepenning
// based on sample code from labs
//
//*****************************************************************************

#include "system.h"

//*****************************************************************************
// Global variables
//*****************************************************************************

/* Flag for polling the buttons */
static uint32_t buttonFlag = 0;

/* Flag for turning off the motors */
static uint8_t motor_disable = 0;


// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
void
ADCIntHandler(void) {
    uint32_t ulValue;
    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf(&g_inBuffer, ulValue);

    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}


// Function to initialize sequence 3 step 0 sample channel 9
// and enable interrupts.
void
initADC(void) {
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 9 (ADC_CTL_CH9) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                                              ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Register the interrupt handler
    ADCIntRegister(ADC0_BASE, 3, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}


// The system clock
void
initClocks(void) {
    // Set the clock rate to 20 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
}


// SysTick interrupt
void
initSysTick(void) {
    //
    // Set up the period for the SysTick timer.  The SysTick
    // timer period is set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}


// The interrupt handler for SysTick interrupt.
void SysTickIntHandler(void) {
    // Initiate a conversion
    ADCProcessorTrigger(ADC0_BASE, 3);
    // Count the number of ADC samples.
    g_ulSampCnt++;
    // Set up the flag when it's time to poll button.
    buttonFlag = 1;
    // Set up the flag when it's time to update the output of Pi-controller.
    pwmFlag = 1;
    // Set up the flag to update the OLED display according to display update rate.
    if (g_ulSampCnt % DISPLAY_RATE == 0) {
        displayFlag = 1;
    }
    // When the helicopter is in the landing mode,
    // decrement the desired altitude till it reaches the value of the reference landed altitude.
    //Update the mode to landed and set up flag to turn off the motors.
    if (heliMode == LANDING && (meanVal >= desired_altitude && desired_altitude < landed_altitude)) {
        desired_altitude += MAX_REDUCE * 10;
        desired_yaw = 0;
        if (desired_altitude >= landed_altitude) {
            heliMode = LANDED;
            motor_disable = 1;
        }
    }
    // When the flag is set and the helicopter has landed and turns back to the orientation,
    // turn off the motors and clear the flag.
    // Tolerance margins are set for altitude and yaw to make the code compatible for different rigs.
    if (motor_disable && (meanVal >= (desired_altitude - MAX_REDUCE)) && (yaw <= YAW_NOISE_MARGIN)) {
        PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
        PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
        PWMGenDisable(PWM_MAIN_BASE, PWM_MAIN_GEN);
        PWMGenDisable(PWM_MAIN_BASE, PWM_TAIL_GEN);
        motor_disable = 0;
    }
}


// Function to toggle the buttonFlag
uint32_t
getButtonFlag(void) {
    uint32_t old = buttonFlag;
    buttonFlag = 0;
    return old;
}


// The interrupt handler for the soft reset interrupt.
void softResetIntHandler(void) {
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
    SysCtlReset();
}


// The soft reset interrupt
void initSoftReset(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
    GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN);
    GPIOIntRegister(GPIO_PORTA_BASE, softResetIntHandler);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6);
}


// The initialization of the watchdog timer
void initWatchDogTimer(void) {
    //
    // Enable the Watchdog 0 peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    //
    // Wait for the Watchdog 0 module to be ready.
    //
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG0)) {
    }
    //
    // Check to see if the registers are locked, and if so, unlock them.
    //
    if (WatchdogLockState(WATCHDOG0_BASE) == true) {
        WatchdogUnlock(WATCHDOG0_BASE);
    }
    //
    // Initialize the watchdog timer.
    //
    WatchdogReloadSet(WATCHDOG0_BASE, 0xFEEFEE);
    //
    // Enable the reset.
    //
    WatchdogResetEnable(WATCHDOG0_BASE);
    //
    // Enable the watchdog timer.
    //
    WatchdogEnable(WATCHDOG0_BASE);
}


// Function to reload the watchdog timer
void reloadWatchDogTimer(void) {
    if (WatchdogLockState(WATCHDOG0_BASE) == true) {
        WatchdogUnlock(WATCHDOG0_BASE);
    }

    WatchdogReloadSet(WATCHDOG0_BASE, 0xFEEFEE);

    WatchdogLock(WATCHDOG0_BASE);
}
