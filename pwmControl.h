//*****************************************************************************
//
// Functions for controlling the PWM output
// fripmgroup2
// Authors:  Lei Li, Junwei Liang, Michael Redepenning
//
//*****************************************************************************

#ifndef PWMCONTROL_H
#define PWMCONTROL_H

#include <stdint.h>
#include <stdio.h>
#include "buttons4.h"
#include "altitude.h"
#include "yaw.h"
#include "system.h"

//*****************************************************************************
// Constants
//*****************************************************************************

#define MAX_REDUCE 8
#define MAX_STRING_LEN 17
#define START_ERROR 5 //for moving to the reference slot

enum heliMode {
    LANDED = -1, FLYING, LANDING
};

// The sample interval of the integrator
#define SAMPLE_INTERVAL 0.01
// Offset for integrator
#define OFFSET_DEFAULT 0
#define OFFSET_MAIN 3

// Gains and limits for main PI controller
#define MAIN_KP 0.03
#define MAIN_KI 0.03
#define MAIN_MAX_OUT 90
#define MAIN_MIN_OUT 10

// Gains and limits for tail PI controller
#define TAIL_KP 0.6
#define TAIL_KI 0.4
#define TAIL_MAX_OUT 60
#define TAIL_MIN_OUT 10

// Gains and limits for reference-finding PI controller
#define REF_KP 1.0
#define REF_KI 0.0001
#define REF_MAX_OUT 50
#define REF_MIN_OUT 35

// PWM configuration
#define PWM_START_RATE_HZ  250
#define PWM_DUTY_MIN      5
#define PWM_DUTY_MAX      95
#define PWM_START_DUTY    30
#define PWM_DIVIDER_CODE  SYSCTL_PWMDIV_4
#define PWM_DIVIDER       4
#define PWM_TAIL_STABLE_DUTY 10
#define PWM_TAIL_SPIN_DUTY 28

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

//  PWM Hardware Details M1PWM5
//  ---Tail Rotor PWM: PF1, J3-10
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1

// The Hardware Details of SW1 (high = up) PA7
#define SW1_BUT_PERIPH  SYSCTL_PERIPH_GPIOA
#define SW1_BUT_PORT_BASE  GPIO_PORTA_BASE
#define SW1_BUT_PIN  GPIO_PIN_7

//*****************************************************************************
// Global Values
//*****************************************************************************
extern uint32_t g_ulSampCnt;
extern int32_t degree;
extern uint32_t ui32MainDuty;
extern uint32_t tailDuty;
extern uint32_t control;
extern uint32_t pwmDuty;
extern uint32_t landed_altitude;
extern int32_t max_altitude;
extern int32_t meanVal;
extern uint32_t yawRefFlag;
extern int32_t yaw;
extern uint32_t pi_control_enable;
extern uint32_t enable_landing;

// The struct of Pi-controller
typedef struct {
    /* Controller gains */
    double Kp;
    double Ki;
    /* Output limits */
    double limMin;
    double limMax;
    /* Sample time (in seconds) */
    double T;
    /* Controller: "memory" */
    double integrator;
    /* Controller output */
    double out;
} PIDController_s;

/*********************************************************
 * initialisePWM
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 *********************************************************/

void initialiseMainPWM(void);

/*********************************************************
 * initialisePWM
 * M1PWM5 (J3-10, PF1) is used for the tail rotor motor
 *********************************************************/

void initialiseTailPWM(void);

/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/

void setMainPWM(uint32_t ui32Freq, uint32_t ui32MainDuty);

/********************************************************
 * Function to set the freq, duty cycle of M1PWM5
 ********************************************************/

void setTailPWM(uint32_t ui32Freq, uint32_t ui32TailDuty);

/********************************************************
 * The initialization function for a Pi-controller
 ********************************************************/

void initPiController(PIDController_s *controller, double Kp, double Ki,
                      double outMax, double outMin, double samp_interval);

/********************************************************
 * The specific set up for three Pi-controllers
 * which will be used for different purpose
 ********************************************************/

void InitPIMainController(void);

void InitPITailController(void);

void ref_InitPITailController(void);

/********************************************************
 * The implementation of a Pi-controller
 ********************************************************/

double piUpdate(PIDController_s *controller, int32_t desired_value,
                int32_t actual_value, int32_t offset);

/**********************************************************************
 * Functions to update the duty cycle of the main and the tail motor
 **********************************************************************/

void piMainMotorUpdate(void);

void piTailMotorUpdate(void);

void ref_piTailMotorUpdate(void);

/**********************************************************************
 * Function initialise the GPIO pin and register the GPIO interrupt
 * for the slide button SW1.
 **********************************************************************/

void initSW1(void);

/**********************************************************************
 * Function responses to the GPIO interrupt when the slide button
 * SW1 has been pushed.
 **********************************************************************/

void SW1IntHandler(void);


#endif
