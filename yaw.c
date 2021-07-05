//*****************************************************************************
//
// Functions for the calculation of the helicopter yaw angle
// fripmgroup2
// Authors:  Lei Li, Junwei Liang, Michael Redepenning
//
//*****************************************************************************

#include "yaw.h"

//*****************************************************************************
// Global variables
//*****************************************************************************

/* The rotation direction of the helicopter.
 * This can be ANTICLOCKWISE, STILL, or CLOCKWISE
 */
static int32_t direction = STILL;

/* The left and right button states.
 * This can be RELEASED, PUSHED, or NO_CHANGE.
 */
static uint8_t leftButState = NO_CHANGE;
static uint8_t rightButState = NO_CHANGE;

/* The pin outputs of channel A and channel B
 * which will be used for quadratic decoding
 */
static int32_t initialOrientationA = 0;
static int32_t initialOrientationB = 0;
static int32_t currentStateA = 0;
static int32_t currentStateB = 0;

/* The disk in the testing station has 112 slots,
 * and the value of the yaw will be 448 for a whole revolution.
 * The scale variable will be used for converting the value of yaw to degree.
 */
static float scale = 360.0 / 448.0;

/* a lookup table for detecting the rotation direction */
static int32_t dir[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

/* A counter for channel interrupt */
int32_t yaw = 0;


// Function to initialize the reference of orientation direction.
void orientationInitial(void) {
    initialOrientationA = (GPIOPinRead(GPIO_PORTB_BASE, CHANNEL_A) & FIRST_BIT);
    initialOrientationB = ((GPIOPinRead(GPIO_PORTB_BASE, CHANNEL_B) & SECOND_BIT) >> 1);
    yaw = 0;
}


// Function to read the input of PB0 and PB1 and find the rotation direction of the helicopter.
void checkDirection(void) {
    // Read the output of the GPIO pins
    currentStateA = (GPIOPinRead(GPIO_PORTB_BASE, CHANNEL_A) & FIRST_BIT);
    currentStateB = ((GPIOPinRead(GPIO_PORTB_BASE, CHANNEL_B) & SECOND_BIT) >> 1);
    // Convert the results into a 4 bits binary number which will be used as the index of the lookup table
    int32_t index = (initialOrientationA << 3) + (initialOrientationB << 2) + (currentStateA << 1) + currentStateB;
    // Find direction in the lookup table
    direction = dir[index];
    // Update the previous GPIO outputs value to the current value
    initialOrientationA = currentStateA;
    initialOrientationB = currentStateB;
}


// Function to calculate the helicopter yaw angle according to the rotation direction.
void calculateYaw(void) {
    switch (direction) {
        case ANTICLOCKWISE:
            yaw--;
            break;
        case CLOCKWISE:
            yaw++;
            break;
        case STILL:
            break;
    }
}


// Function to convert the value of yaw into degree.
void calculateDegree(void) {
    degree = (int32_t)(yaw * scale) % MAX_DEGREE;
    desired_degree = (int32_t)(desired_yaw * scale) % MAX_DEGREE;
}


// The GPIO interrupt for yaw angle.
void initYawGPIO(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Set data direction register as input
    GPIODirModeSet(GPIO_PORTB_BASE, CHANNEL_A, GPIO_DIR_MODE_IN);
    GPIODirModeSet(GPIO_PORTB_BASE, CHANNEL_B, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTB_BASE, CHANNEL_A, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTB_BASE, CHANNEL_B, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    // Detect both rising and falling edges for channel A and channel B
    GPIOIntTypeSet(GPIO_PORTB_BASE, CHANNEL_A | CHANNEL_B, GPIO_BOTH_EDGES);
    GPIOIntRegister(GPIO_PORTB_BASE, GPIOYawIntHandler);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
}


// Function responses to the GPIO interrupt for yaw angle.
void GPIOYawIntHandler(void) {
    // set up the flag for display the value of the degree
    yawFlag = 1;
    // update the value of the yaw variable
    // need to call these two functions inside the interrupt handler so we won't lost any sample
    checkDirection();
    calculateYaw();
    // Increment the value of the desired yaw during the procedure of finding reference orientation
    if (find_yaw_ref_enable && ((desired_yaw - yaw) < 30)) {
        desired_yaw++;
    }
    // Clear the interrupt
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
}


// The GPIO interrupt for the reference orientation.
void getYawRef(void) {
    SysCtlPeripheralEnable(YAW_REF_PERIPH);
    GPIOPinTypeGPIOInput(YAW_REF_PORT_BASE, YAW_REF_PIN);
    GPIOPadConfigSet(YAW_REF_PORT_BASE, YAW_REF_PIN, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(YAW_REF_PORT_BASE, YAW_REF_PIN, GPIO_FALLING_EDGE);
    GPIOIntRegister(YAW_REF_PORT_BASE, yawRefIntHandler);
    GPIOIntEnable(YAW_REF_PORT_BASE, GPIO_INT_PIN_4);
}


// Function responses to the GPIO interrupt for the reference orientation.
void yawRefIntHandler(void) {
    // Clear up the value of the desired yaw for Pi-controller.
    desired_yaw = 0;
    // Initialize the reference of orientation direction.
    orientationInitial();
    // Disable the GPIO interrupt
    // so this function won't be invoked when the helicopter rotates to the reference orientation next time.
    GPIOIntDisable(YAW_REF_PORT_BASE, GPIO_INT_PIN_4);
    // Clear up the interrupt
    GPIOIntClear(YAW_REF_PORT_BASE, GPIO_INT_PIN_4);
    // Change anther Pi-controller for tail motor.
    InitPITailController();
    // Clear up the flag for the whole procedure of finding reference orientation.
    find_yaw_ref_enable = 0;

}


// Update the value of desired yaw when the LEFT button or the RIGHT button has been pushed.
void updateDesiredYaw(void) {

    // check state of LEFT button
    leftButState = checkButton(LEFT);
    switch (leftButState) {
        case PUSHED:
            desired_yaw -= 7;
            break;
        case RELEASED:
            break;
    }
    // check state of RIGHT button
    rightButState = checkButton(RIGHT);
    switch (rightButState) {
        case PUSHED:
            desired_yaw += 7;
            break;
        case RELEASED:
            break;
    }
}
