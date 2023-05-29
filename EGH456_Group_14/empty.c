/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */
#include <stdbool.h>
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/m3/Timer.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
// #include <ti/drivers/EMAC.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/USBMSCHFatFs.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

/* DriverLib Header files */
#include <driverlib/GPIO.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>

/* inc Headers */
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_timer.h>

/* Board Header file */
#include "Board.h"

/* Sensor Header files */
#include "./drivers/OPT3001.h"
#include "./drivers/BMI160.h"

/* Motor Header Files */
#include "./drivers/motorLib/motorlib.h"

/* Task Setup */
#define TASKSTACKSIZE   512

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

/* I2C Setup */
I2C_Handle i2c;
I2C_Params i2cParams;


/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
Void heartBeatFxn(UArg arg0, UArg arg1)
{
    // init sensors
    initBMI160(i2c);
    initOPT3001(i2c);

    uint8_t convertedLux = 0;
    uint8_t *acceleration;
    while (1) {
//        Task_sleep((unsigned int)arg0);
//        GPIO_toggle(Board_LED0);
        convertedLux = readLuxOPT3001(i2c);
        acceleration = readBMI160(i2c);

        System_printf("Lux: %d --- Acceleration(x,y,z): %d, %d, %d\n", convertedLux, acceleration[0], acceleration[1], acceleration[2]);
        System_flush();

    }
}
//*****************************************************************************
//
// Motor Control
//
//*****************************************************************************
#define MOTOR_ACCELERATION_RPMs 750 //Note that this is measured in RPM per Second
#define MOTOR_ESTOP_RPMs 1000 //Note that this is measured in RPM per Second
#define MOTOR_CURR_MAX //we need to decide what this is
#define MOTOR_TEMP_MAX //we need to decide what this is
#define MOTOR_MAX_DUTY 100



#define MOTOR_HALL_A_PIN GPIO_PIN_3
#define MOTOR_HALL_A_PORT GPIO_PORTM_BASE
#define MOTOR_HALL_A_INT GPIO_INT_PIN_3
#define MOTOR_HALL_A_ISR INT_GPIOM_TM4C129

#define MOTOR_HALL_B_PIN GPIO_PIN_2
#define MOTOR_HALL_B_PORT GPIO_PORTH_BASE
#define MOTOR_HALL_B_INT GPIO_INT_PIN_2
#define MOTOR_HALL_B_ISR INT_GPIOH_TM4C129

#define MOTOR_HALL_C_PIN GPIO_PIN_2
#define MOTOR_HALL_C_PORT GPIO_PORTN_BASE
#define MOTOR_HALL_C_INT GPIO_INT_PIN_2
#define MOTOR_HALL_C_ISR INT_GPION_TM4C129

//#define MOTOR_RPM_TIMER_PERIPH SYSCTL_PERIPH_TIMER3
//#define MOTOR_RPM_TIMER_BASE TIMER3_BASE
//#define MOTOR_RPM_TIMER TIMER_A
//#define MOTOR_RPM_TIMER_INT INT_TIMER3A
//#define MOTOR_RPM_TIMER_TIMEOUT TIMER_TIMA_TIMEOUT

/* Motor Funcs */
void motor_init(void);
void motor_driver(UArg arg0);
void motor_initISR();
void motor_start();
void motor_initHall(void);
void motor_initRPM();
void motor_controller();
void motor_initRPM();


/* Motor Globals */
bool motor_hallStates[3];
int motor_edgeCount = 0;
Hwi_Handle motor_Hwi_A, motor_Hwi_B, motor_Hwi_C;
Timer_Handle motor_rpm_Timer;

void motor_initHall(void)
{

    GPIOPinTypeGPIOInput(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN);
    GPIOIntTypeSet(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN,GPIO_HIGH_LEVEL);
    GPIOIntEnable(MOTOR_HALL_A_PORT,MOTOR_HALL_A_INT);

    GPIOPinTypeGPIOInput(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN);
    GPIOIntTypeSet(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN,GPIO_HIGH_LEVEL);
    GPIOIntEnable(MOTOR_HALL_B_PORT,MOTOR_HALL_B_INT);

    GPIOPinTypeGPIOInput(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN);
    GPIOIntTypeSet(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN,GPIO_HIGH_LEVEL);
    GPIOIntEnable(MOTOR_HALL_C_PORT,MOTOR_HALL_C_INT);

}

void motor_start(){
    enableMotor();
    setDuty(50);

//    motor_hallStates[0] = GPIOPinRead(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN);
//    motor_hallStates[1] = GPIOPinRead(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN);
//    motor_hallStates[2] = GPIOPinRead(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN);
//    updateMotor(motor_hallStates[0],motor_hallStates[1],motor_hallStates[2]);
}

void motor_clearISR(){
    Hwi_clearInterrupt(MOTOR_HALL_A_ISR);
    Hwi_clearInterrupt(MOTOR_HALL_B_ISR);
    Hwi_clearInterrupt(MOTOR_HALL_C_ISR);
}

void motor_initISR(){
    /* Create Motor Driving HWI's */
    Error_Block motor_HwiError;
    Hwi_Params motor_HwiParams;

    Hwi_Params_init(&motor_HwiParams);
    motor_HwiParams.maskSetting = Hwi_MaskingOption_SELF;

    motor_Hwi_A = Hwi_create(MOTOR_HALL_A_ISR,(Hwi_FuncPtr)motor_driver,&motor_HwiParams,&motor_HwiError);
    errorCheck(&motor_HwiError);
    motor_Hwi_B = Hwi_create(MOTOR_HALL_B_ISR,(Hwi_FuncPtr)motor_driver,&motor_HwiParams,&motor_HwiError);
    errorCheck(&motor_HwiError);
    motor_Hwi_C = Hwi_create(MOTOR_HALL_C_ISR,(Hwi_FuncPtr)motor_driver,&motor_HwiParams,&motor_HwiError);
    errorCheck(&motor_HwiError);

}

void motor_initRPM(){
#define MOTOR_RPM_TIMER_PERIPH SYSCTL_PERIPH_TIMER3
#define MOTOR_RPM_TIMER_BASE TIMER3_BASE
#define MOTOR_RPM_TIMER TIMER_A
#define MOTOR_RPM_TIMER_INT INT_TIMER3A
#define MOTOR_RPM_TIMER_TIMEOUT TIMER_TIMA_TIMEOUT

    SysCtlPeripheralEnable(MOTOR_RPM_TIMER_PERIPH);

    Error_Block rpm_TimerError;
    Timer_Params rpm_TimerParams;

    Timer_Params_init(&rpm_TimerParams);
    rpm_TimerParams.period = 10000; //0.01s

    motor_rpm_Timer = Timer_create(1,(Timer_FuncPtr)motor_controller,&rpm_TimerParams, &rpm_TimerError);
    errorCheck(&rpm_TimerError);
}

void motor_controller(){
    int j = 0;
}

void motor_driver(UArg arg0)
{
    motor_clearISR();

    motor_hallStates[0] = GPIOPinRead(MOTOR_HALL_A_PORT, MOTOR_HALL_A_PIN);
    motor_hallStates[1] = GPIOPinRead(MOTOR_HALL_B_PORT, MOTOR_HALL_B_PIN);
    motor_hallStates[2] = GPIOPinRead(MOTOR_HALL_C_PORT, MOTOR_HALL_C_PIN);
    updateMotor(motor_hallStates[0],motor_hallStates[1],motor_hallStates[2]);

    motor_edgeCount++;
}

void motor_init(void){
    motor_initHall();
//    motor_initRPM();
    motor_initISR();
//    motor_initRPM();
    Error_Block motorError;
    bool initSuccess = initMotorLib(MOTOR_MAX_DUTY, &motorError);
    errorCheck(&motorError);


    motor_start();

}

void errorCheck(Error_Block *eb){
    if (Error_check(&eb)) {
        // handle the error
        GPIO_toggle(Board_LED1);
    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;
    /* Call board init functions */
    Board_initGeneral();
    // Board_initEMAC();
    Board_initGPIO();
    Board_initI2C();
    // Board_initSDSPI();
    // Board_initSPI();
    // Board_initUART();
    // Board_initUSB(Board_USBDEVICE);
    // Board_initUSBMSCHFatFs();
    // Board_initWatchdog();
    // Board_initWiFi();
    motor_init();

    /* create and open i2c port*/
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_OPT3001, &i2cParams);
    if (i2c == NULL) {
        System_abort("Error initializing I2C\n");
    } else {
        System_printf("I2C Initialized!\n");
        System_flush();
    }

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
//    Task_construct(&task0Struct, (Task_FuncPtr)heartBeatFxn, &taskParams, NULL);





    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
