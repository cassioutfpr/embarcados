#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverleds.h" // device drivers
#include "cmsis_os2.h" // CMSIS-RTOS
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "system_TM4C1294.h" 

#define ENCODER_TICKS 24000000
#define ENCODER_PERIOD_IN_MS 200
#define DIRECTION_SAMPLES_SIZE 5
#define SPEED_SAMPLES_SIZE 5

osThreadId_t tid_calculateRPM;          
osThreadId_t tid_control;               
osThreadId_t tid_userInteraction;  
osThreadId_t tid_readSampleDirection;
osThreadId_t tid_calculateSpeed;
osThreadId_t tid_readSampleSpeed;
osThreadId_t tid_calculateDirection;
osMutexId_t phases_mut_id;

int direction_samples[DIRECTION_SAMPLES_SIZE];
bool direction_samples_completed = false;
int actual_direction;

int last_speed_read;
int speed_samples[SPEED_SAMPLES_SIZE];
bool speed_samples_completed = false;
int actual_speed_rpm;

/*----------------------------------------------------------------------------
 *      Thread 5 'calculateDirection'
 *---------------------------------------------------------------------------*/
void calculateDirection (void *argument) {
  osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
  
  while(1) {
    osDelay(100);
   
    int clockwise = 0, counterclockwise = 0;
    
    for(int i = 0 ; i < DIRECTION_SAMPLES_SIZE ; i++) {
      if(direction_samples[i] == 1) clockwise++;
      else counterclockwise++;
      
      if(clockwise > counterclockwise) actual_direction = 1;
      else actual_direction = -1;
    }
  }
}

/*----------------------------------------------------------------------------
 *      Thread 4 'readSampleDirection'
 *---------------------------------------------------------------------------*/
void readSampleDirection (void *argument) {
  while(1) {
    for(int i = 0 ; i < DIRECTION_SAMPLES_SIZE ; i++) {
      osDelay(100);
      direction_samples[i] = QEIDirectionGet(QEI0_BASE);
    }
    if(!direction_samples_completed) {
      direction_samples_completed = !direction_samples_completed;
      osThreadFlagsSet(tid_calculateDirection, 0x0001);
    }
  }
}

/*----------------------------------------------------------------------------
 *      Thread 3 'calculateSpeed'
 *---------------------------------------------------------------------------*/
void calculateSpeed (void *argument) {
  osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
  
  while(1) {
    osDelay(100);

    int pulses_accumulator = 0;
    float average_pulses_per_second;
      
    for(int i = 0 ; i < SPEED_SAMPLES_SIZE ; i++) {
      pulses_accumulator += speed_samples[i];
    }
    
    average_pulses_per_second = ((((float)pulses_accumulator)/SPEED_SAMPLES_SIZE)/4)*5;
    actual_speed_rpm = (average_pulses_per_second/18)*60;
  }
}

/*----------------------------------------------------------------------------
 *      Thread 2 'readSampleSpeed'
 *---------------------------------------------------------------------------*/
void readSampleSpeed (void *argument) {
  while(1) {
    for(int i = 0 ; i < SPEED_SAMPLES_SIZE ; i++) {
      osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
      speed_samples[i] = last_speed_read;
    }
    if(!speed_samples_completed) {
      speed_samples_completed = !speed_samples_completed;
      osThreadFlagsSet(tid_calculateSpeed, 0x0001);
    }
  }
}

/*----------------------------------------------------------------------------
 *      Thread 1 'calculateRPM'
 *---------------------------------------------------------------------------*/
void calculateRPM(void *argument) {
  for (;;) {
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);    /* wait for an event flag 0x0001 */
  }
}

void QEI0Handler(void) {
  QEIIntClear(QEI0_BASE, QEI_INTTIMER);
  last_speed_read = QEIVelocityGet(QEI0_BASE);
  osThreadFlagsSet(tid_readSampleSpeed, 0x0001);   
}

/*----------------------------------------------------------------------------
 *      Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
void appMain (void *argument) {
//  tid_calculateRPM = osThreadNew(calculateRPM, NULL, NULL);
//  tid_control = osThreadNew(control, NULL, NULL);
//  tid_userInteraction = osThreadNew(userInteraction, NULL, NULL);
  tid_readSampleSpeed = osThreadNew(readSampleSpeed, NULL, NULL);
  tid_calculateSpeed = osThreadNew(calculateSpeed, NULL, NULL);
  tid_readSampleDirection = osThreadNew(readSampleDirection, NULL, NULL);
  tid_calculateDirection = osThreadNew(calculateDirection, NULL, NULL);
  
  osThreadFlagsSet(tid_userInteraction, 0x0001);       

  osDelay(osWaitForever);
  while(1);
}

void enableQEI() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)){}
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0)){}
  QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 1);
  GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_1 | GPIO_PIN_2);
  GPIOPinConfigure(GPIO_PL1_PHA0);
  GPIOPinConfigure(GPIO_PL2_PHB0);
  
  QEIPositionSet(QEI0_BASE, 0);
  QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, ENCODER_TICKS);
  QEIEnable(QEI0_BASE);
  QEIVelocityEnable(QEI0_BASE);
  
  QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
  QEIIntRegister(QEI0_BASE, &QEI0Handler);
}

void enablePwm() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinConfigure(GPIO_PF2_M0PWM2);
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
  PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) {}
  
  PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DBG_RUN | PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 60000);
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 30000);

  PWMGenEnable(PWM0_BASE, PWM_GEN_1);

  PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT), true);
}

int main (void) {
  enableQEI();
  enablePwm();
  osKernelInitialize();                
  osThreadNew(appMain, NULL, NULL);    
  if (osKernelGetState() == osKernelReady) {
    osKernelStart();               
  }
}

