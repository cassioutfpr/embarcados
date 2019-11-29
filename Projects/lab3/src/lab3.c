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
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#define ENCODER_TICKS 24000000
#define ENCODER_PERIOD_IN_MS 200
#define DIRECTION_SAMPLES_SIZE 5
#define SPEED_SAMPLES_SIZE 5

#define integrateMax 3300
#define integrateMin 2700

#define P_GAIN 2
#define I_GAIN 0.01
#define D_GAIN 2000

osThreadId_t tid_control;               
osThreadId_t tid_userReset;  
osThreadId_t tid_readSampleDirection;
osThreadId_t tid_calculateSpeed;
osThreadId_t tid_readSampleSpeed;
osThreadId_t tid_calculateDirection;
osThreadId_t tid_changeDirectionJustForFun;
osMutexId_t phases_mut_id;

bool userTyping = true;

int direction_samples[DIRECTION_SAMPLES_SIZE];
bool direction_samples_completed = false;
int actual_direction;

int last_speed_read;
int speed_samples[SPEED_SAMPLES_SIZE];
bool speed_samples_completed = false;
int actual_speed_rpm;

int setpoint;
int set_direction;
int rpm_control_output;

int integrateSum;

int last_position = 0;

extern void UARTStdioIntHandler(void);

void userCommunication() {
  userTyping = true;
  
  UARTprintf("Bem Vindo!\nDigite o RPM no formato XXXX. \n O valor do RPM deve ser de 2000 ate 5000. \nPara alterar o RPM após a inicialização aperte R.\n");
  UARTprintf("Digite o RPM: ");
  
  while(!UARTCharsAvail(UART0_BASE))
  {
  }
  
  int set_rpm = 0;
  
  char pressed = UARTCharGet(UART0_BASE);
  UARTprintf("%c", pressed);
  set_rpm += (pressed-48)*1000;
  
  pressed = UARTCharGet(UART0_BASE);
  UARTprintf("%c", pressed);
  set_rpm += (pressed-48)*100;
  
  pressed = UARTCharGet(UART0_BASE);
  UARTprintf("%c", pressed);
  set_rpm += (pressed-48)*10;
  
  pressed = UARTCharGet(UART0_BASE);
  UARTprintf("%c\n", pressed);
  set_rpm += (pressed-48);
  
  UARTprintf("Digite o sentido (A ou H): ");
  
  pressed = UARTCharGet(UART0_BASE);
  UARTprintf("%c\n", pressed);
  
  if(pressed == 'a')
    set_direction = 1;
  else
    set_direction = -1;
  
  setpoint = set_rpm;
  
  userTyping = false;
}


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
    
    if(set_direction != actual_direction) {
      switch(set_direction) {
        case -1:
          GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);
          GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
          break;
        case 1:
          GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
          GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
          break;  
      }
    }
  }
}

/*----------------------------------------------------------------------------
 *      Thread 4 'readSampleDirection'
 *---------------------------------------------------------------------------*/
void readSampleDirection (void *argument) {
  while(1) {
    for(int i = 0 ; i < DIRECTION_SAMPLES_SIZE ; i++) {
      osDelay(20);
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
    osDelay(1);

    int pulses_accumulator = 0;
    float average_pulses_per_second;
      
    for(int i = 0 ; i < SPEED_SAMPLES_SIZE ; i++) {
      pulses_accumulator += speed_samples[i];
    }
    
    average_pulses_per_second = ((((float)pulses_accumulator)/SPEED_SAMPLES_SIZE)/4)*5;
    actual_speed_rpm = (average_pulses_per_second/18)*60;
    volatile int a = actual_speed_rpm;
    osThreadFlagsSet(tid_control, 0x0001);
    if(!userTyping) {
      UARTprintf("%d\n", actual_speed_rpm);
    }
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
 *      Thread 1 'control'
 *---------------------------------------------------------------------------*/
void control(void *argument) {
  while(1) {
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
    volatile float duty_cycle = 0.0;
    
    // P
    int error = (setpoint - actual_speed_rpm) * P_GAIN;
    int p = setpoint + error;
    
    // I
    int i;
    integrateSum += error;

    if (integrateSum > integrateMax) {
            integrateSum = integrateMax;
    }
    else if (integrateSum < integrateMin) {
            integrateSum = integrateMin;
    }

    i = I_GAIN * (float)integrateSum;
    
    // D
    int d = D_GAIN * (last_position - actual_speed_rpm);
    last_position = actual_speed_rpm;
    
    // PID
    rpm_control_output = p + i + d;
    
    if(rpm_control_output > 5000)
      rpm_control_output = 5000;
    if(rpm_control_output < 0)
      rpm_control_output = setpoint;
    
    duty_cycle = (60000.0*rpm_control_output)/5000.0;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (int)duty_cycle);
  }
}

/*----------------------------------------------------------------------------
 *      Thread 0 'userReset'
 *---------------------------------------------------------------------------*/
void userReset (void *argument) {
  while(1) {
    osDelay(500);
    char user = UARTCharGet(UART0_BASE);
    
    if(user == 'r')
      userCommunication();
  }
}

void UARTInit(void){
  // Enable the GPIO Peripheral used by the UART.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

  // Enable UART0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

  // Configure GPIO Pins for UART mode.
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  // Initialize the UART for console I/O.
  UARTStdioConfig(0, 100000, SystemCoreClock);
}

void UART0_Handler(void){
  UARTStdioIntHandler();
}

void QEI0Handler(void) {
  QEIIntClear(QEI0_BASE, QEI_INTTIMER);
  last_speed_read = QEIVelocityGet(QEI0_BASE);
  osThreadFlagsSet(tid_readSampleSpeed, 0x0001);   
}

void changeDirectionJustForFun(void *argument) {
  while(1) {
    osDelay(2000);
    set_direction *= -1;
  }
}

/*----------------------------------------------------------------------------
 *      Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
void appMain (void *argument) {
  tid_control = osThreadNew(control, NULL, NULL);
  tid_userReset = osThreadNew(userReset, NULL, NULL);
  tid_readSampleSpeed = osThreadNew(readSampleSpeed, NULL, NULL);
  tid_calculateSpeed = osThreadNew(calculateSpeed, NULL, NULL);
  tid_readSampleDirection = osThreadNew(readSampleDirection, NULL, NULL);
  tid_calculateDirection = osThreadNew(calculateDirection, NULL, NULL);
//  tid_changeDirectionJustForFun = osThreadNew(changeDirectionJustForFun, NULL, NULL);
  
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
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) {}
  
  PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DBG_RUN | PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 60000);
}

void enableMotor() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}
  
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
  GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
  
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);
  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
  GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
  
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 10);
  PWMGenEnable(PWM0_BASE, PWM_GEN_1);
  PWMOutputState(PWM0_BASE, (PWM_OUT_2_BIT), true);
}

int main (void) {
  UARTInit();
  userCommunication();
  enableQEI();
  enablePwm();
  enableMotor();
  osKernelInitialize();               
  osThreadNew(appMain, NULL, NULL);    
  if (osKernelGetState() == osKernelReady) {
    osKernelStart();               
  }
}