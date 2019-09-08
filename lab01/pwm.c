#include <stdint.h>
#include <stdbool.h>
// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "system_TM4C1294.h" 
#define PIN_ON 1
#define PIN_OFF 0
#define CLOCK 24000000
#define INSTRUCTIONS_PER_MICROSECONDS CLOCK/3
#define LOOP_DELAY_IN_MICROSECONDS 1000
#define TIME_SPENT_TO_PRINT 1000000/LOOP_DELAY_IN_MICROSECONDS

//void UART0init (void);
//void UART0_Tx (uint8_t data);
//uint8_t PortE_Input();

int timeSpentOnHigh;
int timeSpentOnLow;
int timeSpentUntilLastPrint;

extern void UARTStdioIntHandler(void);

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
} // UARTInit

void sendUart(int period, float frequency, float dutyCycle)
{
  int integer_frequency, integer_duty_cycle;
  int decimal_frequency, decimal_duty_cycle;
  int centesimal_frequency;
  
  integer_frequency = (int)frequency;
  decimal_frequency = (int)(frequency - (float)integer_frequency)*10;
  centesimal_frequency = (int)(frequency - (float)integer_frequency - (float)decimal_frequency/10)*100;
  integer_duty_cycle = (int)dutyCycle;
  decimal_duty_cycle = (int)(dutyCycle - (float)integer_duty_cycle)*10;
  
  UARTprintf("Periodo %d us\nFrequencia %d.%d%d Hz\nDuty Cycle %d.%d\n", period, integer_frequency, decimal_frequency, centesimal_frequency, integer_duty_cycle, decimal_duty_cycle);
} //sendUart

void computeParameters()
{
  int periodus      = timeSpentOnLow + timeSpentOnHigh;
  float frequency   = 1.0/(float)(periodus/1000000.0);
  float duty_cycle  = (float)timeSpentOnHigh/periodus;
  
  if(timeSpentUntilLastPrint >= TIME_SPENT_TO_PRINT)
  {
    sendUart(periodus, frequency, duty_cycle*100.0);
    timeSpentUntilLastPrint = 0;
  }
} //computeParameters

void computePwm(int numberOfLoops, int previousState, int currentState)
{
  if(previousState == PIN_ON)
  {
    timeSpentOnHigh = LOOP_DELAY_IN_MICROSECONDS * numberOfLoops;
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4); // Acende LED D3
  }
  else if(previousState == PIN_OFF)
  {
    timeSpentOnLow = LOOP_DELAY_IN_MICROSECONDS * numberOfLoops;
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0); // Apaga LED D3
  } 
  
  if(timeSpentOnLow != 0 && timeSpentOnHigh != 0)
  {
    computeParameters();
  } 
} //computePwm

void readPwm()
{
  int previous_state  = 0;
  int current_state   = 0;
  int number_of_loops = 0;
  
  while(1)
  {
    number_of_loops++;
    timeSpentUntilLastPrint++;
    current_state = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0);
    if(current_state != previous_state)
    {
      computePwm(number_of_loops, previous_state, current_state);
      previous_state = current_state;
      number_of_loops = 0;
    }
    SysCtlDelay(INSTRUCTIONS_PER_MICROSECONDS/LOOP_DELAY_IN_MICROSECONDS); //1000u*24000000/3 delay de 100us
  }
} //readPwm

void UART0_Handler(void){
  UARTStdioIntHandler();
} // UART0_Handler

void main(void){
  UARTInit();
  //UARTprintf("Sistemas Embarcados - 2019/2\n");
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Habilita GPIO F (LED D3 = PF4, LED D4 = PF0)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); // Aguarda final da habilitação
    
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4); // LEDs D3 e D4 como saída
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0); // LEDs D3 e D4 apagados
  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Habilita GPIO J (push-button SW1 = PJ0, push-button SW2 = PJ1)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)); // Aguarda final da habilitação
  
  GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO PORTE# como entrada.
  GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  
  timeSpentOnHigh  = 0;
  timeSpentOnLow   = 0; 
  timeSpentUntilLastPrint = 0;
  
  readPwm();

} // main