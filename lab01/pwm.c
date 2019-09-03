#include <stdint.h>
#include <stdbool.h>
// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#define GPIO_READ_INSTRUCTIONS_COST 500

void UART0init (void);
void UART0_Tx (uint8_t data);
void UART0char_Tx (uint8_t data);

int time_state_0  = 0;
int time_state_1  = 0; 

void computeParameters(int time_state_0, int time_state_1)
{
  int period      = time_state_0 + time_state_1;
  int frequency   = 1/period;
  int duty_cycle  = time_state_0/(time_state_0 + time_state_1);
  sendUart(period, frequency, duty_cycle);
} //computeParameters

void sendUart(int period, int frequency, int dutyCycle)
{
  UART0char_Tx (period);
  UART0_Tx ('\n');
  UART0char_Tx (frequency);
  UART0_Tx ('\n');
  UART0char_Tx (dutyCycle);
  UART0_Tx ('\n');
} //sendUart

void computePwm(int numberOfLoops, int *previousState, int currentState)
{
  if(time_state_0 == 0)
  {
    time_state_0 = GPIO_READ_INSTRUCTIONS_COST * numberOfLoops;
  }
  else if(time_state_1 == 0)
  {
    time_state_1 = GPIO_READ_INSTRUCTIONS_COST * numberOfLoops; 
  }
  else if(time_state_0 != 0 && time_state_0 != 0)
  {
    computeParameters(time_state_0, time_state_1);
    time_state_0 = 0;
    time_state_1 = 0;
  } 
  (*previousState) = currentState;
} //computePwm

void readPwm()
{
  int previous_state  = PIN_OFF;
  int current_state   = PIN_OFF;
  int number_of_loops = 0;

  UART0init();
  
  while(1)
  {
    number_of_loops++;
    currentState = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0);
    if(currentState != previousState)
    {
      computePwm(number_of_loops, &previous_state, current_state);
      number_of_loops = 0;
    }
  }

} //readPwm


void main(void){

  SysTickPeriodSet(12000000); // f = 1Hz para clock = 24MHz

  GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO PORTJ# como entrada.
  GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

  readPwm();

} // main