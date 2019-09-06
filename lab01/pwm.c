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
uint8_t PortE_Input();

int timeSpentOnHigh  = 0;
int timeSpentOnLow   = 0; 

void sendUart(int period, int frequency, int dutyCycle)
{
  UART0_Tx (period);
  UART0_Tx ('\n');
  UART0_Tx (frequency);
  UART0_Tx ('\n');
  UART0_Tx (dutyCycle);
  UART0_Tx ('\n');
} //sendUart

void computeParameters()
{
  int period      = timeSpentOnLow + timeSpentOnHigh;
  int frequency   = 1/period;
  int duty_cycle  = timeSpentOnHigh/period;
  sendUart(period, frequency, duty_cycle);
} //computeParameters


void computePwm(int numberOfLoops, int previousState, int currentState)
{
  if(timeSpentOnHigh == 0 && currentState == PIN_ON)
  {
    timeSpentOnHigh = GPIO_READ_INSTRUCTIONS_COST * numberOfLoops;
  }
  else if(timeSpentOnLow == 0 && currentState == PIN_OFF)
  {
    timeSpentOnLow = GPIO_READ_INSTRUCTIONS_COST * numberOfLoops; 
  } 
  
  if(timeSpentOnLow != 0 && timeSpentOnHigh != 0)
  {
    computeParameters();
  } 
} //computePwm

void readPwm()
{
  int previous_state  = PIN_OFF;
  int current_state   = PIN_OFF;
  int number_of_loops = 0;
  
  while(1)
  {
    number_of_loops++;
    currentState = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0);
    if(currentState != previousState)
    {
      computePwm(number_of_loops, previous_state, current_state);
      previousState = currentState;
      number_of_loops = 0;
    }
  }

} //readPwm


void main(void){

  //SysTickPeriodSet(12000000); // f = 1Hz para clock = 24MHz
  UART0init();

  GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO PORTE# como entrada.
  GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

  readPwm();

} // main