#include <stdint.h>
#include <stdbool.h>
// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"

uint8_t LED_D1 = 0;
bool high = true;

void timeOutHandler(void)
{
  
}

void handlerEntrancePwm(void)
{
  if(high){
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
    high = false;
    int timeOnHigh = TimerValueGet(TIMER0_BASE, TIMER_A);
    TimerDisable(TIMER0_BASE, TIMER_A);
    //Verificar se disable ou outra função dá clear no TIMER_A
    TimerLoadSet(TIMER0_BASE, TIMER_A, 3000);
    TimerEnable(TIMER0_BASE, TIMER_A);
  }
  else
  {
     GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);
     high = true;
     int timeOnHigh = TimerValueGet(TIMER0_BASE, TIMER_A);
     TimerDisable(TIMER0_BASE, TIMER_A);
     //Verificar se disable ou outra função dá clear no TIMER_A
     TimerLoadSet(TIMER0_BASE, TIMER_A, 3000);
     TimerEnable(TIMER0_BASE, TIMER_A);
  }
  
}

void gpio_initialization()
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Habilita GPIO E
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)); // Aguarda final da habilitação
  GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0); // Pino 0 como entrada
  
  GPIOIntRegister(GPIO_PORTE_BASE, handlerEntrancePwm);
  
  GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);
  //GPIO_FALLING_EDGE; GPIO_BOTH_EDGES, GPIO_LOW_LEVEL, GPIO_HIGH_LEVEL
  
  GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_0);
}

void main(void){
  //// Enable the Timer0 peripheral//
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  //// Wait for the Timer0 module to be ready.//
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){}
  
  /* FUNCTIONS TO BE USED LATER
  TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT_UP);
  
  TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  
  TimerEnable(TIMER0_BASE, TIMER_A);
  
  TimerValueGet(TIMER0_BASE, TIMER_A);
  
  TimerIntRegister(TIMER0_BASE, TIMER_A, timeOutHandler);
  
  TimerDisable(uint32_t ui32Base,uint32_t ui32Timer);
  */
  //UARTInit();
  gpio_initialization();
} // main
