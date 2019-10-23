// S12 - G03 - Lab2
// Cássio Morales - 1612239
// Luiz Agner - 1612280

#include <stdint.h>
#include <stdbool.h>
// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "system_TM4C1294.h" 
#define TIME_OUT 14000000
#define CLOCK 120000000
#define CLOCK_US 120
#define INSTRUCTIONS_PER_MICROSECONDS CLOCK/3000000
#define LOOP_DELAY_IN_MICROSECONDS 1000
#define TIME_SPENT_TO_PRINT 1000000/LOOP_DELAY_IN_MICROSECONDS

uint8_t LED_D1 = 0;

enum states{
  HIGH = 0,
  LOW  = 1
};

bool high = false;
int timeSpentOnHigh = 0;
int timeSpentOnLow = 0;
int timeSpentUntilLastPrint = 0;

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

void resetSysTick(void)
{
    uint32_t your_32_bit_value = 0x00000000;
    uint32_t volatile * const mem_map_register = (uint32_t volatile *) 0xE000E018;
    *mem_map_register = your_32_bit_value;
}

void UART0_Handler(void){
  UARTStdioIntHandler();
} // UART0_Handler

void timeOutHandler(void)
{
  UARTprintf("TIMEOUT\n"); 
} // SysTick_Handler

void sendUart(int period, float frequency, float dutyCycle)
{
  volatile int integer_frequency, integer_duty_cycle;
  volatile int decimal_frequency, decimal_duty_cycle;
  volatile int centesimal_frequency;
  
  integer_frequency = (int)frequency;
  decimal_frequency = (int)((frequency - (float)integer_frequency)*10);
  centesimal_frequency = (int)((frequency - (float)integer_frequency - (float)decimal_frequency/10)*100);
  integer_duty_cycle = (int)dutyCycle;
  decimal_duty_cycle = (int)((dutyCycle - (float)integer_duty_cycle)*10);
  
  UARTprintf("Periodo %d us\nFrequencia %d.%d%d Hz\nDuty Cycle %d.%d\n", period, integer_frequency, decimal_frequency, centesimal_frequency, integer_duty_cycle, decimal_duty_cycle);
} //sendUart

void computeParametersAndSendToUART()
{
  int periodus      = timeSpentOnLow + timeSpentOnHigh;
  float frequency   = 1.0/(float)(periodus/1000000.0);
  float duty_cycle  = (float)timeSpentOnHigh/periodus;
  
  sendUart(periodus, frequency, duty_cycle*100.0);
}

void handlerEntrancePwm(void)
{
   volatile int local;  
   if(high) 
   {
      GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
      high = false;
      timeSpentOnLow = ((TIME_OUT - SysTickValueGet()))/CLOCK_US;
   }
   else
   {
      GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);
      high = true;
      timeSpentOnHigh = ((TIME_OUT - SysTickValueGet()))/CLOCK_US;
   }
   
   local = timeSpentOnLow;
  if(timeSpentUntilLastPrint >= TIME_SPENT_TO_PRINT)
  {
    computeParametersAndSendToUART();
    timeSpentUntilLastPrint = 0;
  }
  GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_0);
  resetSysTick();
}

void handler_timers(void)
{
  
}

void gpio_initialization()
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)); 
  
  GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);//Configures pin(s) for use by the Timer peripheral.
    //#1 parameter: the base address of the GPIO port.
    //#2parameter: the bit-packed representation of the pin(s)
  
  GPIOPinConfigure(GPIO_PD0_T0CCP0); // Configures the alternate function of a GPIO pin.
      
  //GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);
  //GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);
  //GPIOIntRegister(GPIO_PORTD_BASE, handlerEntrancePwm);
  //GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_0);
}

void timer_initialization(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
  TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_CAP_TIME_UP));
  TimerControlEvent(TIMER0_BASE, TIMER_BOTH, TIMER_EVENT_POS_EDGE);
  //TIMER_EVENT_POS_EDGE
  //TIMER_EVENT_NEG_EDGE
  //TIMER_EVENT_BOTH_EDGES
  
  //TimerLoadSet(TIMER0_BASE, TIMER_BOTH, 0xFFFF);
  TimerEnable(TIMER0_BASE, TIMER_BOTH);
  TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);
  TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT | TIMER_CAPB_EVENT);
  TimerIntRegister(TIMER0_BASE, TIMER_BOTH, handler_timers);
}
void main(void)
{
  UARTInit();
  gpio_initialization();
  timer_initialization();  
   
  while(1) {
    timeSpentUntilLastPrint++;
    SysCtlDelay(INSTRUCTIONS_PER_MICROSECONDS*LOOP_DELAY_IN_MICROSECONDS);
  }
} // main
