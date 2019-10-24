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

bool high = true;
bool firstTime = true;
bool timeout = false;
int timeSpentOnHigh = 0;
int timeSpentOnLow = 0;
int timeSpentUntilLastPrint = 0;
int startTime = 0;
int endTime = 0;

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

void resetSysTick(void)
{
    uint32_t your_32_bit_value = 0x00000000;
    uint32_t volatile * const mem_map_register = (uint32_t volatile *) 0xE000E018;
    *mem_map_register = your_32_bit_value;
}

void UART0_Handler(void){
  UARTStdioIntHandler();
}

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
}

void computeParametersAndSendToUART()
{
  int periodus      = timeSpentOnLow + timeSpentOnHigh;
  float frequency   = 1.0/(float)(periodus/1000000.0);
  float duty_cycle  = (float)timeSpentOnHigh/periodus;
  
  sendUart(periodus, frequency, duty_cycle*100.0);
}

void reset_timer() 
{
  TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT);
  TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
  TimerLoadSet(TIMER0_BASE, TIMER_A, 0xFFFF);
  TimerLoadSet(TIMER0_BASE, TIMER_B, 0xFFFF);
}

void handler_timers(void)
{
  if(high)
  {
    timeSpentOnLow = (65535 - TimerValueGet(TIMER0_BASE, TIMER_A))/CLOCK_US;
    high = false;
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
    reset_timer();
  }
  else
  {
    timeSpentOnHigh = (65535 - TimerValueGet(TIMER0_BASE, TIMER_A))/CLOCK_US;
    high = true;
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    reset_timer();
  }
    
  if(!timeout) 
  {
    if(timeSpentUntilLastPrint >= TIME_SPENT_TO_PRINT)
    {
      computeParametersAndSendToUART();
      timeSpentUntilLastPrint = 0;
    }
  }
  
  timeout = false;
}

void handler_timeout(void){
  if(timeSpentUntilLastPrint >= TIME_SPENT_TO_PRINT)
  {
    if(!high) {
      UARTprintf("Periodo infinito us\nFrequencia 0 Hz\nDuty Cycle 100\n");
    } else {
      UARTprintf("Periodo infinito us\nFrequencia 0 Hz\nDuty Cycle 0\n");
    }
    timeSpentUntilLastPrint = 0;
  }
  reset_timer();
  timeout = true;
}

void gpio_initialization()
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)); 
  
  GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
  
  GPIOPinConfigure(GPIO_PD0_T0CCP0);
}

void timer_initialization(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
  TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME | TIMER_CFG_B_PERIODIC));
  TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  
  TimerLoadSet(TIMER0_BASE, TIMER_B, 0xFFFF);
  TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);
  TimerIntRegister(TIMER0_BASE, TIMER_A, handler_timers);
  TimerIntRegister(TIMER0_BASE, TIMER_B, handler_timeout);
  TimerEnable(TIMER0_BASE, TIMER_A);
  TimerEnable(TIMER0_BASE, TIMER_B);
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
}
