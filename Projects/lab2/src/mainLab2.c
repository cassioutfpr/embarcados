// S12 - G03 - Lab1
// Cássio Morales - 1612239
// Luiz Agner - 1612280
#include <stdint.h>
#include <stdbool.h>
// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#define TIME_OUT 1000000

uint8_t LED_D1 = 0;

enum states{
  HIGH = 0,
  LOW  = 1
};

bool high = false;
bool high2 = false;
volatile int time_on_high = 0;
volatile int time_on_low = 0;

void handler2(void){
  if(high2)
  {
     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4); // Apaga LED D4
     time_on_high = 16000000;
     high2 = false;
  }
  else{
     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0); // Apaga LED D4
    time_on_low = 16000000;
    high2 = true;
  }
  
  SysTickIntDisable();
  SysTickDisable();
  SysTickPeriodSet(12000000);
  SysTickIntEnable();
  SysTickEnable();
  SysTickIntRegister(handler2);
  //MANDAR PRA UART
    
} // SysTick_Handler

void handlerEntrancePwm(void)
{
  SysTickIntDisable();
  SysTickDisable();
  SysTickPeriodSet(12000000); // f = 1Hz para clock = 24MHz
  SysTickIntRegister(handler2);
  SysTickIntEnable();
  SysTickEnable();
   volatile int loco = 0;
   if(high) // Testa estado do push-button SW2
   {
      GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 1); // Apaga LED D4
      high = false;
      loco = SysTickValueGet();
      loco++;
      time_on_low = SysTickValueGet();
   }
   else
   {
      GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);
      GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0); // Acende LED D4
      high = true;
      time_on_high = SysTickValueGet();
   }
  GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_0);
}

void gpio_initialization()
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Habilita GPIO E
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)); // Aguarda final da habilitação
  GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0); // Pino 0 como entrada
  GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);
  GPIOIntRegister(GPIO_PORTE_BASE, handlerEntrancePwm);
  GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_0);

}

void main(void){

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Habilita GPIO F (LED D3 = PF4, LED D4 = PF0)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); // Aguarda final da habilitação
    
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4); // LEDs D3 e D4 como saída
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0); // LEDs D3 e D4 apagados
  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
  
  //UARTInit();
  gpio_initialization();
  int opalele = 0;
  while(1){
    opalele++;
    opalele--;
  }
} // main
