// gpio.c

#include <stdint.h>
#include "tm4c1294ncpdt.h"

//#define GPIO_PORTEKLM (0x0E30) //bits 8 e 12
#define GPIO_PORTAmask (0x01)
//
void SysTick_Wait1ms(uint32_t delay);

void UART0init () {
	//1a. Habilitar o clock no módulo UART no registrador RCGCUART (cada bit representa uma UART) 
	//pag388
	SYSCTL_RCGCUART_R |= 0x01;//escrever 1 no BIT0 liga a UART0
	//1b.   e esperar até que a respectiva UART esteja pronta para ser acessada no registrador PRUART
	//(cada bit representa uma UART).
  	while((SYSCTL_PRUART_R & (0x01) ) != (0x01) ){};

	//2. Garantir que a UART esteja desabilitada antes de fazer as alterações (limpar o bit UARTEN)
	//no registrador UARTCTL (Control).
	//pag1188
	//tem parada que é mais facil achar pelo endereco e descobre o nome:
	//UART0 base: 0x4000.C000 + Offset 0x030 = UART0CTL: 0x4000.C030
	UART0_CTL_R &= ~0x01;

	//3. Escrever o baud-rate nos registradores UARTIBRD e UARTFBRD
	//pag1184
	//UART0 base: 0x4000.C000 + Offset 0x024
	//usando o exemplo do professor
	//Para um baud rate de 19200bps e um clock de 80MHz.
	//BRD = 80.000.000/(16*19.200) = 260,4167
	UART0_IBRD_R = 260;
	UART0_FBRD_R = 27;
	
	//4. Configurar o registrador UARTLCRH para o número de bits, paridade, stop bits e fila
	//1186
	//UART Line Control (UARTLCRH), offset 0x02C
	UART0_LCRH_R = 0x70;//8bits de dados, sem bit de paridade e um stop bitn e FIFO
	
	//5. Garantir que a fonte de clock seja o clock do sistema no registrador UARTCC escrevendo 0
	//(ou escolher qualquer uma das outras fontes de clock)
	//pag1213
	//UART Clock Configuration (UARTCC), offset 0xFC8
	//UART0 base: 0x4000.C000 + Offset 0xFC8 = 0x4000.CFC8
	UART0_CC_R = 0x0;

	//6. Habilitar as flags RXE, TXE e UARTEN no registrador UARTCTL (habilitar a recepção, transmissão e a UART)
	//pag1188
	UART0_CTL_R |= 0x301;

	//7. Habilitar o clock no módulo GPIO no registrador RCGGPIO (cada bit representa uma GPIO) 
	//e esperar até que a respectiva GPIO esteja pronta para ser acessada no registrador PRGPIO
	//(cada bit representa uma GPIO).
	//Na Tiva, a UART0 está conectada aos pinos PA0 (U0Rx) e PA1 (U0Tx) e que por sua vez já 
	//passam por um conversor USB

	//1a. Ativar o clock para a porta setando o bit correspondente no registrador RCGCGPIO
	SYSCTL_RCGCGPIO_R |= GPIO_PORTAmask;
	//1b.   após isso verificar no PRGPIO se a porta está pronta para uso.
  	while((SYSCTL_PRGPIO_R & (GPIO_PORTAmask) ) != (GPIO_PORTAmask) ){};
	
	//8. Desabilitar a funcionalidade analógica no registrador GPIOAMSEL.
	GPIO_PORTA_AHB_AMSEL_R &= ~0x3;//BIT1 | BIT0

	//9. Escolher a função alternativa dos pinos respectivos TX e RX no registrador GPIOPCTL
	//GPIO Port Control (GPIOPCTL)
	//aqui sao 4bits pra cada pino!
	//PAG788 diz as posicoes dos bits de cada pino
	//(verificar a tabela 10-2 no datasheet páginas 743-746) diz as funcoes
	GPIO_PORTA_AHB_PCTL_R |= 0x11;
	
	//10.Habilitar os bits de função alternativa no registrador GPIOAFSEL nos pinos respectivos à UART.
	GPIO_PORTA_AHB_AFSEL_R |= 0X03; //BIT1 | BIT0
	
	//11.Configurar os pinos como digitais no registrador GPIODEN.
	GPIO_PORTA_AHB_DEN_R |= 0x03; //BIT1 | BIT0
}

uint8_t UART0_Rx (void) {
	//isso faz aguardar enquanto o buffer esta vazio
	//FR = Flag Register pag 1180
	//RxFE = Rx FIFO Empty Flag
	while((UART0_FR_R & 0x10) > 0);
	
	return ((uint8_t) (UART0_DR_R & 0XFF));
}

void UART0_Tx (uint8_t data) {
	//isso faz esperar enquanto o buffer esta cheio 
	//FR = Flag Register
	//TxFF = Tx FIFO Full Flag
	while ((UART0_FR_R & 0x20) > 0);
		
	
	UART0_DR_R = data;
}

void UART0char_Tx (uint8_t data) {
	//isso faz esperar enquanto o buffer esta cheio 
	//FR = Flag Register
	//TxFF = Tx FIFO Full Flag
	char datachar[4];
	sprintf(datachar, "%d", data);
	
	while ((UART0_FR_R & 0x20) > 0);
		
	UART0_DR_R = datachar[0];
	UART0_DR_R = datachar[1];
	UART0_DR_R = datachar[2];
}