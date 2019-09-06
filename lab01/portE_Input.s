; -------------------------------------------------------------------------------
        THUMB                        ; Instruções do tipo Thumb-2
; -------------------------------------------------------------------------------
; Declarações EQU - Defines
; ========================
; Definições de Valores
BIT0	EQU 2_0001
BIT1	EQU 2_0010
BIT2	EQU 2_00000100
BIT3	EQU 2_00001000
BIT4	EQU 2_00010000
BIT5	EQU 2_00100000
BIT6	EQU 2_01000000
BIT7	EQU 2_10000000
	

; Definições dos Ports
; PORT E
GPIO_PORTE_AHB_LOCK_R    	EQU    0x4005C520
GPIO_PORTE_AHB_CR_R      	EQU    0x4005C524
GPIO_PORTE_AHB_AMSEL_R   	EQU    0x4005C528
GPIO_PORTE_AHB_PCTL_R    	EQU    0x4005C52C
GPIO_PORTE_AHB_DIR_R     	EQU    0x4005C400
GPIO_PORTE_AHB_AFSEL_R   	EQU    0x4005C420
GPIO_PORTE_AHB_DEN_R     	EQU    0x4005C51C
GPIO_PORTE_AHB_PUR_R     	EQU    0x4005C510	
GPIO_PORTE_AHB_DATA_R    	EQU    0x4005C3FC
GPIO_PORTE               	EQU    2_000000000010000

; -------------------------------------------------------------------------------
; Área de Código - Tudo abaixo da diretiva a seguir será armazenado na memória de 
;                  código
        AREA    |.text|, CODE, READONLY, ALIGN=2

		; Se alguma função do arquivo for chamada em outro arquivo	
        EXPORT PortE_Input

; -------------------------------------------------------------------------------
; Função PortE_Input
; Parâmetro de entrada: Não tem
; Parâmetro de saída: R0 --> o valor da leitura
PortE_Input
	LDR	R1, =GPIO_PORTE_AHB_DATA_R
	LDR R0, [R1]                            
	BX LR									


    ALIGN                           ; garante que o fim da seção está alinhada 
    END                             ; fim do arquivo