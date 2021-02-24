;*****************************************************************************************************
;**																									**
;**                    ��������� ��� ���������� ������ ��� ������ ��������							**
;**											PUBLICIS												**
;**                         ��� �� ATmega162 �������� ������� 8.00 ��								**
;**																									**
;*****************************************************************************************************
/*  Fuse Bits:
EXTENDED 0xFF
HICH     0xD8
LOW      0xC2
*/

.include "m162def.inc"
 
    
;	���������� �������� ����������

.def	mulL=r0
.def	mulH=r1
.def	adwL=r2
.def	adwH=r3
.def	tempL=r4
.def	tempH=r5
.def	pause_count=r6
.def	encoderL=r7
.def	encoderH=r8
;.def	=r9
;.def	=r10
;.def	=r11
;.def	=r12
;.def	=r13
;.def	=r14

.def	step_zoom=r15
.def	adress7219=r16						; ����� ������� ����������
.def	temp=r17
.def	temp1=r18
.def	temp2=r19
.def	count=r20
.def	data7219=r21						; ��������� ����� �� ���������
.def	count_ASF_motor=r22					; ������� ����� ��������� ������� (ASF Motor)
.def	step_motorL=r23						; ������� ����� ��������� �������� �����
.def	step_motorH=r24
.def	enc_step=r25

;.def	=r26								; XL
;.def	=r27								; XH
.def	reductorL=r28						; YL ������� �������� � �������� ���� ��������
.def	reductorH=r29						; YH
;.def	=r30								; ZL
;.def	=r31								; ZH


;------------------------------------------------------------------------------
;   ���� � ������ ��� ��������� �� MAX7219CWG
.equ	Port_7219_Data=PORTB				; ����� DATA
.equ	Pin_7219_Data=7
.equ	Port_7219_Clk=PORTB					; ����� Clk
.equ	Pin_7219_Clk=5
.equ	Port_7219_CS=PORTB					; ����� CS
.equ	Pin_7219_CS=6

;--------------------------------------------------------------------------------------------------
;   ������� �������

;	�������� ����������� ����� �� ����������� (������ � ����� �����)
.equ	Pin_stol_home=PINE					; �������� ���������� ��������� �����
.equ	N_stol_home=2
.equ	Pin_stol_end=PINC					; �������� ��������� ��������� �����
.equ	N_stol_end=7

;	����� �� ��������������� ������
.equ	Pin_laser_in=PINE					; �������������� ������
.equ	N_laser_in=1

;	����� �� ������
.equ	Pin_power=PINC						; ������ ���������/���������� ��������
.equ	N_power=2							; "PINC0"
.equ	Pin_but_home=PIND					; ������ "Home"
.equ	N_but_home=6
.equ	Pin_but_forward=PINC				; ������ "Forward"
.equ	N_but_forward=0						; "PINC4"
.equ	Pin_but_reverse=PINC				; ������ "Revers"
.equ	N_but_reverse=1						; "PINC5"

;	���� �� ����������� ���������� �������� "INK" � "PAPER"
.equ	Pin_led_paper=PINC					; ���� �� ����������, ��������� ������
.equ	N_led_paper=4
.equ	Pin_led_ink=PINC					; ���� �� ����������, ���������������� ��������� ������
.equ	N_led_ink=5


;	���� �� ��������� �������
.equ	Pin_pickup_ready=PIND				; ������ ���������� �� ��������� �������
.equ	N_pickup_ready=5

;--------------------------------------------------------------------------------------------------
;	������ �� �������������� ����������

;	���������� ������� ����������� ������ ��������
.equ	Port_laser_out=PORTE				; ���������� �������
.equ	N_laser_out=0

;	���������� ��������
.equ	Port_zumer=PORTA					; ���������� ��������
.equ	N_zumer=1


;	���������� ��������� ��������� ����������� ����� � �������������� ���������
.equ	Port_stol_step=PORTB				; Step ������ �������� ������ ��������� �����
.equ	N_stol_step=1
.equ	Port_stol_dir=PORTB					; Direction ������ �������� ������ ��������� �����
.equ	N_stol_dir=0
.equ	Port_stol_en=PORTA					; Enable ������ �������� ������ ��������� �����
.equ	N_stol_en=0
.equ	Port_stol_SW1=PORTB					; SW1 ������ ����������� ��������� �������� �����
.equ	N_stol_SW1=2
.equ	Port_stol_SW2=PORTB					; SW2 ������ ����������� ��������� �������� �����
.equ	N_stol_SW2=3
.equ	Port_stol_SW3=PORTB					; SW3 ������ ����������� ��������� �������� �����
.equ	N_stol_SW3=4

;	������ ��� ������������ ������ �������
.equ	Port_PE_enable=PORTD				; ������ �������� ������� ������ ��� ������
.equ	N_PE_enable=4

.equ	Port_paper=PORTD					; ������ ������������� ������
.equ	N_paper=7

.equ	Port_pause=PORTC					; ������ ����� ��� ������
.equ	N_pause=6

;	�������������� ������
.equ	Port_L1=PORTA						; �������������� ����� L1
.equ	N_L1=5
.equ	Port_L2=PORTA						; �������������� ����� L2
.equ	N_L2=6
.equ	Port_L3=PORTA						; �������������� ����� L3
.equ	N_L3=7

;	������ �� RGB
.equ	Port_R=PORTA						; ����� �� ����
.equ	N_R=2
.equ	Port_G=PORTA						; ����� �� ����
.equ	N_G=3
.equ	Port_B=PORTA						; ����� �� ����
.equ	N_B=4


;------------------------------------------------------------------------------ 
    ; ��� ������� ������. � ��� ���������� ����������� ������

.dseg

Digit:			.byte 8
BCD:			.byte 5



.cseg

; ***** INTERRUPT VECTORS ************************************************

.org $000
     jmp RESET                              ; ��������� ������
.org $002
     jmp aINT0                              ;$002 - ������� ���������� 0
.org $004
     jmp aINT1                              ;$004 - ������� ���������� 1
.org $006
     jmp aOC2                               ;$006 - ���������� �������/������� �2
.org $008
     jmp aOVF2                              ;$008 - ������������ �������/������� �2
.org $00A
     jmp aICP1                              ;$00A - ������ �������/������� �1 
.org $00C
     jmp aOC1A                              ;$00C - ���������� � �������/������� �1
.org $00E
     jmp aOC1B                              ;$00E - ���������� � �������/������� �1
.org $0010
     jmp aOVF1                              ;$010 - ������������ �������/������� �1
.org $0012
     jmp aOVF0                              ;$012 - ������������ �������/������� �0 
.org $0014
     jmp aSPI                               ;$014 - �������� �� SPI ���������
.org $0016
     jmp aURXC                              ;$016 - UART ����� ��������
.org $0018
     jmp aUDRE                              ;$018 - ������� ������ UART ���� 
.org $001A
     jmp aUTXC                              ;$01A - UART �������� ���������
.org $001C
     jmp aADCC                              ;$01C - �������������� ADC ���������
.org $001E
     jmp aERDY                              ;$01E - EEPROM �����
.org $0020
     jmp aACI                               ;$020 - ���������� ����������
.org $0022
     jmp aTWI                               ;$022 - ���������� �� ������ Two-Wire 
.org $0024
     jmp aINT2                              ;$024 - ������� ����������
.org $0026
     jmp aOC0                               ;$026 - ���������� �������/������� �0
.org $0028
     jmp aSPMR                              ;$028 - ����������SPM

;************************************************************************;
;************************************************************************;
;**                                                                    **;
;*               ��� ��������� ������� �� �������� ����                 *;
;*                                                                      *;
;**                                                                    **;
;************************************************************************;
;************************************************************************;

RESET:										; ��������� ������
;aINT0:										;$002 - ������� ���������� 0
aINT1:										;$004 - ������� ���������� 1
 aOC2:										;$006 - ���������� �������/������� �2
aOVF2:										;$008 - ������������ �������/������� �2
aICP1:										;$00A - ������ �������/������� �1 
aOC1A:										;$00C - ���������� � �������/������� �1
aOC1B:										;$00E - ���������� � �������/������� �1
aOVF1:										;$010 - ������������ �������/������� �1
aOVF0:										;$012 - ������������ �������/������� �0
 aSPI:										;$014 - �������� �� SPI ���������
aURXC:										;$016 - UART ����� ��������
aUDRE:										;$018 - ������� ������ UART ����
aUTXC:										;$01A - UART �������� ���������
aADCC:										;$01C - �������������� ADC ���������
aERDY:										;$01E - EEPROM �����
 aACI:										;$020 - ���������� ����������
 aTWI:										;$022 - ���������� �� ������ Two-Wire
aINT2:										;$024 - ������� ����������
 aOC0:										;$026 - ���������� �������/������� �0
aSPMR:										;$028 - ����������SPM



	; ������������� ����� 
 ldi 	temp, LOW(RAMEND)
 out	spl, Temp
 ldi 	temp, HIGH(RAMEND)
 out	sph, Temp



;	������������� ������

; ���������� ����������� ������
 ldi	temp,0b11111111						; ������ �������� �� ����� ����� A
 out	DDRA,temp       

 ldi	temp,0b11111111						; ������ �������� �� ����� ����� B
 out	DDRB,temp       

 ldi	temp,0b01001000						; ������ �������� �� ����� ����� C
 out	DDRC,temp       

 ldi	temp,0b10010001						; ������ �������� �� ����� ����� D
 out	DDRD,temp       

 ldi	temp,0b00000001						; ������ �������� �� ����� ����� D
 out	DDRE,temp       

; ������������� �������� �������� ������
 ldi	temp,0b00000000						; ������������� ����� A
 out	PORTA,temp 

 ldi	temp,0b00000000						; ������������� ����� B
 out	PORTB,temp 

 ldi	temp,0b00110000						; ������������� ����� C
 out	PORTC,temp

 ldi	temp,0b00000000						; ������������� ����� D
 out	PORTD,temp
 
 ldi	temp,0b00000000						; ������������� ����� D
 out	PORTE,temp
        

;------------------------------------------------------------------------------
; ��������� � ����������� ������� ���������� �� ������������ ������ ������� �� ����� INT0
 ldi	temp,0b01000000
 	; 7 - INT1
	; 6 - INT0
	; 5 - INT2
	; 4 - PCIE1
	; 3 - PCIE0
	; 2 -
	; 1 - IVSEL
	; 0 - IVCE
 out	GICR,temp								; ��������� ������� ���������� INT0

 ldi	temp,0b00000011
	; 7 - SRE
	; 6 - SRW10
	; 5 - SE
	; 4 - SM1
	; 3 - ISC11
	; 2 - ISC10
	; +1 - ISC01
	; +0 - ISC00
 out	MCUCR,temp								; �� ������������ ������ ������� �� ������ INT0


;****************************************************************************************************
;*																									*
;*																									*
;*                ���������� ��������� � ������������� ���������� ���������							*
;*																									*
;*																									*
;****************************************************************************************************
 /*
; Micro step = 1 (pulse=200)
 sbi	Port_stol_SW1,N_stol_SW1			; SW1 - on
 sbi	Port_stol_SW2,N_stol_SW2			; SW2 - on
 cbi	Port_stol_SW3,N_stol_SW3			; SW3 - off
; Micro step = 2/A (pulse=400)
 sbi	Port_stol_SW1,N_stol_SW1			; SW1 - on
 cbi	Port_stol_SW2,N_stol_SW2			; SW2 - off
 sbi	Port_stol_SW3,N_stol_SW3			; SW3 - on
; Micro step = 2/B (pulse=400)
 cbi	Port_stol_SW1,N_stol_SW1			; SW1 - off
 sbi	Port_stol_SW2,N_stol_SW2			; SW2 - on
 sbi	Port_stol_SW3,N_stol_SW3			; SW3 - on
; Micro step = 4 (pulse=800)
 sbi	Port_stol_SW1,N_stol_SW1			; SW1 - on
 cbi	Port_stol_SW2,N_stol_SW2			; SW2 - off
 cbi	Port_stol_SW3,N_stol_SW3			; SW3 - off
; Micro step = 8 (pulse=1600)
 cbi	Port_stol_SW1,N_stol_SW1			; SW1 - off
 sbi	Port_stol_SW2,N_stol_SW2			; SW2 - on
 cbi	Port_stol_SW3,N_stol_SW3			; SW3 - off
; Micro step = 16 (pulse=3200)
 cbi	Port_stol_SW1,N_stol_SW1			; SW1 - off
 cbi	Port_stol_SW2,N_stol_SW2			; SW2 - off
 sbi	Port_stol_SW3,N_stol_SW3			; SW3 - on
; Micro step = 32 (pulse=6400)
 cbi	Port_stol_SW1,N_stol_SW1			; SW1 - off
 cbi	Port_stol_SW2,N_stol_SW2			; SW2 - off
 cbi	Port_stol_SW3,N_stol_SW3			; SW3 - off
*/
 rcall	MAX7219_Init						; �������������� ���������
 ldi	temp,8
 sts	Digit,temp							; ������� ����� ���� ������������� �� ��������� 
 ldi	temp,8
 sts	Digit+1,temp						; ������� ����� ���� ������������� �� ��������� 
 ldi	temp,8
 sts	Digit+2,temp						; ������� ����� ���� ������������� �� ��������� 
 ldi	temp,8
 sts	Digit+3,temp						; ������� ����� ���� ������������� �� ��������� 
 ldi	temp,8
 sts	Digit+4,temp						; ������� ����� ���� ������������� �� ��������� 
 ldi	temp,8
 sts	Digit+5,temp						; ������� ����� ���� ������������� �� ��������� 
 ldi	temp,8
 sts	Digit+6,temp						; ������� ����� ���� ������������� �� ��������� 
 ldi	temp,8
 sts	Digit+7,temp						; ������� ����� ���� ������������� �� ��������� 
 rcall	MAX7219_RAM							;
 rcall	d500ms
 rcall	d500ms
 rcall	MAX7219_CLEAR						; ������� ���������

;**************************************************************************************************
;	������������� �������� ��� �������� � �������� ���� ��� �������� ���������
 ldi	temp,4								; �������� ��� ��������
 mov	step_zoom,temp

; Micro step = 4 (pulse=800)
 sbi	Port_stol_SW1,N_stol_SW1			; SW1 - on
 cbi	Port_stol_SW2,N_stol_SW2			; SW2 - off
 cbi	Port_stol_SW3,N_stol_SW3			; SW3 - off
;**************************************************************************************************

 sbi	Port_stol_en,N_stol_en				; ��������� �������
 cbi	Port_PE_enable,N_PE_enable			; ���������� ������ ������� ������

 cbi	Port_R,N_R							; ��������� ������� ����
 cbi	Port_G,N_G							; ��������� ������� ����
 sbi	Port_B,N_B							; �������� ����� ����

 cbi	Port_zumer,N_zumer					; ��������� �������

 sei										; ��������� ���������� ��� ���������� �������� � ��������



;*****************************************************************************************************

 ldi	temp,1								; ������� 1 1 1 1 �� ���������, ��������� � �������� ���� 
 sts	Digit,temp
 sts	Digit+2,temp
 sts	Digit+4,temp
 sts	Digit+6,temp
 sts	Digit,temp
 rcall	MAX7219_RAM							; ��������� ���������� �� ����������

;--------------------------------------------
;	������� ���� �� ��� �������, ��� �� �� ���� ������ �����
;rjmp	MAIN_LOOP							; ��� �����

;	������� ���� � ��������� ��������� ��� ���������� ������ �����
 sbi	Port_zumer,N_zumer					; �������� �������
 sbi	Port_R,N_R							; �������� ������� ����
 cbi	Port_G,N_G							; ��������� ������� ����
 cbi	Port_B,N_B							; �������� ����� ����
 rcall	d500ms

MOVE_HOME:
 rcall	MAX7219_SEND						; ��������� ���������� �� ����������

;	������� ��������� ����� � �������� �����������
 sbi	Port_stol_dir,N_stol_dir			; ������ ��������� ����� � �������� �����������
 cbi	Port_stol_en,N_stol_en				; ���������� �������
 cbi	Port_stol_step,N_stol_step			; ������ ���� ���
 rcall	d05ms
 sbi	Port_stol_step,N_stol_step
 rcall	d035ms
;	��������� ������� ���������� � ��������� ��������� �����. ���� ��������, ����� ���� �� �������
 sbic	Pin_stol_home,N_stol_home			; ���� �� ��������� ������ ���������� ��������� �����, ���� ����� ���������
 rjmp	MOVE_END							;  �������� ������, ������� ���� �� ������� � ��������� ���������
 rjmp	MOVE_HOME							; ���� ������� �� ���������, ����� ������� ���� ������ � ������

;--------------------------------------------
;	������� �� 10 ����� ������
MOVE_END:
 rcall	d50ms
 ldi	count,10							; ������� ���� ������ �� 10 �����, ��� �� ������������ ������ ������ �����
 rcall	d50ms
MOVE_END_10:
 rcall	MAX7219_SEND
 cbi	Port_stol_dir,N_stol_dir			; ������ ��������� ����� � ������ �����������
 cbi	Port_stol_en,N_stol_en				; ���������� �������
 cbi	Port_stol_step,N_stol_step			; ������ ���� ���
 rcall	d10ms				
 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; ��������� �������
 dec	count								; ��������� ������� ������
 brne	MOVE_END_10							; ����� ������� ��� ����, ����������� ���� � �������

 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; ��������� ������� ���������

 sbi	Port_R,N_R							; �������� ������� ����
 cbi	Port_G,N_G							; ��������� ������� ����
 cbi	Port_B,N_B							; ��������� ����� ����

 cbi	Port_zumer,N_zumer					; ��������� �������

 ldi	temp,2								; ������� 2 2 2 2 2 �� ���������
 sts	Digit,temp
 sts	Digit+2,temp
 sts	Digit+4,temp
 sts	Digit+6,temp
 rcall	MAX7219_RAM							; ��������� ���������� �� ����������


;--------------------------------------------
;	���� ������������� �������
;rjmp	MAIN_LOOP							; ��� �����

PICKUP_WAIT_OFF:							; ���� ������������� ��������� �������
 rcall	d10ms
 sbis	Pin_pickup_ready,N_pickup_ready
 rjmp	PICKUP_WAIT_OFF
 rcall	d500ms
 rcall	d500ms
 rcall	d500ms
 rcall	d500ms
 rcall	d500ms
 rcall	d500ms

;--------------------------------------------
;	������� ���� � "Home" ��� ���������� ������ ������� ������, ���� �������� ��� �������������
 ldi	count,40							; ��������� ����� �������
ERROR_PAPER:
 rcall	d500ms								; �������� �������� 0,5 �
 dec	count								;  � ������� �����
 brne	ERROR_PAPER							; ���� �� ��������, �� ������� ������
 rjmp	HOME_SKIP							; ���� ��������� �� ����� �������, �� ��� ��� ������� ���� � "Home"

 cbi	Port_R,N_R							; ��������� ������� ����
 cbi	Port_G,N_G							; ��������� ������� ����
 sbi	Port_B,N_B							; �������� ����� ����


;*********************************************************************************************************
;*********************************************************************************************************
;**																										**
;**																										**
;**                                     �������� ���� ���������											**
;**																										**
;**																										**
;*********************************************************************************************************
;*********************************************************************************************************

MAIN_LOOP:
 rcall	MAX7219_CLEAR						; ������� ���������
 ldi	temp,0								; ������� ����� 0 �� ��������� (������� ����)
 sts	Digit,temp


 mov	adwH,reductorH
 mov	adwL,reductorL
 rcall	Bin2ToBCD5
 lds	temp,BCD+4
 sts	Digit+6,temp
 lds	temp,BCD+3
 sts	Digit+5,temp
 lds	temp,BCD+2
 sts	Digit+4,temp
 lds	temp,BCD+1
 sts	Digit+3,temp
 lds	temp,BCD
 sts	Digit+2,temp
 rcall	MAX7219_RAM							; ��������� ���������� �� ����������

MAIN_LOOP1:
 sbis	Pin_but_home,N_but_home				; ��������� ��������� ������ "HOME"
 rjmp	HOME								;  ���� ������, ����� ������� ���� � ������� ������

 sbis	Pin_but_reverse,N_but_reverse		; ��������� ��������� ������ "Reverse"
 rjmp	REVERSE								;  ���� ������, ����� ��������� ���� � �����

; rjmp	PRINT_SKIP
 sbic	Pin_pickup_ready,N_pickup_ready		; ��������� ������ ������� ������
 rjmp	PRINT								;  ���� ������� �������� ������, ������ ��������

 cbi	Port_R,N_R							; ��������� ������� ����
 cbi	Port_G,N_G							; ��������� ������� ����
 sbi	Port_B,N_B							; �������� ����� ����

 rcall	d10ms

rjmp	MAIN_LOOP1							; ��������� �������� ���� ����������









;*****************************************************************************************************
;*****************************************************************************************************
;**																									**
;**																									**
;**                                            ��������� 											**
;**																									**
;**																									**
;*****************************************************************************************************
; step_motorH:step_motorL - ����, ������� ������ ������� ���������
; reductorH:reductorL     - ����, ������� ���������� ������

PRINT:
 rcall	d1ms								; ��������� ������� ���������
 sbis	Pin_pickup_ready,N_pickup_ready			
 rjmp	MAIN_LOOP							; ���� ������ ������������, ����� ��������� � ������� ���� ���������

; Micro step = 4 (pulse=800)
 sbi	Port_stol_SW1,N_stol_SW1			; SW1 - on
 cbi	Port_stol_SW2,N_stol_SW2			; SW2 - off
 cbi	Port_stol_SW3,N_stol_SW3			; SW3 - off

PRINT_SKIP:
 rcall	MAX7219_CLEAR						; ������� ���������
 ldi	temp,3								; ������� ����� 3 �� ��������� (����� ������)
 sts	Digit,temp
 rcall	MAX7219_RAM							; ��������� ���������� �� ����������

 clr	encoderL
 clr	encoderH
 clr	reductorL
 clr	reductorH
 clr	step_motorL
 clr	step_motorH
 clr	enc_step

 cbi	Port_R,N_R							; ��������� ������� ����
 sbi	Port_G,N_G							; �������� ������� ����
 cbi	Port_B,N_B							; ��������� ����� ����

 sbi	Port_PE_enable,N_PE_enable			; ���������� ������ ������� ������ �� ����� �������� �����
 sbi	Port_L3,N_L3

;********************************************
;********************************************
;	��������������� ������� ������
PRINT_HOD:
;--------------------------------------------
;	��������� ������ ��������� ��������� �����. ���� ��������, ����� ���� �� �������
 sbic	Pin_stol_end,N_stol_end				; ���� �� ��������� ������ ��������� ��������� �����, ���� ����� ���������
 rjmp	DVIGATEL_STOP						;  �������� ������, ������� ���� �� ������� � ��������� ��������� 
;	��������� ������ �� ����������
 sbis	Pin_but_home,N_but_home				; ��������� ��������� ������ "HOME"
 rjmp	HOME								; ��� �������� � ��������� "HOME" ��� ������
 sbis	Pin_but_reverse,N_but_reverse		; ��������� ��������� ������ "Reverse"
 rjmp	REVERSE
;--------------------------------------------
/*
 mov	adwH,step_motorH
 mov	adwL,step_motorL
 rcall	Bin2ToBCD5
 lds	temp,BCD+4
 sts	Digit+3,temp
 lds	temp,BCD+3
 sts	Digit+2,temp
 lds	temp,BCD+2
 sts	Digit+1,temp
 mov	adwH,reductorH
 mov	adwL,reductorL
 rcall	Bin2ToBCD5
 lds	temp,BCD+4
 sts	Digit+7,temp
 lds	temp,BCD+3
 sts	Digit+6,temp
 lds	temp,BCD+2
 sts	Digit+5,temp
 rcall	MAX7219_RAM							; ��������� ���������� �� ����������
*/
 mov	adwH,encoderH
 mov	adwL,encoderL
 rcall	Bin2ToBCD5
 lds	temp,BCD+4
 sts	Digit+6,temp
 lds	temp,BCD+3
 sts	Digit+5,temp
 lds	temp,BCD+2
 sts	Digit+4,temp
 lds	temp,BCD+1
 sts	Digit+3,temp
 lds	temp,BCD
 sts	Digit+2,temp
 rcall	MAX7219_RAM							; ��������� ���������� �� ����������

;	��������� �������� �������� ����� ��������� � ��������. ���� ��� �����, ����� ��������� �� ��������.
;	�.�. �� ������ �����, ����� ���� �������� ����� �������� (��� ������� ������)
COMPARE_STEP:
 cp		reductorL,step_motorL				; ���������� ������� ����� �����
 cpc	reductorH,step_motorH				; ���������� ������� ����� ����� � ������ ��������
 breq	PRINT_HOD							; ���� ���������� ��������� �����, ����� �� ������� ����


 cp		reductorL,step_motorL				; ���������� ������� ����� �����
 cpc	reductorH,step_motorH				; ���������� ������� ����� ����� � ������ ��������
 brpl	STEP_FORWARD						; ������� �� ������������� ��������

 cp		reductorL,step_motorL				; ���������� ������� ����� �����
 cpc	reductorH,step_motorH				; ���������� ������� ����� ����� � ������ ��������
 brmi	STEP_REVERSE						; ������� �� ������������� ��������


STEP_REVERSE:								; ������ ��������� � �������� �����������
 rcall	d035ms
 sbi	Port_stol_dir,N_stol_dir			; ������ ��������� ����� � �������� �����������
 cbi	Port_stol_en,N_stol_en				; ���������� �������
 cbi	Port_stol_step,N_stol_step			; ������ ���� ���
 rcall	d05ms
 sbi	Port_stol_step,N_stol_step			; ������ ���
 rcall	d05ms	

 mov	ZL,step_motorL						; ���������� �������������� ��������
 mov	ZH,step_motorH
 sbiw	ZH:ZL,1								; ����������� ������� ���������� ����� �� 1
 mov	step_motorL,ZL						; ������ �������� �������
 mov	step_motorH,ZH

rjmp	PRINT_HOD


STEP_FORWARD:								; ������ ��������� � ������ �����������
 cbi	Port_stol_dir,N_stol_dir			; ������ ��������� ����� � ������ �����������
 cbi	Port_stol_en,N_stol_en				; ���������� �������
 cbi	Port_stol_step,N_stol_step			; ������ ���� ���
 rcall	d05ms
 sbi	Port_stol_step,N_stol_step			; ������ ���
 rcall	d05ms	

 mov	ZL,step_motorL						; ���������� �������������� ��������
 mov	ZH,step_motorH
 adiw	ZH:ZL,1								; ����������� ������� ���������� ����� �� 1
 mov	step_motorL,ZL						; ������ �������� �������
 mov	step_motorH,ZH

rjmp	PRINT_HOD


 cbi	Port_R,N_R							; ��������� ������� ����
 sbi	Port_G,N_G							; �������� ������� ����
 cbi	Port_B,N_B							; ��������� ����� ����

rjmp	PRINT_HOD

DVIGATEL_STOP:
 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; ��������� �������

 sbi	Port_R,N_R							; ��������� ������� ����
 cbi	Port_G,N_G							; ��������� ������� ����
 cbi	Port_B,N_B							; ��������� ����� ����
 
 rcall	d500ms
 rcall	d500ms
 rcall	d500ms
 rcall	d500ms

PRINT_END:
 rcall	d10ms
;	��������� ������ �� ����������
 sbis	Pin_but_home,N_but_home				; ��������� ��������� ������ "HOME"
 rjmp	HOME								; ��� �������� � ��������� "HOME" ��� ������
 sbis	Pin_but_reverse,N_but_reverse		; ��������� ��������� ������ "Reverse"
 rjmp	REVERSE


; rjmp	HOME_SKIP							; ����������� �����
 rjmp	PRINT_END



;****************************************************************************************************
;	������ ������ "HOME". ������� ���� � ��������� ��� ������ ���������
;****************************************************************************************************
HOME:
 rcall	d1ms								; ��������� ������� ���������
 sbic	Pin_but_home,N_but_home			
 rjmp	MAIN_LOOP							; ���� ������ ������������, ����� ��������� � ������� ���� ���������

HOME_SKIP:

 clr	reductorH
 clr	reductorL

 cbi	Port_R,N_R							; ��������� ������� ����
 sbi	Port_G,N_G							; �������� ������� ����
 cbi	Port_B,N_B							; ��������� ����� ����

 rcall	MAX7219_CLEAR						; ������� ���������
 ldi	temp,1								; ������� ����� 1 �� ��������� (�������� ����� � ������ ������)
 sts	Digit,temp

;--------------------------------------------
HOME_GO:									; ������� ���� �� ������
;	������� �������� ���� �� ���������
 mov	adwH,reductorH
 mov	adwL,reductorL
 rcall	Bin2ToBCD5
 lds	temp,BCD+4
 sts	Digit+6,temp
 lds	temp,BCD+3
 sts	Digit+5,temp
 lds	temp,BCD+2
 sts	Digit+4,temp
 lds	temp,BCD+1
 sts	Digit+3,temp
 lds	temp,BCD
 sts	Digit+2,temp
 rcall	MAX7219_RAM							; ��������� ���������� �� ����������

SPEED_HOME1:								; � ������ �������� �������� ����� ������������
;	������� ��������� ����� � �������� �����������
 sbi	Port_stol_dir,N_stol_dir			; ������ ��������� ����� � �������� �����������
 cbi	Port_stol_en,N_stol_en				; ���������� �������
 cbi	Port_stol_step,N_stol_step			; ������ ���� ���
 rcall	d035ms
 sbi	Port_stol_step,N_stol_step
 rcall	d035ms
 adiw	reductorH:reductorL,1				; ���������� ���
;	��������� ������� ���������� � ��������� ��������� �����. ���� ��������, ����� ���� �� �������
 sbic	Pin_stol_home,N_stol_home			; ���� �� ��������� ������ ���������� ��������� �����, ���� ����� ���������
 rjmp	HOME_END							;  �������� ������, ������� ���� �� ������� � ��������� ���������
; sbic	Pin_stol_end,N_stol_end				; ���� �� ��������� ������ ��������� ��������� �����, ���� ����� ���������
; rjmp	HOME_END							;  �������� ������, ������� ���� �� ������� � ��������� ��������� 
 rjmp	HOME_GO								; ���� ������� �� ���������, ����� ������� ���� ������ � ������

;--------------------------------------------
HOME_END:
 ldi	count,10							; ������� ���� ������ �� 10 �����, ��� �� ������������ ������ ������ �����
 rcall	d100ms
HOME_END_10:
 cbi	Port_stol_dir,N_stol_dir			; ������ ��������� ����� � ������ �����������
 cbi	Port_stol_en,N_stol_en				; ���������� �������
 cbi	Port_stol_step,N_stol_step			; ������ ���� ���
 rcall	d10ms				
 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; ��������� �������
 dec	count								; ��������� ������� ������
 brne	HOME_END_10							; ����� ������� ��� ����, ����������� ���� � �������

 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; ��������� ������� ���������

 cbi	Port_PE_enable,N_PE_enable			; ���������� ������ ������� ������
 cbi	Port_L3,N_L3

;--------------------------------------------
;	��������� ��������� ���������� "Paper" ����� �������������� ������� ������. ���� �� �����, ����� ������������
;    �������� ������ �������� ������. ���� �� �����, ����� �� �������� ������ "Paper"
 rcall	d025ms								; ����� ������������ �������� �� 0,25��
 ldi	YL,low(8000)						; �������� � YH:YL ���������
 ldi	YH,high(8000)

PAPER_PUSH:
 sbis	Pin_led_paper,N_led_paper			; ��������� ��������� ���������� "Paper" ��������.
 rjmp	PUSH_BUTTON							; ���� �� ���������, ������ �������� ������ "Paper" �� ���������� ��������
 
 rcall	d025ms								; ����� ������������ �������� �� 0,25��
 sbiw	YL,1								; ��������� ������� �� ����������� YH:YL
 brne	PAPER_PUSH							; ���� ��������� �� ����� ����, ������� � ������ �����

 sbi	Port_zumer,N_zumer					; ������ ��������
 cbi	Port_R,N_R							; ��������� ������� ����
 cbi	Port_G,N_G							; ��������� ������� ����
 sbi	Port_B,N_B							; �������� ����� ����
 rcall	d500ms
 cbi	Port_zumer,N_zumer					; ��������� �������

 rjmp	PUSH_BUTTON_END						; ��������� � ������� ���� �� ������� ������ �������������

;--------------------------------------------
PUSH_BUTTON:
 sbi	Port_paper,N_paper					; �������� ������ "paper"
 rcall	d500ms								; ��������� �������� ����� �������� �����
 cbi	Port_paper,N_paper					; ��������� ������ "paper"

 sbi	Port_zumer,N_zumer					; ������ ��������
 cbi	Port_R,N_R							; ��������� ������� ����
 cbi	Port_G,N_G							; ��������� ������� ����
 sbi	Port_B,N_B							; �������� ����� ����
 rcall	d500ms
 cbi	Port_zumer,N_zumer					; ��������� �������

;--------------------------------------------
PUSH_BUTTON_END:

rjmp	MAIN_LOOP							; ��������� � ������� ���� ������� ������ �������������







;*****************************************************************************************************
;	��� ������� ������ "Reverse" �� ���������� �������� ����������� ���� � ����� � ���������� �������
;    ���������� �� ����� ����������� �����
;*****************************************************************************************************
REVERSE:
 rcall	d1ms								; ��������� ������� ���������
 sbic	Pin_but_reverse,N_but_reverse			
 rjmp	MAIN_LOOP							; ���� ������ ������������, ����� ��������� � ������� ���� ���������

 clr	reductorH
 clr	reductorL

 rcall	MAX7219_CLEAR						; ������� ���������
 ldi	temp,2								; ������� ����� 2 �� ��������� (�������� ����� � �����)
 sts	Digit,temp
 rcall	MAX7219_RAM							; ��������� ���������� �� ����������

REVERSE_HOD:
 cbi	Port_R,N_R							; ��������� ������� ����
 sbi	Port_G,N_G							; �������� ������� ����
 cbi	Port_B,N_B							; ��������� ����� ����

;	������� ��������� ����� � �������� �����������
 cbi	Port_stol_dir,N_stol_dir			; ������ ��������� ����� � ������ �����������
 cbi	Port_stol_en,N_stol_en				; ���������� �������
 cbi	Port_stol_step,N_stol_step			; ������ ���� ���
 rcall	d035ms
 sbi	Port_stol_step,N_stol_step
 rcall	d035ms

 adiw	reductorH:reductorL,1

;	������� �������� ���� �� ���������
 mov	adwH,reductorH
 mov	adwL,reductorL
 rcall	Bin2ToBCD5
 lds	temp,BCD+4
 sts	Digit+6,temp
 lds	temp,BCD+3
 sts	Digit+5,temp
 lds	temp,BCD+2
 sts	Digit+4,temp
 lds	temp,BCD+1
 sts	Digit+3,temp
 lds	temp,BCD
 sts	Digit+2,temp
 rcall	MAX7219_RAM							; ��������� ���������� �� ����������

 sbic	Pin_stol_end,N_stol_end				; ���� �� ��������� ������ ��������� ��������� �����, ���� ����� ���������
 rjmp	REVERSE_END							;  �������� ������, ������� ���� �� ������� � ��������� ��������� 

 rjmp	REVERSE_HOD


;--------------------------------------------
REVERSE_END:
 cbi	Port_R,N_R							; ��������� ������� ����
 cbi	Port_G,N_G							; ��������� ������� ����
 sbi	Port_B,N_B							; �������� ����� ����

 sbi	Port_zumer,N_zumer					; ������ ���� ���
 rcall	d500ms
 rcall	d500ms
 cbi	Port_zumer,N_zumer

 sbi	Port_stol_en,N_stol_en				; ��������� ������� ���������

rjmp	MAIN_LOOP






;****************************************************************************************************
;	��� ��������� ���������� "Paper" �� ���������� �������� ������������� ������� ������ � ����
;    ������� ����� �� ���������� ���������
;****************************************************************************************************
LED_PAPER_ON:
 sbi	Port_R,N_R							; �������� ������� ����
 cbi	Port_G,N_G							; ��������� ������� ����
 cbi	Port_B,N_B							; ��������� ����� ����

; cli										; ��������� ���������� �� ����� �������� �����

 sbis	Pin_but_home,N_but_home				; ��������� ��������� ������ "HOME"
 rjmp	HOME								; ��� �������� � ��������� "HOME" ��� ������

 sbis	Pin_but_reverse,N_but_reverse		; ��������� ��������� ������ "Reverse"
 rjmp	REVERSE

 sbis	Pin_led_paper,N_led_paper			; ��������� ��������� ���������� "Paper" �� ���������� ��������
 rjmp	LED_PAPER_ON						; ���� �����, ����� �������� ������� ���������

;-----------------------------------------------------------------------------------------------------------------
;	��������� ��������� ����������. ���� �� �������, �� ��������� � ������� ����
;	��������� �������� ���������� � ������� 600 ��. ���� �� ���������, ������ �� �������
LED_PAPER_BLINK:
 cbi	Port_R,N_R							; ��������� ������� ����
 cbi	Port_G,N_G							; ��������� ������� ����
 cbi	Port_B,N_B							; ��������� ����� ����

 rcall	d025ms								; ����� ������������ �������� �� 0,25��
 ldi	YL,low(8000)						; �������� � YH:YL ���������
 ldi	YH,high(8000)

PAPER_PAUSE:
 sbis	Pin_led_paper,N_led_paper			; ��������� ��������� ���������� "Paper" ��������.
 rjmp	LED_PAPER_ON						; ���� �� ���������, ������ �� ������� � ��������� � ������ �����

 rcall	d025ms								; ����� ������������ �������� �� 0,25��
 sbiw	YL,1								; ��������� ������� �� ����������� YH:YL
 brne	PAPER_PAUSE							; ���� ��������� �� ����� ����, �������
											; �� ����� d600_1
ret											; ���� ����� 600 �� ��������� �����, ������ ��� ���������, ���������� ������ ��������											; ��������� ��� �������


;************************************************************************************
;*																					*
;*                           ����� ��������� �� ���������                           *
;*         ���������� adress7219 �������� ������ ��� ������ ������ ������			*
;*         ���������� data7219 �������� ����� ��� ������ �� ������� ����������		*
;*																					*
;************************************************************************************

;   ��������: adress7219, data7219, temp2 
;-----------------------------------------------------------------------------------
; ������� ��� ������ ������������ ��������� �� �������������� �������

MAX7219_SEND: 
 ldi	temp2,8								; �������� ������ ��������
 cbi	Port_7219_Clk,Pin_7219_Clk			; c��������� ������ CLK 
 cbi	Port_7219_CS,Pin_7219_CS			; ��������� ������ 7219 �������� CS=0
 nop
MAX7219_LOOP8:								;
 cbi	Port_7219_Clk,Pin_7219_Clk			; c��������� ������ CLK 
 sbrc	adress7219,7						; g�������� ������� ���
 sbi	Port_7219_Data,Pin_7219_Data		;  �������� ��������������
 sbrs	adress7219,7						;  �������� � �����
 cbi	Port_7219_Data,Pin_7219_Data		;  ������� Data
 lsl	adress7219							; c���� ����� (��������� ���)
 sbi	Port_7219_Clk,Pin_7219_Clk			; p����� ���� �� ������ CLK
 dec	temp2								; �������� ����������� �����
 brne	MAX7219_LOOP8
 cbi	Port_7219_Clk,Pin_7219_Clk			; �������� ������ �� ����� Clk

 ldi	temp2,8								; �������� ������ ��������
MAX7219_LOOP16:								;
 cbi	Port_7219_Clk,Pin_7219_Clk			; c��������� ������ CLK 
 sbrc	data7219,7							; g�������� ������� ���
 sbi	Port_7219_Data,Pin_7219_Data		;  �������� ��������������
 sbrs	data7219,7							;  �������� � �����
 cbi	Port_7219_Data,Pin_7219_Data		;  ������� Data
 lsl	data7219							; c���� ����� (��������� ���)
 sbi	Port_7219_Clk,Pin_7219_Clk			; p����� ���� �� ������ CLK
 dec	temp2								; �������� ����������� �����
 brne	MAX7219_LOOP16
 cbi	Port_7219_Clk,Pin_7219_Clk			; �������� ������ �� ����� Clk

 sbi	Port_7219_CS,Pin_7219_CS			; ��������� ������ 7219 �������� CS=1

ret

;-----------------------------------------------------------------------------------
;	������� ���������� ����������� ������ �������� ����������
MAX7219_RAM:							; ������� ���������� ���������� Digit0...7

 ldi	adress7219,0x08						; ������� � 1 �������
 lds	data7219,Digit						; ��������� �������� 1 �������
 rcall	MAX7219_SEND

 ldi	adress7219,0x07						; ������� � 2 �������
 lds	data7219,Digit+1					; ��������� �������� 2 �������
 rcall	MAX7219_SEND

 ldi	adress7219,0x06						; ������� � 3 �������
 lds	data7219,Digit+2					; ��������� �������� 3 �������
 rcall	MAX7219_SEND

 ldi	adress7219,0x05						; ������� � 4 �������
 lds	data7219,Digit+3					; ��������� �������� 4 �������
 rcall	MAX7219_SEND

 ldi	adress7219,0x04						; ������� � 5 �������
 lds	data7219,Digit+4					; ��������� �������� 5 �������
 rcall	MAX7219_SEND

 ldi	adress7219,0x03						; ������� � 6 �������
 lds	data7219,Digit+5					; ��������� �������� 6 �������
 rcall	MAX7219_SEND

 ldi	adress7219,0x02						; ������� � 7 �������
 lds	data7219,Digit+6					; ��������� �������� 7 �������
 rcall	MAX7219_SEND

 ldi	adress7219,0x01						; ������� � 8 �������
 lds	data7219,Digit+7					; ��������� �������� 8 �������
 rcall	MAX7219_SEND

ret


;-----------------------------------------------------------------------------------
;	������������� ���������� �� MAX7219
MAX7219_Init:
 ldi	adress7219,0x0F						; ���� �����. �������� ��� ����������
 ldi	data7219,0x00						; ��������� �������� �����
 rcall	MAX7219_SEND

 ldi	adress7219,0x0C						; ����� ������ shutdown
 ldi	data7219,0x01						; ��������� ������ ����������
 rcall	MAX7219_SEND

 ldi	adress7219,0x0B						; ���������� ������������ ��������
 ldi	data7219,0x07						; ���������� ������ ��������
 rcall	MAX7219_SEND

 ldi	adress7219,0x09						; ������� ��������
 ldi	data7219,0b11111111					; ��������� �������
 rcall	MAX7219_SEND

 ldi	adress7219,0x0A						; �������� �������
 ldi	data7219,0x0F						; ������������ �������
 rcall	MAX7219_SEND

;-----------------------------------------------------------------------------------
;	������� ���������
MAX7219_Clear:
 ldi	adress7219,0x08
 ldi	data7219,15
 rcall	MAX7219_SEND

 ldi	adress7219,0x07
 ldi	data7219,15
 rcall	MAX7219_SEND

 ldi	adress7219,0x06
 ldi	data7219,15
 rcall	MAX7219_SEND

 ldi	adress7219,0x05
 ldi	data7219,15
 rcall	MAX7219_SEND

 ldi	adress7219,0x04
 ldi	data7219,15
 rcall	MAX7219_SEND

 ldi	adress7219,0x03
 ldi	data7219,15
 rcall	MAX7219_SEND

 ldi	adress7219,0x02
 ldi	data7219,15
 rcall	MAX7219_SEND

 ldi	adress7219,0x01
 ldi	data7219,15
 rcall	MAX7219_SEND

 ldi	temp,15
 sts	Digit,temp							; 
 ldi	temp,15
 sts	Digit+1,temp						; 
 ldi	temp,15
 sts	Digit+2,temp						; 
 ldi	temp,15
 sts	Digit+3,temp						; 
 ldi	temp,15
 sts	Digit+4,temp						; 
 ldi	temp,15
 sts	Digit+5,temp						; 
 ldi	temp,15
 sts	Digit+6,temp						; 
 ldi	temp,15
 sts	Digit+7,temp						; 

ret




;***********************************************************************************
;*                                                                                 *
;*                                                                                 *
;*            �������������� ��������� ����� � ����� ��� ����������                *
;*                                                                                 *
;*                                                                                 *
;***********************************************************************************

;   ���������� ����������: temp, temp1, adwL, adwH, mulL, mulH
                          
; Bin2ToBcd5
; ==========
; converts a 16-bit-binary to a 5-digit-BCD
; In: 16-bit-binary in adwH,adwL
; Out: 5-digit-BCD
; Used registers:temp
; Called subroutines: Bin2ToDigit
;
Bin2ToBCD5:
 ldi	temp,high(10000)					; ��������� 10000 ������
 mov	mulH,temp
 ldi	temp,low(10000)
 mov	mulL,temp
 rcall	Bin2ToDigit							; ���������, ��������� � ���������� temp
 sts	BCD,temp							; ���� �� ����, ����� ���������� ���������
Bin2ToBCD4:
 ldi	temp,high(1000)						; ��������� 1000 ������
 mov	mulH,temp
 ldi	temp,low(1000)
 mov	mulL,temp
 rcall	Bin2ToDigit							; ���������
 sts	BCD+1,temp							; ��������� � ���������� temp 
Bin2ToBCD3:
 clr	mulH								;  ��������� 100 ������
 ldi	temp,100
 mov	mulL,temp
 rcall	Bin2ToDigit							; ���������
 sts	BCD+2,temp							; ��������� � ���������� temp
Bin2ToBCD2:
 clr	mulH								; ��������� 10 ������
 ldi	temp,10
 mov	mulL,temp
 rcall	Bin2ToDigit							; ���������
 sts	BCD+4,adwL							; ������� �������� � adiw0
 sts	BCD+3,temp							; ��������� � ���������� temp
ret

; Bin2ToDigit
; ===========
; converts one decimal digit by continued subraction of a binary coded decimal
; Used by: Bin2ToBcd5
; In: 16-bit-binary in adw1,adw0, binary coded decimal in mul0,mul1
; Out: Result in temp
; Used registers: adiw0,adiw1, mul0,mul1, temp
; Called subroutines: -

Bin2ToDigit:
 clr	temp								; digit count is zero
Bin2ToDigita:
 cp		adwH,mulH							; Number bigger than decimal?
 brcs	Bin2ToDigitc						; MSB smaller than decimal
 brne	Bin2ToDigitb						; MSB bigger than decimal
 cp		adwL,mulL							; LSB bigger or equal decimal
 brcs	Bin2ToDigitc						; LSB smaller than decimal
Bin2ToDigitb:
 sub	adwL,mulL							; Subtract LSB decimal
 sbc	adwH,mulH							; Subtract MSB decimal
 inc	temp								; Increment digit count
 rjmp	Bin2ToDigita						; Next loop
Bin2ToDigitc:
ret											; done


;*************************************************************************************
;**																					**
;**																					**
;**             ��������� ���������� - ; 0x0001 - ������� ���������� 0              **
;**																					**
;**																					**
;*************************************************************************************
; enc_step            - �������� � ��������
; step_zoom           - ���������� �������� ��������
; reductorH:reductorL - ���������� �����, ������� �� ������� ���� ����������� ����


/*
aINT0:
 sbic	PIND,1									; ��������� ��������� ��������
 rjmp	ENC_Plus								; ������� ��������� ������
 sbis	PIND,1									; ��������� ��������� ��������
 rjmp	ENC_Minus								; ������� ��������� �����
;--------------------------------------------------------------------------------------------------
ENC_Plus:										; �������� ���� ��� �������� �������� ������
 brts	ENC_Plus_T1								; ��������� ���� �����. ���� ���� ����� ����, ����� ��������� �� ������ ENC_Plus_T1
 brtc	ENC_Plus_T0 							; ���� ���� ����� �����, �� ��������� �� ������ ENC_Plus_T0

ENC_Plus_T1:
 inc	enc_step
 cpse	step_zoom,enc_step						; ���������� ���������� ����� �������� � ���������� ���������
reti											; ���� �� �����, �� ������� �� ����������
 clr	enc_step								; ���� �����, ����� step_enc=0
 adiw	reductorH:reductorL,1					;                   reductor=reductor+1 
reti

ENC_Plus_T0:
 clt											; ������������� ���� ����� '-' 
 dec	enc_step								; step_enc=step_enc-1
 clr	temp
 cpse	enc_step,temp							; ���������, ���� step_enc=0
reti											; ���� �� ����� ����, �� ������� �� ����������
 set											; ���� ����� ����, ����� ������������� ���� '+' � ������� �� ����������
reti											
 
;--------------------------------------------------------------------------------------------------
ENC_Minus:										; �������� ���� ��� �������� �������� �����
 brts	ENC_Minus_T1							; ��������� ���� �����. ���� ���� ����� ����, ����� ��������� �� ������ ENC_Minus_T1
 brtc	ENC_Minus_T0 							; ���� ���� ����� �����, �� ��������� �� ������ ENC_Minus_T0
 
ENC_Minus_T1:
 clr	temp
 cpse	enc_step,temp							; ���������, ���� step_enc=0
 rjmp	ENC_Minus_T1_0							; ���� �� ����� ����, ����� ��������� �� ����� ENC_Minus_T1_0
 clt											; ���� ����� ����, ����� ����='-'
 inc	enc_step								;                        step_enc=step_enc+1
reti

ENC_Minus_T1_0:									; ���� �� ����� ����, ����� step_enc=step_enc-1
 dec	enc_step
reti
 
ENC_Minus_T0:
 inc	enc_step
 cpse	step_zoom,enc_step						; ���������� ���������� ����� �������� � ���������� ���������
reti											; ���� �� �����, �� ������� �� ����������
 clr	enc_step								; ���� �����, ����� step_enc=0
 sbiw	reductorH:reductorL,1					;                   reductor=reductor-1 
 set											;                   � ����='+'
reti

*/
aINT0:
 in		temp,SREG
 push	temp
 push	XL
 push	XH

 mov	XL,encoderL							; ����������� �������� ����� �������� � �������� ��� ���������� 
 mov	XH,encoderH

 sbic	PIND,3								; ��������� ��������� ��������
 rjmp	ENC_Plus							; ������� ��������� ������
 sbis	PIND,3								; ��������� ��������� ��������
 rjmp	ENC_Minus							; ������� ��������� �����


;--------------------------------------------------------------------------------------------------
ENC_Plus:									; �������� ���� ��� �������� �������� ������
 inc	enc_step
 adiw	XH:XL,1
 cpse	step_zoom,enc_step					; ���������� ���������� ����� �������� � ���������� ���������
 rjmp	INT0_EXIT							; ������� �� ����������

 sub	enc_step,step_zoom					; ���� �����, ����� step_enc = step_enc - step_zoom

 adiw	reductorH:reductorL,1				; ����������� ���������� ���������� �����. ���� ������ ������
  											;  ��� ���, ����� �������, ��� ������ ����������� � ����������� ����
rjmp	INT0_EXIT							; ������� �� ���������� � ��������������� �������� �� �����
; out	SREG,temp
; pop	temp
;reti


;--------------------------------------------------------------------------------------------------
ENC_Minus:									; �������� ���� ��� �������� �������� �����
 inc	enc_step
 sbiw	XH:XL,1
 cpse	step_zoom,enc_step					; ���������� ���������� ����� �������� � ���������� ���������
 rjmp	INT0_EXIT							; ������� �� ����������

 sub	enc_step,step_zoom					; ���� �����, ����� step_enc=0

 sbiw	reductorH:reductorL,1				; ����������� ���������� ���������� �����. ���� ������ ������
 											;  ��� ���, ����� �������, ��� ������ ����������� � ����������� ����
 rjmp	INT0_EXIT							; ������� �� ���������� � ��������������� �������� �� �����
; out	SREG,temp
; pop	temp
;reti


INT0_EXIT:
 mov	encoderL,XL							; ����������� ������� �������� ����� �������� � �������� �������� 
 mov	encoderH,XH
 
 pop	XH
 pop	XL
 out	SREG,temp
 pop	temp
reti										; ���� �� �����, �� ������� �� ����������



;************************************************************************************
;************************************************************************************
;**                                                                                **
;**                                                                                **
;**                 ������������ �������� ��� ������� ������ 8���                  **
;**                                                                                **
;**                                                                                **
;************************************************************************************
;************************************************************************************
;-------------------------------------------------------------------------
;	�������� �� 0,25�� (8Mhz), period  0,481 ms,  frequency 2 kHz

d025ms:
 ldi ZL,low(497)                            ; �������� � YH:YL ��������� 497
 ldi ZH,high(497)
d025_1:
 sbiw ZL,1                                  ; ��������� �� ����������� YH:YL �������
 brne d025_1                                 ; ���� ���� Z<>0 (��������� ����������
                                            ;  ���������� ������� �� ����� ����), ��
									        ;  ������� �� ����� d05_1
ret

;-------------------------------------------------------------------------
;	�������� �� 0,35�� (8Mhz), period   ms,  frequency  kHz

d035ms:
 ldi ZL,low(700)                            ; �������� � YH:YL ���������
 ldi ZH,high(700)
d035_1:
 sbiw ZL,1                                  ; ��������� �� ����������� YH:YL �������
 brne d035_1                                 ; ���� ���� Z<>0 (��������� ����������
                                            ;  ���������� ������� �� ����� ����), ��
									        ;  ������� �� ����� d05_1
ret





;-------------------------------------------------------------------------
;	�������� �� 0,5�� (8Mhz), period 0,999 ms,  frequency 1 kHz

d05ms:
 ldi ZL,low(1035)                            ; �������� � YH:YL ��������� 497
 ldi ZH,high(1035)
d05_1:
 sbiw ZL,1                                  ; ��������� �� ����������� YH:YL �������
 brne d05_1                                 ; ���� ���� Z<>0 (��������� ����������
                                            ;  ���������� ������� �� ����� ����), ��
									        ;  ������� �� ����� d05_1
ret


;-------------------------------------------------------------------------
;	�������� 1 ms (8MHz), period 2 ms, frequency 479,35 Hz
d1ms:  
 ldi temp,12
m1ms:
 ldi temp1,240
m11ms:
 dec temp1
 brne m11ms
 dec temp
 brne m1ms
ret
;-------------------------------------------------------------------------
;	�������� 3 ms (8MHz), period 6 ms, frequency 164,3 Hz
d3ms:  
 ldi temp,35
m:
 ldi temp1,240
m1:
 dec temp1
 brne m1
 dec temp
 brne m
ret

;-------------------------------------------------------------------------
;	�������� 10 ms (8Mhz), period 20,2 ms,  frequency 49 Hz
d10ms:  
 ldi temp,110
m10:
 ldi temp1,250
m11:
 dec temp1
 brne m11
 dec temp
 brne m10
ret

;-----------------------------------------------------------------------------------
;	�������� 37 ms (8Mhz), period 74 ms,  frequency 13,51 Hz
d37ms:
 ldi XL,low(154)                            ; �������� � YH:YL ��������� 100
 ldi XH,high(154)
d37_1:
 rcall d025ms                               ; ����� ������������ �������� �� 0,25��
 sbiw XL,1                                  ; ��������� ������� �� ����������� XH:XL
 brne d37_1                                 ; ���� ��������� �� ����� ����, �������
                                            ; �� ����� d50_1
ret



;-------------------------------------------------------------------------
;	�������� �� 50 ms (8Mhz), period  0,1 s,  frequency 10 Hz
d50ms:
 ldi XL,low(208)                            ; �������� � YH:YL ��������� 100
 ldi XH,high(208)
d50_1:
 rcall d025ms                               ; ����� ������������ �������� �� 0,25��
 sbiw XL,1                                  ; ��������� ������� �� ����������� XH:XL
 brne d50_1                                 ; ���� ��������� �� ����� ����, �������
                                            ; �� ����� d50_1
ret


;-------------------------------------------------------------------------
;	�������� �� 67 ms (8Mhz), period  0,1342 s,  frequency  7,452 Hz
d67ms:
 ldi XL,low(279)                            ; �������� � YH:YL ��������� 100
 ldi XH,high(279)
d67_1:
 rcall d025ms                               ; ����� ������������ �������� �� 0,25��
 sbiw XL,1                                  ; ��������� ������� �� ����������� XH:XL
 brne d67_1                                 ; ���� ��������� �� ����� ����, �������
                                            ; �� ����� d50_1
ret




;-------------------------------------------------------------------------
;	�������� �� 90ms (8Mhz), period 0,1804 s,  frequency  5,55 Hz
d90ms:
 ldi XL,low(375)                            ; �������� � YH:YL ��������� 200
 ldi XH,high(375)
d90_1:
 rcall d025ms                               ; ����� ������������ �������� �� 0,25��
 sbiw XL,1                                  ; ��������� ������� �� ����������� XH:XL
 brne d90_1                                ; ���� ��������� �� ����� ����, �������
                                            ; �� ����� d100_1
ret



;-------------------------------------------------------------------------
;	�������� �� 100ms (8Mhz), period 0,2 s,  frequency 5 Hz
d100ms:
 ldi XL,low(416)                            ; �������� � YH:YL ��������� 200
 ldi XH,high(416)
d100_1:
 rcall d025ms                               ; ����� ������������ �������� �� 0,25��
 sbiw XL,1                                  ; ��������� ������� �� ����������� XH:XL
 brne d100_1                                ; ���� ��������� �� ����� ����, �������
                                            ; �� ����� d100_1
ret



;-------------------------------------------------------------------------
;	�������� �� 500��
d500ms:
 ldi XL,low(2000)                            ; �������� � YH:YL ��������� 200
 ldi XH,high(2000)

d500_1:
 rcall d025ms                               ; ����� ������������ �������� �� 0,25��
 sbiw XL,1                                  ; ��������� ������� �� ����������� XH:XL
 brne d500_1                                ; ���� ��������� �� ����� ����, �������
                                            ; �� ����� d100_1

ret





;***************************************************************************************************

.exit                                       ; ����� ���������






PRINT_HOD:
;	��������� ������ ��������� ��������� �����. ���� ��������, ����� ���� �� �������
 sbic	Pin_stol_end,N_stol_end				; ���� �� ��������� ������ ��������� ��������� �����, ���� ����� ���������
 rjmp	DVIGATEL_STOP						;  �������� ������, ������� ���� �� ������� � ��������� ��������� 

;	��������� ������ �� ����������
 sbis	Pin_but_home,N_but_home				; ��������� ��������� ������ "HOME"
 rjmp	HOME								; ��� �������� � ��������� "HOME" ��� ������
 sbis	Pin_but_reverse,N_but_reverse		; ��������� ��������� ������ "Reverse"
 rjmp	REVERSE

;--------------------------------------------
 cpi	reductorL,0							; count_enc �������������� � ����������
 clr	temp								; ���������� ���������� ���������� ����� � ����������
 cpc	reductorH,temp						; ���������� ������� ����� � ������ ��������
 brsh	PRINT_WAIT							; ���� ����� �� ����� ������, ����� ����������� ������ � ����������� ����



 cbi	Port_stol_dir,N_stol_dir			; ������ ��������� ����� � ������ �����������
 cbi	Port_stol_en,N_stol_en				; ���������� �������
 cbi	Port_stol_step,N_stol_step			; ������ ���� ���
 rcall	d05ms
 sbi	Port_stol_step,N_stol_step			; ������ ���
 rcall	d035ms	




PRINT_WAIT:
 mov	adwH,reductorH
 mov	adwL,reductorL
 rcall	Bin2ToBCD5
 lds	temp,BCD+4
 sts	Digit+6,temp
 lds	temp,BCD+3
 sts	Digit+5,temp
 lds	temp,BCD+2
 sts	Digit+4,temp
 lds	temp,BCD+1
 sts	Digit+3,temp
 lds	temp,BCD
 sts	Digit+2,temp
 rcall	MAX7219_RAM							; ��������� ���������� �� ����������
 
 cbi	Port_R,N_R							; ��������� ������� ����
 sbi	Port_G,N_G							; �������� ������� ����
 cbi	Port_B,N_B							; ��������� ����� ����

rjmp	PRINT_HOD

DVIGATEL_STOP:
 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; ��������� �������

;--------------------------------------------
PRINT_END:
 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; ��������� �������

 sbi	Port_R,N_R							; ��������� ������� ����
 cbi	Port_G,N_G							; ��������� ������� ����
 cbi	Port_B,N_B							; �������� ����� ����

 rjmp	REVERSE_HOD


