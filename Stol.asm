;*****************************************************************************************************
;**																									**
;**                    Программа для управления столом при печати принтера							**
;**											PUBLICIS												**
;**                         тип МК ATmega162 Тактовая частота 8.00 МГ								**
;**																									**
;*****************************************************************************************************
/*  Fuse Bits:
EXTENDED 0xFF
HICH     0xD8
LOW      0xC2
*/

.include "m162def.inc"
 
    
;	определяем регистры переменных

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
.def	adress7219=r16						; номер позиции индикатора
.def	temp=r17
.def	temp1=r18
.def	temp2=r19
.def	count=r20
.def	data7219=r21						; выводимое число на индикатор
.def	count_ASF_motor=r22					; счетчик шагов двигателя захвата (ASF Motor)
.def	step_motorL=r23						; счетчик шагов двигателя вращения стола
.def	step_motorH=r24
.def	enc_step=r25

;.def	=r26								; XL
;.def	=r27								; XH
.def	reductorL=r28						; YL счетчик сигналов с енкодера вала принтера
.def	reductorH=r29						; YH
;.def	=r30								; ZL
;.def	=r31								; ZH


;------------------------------------------------------------------------------
;   Порт и выводы для индикации на MAX7219CWG
.equ	Port_7219_Data=PORTB				; Вывод DATA
.equ	Pin_7219_Data=7
.equ	Port_7219_Clk=PORTB					; Вывод Clk
.equ	Pin_7219_Clk=5
.equ	Port_7219_CS=PORTB					; Вывод CS
.equ	Pin_7219_CS=6

;--------------------------------------------------------------------------------------------------
;   Входные сигналы

;	оптопары перемещения стола по горизонтали (начало и конец стола)
.equ	Pin_stol_home=PINE					; оптопара начального положения стола
.equ	N_stol_home=2
.equ	Pin_stol_end=PINC					; оптопара конечного положения стола
.equ	N_stol_end=7

;	входы от фототранзистора лазера
.equ	Pin_laser_in=PINE					; фототранзистор лазера
.equ	N_laser_in=1

;	входы от кнопок
.equ	Pin_power=PINC						; кнопка включения/выключения принтера
.equ	N_power=2							; "PINC0"
.equ	Pin_but_home=PIND					; кнопка "Home"
.equ	N_but_home=6
.equ	Pin_but_forward=PINC				; кнопка "Forward"
.equ	N_but_forward=0						; "PINC4"
.equ	Pin_but_reverse=PINC				; кнопка "Revers"
.equ	N_but_reverse=1						; "PINC5"

;	вход от индикаторов клавиатуры принтера "INK" и "PAPER"
.equ	Pin_led_paper=PINC					; вход от светодиода, индикатор бумаги
.equ	N_led_paper=4
.equ	Pin_led_ink=PINC					; вход от светодиода, сигнализируещего окончания чернил
.equ	N_led_ink=5


;	вход от эмулятора захвата
.equ	Pin_pickup_ready=PIND				; сигнал готовности от эмулятора захвата
.equ	N_pickup_ready=5

;--------------------------------------------------------------------------------------------------
;	Выходы на исполнительные устройства

;	управление лазером определения уровня носителя
.equ	Port_laser_out=PORTE				; управление лазером
.equ	N_laser_out=0

;	управление пищалкой
.equ	Port_zumer=PORTA					; управление пищалкой
.equ	N_zumer=1


;	управление драйвером двигателя премещением стола в горизонтальном положении
.equ	Port_stol_step=PORTB				; Step сигнал драйвера мотора движением стола
.equ	N_stol_step=1
.equ	Port_stol_dir=PORTB					; Direction сигнал драйвера мотора движением стола
.equ	N_stol_dir=0
.equ	Port_stol_en=PORTA					; Enable сигнал драйвера мотора движением стола
.equ	N_stol_en=0
.equ	Port_stol_SW1=PORTB					; SW1 сигнал управлением скоростью движения стола
.equ	N_stol_SW1=2
.equ	Port_stol_SW2=PORTB					; SW2 сигнал управлением скоростью движения стола
.equ	N_stol_SW2=3
.equ	Port_stol_SW3=PORTB					; SW3 сигнал управлением скоростью движения стола
.equ	N_stol_SW3=4

;	выходы для эмулирования работы захвата
.equ	Port_PE_enable=PORTD				; сигнал эмуляции наличии бумаги при печати
.equ	N_PE_enable=4

.equ	Port_paper=PORTD					; сигнал подтверждения бумаги
.equ	N_paper=7

.equ	Port_pause=PORTC					; сигнал паузы при печати
.equ	N_pause=6

;	дополнительные выходы
.equ	Port_L1=PORTA						; дополнительный выход L1
.equ	N_L1=5
.equ	Port_L2=PORTA						; дополнительный выход L2
.equ	N_L2=6
.equ	Port_L3=PORTA						; дополнительный выход L3
.equ	N_L3=7

;	выходы на RGB
.equ	Port_R=PORTA						; выход на цвет
.equ	N_R=2
.equ	Port_G=PORTA						; выход на цвет
.equ	N_G=3
.equ	Port_B=PORTA						; выход на цвет
.equ	N_B=4


;------------------------------------------------------------------------------ 
    ; это сегмент данных. В нем выделяется оперативная память

.dseg

Digit:			.byte 8
BCD:			.byte 5



.cseg

; ***** INTERRUPT VECTORS ************************************************

.org $000
     jmp RESET                              ; Обработка сброса
.org $002
     jmp aINT0                              ;$002 - Внешнее прерывание 0
.org $004
     jmp aINT1                              ;$004 - Внешнее прерывание 1
.org $006
     jmp aOC2                               ;$006 - Совпадение таймера/счетчик Т2
.org $008
     jmp aOVF2                              ;$008 - Переполнение таймера/счетчик Т2
.org $00A
     jmp aICP1                              ;$00A - Захват таймера/счетчик Т1 
.org $00C
     jmp aOC1A                              ;$00C - Совпадение А таймера/счетчик Т1
.org $00E
     jmp aOC1B                              ;$00E - Совпадение В таймера/счетчик Т1
.org $0010
     jmp aOVF1                              ;$010 - Переполнение таймера/счетчик Т1
.org $0012
     jmp aOVF0                              ;$012 - Переполнение таймера/счетчик Т0 
.org $0014
     jmp aSPI                               ;$014 - Передача по SPI завершена
.org $0016
     jmp aURXC                              ;$016 - UART прием завершен
.org $0018
     jmp aUDRE                              ;$018 - Регистр данных UART пуст 
.org $001A
     jmp aUTXC                              ;$01A - UART передача завершена
.org $001C
     jmp aADCC                              ;$01C - Преобразование ADC завершено
.org $001E
     jmp aERDY                              ;$01E - EEPROM готов
.org $0020
     jmp aACI                               ;$020 - Аналоговый компаратор
.org $0022
     jmp aTWI                               ;$022 - Прерывание от модуля Two-Wire 
.org $0024
     jmp aINT2                              ;$024 - Внешнее прерывание
.org $0026
     jmp aOC0                               ;$026 - Совпадение таймера/счетчик Т0
.org $0028
     jmp aSPMR                              ;$028 - ГотовностьSPM

;************************************************************************;
;************************************************************************;
;**                                                                    **;
;*               При включении питания мы попадаем сюда                 *;
;*                                                                      *;
;**                                                                    **;
;************************************************************************;
;************************************************************************;

RESET:										; Обработка сброса
;aINT0:										;$002 - Внешнее прерывание 0
aINT1:										;$004 - Внешнее прерывание 1
 aOC2:										;$006 - Совпадение таймера/счетчик Т2
aOVF2:										;$008 - Переполнение таймера/счетчик Т2
aICP1:										;$00A - Захват таймера/счетчик Т1 
aOC1A:										;$00C - Совпадение А таймера/счетчик Т1
aOC1B:										;$00E - Совпадение В таймера/счетчик Т1
aOVF1:										;$010 - Переполнение таймера/счетчик Т1
aOVF0:										;$012 - Переполнение таймера/счетчик Т0
 aSPI:										;$014 - Передача по SPI завершена
aURXC:										;$016 - UART прием завершен
aUDRE:										;$018 - Регистр данных UART пуст
aUTXC:										;$01A - UART передача завершена
aADCC:										;$01C - Преобразование ADC завершено
aERDY:										;$01E - EEPROM готов
 aACI:										;$020 - Аналоговый компаратор
 aTWI:										;$022 - Прерывание от модуля Two-Wire
aINT2:										;$024 - Внешнее прерывание
 aOC0:										;$026 - Совпадение таймера/счетчик Т0
aSPMR:										;$028 - ГотовностьSPM



	; Инициализация стека 
 ldi 	temp, LOW(RAMEND)
 out	spl, Temp
 ldi 	temp, HIGH(RAMEND)
 out	sph, Temp



;	Инициализация портов

; определяем направление портов
 ldi	temp,0b11111111						; запись сигналов на вывод порта A
 out	DDRA,temp       

 ldi	temp,0b11111111						; запись сигналов на вывод порта B
 out	DDRB,temp       

 ldi	temp,0b01001000						; запись сигналов на вывод порта C
 out	DDRC,temp       

 ldi	temp,0b10010001						; запись сигналов на вывод порта D
 out	DDRD,temp       

 ldi	temp,0b00000001						; запись сигналов на вывод порта D
 out	DDRE,temp       

; устанавливаем исходные значения портов
 ldi	temp,0b00000000						; инициализация порта A
 out	PORTA,temp 

 ldi	temp,0b00000000						; инициализация порта B
 out	PORTB,temp 

 ldi	temp,0b00110000						; инициализация порта C
 out	PORTC,temp

 ldi	temp,0b00000000						; инициализация порта D
 out	PORTD,temp
 
 ldi	temp,0b00000000						; инициализация порта D
 out	PORTE,temp
        

;------------------------------------------------------------------------------
; Разрешаем и настраиваем внешнее прерывание по нарастающему фронту сигнала на входе INT0
 ldi	temp,0b01000000
 	; 7 - INT1
	; 6 - INT0
	; 5 - INT2
	; 4 - PCIE1
	; 3 - PCIE0
	; 2 -
	; 1 - IVSEL
	; 0 - IVCE
 out	GICR,temp								; разрешаем внешнее прерывание INT0

 ldi	temp,0b00000011
	; 7 - SRE
	; 6 - SRW10
	; 5 - SE
	; 4 - SM1
	; 3 - ISC11
	; 2 - ISC10
	; +1 - ISC01
	; +0 - ISC00
 out	MCUCR,temp								; по нарастающему фронту сигнала на выводе INT0


;****************************************************************************************************
;*																									*
;*																									*
;*                Подготовка периферии и инициализация переменных программы							*
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
 rcall	MAX7219_Init						; инициадизируем индикатор
 ldi	temp,8
 sts	Digit,temp							; выводим номер шага инициализации на индикатор 
 ldi	temp,8
 sts	Digit+1,temp						; выводим номер шага инициализации на индикатор 
 ldi	temp,8
 sts	Digit+2,temp						; выводим номер шага инициализации на индикатор 
 ldi	temp,8
 sts	Digit+3,temp						; выводим номер шага инициализации на индикатор 
 ldi	temp,8
 sts	Digit+4,temp						; выводим номер шага инициализации на индикатор 
 ldi	temp,8
 sts	Digit+5,temp						; выводим номер шага инициализации на индикатор 
 ldi	temp,8
 sts	Digit+6,temp						; выводим номер шага инициализации на индикатор 
 ldi	temp,8
 sts	Digit+7,temp						; выводим номер шага инициализации на индикатор 
 rcall	MAX7219_RAM							;
 rcall	d500ms
 rcall	d500ms
 rcall	MAX7219_CLEAR						; очищаем индикатор

;**************************************************************************************************
;	устанавливаем делитель для енкодера и делитель шага для драйвера двигателя
 ldi	temp,4								; делитель для енкодера
 mov	step_zoom,temp

; Micro step = 4 (pulse=800)
 sbi	Port_stol_SW1,N_stol_SW1			; SW1 - on
 cbi	Port_stol_SW2,N_stol_SW2			; SW2 - off
 cbi	Port_stol_SW3,N_stol_SW3			; SW3 - off
;**************************************************************************************************

 sbi	Port_stol_en,N_stol_en				; отключаем драйвер
 cbi	Port_PE_enable,N_PE_enable			; сбрасываем датчик наличия бумаги

 cbi	Port_R,N_R							; отключаем красный цвет
 cbi	Port_G,N_G							; отключаем зеленый цвет
 sbi	Port_B,N_B							; включаем синий цвет

 cbi	Port_zumer,N_zumer					; отключаем пищалку

 sei										; разрешаем прерывание для считывания сигналов с енкодера



;*****************************************************************************************************

 ldi	temp,1								; выводим 1 1 1 1 на индикатор, программа в основном меню 
 sts	Digit,temp
 sts	Digit+2,temp
 sts	Digit+4,temp
 sts	Digit+6,temp
 sts	Digit,temp
 rcall	MAX7219_RAM							; обновляем информацию на индикаторе

;--------------------------------------------
;	убираем стол из под головки, что бы не было ошибки глаза
;rjmp	MAIN_LOOP							; для ТЕСТА

;	двигаем стол в начальное положение для устранения ошибки глаза
 sbi	Port_zumer,N_zumer					; включаем пищалку
 sbi	Port_R,N_R							; включаем красный цвет
 cbi	Port_G,N_G							; отключаем зеленый цвет
 cbi	Port_B,N_B							; отлючаем синий цвет
 rcall	d500ms

MOVE_HOME:
 rcall	MAX7219_SEND						; обновляем информацию на индикаторе

;	вращаем двигатель стола в обратном направлении
 sbi	Port_stol_dir,N_stol_dir			; крутим двигатель стола в обратном направлении
 cbi	Port_stol_en,N_stol_en				; активируем драйвер
 cbi	Port_stol_step,N_stol_step			; делаем один шаг
 rcall	d05ms
 sbi	Port_stol_step,N_stol_step
 rcall	d035ms
;	проверяем датчики начального и конечного положения стола. если сработал, тогда стол не двигаем
 sbic	Pin_stol_home,N_stol_home			; пока не сработает датчик начального положения стола, стол будет двигаться
 rjmp	MOVE_END							;  сработал датчик, поэтому стол не двигаем и отключаем двигатель
 rjmp	MOVE_HOME							; если датчики не сработали, тогда двигаем стол дальше к началу

;--------------------------------------------
;	двигаем на 10 шагов вперед
MOVE_END:
 rcall	d50ms
 ldi	count,10							; двигаем стол вперед на 10 шагов, что бы активировать датчик начало стола
 rcall	d50ms
MOVE_END_10:
 rcall	MAX7219_SEND
 cbi	Port_stol_dir,N_stol_dir			; крутим двигатель стола в прямом направлении
 cbi	Port_stol_en,N_stol_en				; активируем драйвер
 cbi	Port_stol_step,N_stol_step			; делаем один шаг
 rcall	d10ms				
 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; отключаем драйвер
 dec	count								; уменьшаем счетчик циклов
 brne	MOVE_END_10							; когда сделали все шаги, заканциваем цикл и выходим

 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; отключаем драйвер двигателя

 sbi	Port_R,N_R							; включаем красный цвет
 cbi	Port_G,N_G							; отключаем зеленый цвет
 cbi	Port_B,N_B							; отключаем синий цвет

 cbi	Port_zumer,N_zumer					; отключаем пищалку

 ldi	temp,2								; выводим 2 2 2 2 2 на индикатор
 sts	Digit,temp
 sts	Digit+2,temp
 sts	Digit+4,temp
 sts	Digit+6,temp
 rcall	MAX7219_RAM							; обновляем информацию на индикаторе


;--------------------------------------------
;	Ждем инициализации захвата
;rjmp	MAIN_LOOP							; для теста

PICKUP_WAIT_OFF:							; ждем инициализации эмулятора захвата
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
;	убираем стол в "Home" для устранения ошибки замятия бумаги, если выскачит при инициализации
 ldi	count,40							; загружаем время таймера
ERROR_PAPER:
 rcall	d500ms								; вызываем задержку 0,5 с
 dec	count								;  и считаем время
 brne	ERROR_PAPER							; если не достигли, то считаем дальше
 rjmp	HOME_SKIP							; если досчитали до конца таймера, то еще раз двигаем стол в "Home"

 cbi	Port_R,N_R							; отключаем красный цвет
 cbi	Port_G,N_G							; отключаем зеленый цвет
 sbi	Port_B,N_B							; включаем синий цвет


;*********************************************************************************************************
;*********************************************************************************************************
;**																										**
;**																										**
;**                                     ОСНОВНОЙ ЦИКЛ ПРОГРАММЫ											**
;**																										**
;**																										**
;*********************************************************************************************************
;*********************************************************************************************************

MAIN_LOOP:
 rcall	MAX7219_CLEAR						; очищаем индикатор
 ldi	temp,0								; выводим цифру 0 на индикатор (главный цикл)
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
 rcall	MAX7219_RAM							; обновляем информацию на индикаторе

MAIN_LOOP1:
 sbis	Pin_but_home,N_but_home				; проверяем состояние кнопки "HOME"
 rjmp	HOME								;  если нажата, тогда двигаем стол в позицию печати

 sbis	Pin_but_reverse,N_but_reverse		; проверяем состояние кнопки "Reverse"
 rjmp	REVERSE								;  если нажата, тогда выдвигаем стол к юзеру

; rjmp	PRINT_SKIP
 sbic	Pin_pickup_ready,N_pickup_ready		; проверяем сигнал захвата бумаги
 rjmp	PRINT								;  если принтер захватил бумагу, значит печатаем

 cbi	Port_R,N_R							; отключаем красный цвет
 cbi	Port_G,N_G							; отключаем зеленый цвет
 sbi	Port_B,N_B							; включаем синий цвет

 rcall	d10ms

rjmp	MAIN_LOOP1							; повторяем основной цикл безконечно









;*****************************************************************************************************
;*****************************************************************************************************
;**																									**
;**																									**
;**                                            Процедуры 											**
;**																									**
;**																									**
;*****************************************************************************************************
; step_motorH:step_motorL - шаги, которые прошел шаговый двигатель
; reductorH:reductorL     - шаги, которые необходимо пройти

PRINT:
 rcall	d1ms								; устраняем дребезг контактов
 sbis	Pin_pickup_ready,N_pickup_ready			
 rjmp	MAIN_LOOP							; если ложное срабатывание, тогда переходим в главный цикл программы

; Micro step = 4 (pulse=800)
 sbi	Port_stol_SW1,N_stol_SW1			; SW1 - on
 cbi	Port_stol_SW2,N_stol_SW2			; SW2 - off
 cbi	Port_stol_SW3,N_stol_SW3			; SW3 - off

PRINT_SKIP:
 rcall	MAX7219_CLEAR						; очищаем индикатор
 ldi	temp,3								; выводим цифру 3 на индикатор (режим печати)
 sts	Digit,temp
 rcall	MAX7219_RAM							; обновляем информацию на индикаторе

 clr	encoderL
 clr	encoderH
 clr	reductorL
 clr	reductorH
 clr	step_motorL
 clr	step_motorH
 clr	enc_step

 cbi	Port_R,N_R							; отключаем красный цвет
 sbi	Port_G,N_G							; включаем зеленый цвет
 cbi	Port_B,N_B							; отключаем синий цвет

 sbi	Port_PE_enable,N_PE_enable			; активируем датчик наличия бумаги во время движения стола
 sbi	Port_L3,N_L3

;********************************************
;********************************************
;	непосредственно процесс печати
PRINT_HOD:
;--------------------------------------------
;	проверяем датчик конечного положения стола. если сработал, тогда стол не двигаем
 sbic	Pin_stol_end,N_stol_end				; пока не сработает датчик конечного положения стола, стол будет двигаться
 rjmp	DVIGATEL_STOP						;  сработал датчик, поэтому стол не двигаем и отключаем двигатель 
;	проверяем кнопки на клавиатуре
 sbis	Pin_but_home,N_but_home				; проверяем состояние кнопки "HOME"
 rjmp	HOME								; для возврата в положение "HOME" при печати
 sbis	Pin_but_reverse,N_but_reverse		; проверяем состояние кнопки "Reverse"
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
 rcall	MAX7219_RAM							; обновляем информацию на индикаторе
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
 rcall	MAX7219_RAM							; обновляем информацию на индикаторе

;	проверяем значения счетчика шагов двигателя и энкодера. если они равны, тогда двигатель не крутится.
;	т.е. мы шагаем тогда, когда была движение диска энкодера (оси привода бумаги)
COMPARE_STEP:
 cp		reductorL,step_motorL				; сравниваем младшую часть числа
 cpc	reductorH,step_motorH				; сравниваем старшую часть числа с учетом переноса
 breq	PRINT_HOD							; если содержимое регистров равно, тогда не двигаем стол


 cp		reductorL,step_motorL				; сравниваем младшую часть числа
 cpc	reductorH,step_motorH				; сравниваем старшую часть числа с учетом переноса
 brpl	STEP_FORWARD						; переход по положительное значение

 cp		reductorL,step_motorL				; сравниваем младшую часть числа
 cpc	reductorH,step_motorH				; сравниваем старшую часть числа с учетом переноса
 brmi	STEP_REVERSE						; переход по отрицательное значение


STEP_REVERSE:								; крутим двигатель в обратном направлении
 rcall	d035ms
 sbi	Port_stol_dir,N_stol_dir			; крутим двигатель стола в обратном направлении
 cbi	Port_stol_en,N_stol_en				; активируем драйвер
 cbi	Port_stol_step,N_stol_step			; делаем один шаг
 rcall	d05ms
 sbi	Port_stol_step,N_stol_step			; делаем шаг
 rcall	d05ms	

 mov	ZL,step_motorL						; занимаемся перекидыванием значений
 mov	ZH,step_motorH
 sbiw	ZH:ZL,1								; увеличиваем счетчик пройденных шагов на 1
 mov	step_motorL,ZL						; кидаем значение обратно
 mov	step_motorH,ZH

rjmp	PRINT_HOD


STEP_FORWARD:								; крутим двигатель в прямом направлении
 cbi	Port_stol_dir,N_stol_dir			; крутим двигатель стола в прямом направлении
 cbi	Port_stol_en,N_stol_en				; активируем драйвер
 cbi	Port_stol_step,N_stol_step			; делаем один шаг
 rcall	d05ms
 sbi	Port_stol_step,N_stol_step			; делаем шаг
 rcall	d05ms	

 mov	ZL,step_motorL						; занимаемся перекидыванием значений
 mov	ZH,step_motorH
 adiw	ZH:ZL,1								; увеличиваем счетчик пройденных шагов на 1
 mov	step_motorL,ZL						; кидаем значение обратно
 mov	step_motorH,ZH

rjmp	PRINT_HOD


 cbi	Port_R,N_R							; отключаем красный цвет
 sbi	Port_G,N_G							; включаем зеленый цвет
 cbi	Port_B,N_B							; отключаем синий цвет

rjmp	PRINT_HOD

DVIGATEL_STOP:
 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; отключаем драйвер

 sbi	Port_R,N_R							; вкключаем красный цвет
 cbi	Port_G,N_G							; отключаем зеленый цвет
 cbi	Port_B,N_B							; отключаем синий цвет
 
 rcall	d500ms
 rcall	d500ms
 rcall	d500ms
 rcall	d500ms

PRINT_END:
 rcall	d10ms
;	проверяем кнопки на клавиатуре
 sbis	Pin_but_home,N_but_home				; проверяем состояние кнопки "HOME"
 rjmp	HOME								; для возврата в положение "HOME" при печати
 sbis	Pin_but_reverse,N_but_reverse		; проверяем состояние кнопки "Reverse"
 rjmp	REVERSE


; rjmp	HOME_SKIP							; автовозврат стола
 rjmp	PRINT_END



;****************************************************************************************************
;	Нажата кнопка "HOME". Двигаем стол в начальное для печати положение
;****************************************************************************************************
HOME:
 rcall	d1ms								; устраняем дребезг контактов
 sbic	Pin_but_home,N_but_home			
 rjmp	MAIN_LOOP							; если ложное срабатывание, тогда переходим в главный цикл программы

HOME_SKIP:

 clr	reductorH
 clr	reductorL

 cbi	Port_R,N_R							; отключаем красный цвет
 sbi	Port_G,N_G							; включаем зеленый цвет
 cbi	Port_B,N_B							; отключаем синий цвет

 rcall	MAX7219_CLEAR						; очищаем индикатор
 ldi	temp,1								; выводим цифру 1 на индикатор (движение стола к началу печати)
 sts	Digit,temp

;--------------------------------------------
HOME_GO:									; двигаем стол на начало
;	выводим значение шага на индикатор
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
 rcall	MAX7219_RAM							; обновляем информацию на индикаторе

SPEED_HOME1:								; в начале скорость движения стола максимальная
;	вращаем двигатель стола в обратном направлении
 sbi	Port_stol_dir,N_stol_dir			; крутим двигатель стола в обратном направлении
 cbi	Port_stol_en,N_stol_en				; активируем драйвер
 cbi	Port_stol_step,N_stol_step			; делаем один шаг
 rcall	d035ms
 sbi	Port_stol_step,N_stol_step
 rcall	d035ms
 adiw	reductorH:reductorL,1				; прибавляем шаг
;	проверяем датчики начального и конечного положения стола. если сработал, тогда стол не двигаем
 sbic	Pin_stol_home,N_stol_home			; пока не сработает датчик начального положения стола, стол будет двигаться
 rjmp	HOME_END							;  сработал датчик, поэтому стол не двигаем и отключаем двигатель
; sbic	Pin_stol_end,N_stol_end				; пока не сработает датчик конечного положения стола, стол будет двигаться
; rjmp	HOME_END							;  сработал датчик, поэтому стол не двигаем и отключаем двигатель 
 rjmp	HOME_GO								; если датчики не сработали, тогда двигаем стол дальше к началу

;--------------------------------------------
HOME_END:
 ldi	count,10							; двигаем стол вперед на 10 шагов, что бы активировать датчик начало стола
 rcall	d100ms
HOME_END_10:
 cbi	Port_stol_dir,N_stol_dir			; крутим двигатель стола в прямом направлении
 cbi	Port_stol_en,N_stol_en				; активируем драйвер
 cbi	Port_stol_step,N_stol_step			; делаем один шаг
 rcall	d10ms				
 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; отключаем драйвер
 dec	count								; уменьшаем счетчик циклов
 brne	HOME_END_10							; когда сделали все шаги, заканциваем цикл и выходим

 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; отключаем драйвер двигателя

 cbi	Port_PE_enable,N_PE_enable			; сбрасываем датчик наличия бумаги
 cbi	Port_L3,N_L3

;--------------------------------------------
;	проверяем состояние светодиода "Paper" перед подтверждением нажатия кнопки. Если он горит, тогда подтверждаем
;    нажатием кнопки загрузки бумаги. Если не горит, тогда не нажимаем кнопку "Paper"
 rcall	d025ms								; вызов подпрограммы задержки на 0,25мс
 ldi	YL,low(8000)						; загрузка в YH:YL константы
 ldi	YH,high(8000)

PAPER_PUSH:
 sbis	Pin_led_paper,N_led_paper			; проверяем состояние светодиода "Paper" принтера.
 rjmp	PUSH_BUTTON							; если он включился, значит нажимаем кнопку "Paper" на клавиатуре принтера
 
 rcall	d025ms								; вызов подпрограммы задержки на 0,25мс
 sbiw	YL,1								; вычитание единицы из содержимого YH:YL
 brne	PAPER_PUSH							; если результат не равен нулю, перейти в начало цикла

 sbi	Port_zumer,N_zumer					; пикаем пищалкой
 cbi	Port_R,N_R							; отключаем красный цвет
 cbi	Port_G,N_G							; отключаем зеленый цвет
 sbi	Port_B,N_B							; включаем синий цвет
 rcall	d500ms
 cbi	Port_zumer,N_zumer					; отключаем пищалку

 rjmp	PUSH_BUTTON_END						; переходим в главный цикл НЕ НАЖИМАЯ кнопки подтверждения

;--------------------------------------------
PUSH_BUTTON:
 sbi	Port_paper,N_paper					; нажимаем кнопку "paper"
 rcall	d500ms								; выполняем задержку после останова стола
 cbi	Port_paper,N_paper					; отпускаем кнопку "paper"

 sbi	Port_zumer,N_zumer					; пикаем пищалкой
 cbi	Port_R,N_R							; отключаем красный цвет
 cbi	Port_G,N_G							; отключаем зеленый цвет
 sbi	Port_B,N_B							; включаем синий цвет
 rcall	d500ms
 cbi	Port_zumer,N_zumer					; отключаем пищалку

;--------------------------------------------
PUSH_BUTTON_END:

rjmp	MAIN_LOOP							; переходим в главный цикл НАЖИМАЯ кнопку подтверждения







;*****************************************************************************************************
;	При нажатии кнопки "Reverse" на клавиатуре принтера выбрасываем стол к юзеру с включенной зеленой
;    подсветкой до конца перемещения стола
;*****************************************************************************************************
REVERSE:
 rcall	d1ms								; устраняем дребезг контактов
 sbic	Pin_but_reverse,N_but_reverse			
 rjmp	MAIN_LOOP							; если ложное срабатывание, тогда переходим в главный цикл программы

 clr	reductorH
 clr	reductorL

 rcall	MAX7219_CLEAR						; очищаем индикатор
 ldi	temp,2								; выводим цифру 2 на индикатор (движение стола к юзеру)
 sts	Digit,temp
 rcall	MAX7219_RAM							; обновляем информацию на индикаторе

REVERSE_HOD:
 cbi	Port_R,N_R							; отключаем красный цвет
 sbi	Port_G,N_G							; включаем зеленый цвет
 cbi	Port_B,N_B							; отключаем синий цвет

;	вращаем двигатель стола в обратном направлении
 cbi	Port_stol_dir,N_stol_dir			; крутим двигатель стола в прямом направлении
 cbi	Port_stol_en,N_stol_en				; активируем драйвер
 cbi	Port_stol_step,N_stol_step			; делаем один шаг
 rcall	d035ms
 sbi	Port_stol_step,N_stol_step
 rcall	d035ms

 adiw	reductorH:reductorL,1

;	выводим значение шага на индикатор
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
 rcall	MAX7219_RAM							; обновляем информацию на индикаторе

 sbic	Pin_stol_end,N_stol_end				; пока не сработает датчик конечного положения стола, стол будет двигаться
 rjmp	REVERSE_END							;  сработал датчик, поэтому стол не двигаем и отключаем двигатель 

 rjmp	REVERSE_HOD


;--------------------------------------------
REVERSE_END:
 cbi	Port_R,N_R							; отключаем красный цвет
 cbi	Port_G,N_G							; отключаем зеленый цвет
 sbi	Port_B,N_B							; включаем синий цвет

 sbi	Port_zumer,N_zumer					; пикаем один раз
 rcall	d500ms
 rcall	d500ms
 cbi	Port_zumer,N_zumer

 sbi	Port_stol_en,N_stol_en				; отключаем драйвер двигателя

rjmp	MAIN_LOOP






;****************************************************************************************************
;	При зажигании светодиода "Paper" на клавиатуре принтера останавливаем процесс печати и ждем
;    решение юзера по дальнейшим действиям
;****************************************************************************************************
LED_PAPER_ON:
 sbi	Port_R,N_R							; включаем красный цвет
 cbi	Port_G,N_G							; отключаем зеленый цвет
 cbi	Port_B,N_B							; отключаем синий цвет

; cli										; запрещаем прерывание во время движения стола

 sbis	Pin_but_home,N_but_home				; проверяем состояние кнопки "HOME"
 rjmp	HOME								; для возврата в положение "HOME" при печати

 sbis	Pin_but_reverse,N_but_reverse		; проверяем состояние кнопки "Reverse"
 rjmp	REVERSE

 sbis	Pin_led_paper,N_led_paper			; проверяем состояние индикатора "Paper" на клавиатуре принтера
 rjmp	LED_PAPER_ON						; если горит, тогда включаем красную подсветку

;-----------------------------------------------------------------------------------------------------------------
;	проверяем состояние светодиода. если он мограет, то переходим в главный цикл
;	проверяем свечение светодиода в течении 600 мс. если он загорелся, значит он моргает
LED_PAPER_BLINK:
 cbi	Port_R,N_R							; отключаем красный цвет
 cbi	Port_G,N_G							; отключаем зеленый цвет
 cbi	Port_B,N_B							; отключаем синий цвет

 rcall	d025ms								; вызов подпрограммы задержки на 0,25мс
 ldi	YL,low(8000)						; загрузка в YH:YL константы
 ldi	YH,high(8000)

PAPER_PAUSE:
 sbis	Pin_led_paper,N_led_paper			; проверяем состояние светодиода "Paper" принтера.
 rjmp	LED_PAPER_ON						; если он включился, значит он моргает и переходим в начало цикла

 rcall	d025ms								; вызов подпрограммы задержки на 0,25мс
 sbiw	YL,1								; вычитание единицы из содержимого YH:YL
 brne	PAPER_PAUSE							; если результат не равен нулю, перейти
											; на метку d600_1
ret											; если через 600 мс светодиод погас, значит все нормально, продолжаем дальше печатать											; выключаем все питание


;************************************************************************************
;*																					*
;*                           Вывод индикации на индикатор                           *
;*         переменная adress7219 содержит данные для выбора режима работы			*
;*         переменная data7219 содержит цифры для вывода на разряде индикатора		*
;*																					*
;************************************************************************************

;   Регистры: adress7219, data7219, temp2 
;-----------------------------------------------------------------------------------
; Регистр для вывода динамической индикации на семисегментный дисплей

MAX7219_SEND: 
 ldi	temp2,8								; Передача команд регистра
 cbi	Port_7219_Clk,Pin_7219_Clk			; cбрасываем сигнал CLK 
 cbi	Port_7219_CS,Pin_7219_CS			; разрешаем работу 7219 сигналом CS=0
 nop
MAX7219_LOOP8:								;
 cbi	Port_7219_Clk,Pin_7219_Clk			; cбрасываем сигнал CLK 
 sbrc	adress7219,7						; gереносим старший бит
 sbi	Port_7219_Data,Pin_7219_Data		;  регистра промежуточного
 sbrs	adress7219,7						;  хранения в линию
 cbi	Port_7219_Data,Pin_7219_Data		;  сигнала Data
 lsl	adress7219							; cдвиг влево (очередной бит)
 sbi	Port_7219_Clk,Pin_7219_Clk			; pапись бита по фронту CLK
 dec	temp2								; итерация внутреннего цикла
 brne	MAX7219_LOOP8
 cbi	Port_7219_Clk,Pin_7219_Clk			; опускаем сигнал на линии Clk

 ldi	temp2,8								; Передача данных регистра
MAX7219_LOOP16:								;
 cbi	Port_7219_Clk,Pin_7219_Clk			; cбрасываем сигнал CLK 
 sbrc	data7219,7							; gереносим старший бит
 sbi	Port_7219_Data,Pin_7219_Data		;  регистра промежуточного
 sbrs	data7219,7							;  хранения в линию
 cbi	Port_7219_Data,Pin_7219_Data		;  сигнала Data
 lsl	data7219							; cдвиг влево (очередной бит)
 sbi	Port_7219_Clk,Pin_7219_Clk			; pапись бита по фронту CLK
 dec	temp2								; итерация внутреннего цикла
 brne	MAX7219_LOOP16
 cbi	Port_7219_Clk,Pin_7219_Clk			; опускаем сигнал на линии Clk

 sbi	Port_7219_CS,Pin_7219_CS			; запрещаем работу 7219 сигналом CS=1

ret

;-----------------------------------------------------------------------------------
;	Выводим содержимое оперативной памяти значений индикатора
MAX7219_RAM:							; выводит содержимое оперативки Digit0...7

 ldi	adress7219,0x08						; выводим в 1 позицию
 lds	data7219,Digit						; считываем значение 1 разряда
 rcall	MAX7219_SEND

 ldi	adress7219,0x07						; выводим в 2 позицию
 lds	data7219,Digit+1					; считываем значение 2 разряда
 rcall	MAX7219_SEND

 ldi	adress7219,0x06						; выводим в 3 позицию
 lds	data7219,Digit+2					; считываем значение 3 разряда
 rcall	MAX7219_SEND

 ldi	adress7219,0x05						; выводим в 4 позицию
 lds	data7219,Digit+3					; считываем значение 4 разряда
 rcall	MAX7219_SEND

 ldi	adress7219,0x04						; выводим в 5 позицию
 lds	data7219,Digit+4					; считываем значение 5 разряда
 rcall	MAX7219_SEND

 ldi	adress7219,0x03						; выводим в 6 позицию
 lds	data7219,Digit+5					; считываем значение 6 разряда
 rcall	MAX7219_SEND

 ldi	adress7219,0x02						; выводим в 7 позицию
 lds	data7219,Digit+6					; считываем значение 7 разряда
 rcall	MAX7219_SEND

 ldi	adress7219,0x01						; выводим в 8 позицию
 lds	data7219,Digit+7					; считываем значение 8 разряда
 rcall	MAX7219_SEND

ret


;-----------------------------------------------------------------------------------
;	Инициализация индикатора на MAX7219
MAX7219_Init:
 ldi	adress7219,0x0F						; тест режим. включает все индикаторы
 ldi	data7219,0x00						; отключаем тестовый режим
 rcall	MAX7219_SEND

 ldi	adress7219,0x0C						; режим работы shutdown
 ldi	data7219,0x01						; разрешаем работу микросхемы
 rcall	MAX7219_SEND

 ldi	adress7219,0x0B						; количество отображаемых разрядов
 ldi	data7219,0x07						; отображаем восемь разрядов
 rcall	MAX7219_SEND

 ldi	adress7219,0x09						; декодер разрядов
 ldi	data7219,0b11111111					; отключаем декодер
 rcall	MAX7219_SEND

 ldi	adress7219,0x0A						; выбираем яркость
 ldi	data7219,0x0F						; максимальная яркость
 rcall	MAX7219_SEND

;-----------------------------------------------------------------------------------
;	Очищаем индикатор
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
;*            Преобразование двоичного числа в число для индикатора                *
;*                                                                                 *
;*                                                                                 *
;***********************************************************************************

;   Использует переменные: temp, temp1, adwL, adwH, mulL, mulH
                          
; Bin2ToBcd5
; ==========
; converts a 16-bit-binary to a 5-digit-BCD
; In: 16-bit-binary in adwH,adwL
; Out: 5-digit-BCD
; Used registers:temp
; Called subroutines: Bin2ToDigit
;
Bin2ToBCD5:
 ldi	temp,high(10000)					; Вычисляем 10000 разряд
 mov	mulH,temp
 ldi	temp,low(10000)
 mov	mulL,temp
 rcall	Bin2ToDigit							; Вычисляем, результат в переменной temp
 sts	BCD,temp							; Если не ноль, тогда записываем результат
Bin2ToBCD4:
 ldi	temp,high(1000)						; Вычисляем 1000 разряд
 mov	mulH,temp
 ldi	temp,low(1000)
 mov	mulL,temp
 rcall	Bin2ToDigit							; Выяисляем
 sts	BCD+1,temp							; Результат в переменной temp 
Bin2ToBCD3:
 clr	mulH								;  Вычисляем 100 разряд
 ldi	temp,100
 mov	mulL,temp
 rcall	Bin2ToDigit							; Вычисляем
 sts	BCD+2,temp							; Результат в переменной temp
Bin2ToBCD2:
 clr	mulH								; Вычисляем 10 разряд
 ldi	temp,10
 mov	mulL,temp
 rcall	Bin2ToDigit							; Вычисляем
 sts	BCD+4,adwL							; Единицы остаются в adiw0
 sts	BCD+3,temp							; Результат в переменной temp
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
;**             Обработка прерывания - ; 0x0001 - Внешнее прерывание 0              **
;**																					**
;**																					**
;*************************************************************************************
; enc_step            - импульсы с енкодера
; step_zoom           - коэфициент делителя енкодера
; reductorH:reductorL - количество шагов, которое на которое надо переместить стол


/*
aINT0:
 sbic	PIND,1									; проверяем состояние энкодера
 rjmp	ENC_Plus								; энкодер вращается вправо
 sbis	PIND,1									; проверяем состояние энкодера
 rjmp	ENC_Minus								; энкодер вращается влево
;--------------------------------------------------------------------------------------------------
ENC_Plus:										; Поподаем сюда при вращении энкодера вправо
 brts	ENC_Plus_T1								; проверяем знак числа. если знак числа плюс, тогда переходим по ссылки ENC_Plus_T1
 brtc	ENC_Plus_T0 							; если знак числа минус, то переходим по ссылки ENC_Plus_T0

ENC_Plus_T1:
 inc	enc_step
 cpse	step_zoom,enc_step						; сравниваем количество шагов энкодера с переменной редуктора
reti											; если не равны, то выходим из прерывания
 clr	enc_step								; если равны, тогда step_enc=0
 adiw	reductorH:reductorL,1					;                   reductor=reductor+1 
reti

ENC_Plus_T0:
 clt											; устанавливаем знак числа '-' 
 dec	enc_step								; step_enc=step_enc-1
 clr	temp
 cpse	enc_step,temp							; проверяем, если step_enc=0
reti											; если не равно нулю, то выходим из прерывания
 set											; если равен нулю, тогда устанавливаем знак '+' и выходим из прерывания
reti											
 
;--------------------------------------------------------------------------------------------------
ENC_Minus:										; Поподаем сюда при вращении энкодера влево
 brts	ENC_Minus_T1							; проверяем знак числа. если знак числа плюс, тогда переходим по ссылки ENC_Minus_T1
 brtc	ENC_Minus_T0 							; если знак числа минус, то переходим по ссылки ENC_Minus_T0
 
ENC_Minus_T1:
 clr	temp
 cpse	enc_step,temp							; проверяем, если step_enc=0
 rjmp	ENC_Minus_T1_0							; если не равео нулю, тогда переходим по сылке ENC_Minus_T1_0
 clt											; если равен нулю, тогда знак='-'
 inc	enc_step								;                        step_enc=step_enc+1
reti

ENC_Minus_T1_0:									; если не равно нулю, тогда step_enc=step_enc-1
 dec	enc_step
reti
 
ENC_Minus_T0:
 inc	enc_step
 cpse	step_zoom,enc_step						; сравниваем количество шагов энкодера с переменной редуктора
reti											; если не равны, то выходим из прерывания
 clr	enc_step								; если равны, тогда step_enc=0
 sbiw	reductorH:reductorL,1					;                   reductor=reductor-1 
 set											;                   и знак='+'
reti

*/
aINT0:
 in		temp,SREG
 push	temp
 push	XL
 push	XH

 mov	XL,encoderL							; перкидываем значение шагов енкодера в регистры для математики 
 mov	XH,encoderH

 sbic	PIND,3								; проверяем состояние энкодера
 rjmp	ENC_Plus							; энкодер вращается вправо
 sbis	PIND,3								; проверяем состояние энкодера
 rjmp	ENC_Minus							; энкодер вращается влево


;--------------------------------------------------------------------------------------------------
ENC_Plus:									; Поподаем сюда при вращении энкодера вправо
 inc	enc_step
 adiw	XH:XL,1
 cpse	step_zoom,enc_step					; сравниваем количество шагов энкодера с переменной редуктора
 rjmp	INT0_EXIT							; выходим из прерывания

 sub	enc_step,step_zoom					; если равны, тогда step_enc = step_enc - step_zoom

 adiw	reductorH:reductorL,1				; увеличиваем количество пройденных шагов. если прошли больше
  											;  чем ххх, тогда считаем, что печать закончелась и выбрасываем стол
rjmp	INT0_EXIT							; выходим из прерывания и восстанавливаем значение из стека
; out	SREG,temp
; pop	temp
;reti


;--------------------------------------------------------------------------------------------------
ENC_Minus:									; Поподаем сюда при вращении энкодера влево
 inc	enc_step
 sbiw	XH:XL,1
 cpse	step_zoom,enc_step					; сравниваем количество шагов энкодера с переменной редуктора
 rjmp	INT0_EXIT							; выходим из прерывания

 sub	enc_step,step_zoom					; если равны, тогда step_enc=0

 sbiw	reductorH:reductorL,1				; увеличиваем количество пройденных шагов. если прошли больше
 											;  чем ххх, тогда считаем, что печать закончелась и выбрасываем стол
 rjmp	INT0_EXIT							; выходим из прерывания и восстанавливаем значение из стека
; out	SREG,temp
; pop	temp
;reti


INT0_EXIT:
 mov	encoderL,XL							; перкидываем обратно значение шагов енкодера в регистры хранения 
 mov	encoderH,XH
 
 pop	XH
 pop	XL
 out	SREG,temp
 pop	temp
reti										; если не равны, то выходим из прерывания



;************************************************************************************
;************************************************************************************
;**                                                                                **
;**                                                                                **
;**                 Формирование задержек при частоте кварца 8мГц                  **
;**                                                                                **
;**                                                                                **
;************************************************************************************
;************************************************************************************
;-------------------------------------------------------------------------
;	Задержка на 0,25мс (8Mhz), period  0,481 ms,  frequency 2 kHz

d025ms:
 ldi ZL,low(497)                            ; Загрузка в YH:YL константы 497
 ldi ZH,high(497)
d025_1:
 sbiw ZL,1                                  ; Вычитание из содержимого YH:YL единицы
 brne d025_1                                 ; Если флаг Z<>0 (результат выполнения
                                            ;  предыдущей команды не равен нулю), то
									        ;  перейти на метку d05_1
ret

;-------------------------------------------------------------------------
;	Задержка на 0,35мс (8Mhz), period   ms,  frequency  kHz

d035ms:
 ldi ZL,low(700)                            ; Загрузка в YH:YL константы
 ldi ZH,high(700)
d035_1:
 sbiw ZL,1                                  ; Вычитание из содержимого YH:YL единицы
 brne d035_1                                 ; Если флаг Z<>0 (результат выполнения
                                            ;  предыдущей команды не равен нулю), то
									        ;  перейти на метку d05_1
ret





;-------------------------------------------------------------------------
;	Задержка на 0,5мс (8Mhz), period 0,999 ms,  frequency 1 kHz

d05ms:
 ldi ZL,low(1035)                            ; Загрузка в YH:YL константы 497
 ldi ZH,high(1035)
d05_1:
 sbiw ZL,1                                  ; Вычитание из содержимого YH:YL единицы
 brne d05_1                                 ; Если флаг Z<>0 (результат выполнения
                                            ;  предыдущей команды не равен нулю), то
									        ;  перейти на метку d05_1
ret


;-------------------------------------------------------------------------
;	Задержка 1 ms (8MHz), period 2 ms, frequency 479,35 Hz
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
;	Задержка 3 ms (8MHz), period 6 ms, frequency 164,3 Hz
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
;	Задержка 10 ms (8Mhz), period 20,2 ms,  frequency 49 Hz
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
;	Задержка 37 ms (8Mhz), period 74 ms,  frequency 13,51 Hz
d37ms:
 ldi XL,low(154)                            ; Загрузка в YH:YL константы 100
 ldi XH,high(154)
d37_1:
 rcall d025ms                               ; Вызов подпрограммы задержки на 0,25мс
 sbiw XL,1                                  ; Вычитание единицы из содержимого XH:XL
 brne d37_1                                 ; Если результат не равен нулю, перейти
                                            ; на метку d50_1
ret



;-------------------------------------------------------------------------
;	Задержка на 50 ms (8Mhz), period  0,1 s,  frequency 10 Hz
d50ms:
 ldi XL,low(208)                            ; Загрузка в YH:YL константы 100
 ldi XH,high(208)
d50_1:
 rcall d025ms                               ; Вызов подпрограммы задержки на 0,25мс
 sbiw XL,1                                  ; Вычитание единицы из содержимого XH:XL
 brne d50_1                                 ; Если результат не равен нулю, перейти
                                            ; на метку d50_1
ret


;-------------------------------------------------------------------------
;	Задержка на 67 ms (8Mhz), period  0,1342 s,  frequency  7,452 Hz
d67ms:
 ldi XL,low(279)                            ; Загрузка в YH:YL константы 100
 ldi XH,high(279)
d67_1:
 rcall d025ms                               ; Вызов подпрограммы задержки на 0,25мс
 sbiw XL,1                                  ; Вычитание единицы из содержимого XH:XL
 brne d67_1                                 ; Если результат не равен нулю, перейти
                                            ; на метку d50_1
ret




;-------------------------------------------------------------------------
;	Задержка на 90ms (8Mhz), period 0,1804 s,  frequency  5,55 Hz
d90ms:
 ldi XL,low(375)                            ; Загрузка в YH:YL константы 200
 ldi XH,high(375)
d90_1:
 rcall d025ms                               ; Вызов подпрограммы задержки на 0,25мс
 sbiw XL,1                                  ; Вычитание единицы из содержимого XH:XL
 brne d90_1                                ; Если результат не равен нулю, перейти
                                            ; на метку d100_1
ret



;-------------------------------------------------------------------------
;	Задержка на 100ms (8Mhz), period 0,2 s,  frequency 5 Hz
d100ms:
 ldi XL,low(416)                            ; Загрузка в YH:YL константы 200
 ldi XH,high(416)
d100_1:
 rcall d025ms                               ; Вызов подпрограммы задержки на 0,25мс
 sbiw XL,1                                  ; Вычитание единицы из содержимого XH:XL
 brne d100_1                                ; Если результат не равен нулю, перейти
                                            ; на метку d100_1
ret



;-------------------------------------------------------------------------
;	Задержка на 500мс
d500ms:
 ldi XL,low(2000)                            ; Загрузка в YH:YL константы 200
 ldi XH,high(2000)

d500_1:
 rcall d025ms                               ; Вызов подпрограммы задержки на 0,25мс
 sbiw XL,1                                  ; Вычитание единицы из содержимого XH:XL
 brne d500_1                                ; Если результат не равен нулю, перейти
                                            ; на метку d100_1

ret





;***************************************************************************************************

.exit                                       ; Конец программы






PRINT_HOD:
;	проверяем датчик конечного положения стола. если сработал, тогда стол не двигаем
 sbic	Pin_stol_end,N_stol_end				; пока не сработает датчик конечного положения стола, стол будет двигаться
 rjmp	DVIGATEL_STOP						;  сработал датчик, поэтому стол не двигаем и отключаем двигатель 

;	проверяем кнопки на клавиатуре
 sbis	Pin_but_home,N_but_home				; проверяем состояние кнопки "HOME"
 rjmp	HOME								; для возврата в положение "HOME" при печати
 sbis	Pin_but_reverse,N_but_reverse		; проверяем состояние кнопки "Reverse"
 rjmp	REVERSE

;--------------------------------------------
 cpi	reductorL,0							; count_enc инкриментируем в прерывании
 clr	temp								; сравниваем количество пройденных шагов с константой
 cpc	reductorH,temp						; сравниваем старшие байты с учетом переноса
 brsh	PRINT_WAIT							; если дошли до конца печати, тогда заканчиваем печать и выбрасываем стол



 cbi	Port_stol_dir,N_stol_dir			; крутим двигатель стола в прямом направлении
 cbi	Port_stol_en,N_stol_en				; активируем драйвер
 cbi	Port_stol_step,N_stol_step			; делаем один шаг
 rcall	d05ms
 sbi	Port_stol_step,N_stol_step			; делаем шаг
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
 rcall	MAX7219_RAM							; обновляем информацию на индикаторе
 
 cbi	Port_R,N_R							; отключаем красный цвет
 sbi	Port_G,N_G							; включаем зеленый цвет
 cbi	Port_B,N_B							; отключаем синий цвет

rjmp	PRINT_HOD

DVIGATEL_STOP:
 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; отключаем драйвер

;--------------------------------------------
PRINT_END:
 sbi	Port_stol_step,N_stol_step
 sbi	Port_stol_en,N_stol_en				; отключаем драйвер

 sbi	Port_R,N_R							; отключаем красный цвет
 cbi	Port_G,N_G							; отключаем зеленый цвет
 cbi	Port_B,N_B							; включаем синий цвет

 rjmp	REVERSE_HOD


