; Traffic Light State Machine
; Created: 5/04/2020
; Author: Christopher Sutton - 44680112

.include "./m328Pdef.inc"
  .DSEG
  .ORG 0x100
  .def  gpiob=r16
  .def  gpioa=r23
sensors: .BYTE 1
state:   .BYTE 1                 ; Setting pointer - state in data segmen
  .CSEG
  .org  0x0000
                                ; start at beginning of program address
  jmp RESET                   ; Reset
  jmp INT0_IR                 ; IRQ0
    jmp INT1_IR                 ; IRQ1
    jmp PCINT0_IR               ; PCINT0
    jmp PCINT1_IR               ; PCINT1
    jmp PCINT2_IR               ; PCINT2
    jmp WDT_IR                  ; Watchdog Timeout
    jmp TIM2_COMPA              ; Timer2 CompareA
    jmp TIM2_COMPB              ; Timer2 CompareB
    jmp TIM2_OVF                ; Timer2 Overflow
    jmp TIM1_CAPT               ; Timer1 Capture
    jmp TIM1_COMPA              ; Timer1 CompareA
    jmp TIM1_COMPB              ; Timer1 CompareB
    jmp TIM1_OVF                ; Timer1 Overflow
    jmp TIM0_COMPA              ; Timer0 CompareA
    jmp TIM0_COMPB              ; Timer0 CompareB
    jmp TIM0_OVF                ; Timer0 Overflow
    jmp SPI_STC                 ; SPI Transfer Complete
    jmp USART_RXC               ; USART RX Complete
    jmp USART_UDRE              ; USART UDR Empty
    jmp USART_TXC               ; USART TX Complete
    jmp ADC_CC                  ; ADC Conversion Complete
    jmp EE_RDY                  ; EEPROM Ready
    jmp ANA_COMP                ; Analog Comparator
    jmp TWI_IR                  ; 2-wire Serial
    jmp SPM_RDY                 ; SPM Ready

;;; And we branch to the final location from here.
;;; Interrupts we cannot handle in this example we just loop at noint
INT1_IR:    rjmp  noint
PCINT0_IR:  rjmp  noint
PCINT1_IR:  rjmp  noint
PCINT2_IR:  rjmp  noint
WDT_IR:     rjmp  noint
TIM2_COMPA: rjmp  noint
TIM2_COMPB: rjmp  noint
TIM2_OVF:   rjmp  noint
TIM1_CAPT:  rjmp  noint
TIM1_COMPA: rjmp  noint
TIM1_COMPB: rjmp  noint
TIM1_OVF:   rjmp  noint
TIM0_COMPA: rjmp  noint
TIM0_COMPB: rjmp  noint
TIM0_OVF:   rjmp  noint
SPI_STC:    rjmp  noint
USART_RXC:  rjmp  noint
USART_UDRE: rjmp  noint
USART_TXC:  rjmp  noint
ADC_CC:     rjmp  noint
EE_RDY:     rjmp  noint
ANA_COMP:   rjmp  noint
TWI_IR:     rjmp  noint
SPM_RDY:    rjmp  noint
error:
noint:
    inc   r16
    rjmp    noint
;;; On reset we branch to here
RESET:                          ; Main program start
    ldi   r16,high(RAMEND)        ; Set Stack Pointer to top of RAM
    out   SPH,r16
    ldi   r16,low(RAMEND)
    out   SPL,r16

    in    r16,MCUCR             ; set interrupt vector to address 0x0002
    ori   r16,(1<<IVCE)
    out   MCUCR,r16
    andi  r16,0xfc
    out   MCUCR,r16

    cbi   EIMSK,INT0            ; disable the interrupt -> configure INT0
    ldi   r16,0b00000010        ; high to low transition interrupt for INT0
    sts   EICRA,r16
    sbi   EIMSK,INT0            ; and enable the interrupt for INT0
    sei                         ; enable interrupts globally


lcd_init_str: .DB  0x0C,0x01
;;; Each line of the message buffer can take 16 Characters
lcd_init_msg1: .DB  "Chris's Magic   "
lcd_init_msg2: .DB  "Traffic Lights  "
lcd_state_msg1:.DB  "STATE:     SS:  "
lcd_state_msg2:.DB  "M:      S:      "
green_msg:     .DB  "GREEN "
red_msg:       .DB  "RED   "
yellow_msg:    .DB  "YELLOW"
ss1_on:        .DB  "12"
digits:        .DB  "0123456789  "


;;; Calling all initialization routines
  call init_spi
  call timer_init
  call init_port_expander
  call lcd_init

  call lcd_startup_msg
  call t2_loop
  call lcd_init_state_msg


;;; Main loop
clr r24                         ; Button State B1-SS1 B2-SS2
lcd_err:
  call clear_mcp_int
  clr   gpioa
  clr   gpiob
  clr   r27
  sts   sensors,r27
  sts   state,r27
mainLoop:
    clr   r27
    sts   sensors,r27
    call  t2_loop
    rjmp  state0


state0:
    call  t2_loop
    call  state_display
    call lcd_set_sensor_state
    lds   r16,state
    inc   r16
    sts   state,r16
    cpi   r16,7
    brge  state6
    rjmp  state0

state6:
    call  t2_loop
    ldi   r16,6
    sts   state,r16
    call  state_display
    call lcd_set_sensor_state
    clr   r16
    lds   r16,sensors
    sbrs  r16,0
    jmp   state7
    jmp   state6


state7:
    call  t2_loop
    ldi   r16,7
    sts   state,r16
    call  state_display
    call lcd_set_sensor_state
    lds   r16,sensors
    sbrs  r16,2
    rjmp  state8
    rjmp  state7

state8:
    call  t2_loop
    ldi   r16,8
    sts   state,r16
    call  state_display
    call lcd_set_sensor_state
    cpi   r16,12
    brge  state13
    rjmp  state8

state13:
    call  t2_loop
    call  state_display
    call lcd_set_sensor_state
    lds   r16,sensors
    sbrc  r16,2
    rjmp  mainLoop
    rjmp  state13

clear_mcp_int:
    ldi   r20,0x10
    call  SPI_Read_Command
    ldi   r20,0x13                ; Address of GPIOB
    call  SPI_Read_Command
    ret

INT0_IR:
    push  r24
    push  r16
    push  r25
    push  r20
    push  r27
    clr   r27
    call  t1_loop
    ldi   r20,0x10
    call  SPI_Read_Command
    clr   r16
    ldi   r20,0x13                ; Address of GPIOB
    call  SPI_Read_Command
    andi  r16,0b00001100
    sbrc  r16,3
    call  set_ss2
    sbrc  r16,2
    call  set_ss1
    call  lcd_set_sensor_state
    pop   r27
    pop   r20
    pop   r25
    pop   r16
    pop   r24
    reti                        ; and we're done with the interrupt

set_ss1:
    lds   r27,sensors
    ori   r27,0b01
    andi  r27,0b11
    sts   sensors,r27
    ret
set_ss2:
    lds   r27,sensors
    ori   r27,0b10
    andi  r27,0b11
    sts   sensors,r27
    ret

led_state_display:
    push  r24
    push  r25
    ldi   r20,0x14
    mov   r21,gpioa
    call  SPI_Send_Command
    pop   r25
    pop   r24
    ret

;;; Initialise I/O ports and peripherals
;;; PB0 LED    Output	1
;;; PB1 ??    Output	1
;;; PB2 !SS    Output	1
;;; PB3 MOSI0    Output	1
;;; PB4 MISO0    Input	0
;;; PB5 SCK    Output	1
;;; PB6 XTAL    X	0
;;; PB7 XTAL    X	0
init_spi:
    ldi	r16,0b00101111        ; set pin directions
    out	DDRB,r16
    sbi	PORTB,2               ; and SS back high

;;; Setup SPI operations
;;; See pp217-218 of the data sheet
    ldi	r16,(1<<SPE)|(1<<MSTR) ; set master SPI, (SPI mode 0 operation is 00)
    out	SPCR,r16               ; SCK is set fosc/4 => 4MHz
    clr	r16                    ; clear interrupt flags and oscillator mode.
    out	SPSR,r16
    ret

;;; Send a command + byte to SPI interface
;;; CMD is in r20, DATA is in r21
;;; r16 is destroyed by this subroutine
;;; SPI_Spec <Device Address><Register Address><Value to write>
SPI_Send_Command:
  ;; Send <Command><Byte> to SPI
  ;; PARAM: r20 - Address
  ;; PARAM(Optional): r21 - Data
    cbi	PORTB,2               ; SS low
    ldi	r16,0x40
    call	SPI_SendByte
    mov	r16,r20
    call	SPI_SendByte
    mov	r16,r21
    call	SPI_SendByte
    sbi	PORTB,2               ; and SS back high
    ret
SPI_Read_Command:
  ;; Read from address
  ;; PARAM: r20 - Address
    cbi	PORTB,2               ; SS low
    ldi	r16,0x41
    call	SPI_SendByte
    mov	r16,r20
    call	SPI_SendByte
    mov	r16,r21
    call	SPI_SendByte
    sbi	PORTB,2               ; and SS back high
    ret
SPI_SendByte:
  ;; Sending the byte over SPI Data Register
    out   SPDR,r16
SPI_wait:
  ;; Waiting for return
    in	r16,SPSR
    sbrs	r16,SPIF
    rjmp	SPI_wait
    in	r16,SPDR
    ret

;;; Timer Helper Functions
;;; Delay = 1 Second, 16,000,000/1024 = 15625
timer_init:
    push	r16
    push	r17
    push	r18
    lds	r18, TIMSK1         ; save current value
    clr	r16                 ; diables all interrupts
    sts	TIMSK1, r16         ; [-][-][-][ICIE1][-][-][OCIE1B][TOIE1]
    sts	TCCR1B, r16         ; stop the clock
    ldi	r17, 0b00000100     ; [-][-][ICF1][-][-][OCF1B][OCF1A][TOV1]
    out	TIFR1, r17          ; Output Compare Match B -> OCF1B
    ldi	r16,0b00000000      ; TCCR1A
    sts	TCCR1A, r16         ; PORTA - Normal, PORTB - Normal, WGM=0000(Normal)
    ldi	r17, HIGH(1562)     ; 1562 is value of counter
    ldi	r16, LOW(1562)
    sts	OCR1BH, r17         ; Setting the top of the compare
    sts	OCR1BL, r16         ; Setting the bottom of the compare
    clr	r17                 ; Clear the current count
    sts	TCNT1H, r17         ; TCNT is how you access the timer counter
    sts	TCNT1L, r17         ; The value that OCR1B is looking to compare
    ldi	r16, 0b00000101     ; Noise = 0; WGM =0000, clk =/1024
    sts	TCCR1B, r16
    ldi	r16, 0b00000000
    sts	TCCR1C, r16
    sts	TIMSK1, r18
    pop	r18
    pop	r17
    pop	r16
    ret

t1_loop:
    push	r17
t1_inner:
    sbis	TIFR1,2
    rjmp	t1_inner
    ldi	r17,0b00000100
    out	TIFR1,r17
    clr	r17                   ; Clear the current count
    sts	TCNT1H, r17           ; TCNT is how you access the timer counter
    sts	TCNT1L, r17           ; The value that OCR1B is looking to compare
    pop	r17
    ret

t2_loop:
    ldi   r16,10
begin_loop:
    call  t1_loop
    dec   r16
    cpi   r16,0
    brne  begin_loop
    ret


;;; We will configure port A as all outputs, port B as all inputs.
;;; use the SPI_send_command to send the SPI commands.
;;; This will require the register address in r20 and the register data in r21
;;; IOCON.BANK DEFAULTS TO 0
init_port_expander:
    ldi   r20,0x0A              ; Setting IOCON
    ldi   r21,0b01000000        ; Turning Mirror ON
    call  SPI_Send_Command
    ldi   r20,0x00              ; IODIRA (port A data direction)
    ldi   r21,0x00              ; all outputs
    call  SPI_Send_Command
    ldi   r20,0x01              ; IODIRB (port B data direction)
    ldi   r21,0xff              ; all inputs
    call  SPI_Send_Command
    ldi   r20,0x0d              ; register GPPUB (port B GPIO Pullups)
    ldi   r21,0b00001100              ; turn on all pullups
    call  SPI_Send_Command
    ldi   r20,0x05              ; GPINTENB (Interrupt on Change)
    ldi   r21,0b00001100              ; setting pins GPINT5 and GPINT4
    call  SPI_Send_Command
    ldi   r20,0x09              ; INTCONB (Compare DEFVAL=1 or Prev Val=0)
    ldi   r21,0b00001100              ; Turning all the pins to DEFVAL
    call  SPI_Send_Command
    ldi   r20,0x07              ; DEFVAL (Sets the compare bit)
    ldi   r21,0b00001100             ; Turn them all to one
    call  SPI_Send_Command
  ret

lcd_init:
    cbi DDRC,4          ; I2C pins as inputs will pullup resistors turned on.
    cbi DDRC,5
    sbi PORTC,4
    sbi PORTC,5
    sbi DDRC,0          ; port C, bit 0 is out debug bit.
    cbi PORTC,0

; initialise I/O ports and peripherals

; PC4 SDA0  Bidirectional
; PC5 SCL0  Bidirectional


; I2C clock rate:  assume wants 40KHz
; Rate:
; SCL = Fosc / (16 + 2(TWBR).(TWPS[1:0]))
;     = 16,000,000 / (16 + 2(TWBR).(TWPS[1:0]))

; 40,000 = 16,000,000 / (16 + 2(TWBR).(TWPS[1:0]))

; (16 + 2(TWBR).(TWPS[1:0])) = 16,000,000 / 40,000

; (16 + 2(TWBR).(TWPS[1:0])) = 400

; 2(TWBR).(TWPS[1:0]) = 400 - 16

; 2(TWBR).(TWPS[1:0]) = 386

; (TWBR).(TWPS[1:0]) = 193

; TWBR = 193,   TWPS[1:0] = 0:0 (scale of 1)

;  					  Setup TWI interface
    ldi		r16,193		; setup TWI frequency scaling
    sts		TWBR,r16	; Two Wire Interface Bit-rate Register
    ldi		r16,0x00
    sts		TWSR,r16

    ldi		r24,0x27	; Setup LCD display at this address (Maybe 0x3f instead)
    call	LCD_Setup
    call	LCD_Clear
    ret

lcd_startup_msg:
  ;; Line 1
    ldi   ZL,LOW(lcd_init_msg1*2)
    ldi   ZH,HIGH(lcd_init_msg1*2)
    ldi   r25,0x0F
    call  LCD_Text
 ;; Line 2
    ldi   r25,0x40
    call  LCD_Position
    ldi   ZL,LOW(lcd_init_msg2*2)
    ldi   ZH,HIGH(lcd_init_msg2*2)
    ldi   r25,0x0F
    call  LCD_Text
    ret

lcd_init_state_msg:
  ;; Line 1
    ldi   r25,0x00
    call  LCD_Position
    ldi   ZL,LOW(lcd_state_msg1*2)
    ldi   ZH,HIGH(lcd_state_msg1*2)
    ldi   r25,0x0F
    call  LCD_Text
 ;; Line 2
    ldi   r25,0x40
    call  LCD_Position
    ldi   ZL,LOW(lcd_state_msg2*2)
    ldi   ZH,HIGH(lcd_state_msg2*2)
    ldi   r25,0x0F
    call  LCD_Text
    ret

lcd_set_sensor_state:
  ;; Setting
    lds   r16,sensors
    cpi   r16,0
    breq  end_set_sensor

    lds   r16,sensors
    cpi   r16,0b01
    breq  commit_1

    lds   r16,sensors
    cpi   r16,0b10
    breq  commit_2

    lds   r16,sensors
    cpi   r16,0b11
    breq  set_both_sensor
end_set_sensor:
  ret

commit_1:
    call  lcd_set_ss1
    rjmp  end_set_sensor

commit_2:
    call  lcd_set_ss2
    rjmp  end_set_sensor

set_both_sensor:
    call  lcd_set_ss1
    call  lcd_set_ss2
    rjmp  end_set_sensor

lcd_set_ss1:
    push  r24
    push  r25
    push  r27
    ldi   r24,0x27
    ldi   r25,0x0E
    call  LCD_Position
    ldi   ZL,LOW(ss1_on*2)
    ldi   ZH,HIGH(ss1_on*2)
    ldi   r25,0x01
    call  LCD_Text
    pop   r27
    pop   r25
    pop   r24
    ret

lcd_set_ss2:
    push  r24
    push  r25
    push  r27
    ldi   r24,0x27
    ldi   r25,0x0F
    call  LCD_Position
    ldi   ZL,LOW(ss1_on*2+1)
    ldi   ZH,HIGH(ss1_on*2+1)
    ldi   r25,0x01
    call  LCD_Text
    pop   r27
    pop   r25
    pop   r24
    ret

state_display:
    push  r24
    push  r25
    ldi   r24,0x27
    ldi   r25,0x06
    call  LCD_Position
    call  load_state
    call  led_state_display
    lds   r16, state
    cpi   r16,10
    brge  double_digits
    call  single_digits
lcd_cont:
    ldi   r25,0x42
    call  LCD_Position
    mov   ZL,YL
    mov   ZH,YH
    ldi   r25,0x06
    call  LCD_Text
    ldi   r25,0x4A
    call  LCD_Position
    mov   ZL,XL
    mov   ZH,XH
    ldi   r25,0x06
    call  LCD_Text
    pop   r25
    pop   r24
    ret

single_digits:
    ldi   r24,0x27
    ldi   r25,0x06
    call  LCD_Position
    ldi   ZL,LOW(digits*2)
    ldi   ZH,HIGH(digits*2)
    lds   r16, state
    add   ZL,r16
    ldi   r25,0x01
    call  LCD_Text
    ldi   r25,0x07
    call  LCD_Position
    ldi   ZL,LOW(digits*2+10)
    ldi   ZH,HIGH(digits*2+10)
    ldi   r25,0x01
    call  LCD_Text
    ret

double_digits:
    ldi   r24,0x27
    ldi   r25,0x06
    call  LCD_Position
    ldi   ZL,LOW(digits*2+1)
    ldi   ZH,HIGH(digits*2+1)
    ldi   r25,0x01
    call  LCD_Text
    ldi   r25,0x07
    call  LCD_Position
    ldi   ZL,LOW(digits*2)
    ldi   ZH,HIGH(digits*2)
    lds   r16, state
    add   ZL,r16
    subi  ZL,10
    ldi   r25,0x01
    call  LCD_Text
    jmp  lcd_cont

load_state:
    lds   r16,state
    cpi   r16,0
    breq  case_0
    cpi   r16,1
    breq  case_0
    cpi   r16,2
    breq  case_0
    cpi   r16,3
    breq  case_0
    cpi   r16,4
    breq  case_0
    cpi   r16,5
    breq  case_0
    cpi   r16,6
    breq  case_0
    cpi   r16,7
    breq  case_1
    cpi   r16,8
    breq  case_2
    cpi   r16,9
    breq  case_2
    cpi   r16,10
    breq  case_2
    cpi   r16,11
    breq  case_2
    cpi   r16,12
    breq  case_3
    cpi   r16,13
    brge  case_4
    ret

case_0:
    ldi   YL,LOW(green_msg*2)
    ldi   YH,HIGH(green_msg*2)
    ldi   XL,LOW(red_msg*2)
    ldi   XH,HIGH(red_msg*2)
    ldi   gpioa, 0b00100001
    ret
case_1:
    ldi   YL,LOW(yellow_msg*2)
    ldi   YH,HIGH(yellow_msg*2)
    ldi   XL,LOW(red_msg*2)
    ldi   XH,HIGH(red_msg*2)
    ldi   gpioa, 0b00010001
    ret
case_2:
    ldi   YL,LOW(red_msg*2)
    ldi   YH,HIGH(red_msg*2)
    ldi   XL,LOW(green_msg*2)
    ldi   XH,HIGH(green_msg*2)
    ldi   gpioa, 0b00010100
    ret
case_3:
    ldi   YL,LOW(red_msg*2)
    ldi   YH,HIGH(red_msg*2)
    ldi   XL,LOW(yellow_msg*2)
    ldi   XH,HIGH(yellow_msg*2)
    ldi   gpioa, 0b00001010
    ret
case_4:
    clr   r16
    sts   state,r16
    rjmp  case_0
; Helper Functions



; Send TWI start address.
; On return Z flag is set if completed correctly
; r15 and r16 destroyed
sendTWI_Start:
    ldi		r16,(1<<TWINT) | (1<<TWSTA) | (1<<TWEN)
    sts		TWCR,r16

    call	waitTWI

    lds		r16,TWSR
    andi	r16,0xf8		; mask out
    cpi		r16,0x08		; TWSR = START (0x08)
    ret

; Send TWI slave address. Address is in r16
; On return Z flag is set if completed correctly
; r15 and r16 destroyed
sendTWI_SLA:
    sts		TWDR,r16
    ldi		r16,(1<<TWINT) | (1<<TWEN)
    sts		TWCR,r16

    call	waitTWI

    lds		r16,TWSR
    andi	r16,0xf8		; mask out
    cpi		r16,0x18		; TWSR = SLA+W sent, ACK received (0x18)
    ret

; Send 8 bits of data as two 4 bit nibbles.
; The data is in r16, the lower 4 bits are in r17
; we assume the TWI operation is waiting for data to be sent.
; r15, r18 and r19 all destroyed
sendTWI_Byte:
    mov		r18,r16
    andi	r18,0xF0
    or		r18,r17
    call	sendTWI_Nibble
    mov		r18,r16
    swap	r18
    andi	r18,0xF0
    or		r18,r17
    call	sendTWI_Nibble
    ret

; send 4 bits of data, changing the enable bit as we send it.
; data is in r18. r15, r18 and r19 are destroyed

sendTWI_Nibble:
    ori		r18,0x04
    sts		TWDR,r18
    ldi		r19,(1<<TWINT) | (1<<TWEN)
    sts		TWCR,r19

    call	waitTWI			; destroys r15

    lds		r19,TWSR
    andi	r19,0xf8		; mask out
    cpi		r19,0x28		; TWSR = data sent, ACK received (0x28)
    brne	sendTWI_Nibble_exit

    andi	r18,0xFB		; set enable bit low

    sts		TWDR,r18
    ldi		r19,(1<<TWINT) | (1<<TWEN)
    sts		TWCR,r19

    call	waitTWI

    lds		r19,TWSR
    andi	r19,0xf8		; mask out
    cpi		r19,0x28		; TWSR = data sent, ACK received (0x28)
sendTWI_Nibble_exit:
    ret

;  Send the data pointed to by the Z register to the TWI interface.
;  r25 contains the number of bytes to send
;  r24 contains the address of the I2C controller
;  r17 contains the lower 4 bits of each nibble to send

SendTWI_Data:
    call	sendTWI_Start
    brne	serror

    mov		r16,r24			; use this address
    add		r16,r16			; and move over the r/w bit
    call	sendTWI_SLA
    brne	serror

    cpi		r25,0x00		; any bytes left?
    breq	sendTWI_done	; if not all done

sendTWI_loop:
    lpm		r16,Z+
    call	sendTWI_Byte
    brne	serror

    dec		r25
    brne	sendTWI_loop

sendTWI_done:
serror:

;;; send stop bit and we're done
sendTWI_Stop:
    ldi		r16,(1<<TWINT) | (1<<TWEN) | (1<<TWSTO)		; and send stop
    sts		TWCR,r16
    ldi		r16,0
sendTWI_Delay:
    dec		r16
    brne	sendTWI_Delay
    ret

; Wait until the TWI (I2C) interface has sent the byte and received an ack/nak
; destroys r15

waitTWI:
    lds	r15,TWCR
    sbrs	r15,TWINT		; wait until transmitted
    rjmp	waitTWI
    ret

; Initialisation strings for the LCD panel

; LCD Position - set the write poswition in the DRAM
; r24 holds the LCD I2C address
; r25 holds the address (0-127)
; r17 holds the lower 4 bits

LCD_Position:
    call	sendTWI_Start
    brne	LCD_serror

    mov		r16,r24			; use this address
    add		r16,r16			; and move over the r/w bit
    call	sendTWI_SLA
    brne	LCD_serror

    mov		r16,r25
    ori		r16,0x80		; set DDRAM address command
    ldi		r17,8			; backlight
    call	sendTWI_Byte

    rjmp	sendTWI_Stop

; LCD Clear - Clears the LCD and places the cursor at location 0
; r24 holds the LCD I2C address
; r17 holds the lower 4 bits

LCD_Clear:
    call	sendTWI_Start
    brne	LCD_serror

    mov		r16,r24			; use this address
    add		r16,r16			; and move over the r/w bit
    call	sendTWI_SLA
    brne	LCD_serror

    ldi		r16,0x01		; set DDRAM address command
    ldi		r17,8			; backlight
    call	sendTWI_Byte

    rjmp	sendTWI_Stop

; LCD_Text - send a string to the LCD for displaying
; Z points to the string,
; r25 holds the number of characters to print,
; r24 is the address of the LCD

LCD_Text:
    call	sendTWI_Start
    brne	LCD_serror

    mov		r16,r24			; use this address
    add		r16,r16			; and move over the r/w bit
    call	sendTWI_SLA
    brne	LCD_serror

    cpi		r25,0x00		; any bytes left?
    breq	LCD_Text_done	; if not all done
    ldi		r17,9			; backlight + data byte
LCD_Text_loop:
    lpm		r16,Z+
    call	sendTWI_Byte
    brne	LCD_serror

    dec		r25
    brne	LCD_Text_loop

LCD_Text_done:
LCD_serror:
    rjmp	sendTWI_Stop

; LCDSetup - setup the LCD display connected at I2C port in r16

LCD_Setup:
    call	sendTWI_Start						; send start bit
    breq	LCD_Setup_0
    jmp		LCD_Setup_Err
LCD_Setup_0:
    mov		r16,r24
    add		r16,r16
    call	sendTWI_SLA
    breq	LCD_Setup_1
    jmp		LCD_Setup_Err
LCD_Setup_1:
    clr		r18
    clr		r19
    call	sendTWI_Nibble
    call	sendTWI_Stop

    ldi		r18,LOW(5)
    ldi		r19,HIGH(5)
; call	delay_ms							; wait 5 ms
; Send the first of three 0x30 to the display

    call	sendTWI_Start						; send start bit
    breq	LCD_Setup_2
    jmp		LCD_Setup_Err
LCD_Setup_2:
    mov		r16,r24
    add		r16,r16
    call	sendTWI_SLA
    breq	LCD_Setup_3
    jmp		LCD_Setup_Err
LCD_Setup_3:
    ldi		r18,0x30
    clr		r19
    call	sendTWI_Nibble
    call	sendTWI_Stop

    ldi		r18,LOW(5)
    ldi		r19,HIGH(5)
;  call	delay_ms							; wait 5 ms

; Send the second of three 0x30 to the display

    call	sendTWI_Start						; send start bit
    brne	LCD_Setup_Err
    mov		r16,r24
    add		r16,r16
    call	sendTWI_SLA
    brne	LCD_Setup_Err
    ldi		r18,0x30
    clr		r19
    call	sendTWI_Nibble
    call	sendTWI_Stop

    ldi		r18,LOW(5)
    ldi		r19,HIGH(5)
;  call	delay_ms							; wait 5 ms

; Send the third of three 0x30 to the display

    call	sendTWI_Start						; send start bit
    brne	LCD_Setup_Err
    mov		r16,r24
    add		r16,r16
    call	sendTWI_SLA
    brne	LCD_Setup_Err
    ldi		r18,0x30
    clr		r19
    call	sendTWI_Nibble
    call	sendTWI_Stop


; Send 0x28 to the display to reset to 4 bit mode

    call	sendTWI_Start						; send start bit
    brne	LCD_Setup_Err
    mov		r16,r24
    add		r16,r16
    call	sendTWI_SLA
    brne	LCD_Setup_Err
    ldi		r18,0x28
    clr		r19
    call	sendTWI_Nibble
    call	sendTWI_Stop

    ldi		ZL,LOW(lcd_init_str*2)
    ldi		ZH,HIGH(lcd_init_str*2)
    ldi		r25,2								; all 2 bytes
    ldi		r17,8								; lower 4 bits zero (Backlight on)
    call	SendTWI_Data
    ret

    LCD_Setup_Err:
    jmp lcd_err
