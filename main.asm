; Traffic Light State Machine
; Created: 5/04/2020
; Author: Christopher Sutton - 44680112
; Project uses LCD at address 0x27

.include "./m328Pdef.inc"
  .DSEG
  .ORG 0x100
  .def  gpiob=r16
  .def  gpioa=r23
state:   .BYTE 1                 ; Setting pointer - state in data segmen
sensor:  .BYTE 1
  .ORG 0x10F
  .CSEG
  .org  0x0000
    jmp RESET                   ; Reset
    jmp INT0_IR                 ; IRQ0  0x0001
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
;INT0_IR:    rjmp  noint
INT1_IR:    rjmp  noint         ;
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


;;; Calling all initialization routines
  call init_spi
  call timer_init
  call init_port_expander


;;; Main loop
clr r24                         ; Button State B1-SS1 B2-SS2
lcd_err:
  call clear_mcp_int
  clr   gpioa
  clr   gpiob
  clr   r27
  sts   sensor,r27
  sts   state,r27
mainLoop:
    clr   r27
    ldi   r16,0
    sts   state,r16
    sts   sensor,r16
    rjmp  state0

state0:
    call  t2_loop
    call  state_display
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
    lds   r16,sensor
    cpi   r16,0
    breq  state6
    rjmp  state7

state7:
    call  t2_loop
    ldi   r16,7
    sts   state,r16
    call  state_display
    rjmp  state8

state8:
    call  t2_loop
    ldi   r16,8
    sts   state,r16
    call  state_display
    call  reset_sensor
    rjmp  state9

state9:
    call  t2_loop
    lds   r16,state
    inc   r16
    sts   state,r16
    call  state_display
    lds   r16,state
    cpi   r16,12
    brge  state12
    rjmp  state9

state12:
    call  t2_loop
    ldi   r16,12
    sts   state,r16
    call  state_display
    lds   r16,sensor
    cpi   r16,1
    brge  repeat_reset
    rjmp  state13

state13:
    call  t2_loop
    ldi   r16,13
    sts   state,r16
    call  state_display
    rjmp  mainLoop

repeat_reset:
    ldi   r16,0
    sts   sensor,r16
    rjmp  state12

reset_sensor:
    ldi   r16,0
    sts   sensor,r16
    ret

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
    in    r16,SREG
    push  r16
    clr   r27
    call  t1_loop
    ldi   r20,0x10
    call  SPI_Read_Command
    clr   r16
    ldi   r20,0x13                ; Address of GPIOB
    call  SPI_Read_Command
    andi  r16,0b00001100
    cpi   r16,0b00001100
    breq  end_int
    cpi   r16,0b00001000
    breq  set_ss2
    cpi   r16,0b00000100
    breq  set_ss1
end_int:
    pop   r16
    out   SREG,r16
    pop   r27
    pop   r20
    pop   r25
    pop   r16
    pop   r24
    reti                        ; and we're done with the interrupt

set_ss1:
    lds   r27,sensor
    ori   r27,0b01
    andi  r27,0b11
    sts   sensor,r27
    rjmp end_int
set_ss2:
    lds   r27,sensor
    ori   r27,0b10
    andi  r27,0b11
    sts   sensor,r27
    rjmp end_int

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
    ldi   r21,0b00000000              ; Turning all the pins to DEFVAL
    call  SPI_Send_Command
    ldi   r20,0x07              ; DEFVAL (Sets the compare bit)
    ldi   r21,0b00001100             ; Turn them all to one
    call  SPI_Send_Command
  ret

;;; Displaying state to LEDs
state_display:
    push  r24
    push  r25
    call  load_state
    call  led_state_display
    pop   r25
    pop   r24
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

;;; Essentially a large switch
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
    breq  case_2
    cpi   r16,13
    breq  case_3
    cpi   r16,14
    breq  case_4
    cpi   r16,15
    brge  case_4
    ret
;;; Y reg for M:Message and X for S:Message
;;; Gpioa used for setting the led's
case_0:
    ldi   gpioa, 0b00100001
    ret
case_1:
    ldi   gpioa, 0b00010001
    ret
case_2:
    ldi   gpioa, 0b00001100
    ret
case_3:
    ldi   gpioa, 0b00001010
    ret
case_4:
    clr   r16
    sts   state,r16
    rjmp  case_0
; Helper Functions
