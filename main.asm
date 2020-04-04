.include "./m328Pdef.inc"

.org    0x0000                  ; start at beginning of program address

call    setup_int               ; Setup IVR to no_interrupt
                                ; On reset we branch to here
RESET:                          ; Main program start
  ldi   r16,high(RAMEND)        ; Set Stack Pointer to top of RAM
  out   SPH,r16
  ldi   r16,low(RAMEND)
  out   SPL,r16
  sei                           ; Enable interrupts

                                ; Calling all initialization routines
  call init_spi
  call timer_init
  call init_port_expander


                                ; Main loop
mainLoop:
  ldi   r20, 0x14               ; Output GPIOA
  ldi   r21, 0xFF
  call  SPI_Send_Command
  call  t1_loop
  rjmp mainLoop
pre_pause:
  call  check_state
  rjmp  mainLoop


check_state:
  ldi   r20,0x13              ; register GPIOB (port B data input)
  call  SPI_Read_Command      ; Read the state of button into r16
  andi  r16,0b0000100         ; test data from pin 3.
  breq  check_win             ; Hold in pause until button is released
  rjmp  mainLoop

;;; Reading the Button Pin3 PORTA and holding if pressed
pause:
  ldi   r20, 0x13
  call  SPI_Read_Command
  andi  r16, 0b0000100
  breq  pause
  rjmp  mainLoop

check_win:
  ldi   r20, 0x14           ; Set to read PORTB
  call  SPI_Read_Command
  cpi   r16, 0b00000100     ; Check to see if PORTB has green lit
  breq  win_sequence
  call  t1_loop               ;
  call  t1_loop               ; Pause after press
  call  t1_loop               ;
  call  pause                 ; Continue to pause if the button is pressed
  ret

win_sequence:
  ldi   r20, 0x14
  ldi   r21, 0xFF
  call  SPI_Send_Command
  call  t1_loop
  ldi   r21, 0x00
  call  SPI_Send_Command
  call  t1_loop
  ldi   r21, 0xFF
  call  SPI_Send_Command
  call  t1_loop
  ldi   r21, 0b0000100
  call  SPI_Send_Command
  ldi   r17, 0b00000001
  rjmp  pause

; Helper soubroutines
; complete interrupt vector table.
setup_int:
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

;    and we branch to the final location from here.
;    Interrupts we cannot handle in this example we just loop at noint
INT0_IR:		rjmp	noint
INT1_IR:		rjmp	noint
PCINT0_IR:		rjmp	noint
PCINT1_IR:		rjmp		noint
PCINT2_IR:		rjmp	noint
WDT_IR:		 rjmp		 noint
TIM2_COMPA:		 rjmp		 noint
TIM2_COMPB:		 rjmp		 noint
TIM2_OVF:		 rjmp		 noint
TIM1_CAPT:		rjmp		noint
TIM1_COMPA:		 rjmp		 noint
TIM1_COMPB:		 rjmp		 noint
TIM1_OVF:		 rjmp		 noint
TIM0_COMPA:		 rjmp		 noint
TIM0_COMPB:		 rjmp		 noint
TIM0_OVF:		 rjmp		 noint
SPI_STC:		rjmp	noint
USART_RXC:		rjmp	noint
USART_UDRE:		 rjmp		noint
USART_TXC:		rjmp		noint
ADC_CC:		 rjmp		noint
EE_RDY:		 rjmp		noint
ANA_COMP:		 rjmp		noint
TWI_IR:		 rjmp		noint
SPM_RDY:		rjmp		noint

error:
noint:		inc	r16
		rjmp	noint
ret

;
;
; Initialise I/O ports and peripherals
;
; PB0 LED    Output	1
; PB1 ??    Output	1
; PB2 !SS    Output	1
; PB3 MOSI0    Output	1
; PB4 MISO0    Input	0
; PB5 SCK    Output	1
; PB6 XTAL    X	0
; PB7 XTAL    X	0
init_spi:
    ldi	r16,0b00101111        ; set pin directions
    out	DDRB,r16
    sbi	PORTB,2               ; and SS back high

; Setup SPI operations
; See pp217-218 of the data sheet
    ldi	r16,(1<<SPE)|(1<<MSTR) ; set master SPI, (SPI mode 0 operation is 00)
    out	SPCR,r16               ; SCK is set fosc/4 => 4MHz
    clr	r16                    ; clear interrupt flags and oscillator mode.
    out	SPSR,r16
    ret

; Send a command + byte to SPI interface
; CMD is in r20, DATA is in r21
; r16 is destroyed by this subroutine
SPI_Send_Command:
    cbi	PORTB,2               ; SS low
    ldi	r16,0x40
    call	SPI_SendByte
    mov	r16,r20
    call	SPI_SendByte
    mov	r16,r21
    call	SPI_SendByte
    sbi	PORTB,2               ; and SS back high
    ret
; Send a command + byte to SPI interface
; CMD is in r20, DATA is in r21 (if necessary)
SPI_Read_Command:
    cbi	PORTB,2               ; SS low
    ldi	r16,0x41
    call	SPI_SendByte
    mov	r16,r20
    call	SPI_SendByte
    mov	r16,r21
    call	SPI_SendByte
    sbi	PORTB,2               ; and SS back high
    ret
; Send one SPI byte (Returned data in r16)
SPI_SendByte:
    out		SPDR,r16
SPI_wait:
    in	r16,SPSR
    sbrs	r16,SPIF
    rjmp	SPI_wait
    in	r16,SPDR
    ret

;;; Helper Functions for LED
set_led:
    ;; Turning the led on and shifting
    ldi	r20,0x14              ; Register OLATA (port A data output)
    mov	r21,r17               ; Value to write
    call	SPI_Send_Command
    cpi	r22,0x01             ; Toggle flag for direction of LED
    brsh	led_up
led_down:
    ;; Checking the pin and shifting the LED down
    cpi	r17,0b00000001
    breq	set_up
    lsr	r17
    rjmp	pre_pause
led_up:
    ;; Checking high pin (Last red) and shifting the LED up
    cpi	r17, 0b00010000
    breq	set_down
    lsl	r17
    rjmp	pre_pause

set_down:
    ;; Setting the toggle to down direction
    ldi	r22,0x00
    call	led_down
set_up:
    ;; Setting the toggle to the up direction
    ldi	r22,0x01
    ldi	r17,0x01
    call	led_up

;;; Timer Helper Functions
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
    ldi	r17, HIGH(1562)   ; 1562 is value of counter
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


; We will configure port A as all outputs, port B as all inputs.
; use the SPI_send_command to send the SPI commands.
; This will require the register address in r20 and the register data in r21
init_port_expander:
    ldi	r20,0x00              ; register IODIRA (port A data direction)
    ldi	r21,0x00              ; all outputs
    call	SPI_Send_Command
    ldi	r20,0x01              ; register IODIRB (port B data direction)
    ldi	r21,0xff              ; all inputs
    call	SPI_Send_Command
    ldi	r20,0x0d              ; register GPPUB (port B GPIO Pullups)
    ldi	r21,0xff              ; turn on all pullups
    call	SPI_Send_Command
    ret
