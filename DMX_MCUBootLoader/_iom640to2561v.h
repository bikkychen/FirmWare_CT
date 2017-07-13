#ifndef ___iom640to2561v_h
#define ___iom640to2561v_h

/* ATmega640..2561 io header file for
 * ImageCraft ICCAVR compiler
 */

/* 2004/12/19 created as iom2560v.h
   2005/03/10 fixed SPI bits
   2005/05/12 fixed EECR "P" bits
   2005/05/16 added GPIOR0..2
   2006/02/11 fixed device #define and CKOUT
   2008/04/21 added USART bits for SPI mode
 */

/* Port L */
#define PINL	(*(volatile unsigned char *)0x109) /* m/m */
#define DDRL	(*(volatile unsigned char *)0x10A) /* m/m */
#define PORTL	(*(volatile unsigned char *)0x10B) /* m/m */

/* Port K */
#define PINK	(*(volatile unsigned char *)0x106) /* m/m */
#define DDRK	(*(volatile unsigned char *)0x107) /* m/m */
#define PORTK	(*(volatile unsigned char *)0x108) /* m/m */

/* Port J */
#define PINJ	(*(volatile unsigned char *)0x103) /* m/m */
#define DDRJ	(*(volatile unsigned char *)0x104) /* m/m */
#define PORTJ	(*(volatile unsigned char *)0x105) /* m/m */

/* Port H */
#define PINH	(*(volatile unsigned char *)0x100) /* m/m */
#define DDRH	(*(volatile unsigned char *)0x101) /* m/m */
#define PORTH	(*(volatile unsigned char *)0x102) /* m/m */

/* Port G */
#define PING	(*(volatile unsigned char *)0x32)
#define DDRG	(*(volatile unsigned char *)0x33)
#define PORTG	(*(volatile unsigned char *)0x34)

/* Port F */
#define PINF	(*(volatile unsigned char *)0x2F)
#define DDRF	(*(volatile unsigned char *)0x30)
#define PORTF	(*(volatile unsigned char *)0x31)

/* Port E */
#define PINE	(*(volatile unsigned char *)0x2C)
#define DDRE	(*(volatile unsigned char *)0x2D)
#define PORTE	(*(volatile unsigned char *)0x2E)

/* Port D */
#define PIND	(*(volatile unsigned char *)0x29)
#define DDRD	(*(volatile unsigned char *)0x2A)
#define PORTD	(*(volatile unsigned char *)0x2B)

/* Port C */
#define PINC	(*(volatile unsigned char *)0x26)
#define DDRC	(*(volatile unsigned char *)0x27)
#define PORTC	(*(volatile unsigned char *)0x28)

/* Port B */
#define PINB	(*(volatile unsigned char *)0x23)
#define DDRB	(*(volatile unsigned char *)0x24)
#define PORTB	(*(volatile unsigned char *)0x25)

/* Port A */
#define PINA	(*(volatile unsigned char *)0x20)
#define DDRA	(*(volatile unsigned char *)0x21)
#define PORTA	(*(volatile unsigned char *)0x22)

/* ADC */
#define ADC 	(*(volatile unsigned int *)0x78)
#define ADCL	(*(volatile unsigned char *)0x78)
#define ADCH	(*(volatile unsigned char *)0x79)
#define ADCSRA	(*(volatile unsigned char *)0x7A)
#define  ADEN     7
#define  ADSC     6
#define  ADATE    5
#define  ADIF     4
#define  ADIE     3
#define  ADPS2    2
#define  ADPS1    1
#define  ADPS0    0
#define ADCSRB	(*(volatile unsigned char *)0x7B)
#define  ACME     6
#define  MUX5     3
#define  ADTS2    2
#define  ADTS1    1
#define  ADTS0    0
#define ADMUX	(*(volatile unsigned char *)0x7C)
#define  REFS1    7
#define  REFS0    6
#define  ADLAR    5
#define  MUX4     4
#define  MUX3     3
#define  MUX2     2
#define  MUX1     1
#define  MUX0     0

/* Analog Comparator Control and Status Register */
#define ACSR	(*(volatile unsigned char *)0x50)
#define  ACD      7
#define  ACBG     6
#define  ACO      5
#define  ACI      4
#define  ACIE     3
#define  ACIC     2
#define  ACIS1    1
#define  ACIS0    0

/* DIDR */
#define DIDR2	(*(volatile unsigned char *)0x7D) /* m/m */
#define  ADC15D   7
#define  ADC14D   6
#define  ADC13D   5
#define  ADC12D   4
#define  ADC11D   3
#define  ADC10D   2
#define  ADC9D    1
#define  ADC8D    0
#define DIDR0	(*(volatile unsigned char *)0x7E) /* m/m */
#define  ADC7D    7
#define  ADC6D    6
#define  ADC5D    5
#define  ADC4D    4
#define  ADC3D    3
#define  ADC2D    2
#define  ADC1D    1
#define  ADC0D    0
#define DIDR1	(*(volatile unsigned char *)0x7F) /* m/m */
#define  AIN1D    1
#define  AIN0D    0

/* USART0 */
#define UBRR0H	(*(volatile unsigned char *)0xC5) /* m/m */
#define UBRR0L	(*(volatile unsigned char *)0xC4) /* m/m */
#define UBRR0	(*(volatile unsigned int *)0xC4) /* m/m */
#define UCSR0C	(*(volatile unsigned char *)0xC2) /* m/m */
#define  UMSEL01  7
#define  UMSEL00  6
#define  UPM01    5
#define  UPM00    4
#define  USBS0    3
#define  UCSZ01   2
#define  UCSZ00   1
#define  UCPOL0   0
/* in SPI mode */
#define  UDORD0   2
#define  UCPHA0   1
#define UCSR0B	(*(volatile unsigned char *)0xC1) /* m/m */
#define  RXCIE0   7
#define  TXCIE0   6
#define  UDRIE0   5
#define  RXEN0    4
#define  TXEN0    3
#define  UCSZ02   2
#define  RXB80    1
#define  TXB80    0
#define UCSR0A	(*(volatile unsigned char *)0xC0) /* m/m */
#define  RXC0     7
#define  TXC0     6
#define  UDRE0    5
#define  FE0      4
#define  DOR0     3
#define  UPE0     2
#define  U2X0     1
#define  MPCM0    0
#define UDR0	(*(volatile unsigned char *)0xC6) /* m/m */

/* USART1 */
#define UBRR1H	(*(volatile unsigned char *)0xCD) /* m/m */
#define UBRR1L	(*(volatile unsigned char *)0xCC) /* m/m */
#define UBRR1	(*(volatile unsigned int *)0xCC) /* m/m */
#define UCSR1C	(*(volatile unsigned char *)0xCA) /* m/m */
#define  UMSEL11  7
#define  UMSEL10  6
#define  UPM11    5
#define  UPM10    4
#define  USBS1    3
#define  UCSZ11   2
#define  UCSZ10   1
#define  UCPOL1   0
/* in SPI mode */
#define  UDORD1   2
#define  UCPHA1   1
#define UCSR1B	(*(volatile unsigned char *)0xC9) /* m/m */
#define  RXCIE1   7
#define  TXCIE1   6
#define  UDRIE1   5
#define  RXEN1    4
#define  TXEN1    3
#define  UCSZ12   2
#define  RXB81    1
#define  TXB81    0
#define UCSR1A	(*(volatile unsigned char *)0xC8) /* m/m */
#define  RXC1     7
#define  TXC1     6
#define  UDRE1    5
#define  FE1      4
#define  DOR1     3
#define  UPE1     2
#define  U2X1     1
#define  MPCM1    0
#define UDR1	(*(volatile unsigned char *)0xCE) /* m/m */

/* USART2 */
#define UBRR2H	(*(volatile unsigned char *)0xD5) /* m/m */
#define UBRR2L	(*(volatile unsigned char *)0xD4) /* m/m */
#define UBRR2	(*(volatile unsigned int *)0xD4) /* m/m */
#define UCSR2C	(*(volatile unsigned char *)0xD2) /* m/m */
#define  UMSEL21  7
#define  UMSEL20  6
#define  UPM21    5
#define  UPM20    4
#define  USBS2    3
#define  UCSZ21   2
#define  UCSZ20   1
#define  UCPOL2   0
/* in SPI mode */
#define  UDORD2   2
#define  UCPHA2   1
#define UCSR2B	(*(volatile unsigned char *)0xD1) /* m/m */
#define  RXCIE2   7
#define  TXCIE2   6
#define  UDRIE2   5
#define  RXEN2    4
#define  TXEN2    3
#define  UCSZ22   2
#define  RXB82    1
#define  TXB82    0
#define UCSR2A	(*(volatile unsigned char *)0xD0) /* m/m */
#define  RXC2     7
#define  TXC2     6
#define  UDRE2    5
#define  FE2      4
#define  DOR2     3
#define  UPE2     2
#define  U2X2     1
#define  MPCM2    0
#define UDR2	(*(volatile unsigned char *)0xD6) /* m/m */

/* USART3 */
#define UBRR3H	(*(volatile unsigned char *)0x135) /* m/m */
#define UBRR3L	(*(volatile unsigned char *)0x134) /* m/m */
#define UBRR3	(*(volatile unsigned int *)0x134) /* m/m */
#define UCSR3C	(*(volatile unsigned char *)0x132) /* m/m */
#define  UMSEL31  7
#define  UMSEL30  6
#define  UPM31    5
#define  UPM30    4
#define  USBS3    3
#define  UCSZ31   2
#define  UCSZ30   1
#define  UCPOL3   0
/* in SPI mode */
#define  UDORD3   2
#define  UCPHA3   1
#define UCSR3B	(*(volatile unsigned char *)0x131) /* m/m */
#define  RXCIE3   7
#define  TXCIE3   6
#define  UDRIE3   5
#define  RXEN3    4
#define  TXEN3    3
#define  UCSZ32   2
#define  RXB83    1
#define  TXB83    0
#define UCSR3A	(*(volatile unsigned char *)0x130) /* m/m */
#define  RXC3     7
#define  TXC3     6
#define  UDRE3    5
#define  FE3      4
#define  DOR3     3
#define  UPE3     2
#define  U2X3     1
#define  MPCM3    0
#define UDR3	(*(volatile unsigned char *)0x136) /* m/m */

/* 2-wire SI */
#define TWBR	(*(volatile unsigned char *)0xB8) /* m/m */
#define TWSR	(*(volatile unsigned char *)0xB9) /* m/m */
#define  TWPS1    1
#define  TWPS0    0
#define TWAR	(*(volatile unsigned char *)0xBA) /* m/m */
#define  TWGCE    0
#define TWDR	(*(volatile unsigned char *)0xBB) /* m/m */
#define TWCR	(*(volatile unsigned char *)0xBC) /* m/m */
#define  TWINT    7
#define  TWEA     6
#define  TWSTA    5
#define  TWSTO    4
#define  TWWC     3
#define  TWEN     2
#define  TWIE     0
#define TWAMR	(*(volatile unsigned char *)0xBD) /* m/m */

/* SPI */
#define SPCR	(*(volatile unsigned char *)0x4C)
#define  SPIE     7
#define  SPE      6
#define  DORD     5
#define  MSTR     4
#define  CPOL     3
#define  CPHA     2
#define  SPR1     1
#define  SPR0     0
#define SPSR	(*(volatile unsigned char *)0x4D)
#define  SPIF     7
#define  WCOL     6
#define  SPI2X    0
#define SPDR	(*(volatile unsigned char *)0x4E)

/* EEPROM */
#define EECR	(*(volatile unsigned char *)0x3F)
#define  EEPM1    5
#define  EEPM0    4
#define  EERIE    3
#define  EEMPE    2
#define  EEMWE    2
#define  EEPE     1
#define  EEWE     1
#define  EERE     0
#define EEDR	(*(volatile unsigned char *)0x40)
#define EEAR	(*(volatile unsigned int *)0x41)
#define EEARL	(*(volatile unsigned char *)0x41)
#define EEARH	(*(volatile unsigned char *)0x42)

/* GTCCR */
#define GTCCR	(*(volatile unsigned char *)0x43)
#define  TSM      7
#define  PSRASY   1
#define  PSRSYNC  0

/* Watchdog Timer Control Register */
#define WDTCSR	(*(volatile unsigned char *)0x60) /* m/m */
#define WDTCR	(*(volatile unsigned char *)0x60) /* m/m */
#define  WDIF     7
#define  WDIE     6
#define  WDP3     5
#define  WDCE     4
#define  WDE      3
#define  WDP2     2
#define  WDP1     1
#define  WDP0     0

/* OCDR */
#define OCDR	(*(volatile unsigned char *)0x51)
#define MONDR	(*(volatile unsigned char *)0x51)

/* Timer/Counter 0 */
#define OCR0B	(*(volatile unsigned char *)0x48)
#define OCR0A	(*(volatile unsigned char *)0x47)
#define TCNT0	(*(volatile unsigned char *)0x46)
#define TCCR0B	(*(volatile unsigned char *)0x45)
#define  FOC0     7
#define  FOC0A    7
#define  FOC0B    6
#define  WGM02    3
#define  CS02     2
#define  CS01     1
#define  CS00     0
#define TCCR0A	(*(volatile unsigned char *)0x44)
#define  COM0A1   7
#define  COM0A0   6
#define  COM0B1   5
#define  COM0B0   4
#define  WGM01    1
#define  WGM00    0

/* Timer/Counter1 */
#define ICR1	(*(volatile unsigned int *)0x86) /* m/m */
#define ICR1L	(*(volatile unsigned char *)0x86) /* m/m */
#define ICR1H	(*(volatile unsigned char *)0x87) /* m/m */
#define OCR1C	(*(volatile unsigned int *)0x8C) /* m/m */
#define OCR1CL	(*(volatile unsigned char *)0x8C) /* m/m */
#define OCR1CH	(*(volatile unsigned char *)0x8D) /* m/m */
#define OCR1B	(*(volatile unsigned int *)0x8A) /* m/m */
#define OCR1BL	(*(volatile unsigned char *)0x8A) /* m/m */
#define OCR1BH	(*(volatile unsigned char *)0x8B) /* m/m */
#define OCR1A	(*(volatile unsigned int *)0x88) /* m/m */
#define OCR1AL	(*(volatile unsigned char *)0x88) /* m/m */
#define OCR1AH	(*(volatile unsigned char *)0x89) /* m/m */
#define TCNT1	(*(volatile unsigned int *)0x84) /* m/m */
#define TCNT1L	(*(volatile unsigned char *)0x84) /* m/m */
#define TCNT1H	(*(volatile unsigned char *)0x85) /* m/m */
#define TCCR1C	(*(volatile unsigned char *)0x82) /* m/m */
#define  FOC1A    7
#define  FOC1B    6
#define  FOC1C    5
#define TCCR1B	(*(volatile unsigned char *)0x81) /* m/m */
#define  ICNC1    7
#define  ICES1    6
#define  WGM13    4
#define  WGM12    3
#define  CS12     2
#define  CS11     1
#define  CS10     0
#define TCCR1A	(*(volatile unsigned char *)0x80) /* m/m */
#define  COM1A1   7
#define  COM1A0   6
#define  COM1B1   5
#define  COM1B0   4
#define  COM1C1   3
#define  COM1C0   2
#define  WGM11    1
#define  WGM10    0

/* Timer/Counter2 */
#define ASSR	(*(volatile unsigned char *)0xB6) /* m/m */
#define  EXCLK    6
#define  AS2      5
#define  TCN2UB   4
#define  OCR2AUB  3
#define  OCR2BUB  2
#define  TCR2AUB  1
#define  TCR2BUB  0
#define OCR2B	(*(volatile unsigned char *)0xB4) /* m/m */
#define OCR2A	(*(volatile unsigned char *)0xB3) /* m/m */
#define TCNT2	(*(volatile unsigned char *)0xB2) /* m/m */
#define TCCR2B	(*(volatile unsigned char *)0xB1) /* m/m */
#define  FOC2A    7
#define  FOC2B    6
#define  WGM22    3
#define  CS22     2
#define  CS21     1
#define  CS20     0
#define TCCR2A	(*(volatile unsigned char *)0xB0) /* m/m */
#define  COM2A1   7
#define  COM2A0   6
#define  COM2B1   5
#define  COM2B0   4
#define  WGM21    1
#define  WGM20    0

/* Timer/Counter3 */
#define OCR3CH	(*(volatile unsigned char *)0x9D) /* m/m */
#define OCR3CL	(*(volatile unsigned char *)0x9C) /* m/m */
#define OCR3C	(*(volatile unsigned int *)0x9C) /* m/m */
#define OCR3BH	(*(volatile unsigned char *)0x9B) /* m/m */
#define OCR3BL	(*(volatile unsigned char *)0x9A) /* m/m */
#define OCR3B	(*(volatile unsigned int *)0x9A) /* m/m */
#define OCR3AH	(*(volatile unsigned char *)0x99) /* m/m */
#define OCR3AL	(*(volatile unsigned char *)0x98) /* m/m */
#define OCR3A	(*(volatile unsigned int *)0x98) /* m/m */
#define ICR3H	(*(volatile unsigned char *)0x97) /* m/m */
#define ICR3L	(*(volatile unsigned char *)0x96) /* m/m */
#define ICR3	(*(volatile unsigned int *)0x96) /* m/m */
#define TCNT3H	(*(volatile unsigned char *)0x95) /* m/m */
#define TCNT3L	(*(volatile unsigned char *)0x94) /* m/m */
#define TCNT3	(*(volatile unsigned int *)0x94) /* m/m */
#define TCCR3C	(*(volatile unsigned char *)0x92) /* m/m */
#define  FOC3A    7
#define  FOC3B    6
#define  FOC3C    5
#define TCCR3B	(*(volatile unsigned char *)0x91) /* m/m */
#define  ICNC3    7
#define  ICES3    6
#define  WGM33    4
#define  WGM32    3
#define  CS32     2
#define  CS31     1
#define  CS30     0
#define TCCR3A	(*(volatile unsigned char *)0x90) /* m/m */
#define  COM3A1   7
#define  COM3A0   6
#define  COM3B1   5
#define  COM3B0   4
#define  COM3C1   3
#define  COM3C0   2
#define  WGM31    1
#define  WGM30    0

/* Timer/Counter4 */
#define OCR4CH	(*(volatile unsigned char *)0xAD) /* m/m */
#define OCR4CL	(*(volatile unsigned char *)0xAC) /* m/m */
#define OCR4C	(*(volatile unsigned int *)0xAC) /* m/m */
#define OCR4BH	(*(volatile unsigned char *)0xAB) /* m/m */
#define OCR4BL	(*(volatile unsigned char *)0xAA) /* m/m */
#define OCR4B	(*(volatile unsigned int *)0xAA) /* m/m */
#define OCR4AH	(*(volatile unsigned char *)0xA9) /* m/m */
#define OCR4AL	(*(volatile unsigned char *)0xA8) /* m/m */
#define OCR4A	(*(volatile unsigned int *)0xA8) /* m/m */
#define ICR4H	(*(volatile unsigned char *)0xA7) /* m/m */
#define ICR4L	(*(volatile unsigned char *)0xA6) /* m/m */
#define ICR4	(*(volatile unsigned int *)0xA6) /* m/m */
#define TCNT4H	(*(volatile unsigned char *)0xA5) /* m/m */
#define TCNT4L	(*(volatile unsigned char *)0xA4) /* m/m */
#define TCNT4	(*(volatile unsigned int *)0xA4) /* m/m */
#define TCCR4C	(*(volatile unsigned char *)0xA2) /* m/m */
#define  FOC4A    7
#define  FOC4B    6
#define  FOC4C    5
#define TCCR4B	(*(volatile unsigned char *)0xA1) /* m/m */
#define  ICNC4    7
#define  ICES4    6
#define  WGM43    4
#define  WGM42    3
#define  CS42     2
#define  CS41     1
#define  CS40     0
#define TCCR4A	(*(volatile unsigned char *)0xA0) /* m/m */
#define  COM4A1   7
#define  COM4A0   6
#define  COM4B1   5
#define  COM4B0   4
#define  COM4C1   3
#define  COM4C0   2
#define  WGM41    1
#define  WGM40    0

/* Timer/Counter5 */
#define OCR5CH	(*(volatile unsigned char *)0x12D) /* m/m */
#define OCR5CL	(*(volatile unsigned char *)0x12C) /* m/m */
#define OCR5C	(*(volatile unsigned int *)0x12C) /* m/m */
#define OCR5BH	(*(volatile unsigned char *)0x12B) /* m/m */
#define OCR5BL	(*(volatile unsigned char *)0x12A) /* m/m */
#define OCR5B	(*(volatile unsigned int *)0x12A) /* m/m */
#define OCR5AH	(*(volatile unsigned char *)0x129) /* m/m */
#define OCR5AL	(*(volatile unsigned char *)0x128) /* m/m */
#define OCR5A	(*(volatile unsigned int *)0x128) /* m/m */
#define ICR5H	(*(volatile unsigned char *)0x127) /* m/m */
#define ICR5L	(*(volatile unsigned char *)0x126) /* m/m */
#define ICR5	(*(volatile unsigned int *)0x126) /* m/m */
#define TCNT5H	(*(volatile unsigned char *)0x125) /* m/m */
#define TCNT5L	(*(volatile unsigned char *)0x124) /* m/m */
#define TCNT5	(*(volatile unsigned int *)0x124) /* m/m */
#define TCCR5C	(*(volatile unsigned char *)0x122) /* m/m */
#define  FOC5A    7
#define  FOC5B    6
#define  FOC5C    5
#define TCCR5B	(*(volatile unsigned char *)0x121) /* m/m */
#define  ICNC5    7
#define  ICES5    6
#define  WGM53    4
#define  WGM52    3
#define  CS52     2
#define  CS51     1
#define  CS50     0
#define TCCR5A	(*(volatile unsigned char *)0x120) /* m/m */
#define  COM5A1   7
#define  COM5A0   6
#define  COM5B1   5
#define  COM5B0   4
#define  COM5C1   3
#define  COM5C0   2
#define  WGM51    1
#define  WGM50    0

/* Oscillator Calibration Register */
#define OSCCAL	(*(volatile unsigned char *)0x66) /* m/m */

/* clock prescaler control register */
#define CLKPR	(*(volatile unsigned char *)0x61) /* m/m */
#define  CLKPCE   7
#define  CLKPS3   3
#define  CLKPS2   2
#define  CLKPS1   1
#define  CLKPS0   0

/* PRR */
#define PRR0	(*(volatile unsigned char *)0x64) /* m/m */
#define  PRTWI    7
#define  PRTIM2   6
#define  PRTIM0   5
#define  PRTIM1   3
#define  PRSPI    2
#define  PRUSART0 1
#define  PRADC    0
#define PRR1	(*(volatile unsigned char *)0x65) /* m/m */
#define  PRTIM5   5
#define  PRTIM4   4
#define  PRTIM3   3
#define  PRUSART3 2
#define  PRUSART2 1
#define  PRUSART1 0

/* MCU */
#define MCUSR	(*(volatile unsigned char *)0x54)
#define  JTRF     4
#define  WDRF     3
#define  BORF     2
#define  EXTRF    1
#define  PORF     0
#define MCUCR	(*(volatile unsigned char *)0x55)
#define  JTD      7
#define  PUD      4
#define  IVSEL    1
#define  IVCE     0

#define SMCR	(*(volatile unsigned char *)0x53)
#define  SM2      3
#define  SM1      2
#define  SM0      1
#define  SE       0

/* SPM Control and Status Register */
#define SPMCSR	(*(volatile unsigned char *)0x57)
#define  SPMIE    7
#define  RWWSB    6
#define  RWWSRE   4
#define  BLBSET   3
#define  PGWRT    2
#define  PGERS    1
#define  SPMEN    0

/* Timer/Counter Interrupts */
#define TIFR0	(*(volatile unsigned char *)0x35)
#define  OCF0B    2
#define  OCF0A    1
#define  TOV0     0
#define TIMSK0	(*(volatile unsigned char *)0x6E) /* m/m */
#define  OCIE0B   2
#define  OCIE0A   1
#define  TOIE0    0
#define TIFR1	(*(volatile unsigned char *)0x36)
#define  ICF1     5
#define  OCF1C    3
#define  OCF1B    2
#define  OCF1A    1
#define  TOV1     0
#define TIMSK1	(*(volatile unsigned char *)0x6F) /* m/m */
#define  ICIE1    5
#define  OCIE1C   3
#define  OCIE1B   2
#define  OCIE1A   1
#define  TOIE1    0
#define TIFR2	(*(volatile unsigned char *)0x37)
#define  OCF2B    2
#define  OCF2A    1
#define  TOV2     0
#define TIMSK2	(*(volatile unsigned char *)0x70) /* m/m */
#define  OCIE2B   2
#define  OCIE2A   1
#define  TOIE2    0
#define TIFR3	(*(volatile unsigned char *)0x38)
#define  ICF3     5
#define  OCF3C    3
#define  OCF3B    2
#define  OCF3A    1
#define  TOV3     0
#define TIMSK3	(*(volatile unsigned char *)0x71) /* m/m */
#define  ICIE3    5
#define  OCIE3C   3
#define  OCIE3B   2
#define  OCIE3A   1
#define  TOIE3    0
#define TIFR4	(*(volatile unsigned char *)0x39)
#define  ICF4     5
#define  OCF4C    3
#define  OCF4B    2
#define  OCF4A    1
#define  TOV4     0
#define TIMSK4	(*(volatile unsigned char *)0x72) /* m/m */
#define  ICIE4    5
#define  OCIE4C   3
#define  OCIE4B   2
#define  OCIE4A   1
#define  TOIE4    0
#define TIFR5	(*(volatile unsigned char *)0x3A)
#define  ICF5     5
#define  OCF5C    3
#define  OCF5B    2
#define  OCF5A    1
#define  TOV5     0
#define TIMSK5	(*(volatile unsigned char *)0x73) /* m/m */
#define  ICIE5    5
#define  OCIE5C   3
#define  OCIE5B   2
#define  OCIE5A   1
#define  TOIE5    0

/* Èxternal Interrupts */
#define EIFR	(*(volatile unsigned char *)0x3C)
#define  INTF7    7
#define  INTF6    6
#define  INTF5    5
#define  INTF4    4
#define  INTF3    3
#define  INTF2    2
#define  INTF1    1
#define  INTF0    0
#define EIMSK	(*(volatile unsigned char *)0x3D)
#define  INT7     7
#define  INT6     6
#define  INT5     5
#define  INT4     4
#define  INT3     3
#define  INT2     2
#define  INT1     1
#define  INT0     0
#define EICRB	(*(volatile unsigned char *)0x6A) /* m/m */
#define  ISC71    7
#define  ISC70    6
#define  ISC61    5
#define  ISC60    4
#define  ISC51    3
#define  ISC50    2
#define  ISC41    1
#define  ISC40    0
#define EICRA	(*(volatile unsigned char *)0x69) /* m/m */
#define  ISC31    7
#define  ISC30    6
#define  ISC21    5
#define  ISC20    4
#define  ISC11    3
#define  ISC10    2
#define  ISC01    1
#define  ISC00    0

/* Pin Change Interrupts */
#define PCIFR	(*(volatile unsigned char *)0x3B)
#define  PCIF2    2
#define  PCIF1    1
#define  PCIF0    0
#define PCICR	(*(volatile unsigned char *)0x68) /* m/m */
#define  PCIE2    2
#define  PCIE1    1
#define  PCIE0    0
#define PCMSK0	(*(volatile unsigned char *)0x6B) /* m/m */
#define PCMSK1	(*(volatile unsigned char *)0x6C) /* m/m */
#define PCMSK2	(*(volatile unsigned char *)0x6D) /* m/m */

/* eXternal Memory Control Register */
#define XMCRB	(*(volatile unsigned char *)0x75) /* m/m */
#define  XMBK     7
#define  XMM2     2
#define  XMM1     1
#define  XMM0     0
#define XMCRA	(*(volatile unsigned char *)0x74) /* m/m */
#define  SRE      7
#define  SRL2     6
#define  SRL1     5
#define  SRL0     4
#define  SRW11    3
#define  SRW10    2
#define  SRW01    1
#define  SRW00    0

/* GPIO */
#define GPIOR0	(*(volatile unsigned char *)0x3E)
#define GPIOR1	(*(volatile unsigned char *)0x4A)
#define GPIOR2	(*(volatile unsigned char *)0x4B)

/* RAM page Z-pointer */
#define RAMPZ	(*(volatile unsigned char *)0x5B)

/* EIND */
#define EIND	(*(volatile unsigned char *)0x5C)

/* Stack Pointer */
#define SP  	(*(volatile unsigned int *)0x5D)
#define SPL 	(*(volatile unsigned char *)0x5D)
#define SPH 	(*(volatile unsigned char *)0x5E)

/* Status REGister */
#define SREG	(*(volatile unsigned char *)0x5F)


/* bits */

/* Port A */
#define  PORTA7   7
#define  PORTA6   6
#define  PORTA5   5
#define  PORTA4   4
#define  PORTA3   3
#define  PORTA2   2
#define  PORTA1   1
#define  PORTA0   0
#define  PA7      7
#define  PA6      6
#define  PA5      5
#define  PA4      4
#define  PA3      3
#define  PA2      2
#define  PA1      1
#define  PA0      0
#define  DDA7     7
#define  DDA6     6
#define  DDA5     5
#define  DDA4     4
#define  DDA3     3
#define  DDA2     2
#define  DDA1     1
#define  DDA0     0
#define  PINA7    7
#define  PINA6    6
#define  PINA5    5
#define  PINA4    4
#define  PINA3    3
#define  PINA2    2
#define  PINA1    1
#define  PINA0    0

/* Port B */
#define  PORTB7   7
#define  PORTB6   6
#define  PORTB5   5
#define  PORTB4   4
#define  PORTB3   3
#define  PORTB2   2
#define  PORTB1   1
#define  PORTB0   0
#define  PB7      7
#define  PB6      6
#define  PB5      5
#define  PB4      4
#define  PB3      3
#define  PB2      2
#define  PB1      1
#define  PB0      0
#define  DDB7     7
#define  DDB6     6
#define  DDB5     5
#define  DDB4     4
#define  DDB3     3
#define  DDB2     2
#define  DDB1     1
#define  DDB0     0
#define  PINB7    7
#define  PINB6    6
#define  PINB5    5
#define  PINB4    4
#define  PINB3    3
#define  PINB2    2
#define  PINB1    1
#define  PINB0    0

/* Port C */
#define  PORTC7   7
#define  PORTC6   6
#define  PORTC5   5
#define  PORTC4   4
#define  PORTC3   3
#define  PORTC2   2
#define  PORTC1   1
#define  PORTC0   0
#define  PC7      7
#define  PC6      6
#define  PC5      5
#define  PC4      4
#define  PC3      3
#define  PC2      2
#define  PC1      1
#define  PC0      0
#define  DDC7     7
#define  DDC6     6
#define  DDC5     5
#define  DDC4     4
#define  DDC3     3
#define  DDC2     2
#define  DDC1     1
#define  DDC0     0
#define  PINC7    7
#define  PINC6    6
#define  PINC5    5
#define  PINC4    4
#define  PINC3    3
#define  PINC2    2
#define  PINC1    1
#define  PINC0    0

/* Port D */
#define  PORTD7   7
#define  PORTD6   6
#define  PORTD5   5
#define  PORTD4   4
#define  PORTD3   3
#define  PORTD2   2
#define  PORTD1   1
#define  PORTD0   0
#define  PD7      7
#define  PD6      6
#define  PD5      5
#define  PD4      4
#define  PD3      3
#define  PD2      2
#define  PD1      1
#define  PD0      0
#define  DDD7     7
#define  DDD6     6
#define  DDD5     5
#define  DDD4     4
#define  DDD3     3
#define  DDD2     2
#define  DDD1     1
#define  DDD0     0
#define  PIND7    7
#define  PIND6    6
#define  PIND5    5
#define  PIND4    4
#define  PIND3    3
#define  PIND2    2
#define  PIND1    1
#define  PIND0    0

/* Port E */
#define  PORTE7   7
#define  PORTE6   6
#define  PORTE5   5
#define  PORTE4   4
#define  PORTE3   3
#define  PORTE2   2
#define  PORTE1   1
#define  PORTE0   0
#define  PE7      7
#define  PE6      6
#define  PE5      5
#define  PE4      4
#define  PE3      3
#define  PE2      2
#define  PE1      1
#define  PE0      0
#define  DDE7     7
#define  DDE6     6
#define  DDE5     5
#define  DDE4     4
#define  DDE3     3
#define  DDE2     2
#define  DDE1     1
#define  DDE0     0
#define  PINE7    7
#define  PINE6    6
#define  PINE5    5
#define  PINE4    4
#define  PINE3    3
#define  PINE2    2
#define  PINE1    1
#define  PINE0    0

/* Port F */
#define  PORTF7   7
#define  PORTF6   6
#define  PORTF5   5
#define  PORTF4   4
#define  PORTF3   3
#define  PORTF2   2
#define  PORTF1   1
#define  PORTF0   0
#define  PF7      7
#define  PF6      6
#define  PF5      5
#define  PF4      4
#define  PF3      3
#define  PF2      2
#define  PF1      1
#define  PF0      0
#define  DDF7     7
#define  DDF6     6
#define  DDF5     5
#define  DDF4     4
#define  DDF3     3
#define  DDF2     2
#define  DDF1     1
#define  DDF0     0
#define  PINF7    7
#define  PINF6    6
#define  PINF5    5
#define  PINF4    4
#define  PINF3    3
#define  PINF2    2
#define  PINF1    1
#define  PINF0    0

/* Port G */
#define  PORTG5   5
#define  PORTG4   4
#define  PORTG3   3
#define  PORTG2   2
#define  PORTG1   1
#define  PORTG0   0
#define  PG5      5
#define  PG4      4
#define  PG3      3
#define  PG2      2
#define  PG1      1
#define  PG0      0
#define  DDG5     5
#define  DDG4     4
#define  DDG3     3
#define  DDG2     2
#define  DDG1     1
#define  DDG0     0
#define  PING5    5
#define  PING4    4
#define  PING3    3
#define  PING2    2
#define  PING1    1
#define  PING0    0

/* Port H */
#define  PORTH7   7
#define  PORTH6   6
#define  PORTH5   5
#define  PORTH4   4
#define  PORTH3   3
#define  PORTH2   2
#define  PORTH1   1
#define  PORTH0   0
#define  PH7      7
#define  PH6      6
#define  PH5      5
#define  PH4      4
#define  PH3      3
#define  PH2      2
#define  PH1      1
#define  PH0      0
#define  DDH7     7
#define  DDH6     6
#define  DDH5     5
#define  DDH4     4
#define  DDH3     3
#define  DDH2     2
#define  DDH1     1
#define  DDH0     0
#define  PINH7    7
#define  PINH6    6
#define  PINH5    5
#define  PINH4    4
#define  PINH3    3
#define  PINH2    2
#define  PINH1    1
#define  PINH0    0

/* Port J */
#define  PORTJ7   7
#define  PORTJ6   6
#define  PORTJ5   5
#define  PORTJ4   4
#define  PORTJ3   3
#define  PORTJ2   2
#define  PORTJ1   1
#define  PORTJ0   0
#define  PJ7      7
#define  PJ6      6
#define  PJ5      5
#define  PJ4      4
#define  PJ3      3
#define  PJ2      2
#define  PJ1      1
#define  PJ0      0
#define  DDJ7     7
#define  DDJ6     6
#define  DDJ5     5
#define  DDJ4     4
#define  DDJ3     3
#define  DDJ2     2
#define  DDJ1     1
#define  DDJ0     0
#define  PINJ7    7
#define  PINJ6    6
#define  PINJ5    5
#define  PINJ4    4
#define  PINJ3    3
#define  PINJ2    2
#define  PINJ1    1
#define  PINJ0    0

/* Port K */
#define  PORTK7   7
#define  PORTK6   6
#define  PORTK5   5
#define  PORTK4   4
#define  PORTK3   3
#define  PORTK2   2
#define  PORTK1   1
#define  PORTK0   0
#define  PK7      7
#define  PK6      6
#define  PK5      5
#define  PK4      4
#define  PK3      3
#define  PK2      2
#define  PK1      1
#define  PK0      0
#define  DDK7     7
#define  DDK6     6
#define  DDK5     5
#define  DDK4     4
#define  DDK3     3
#define  DDK2     2
#define  DDK1     1
#define  DDK0     0
#define  PINK7    7
#define  PINK6    6
#define  PINK5    5
#define  PINK4    4
#define  PINK3    3
#define  PINK2    2
#define  PINK1    1
#define  PINK0    0

/* Port L */
#define  PORTL7   7
#define  PORTL6   6
#define  PORTL5   5
#define  PORTL4   4
#define  PORTL3   3
#define  PORTL2   2
#define  PORTL1   1
#define  PORTL0   0
#define  PL7      7
#define  PL6      6
#define  PL5      5
#define  PL4      4
#define  PL3      3
#define  PL2      2
#define  PL1      1
#define  PL0      0
#define  DDL7     7
#define  DDL6     6
#define  DDL5     5
#define  DDL4     4
#define  DDL3     3
#define  DDL2     2
#define  DDL1     1
#define  DDL0     0
#define  PINL7    7
#define  PINL6    6
#define  PINL5    5
#define  PINL4    4
#define  PINL3    3
#define  PINL2    2
#define  PINL1    1
#define  PINL0    0

/* PCMSK2 */
#define  PCINT23  7
#define  PCINT22  6
#define  PCINT21  5
#define  PCINT20  4
#define  PCINT19  3
#define  PCINT18  2
#define  PCINT17  1
#define  PCINT16  0
/* PCMSK1 */
#define  PCINT15  7
#define  PCINT14  6
#define  PCINT13  5
#define  PCINT12  4
#define  PCINT11  3
#define  PCINT10  2
#define  PCINT9   1
#define  PCINT8   0
/* PCMSK0 */
#define  PCINT7   7
#define  PCINT6   6
#define  PCINT5   5
#define  PCINT4   4
#define  PCINT3   3
#define  PCINT2   2
#define  PCINT1   1
#define  PCINT0   0


/* Lock and Fuse Bits with LPM/SPM instructions */

/* lock bits */
#define  BLB12    5
#define  BLB11    4
#define  BLB02    3
#define  BLB01    2
#define  LB2      1
#define  LB1      0

/* fuses low bits */
#define  CKDIV8   7
#define  CKOUT    6
#define  SUT1     5
#define  SUT0     4
#define  CKSEL3   3
#define  CKSEL2   2
#define  CKSEL1   1
#define  CKSEL0   0

/* fuses high bits */
#define  OCDEN    7
#define  JTAGEN   6
#define  SPIEN    5
#define  WDTON    4
#define  EESAVE   3
#define  BOOTSZ1  2
#define  BOOTSZ0  1
#define  BOOTRST  0

/* extended fuses */
#define  BODLEVEL2 2
#define  BODLEVEL1 1
#define  BODLEVEL0 0


/* Interrupt Vector Numbers */

#define iv_RESET        1
#define iv_INT0         2
#define iv_EXT_INT0     2
#define iv_INT1         3
#define iv_EXT_INT1     3
#define iv_INT2         4
#define iv_EXT_INT2     4
#define iv_INT3         5
#define iv_EXT_INT3     5
#define iv_INT4         6
#define iv_EXT_INT4     6
#define iv_INT5         7
#define iv_EXT_INT5     7
#define iv_INT6         8
#define iv_EXT_INT6     8
#define iv_INT7         9
#define iv_EXT_INT7     9
#define iv_PCINT0       10
#define iv_PCINT1       11
#define iv_PCINT2       12
#define iv_WDT          13
#define iv_TIMER2_COMPA 14
#define iv_TIMER2_COMPB 15
#define iv_TIMER2_OVF   16
#define iv_TIM2_COMPA   14
#define iv_TIM2_COMPB   15
#define iv_TIM2_OVF     16
#define iv_TIMER1_CAPT  17
#define iv_TIMER1_COMPA 18
#define iv_TIMER1_COMPB 19
#define iv_TIMER1_COMPC 20
#define iv_TIMER1_OVF   21
#define iv_TIM1_CAPT    17
#define iv_TIM1_COMPA   18
#define iv_TIM1_COMPB   19
#define iv_TIM1_COMPC   20
#define iv_TIM1_OVF     21
#define iv_TIMER0_COMPA 22
#define iv_TIMER0_COMPB 23
#define iv_TIMER0_OVF   24
#define iv_TIM0_COMPA   22
#define iv_TIM0_COMPB   23
#define iv_TIM0_OVF     24
#define iv_SPI_STC      25
#define iv_USART0_RX    26
#define iv_USART0_RXC   26
#define iv_USART0_DRE   27
#define iv_USART0_UDRE  27
#define iv_USART0_TX    28
#define iv_USART0_TXC   28
#define iv_ANA_COMP     29
#define iv_ANALOG_COMP  29
#define iv_ADC          30
#define iv_EE_RDY       31
#define iv_EE_READY     31
#define iv_TIMER3_CAPT  32
#define iv_TIMER3_COMPA 33
#define iv_TIMER3_COMPB 34
#define iv_TIMER3_COMPC 35
#define iv_TIMER3_OVF   36
#define iv_TIM3_CAPT    32
#define iv_TIM3_COMPA   33
#define iv_TIM3_COMPB   34
#define iv_TIM3_COMPC   35
#define iv_TIM3_OVF     36
#define iv_USART1_RX    37
#define iv_USART1_RXC   37
#define iv_USART1_DRE   38
#define iv_USART1_UDRE  38
#define iv_USART1_TX    39
#define iv_USART1_TXC   39
#define iv_TWI          40
#define iv_TWSI         40
#define iv_SPM_RDY      41
#define iv_SPM_READY    41
#define iv_TIMER4_CAPT  42
#define iv_TIMER4_COMPA 43
#define iv_TIMER4_COMPB 44
#define iv_TIMER4_COMPC 45
#define iv_TIMER4_OVF   46
#define iv_TIM4_CAPT    42
#define iv_TIM4_COMPA   43
#define iv_TIM4_COMPB   44
#define iv_TIM4_COMPC   45
#define iv_TIM4_OVF     46
#define iv_TIMER5_CAPT  47
#define iv_TIMER5_COMPA 48
#define iv_TIMER5_COMPB 49
#define iv_TIMER5_COMPC 50
#define iv_TIMER5_OVF   51
#define iv_TIM5_CAPT    47
#define iv_TIM5_COMPA   48
#define iv_TIM5_COMPB   49
#define iv_TIM5_COMPC   50
#define iv_TIM5_OVF     51
#define iv_USART2_RX    52
#define iv_USART2_RXC   52
#define iv_USART2_DRE   53
#define iv_USART2_UDRE  53
#define iv_USART2_TX    54
#define iv_USART2_TXC   54
#define iv_USART3_RX    55
#define iv_USART3_RXC   55
#define iv_USART3_DRE   56
#define iv_USART3_UDRE  56
#define iv_USART3_TX    57
#define iv_USART3_TXC   57

/*ja*/

#endif
