/*******************************************************
Project : Color Sensor with ATmega16A
Version : 
Date    : 22/01/2018
Author  : Fatemeh Salehi
Company : 
Comments: Color detection system using timer-based sampling


Chip type               : ATmega16A
Program type            : Application
AVR Core Clock frequency: 1.000000 MHz
Memory model            : Small
External RAM size       : 0                  
Data Stack size         : 256
*******************************************************/

#include "color_sensor.h"
#include <lcd.h>
#include <stdio.h>
#include <delay.h>

#define COLOR_SENSOR_S0_PIN    PINB0
#define COLOR_SENSOR_S1_PIN    PINB1
#define COLOR_SENSOR_S2_PIN    PINB2
#define COLOR_SENSOR_S3_PIN    PINB3
#define COLOR_SENSOR_OUT_PIN   PINB4

// Type definitions
typedef enum {
    WAITING_FOR_RISE,
    WAITING_FOR_FALL
} CaptureState;

typedef struct {
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
} ColorValues;

// Private variables
static uint16_t capture_start;
static uint16_t capture_end;
static uint16_t pulse_duration;
static CaptureState capture_state = WAITING_FOR_RISE;
static ColorValues color_readings;
static char lcd_buffer[16];

// Private function prototypes
static void init_ports(void);
static void init_timer(void);
static void init_lcd(void);
static void init_peripherals(void);
static void display_color_values(const ColorValues* colors);

// Implementation of private functions
static void init_ports(void)
{
    // Port A initialization - Used for LCD control
    // Function: All pins as inputs initially
    // State: All pins tri-stated (high impedance)
    DDRA = (0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | 
           (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
    PORTA = (0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | 
            (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

    // Port B initialization - Used for color sensor control
    // Function: Bits 0-4 as outputs, others as inputs
    DDRB = (0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (1<<DDB4) | 
           (1<<DDB3) | (1<<DDB2) | (1<<DDB1) | (1<<DDB0);
    PORTB = (0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | 
            (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

    // Port C initialization - Used for LCD data
    DDRC = (0<<DDC7) | (0<<DDC6) | (0<<DDC5) | (0<<DDC4) | 
           (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
    PORTC = (0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | 
            (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

    // Port D initialization - General purpose I/O
    DDRD = (0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | 
           (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
    PORTD = (0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | 
            (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);
}

void init_timer(void)
{
    // Timer/Counter 0 initialization
    // Clock source: System Clock
    // Clock value: Timer 0 Stopped
    // Mode: Normal top=0xFF
    // OC0 output: Disconnected
    TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<CS02) | (0<<CS01) | (0<<CS00);
    TCNT0=0x00;
    OCR0=0x00;

    // Timer1 initialization for input capture
    // Clock source: System Clock
    // Clock value: 1000.000 kHz
    // Mode: Normal top=0xFFFF
    // OC1A output: Disconnected
    // OC1B output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Rising Edge
    // Timer Period: 65.536 ms
    // Timer1 Overflow Interrupt: Off
    // Input Capture Interrupt: On
    // Compare A Match Interrupt: Off
    // Compare B Match Interrupt: Off
    TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
    TCCR1B=(0<<ICNC1) | (1<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
    TCNT1H=0x00;
    TCNT1L=0x00;
    ICR1H=0x00;
    ICR1L=0x00;
    OCR1AH=0x00;
    OCR1AL=0x00;
    OCR1BH=0x00;
    OCR1BL=0x00;
    
    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: Timer2 Stopped
    // Mode: Normal top=0xFF
    // OC2 output: Disconnected
    ASSR=0<<AS2;
    TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (0<<CS21) | (0<<CS20);
    TCNT2=0x00;
    OCR2=0x00;

    // Timer(s)/Counter(s) Interrupt(s) initialization
    TIMSK=(0<<OCIE2) | (0<<TOIE2) | (1<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (0<<TOIE0);
}

static void init_lcd(void)
{
    // LCD initialization
    // Connections are specified in the
    // Project|Configure|C Compiler|Libraries|Alphanumeric LCD menu:
    // RS - PORTA Bit 0
    // RD - PORTA Bit 1
    // EN - PORTA Bit 2
    // D4 - PORTA Bit 4
    // D5 - PORTA Bit 5
    // D6 - PORTA Bit 6
    // D7 - PORTA Bit 7
    // Characters/line: 16

    lcd_init(16);
    #asm
        .equ __lcd_port=0x15 ;PORTC
    #endasm
}

static void init_peripherals(void)
{
    // External Interrupt(s) initialization
    // INT0: Off
    // INT1: Off
    // INT2: Off
    MCUCR=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
    MCUCSR=(0<<ISC2);
    
    // USART initialization
    // USART disabled
    UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (0<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
    
    // Analog Comparator initialization
    // Analog Comparator: Off
    // The Analog Comparator's positive input is
    // connected to the AIN0 pin
    // The Analog Comparator's negative input is
    // connected to the AIN1 pin
    ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
    SFIOR=(0<<ACME);
    
    // ADC initialization
    // ADC disabled
    ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
    
    // SPI initialization
    // SPI disabled
    SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);
    
    // TWI initialization
    // TWI disabled
    TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);
}

void color_sensor_init(void)
{
    init_ports();
    init_timer();
    init_peripherals();
    init_lcd();
    
    // Initialize sensor frequency scaling (2%)
    PORTB &= ~((1 << COLOR_SENSOR_S0_PIN) | (1 << COLOR_SENSOR_S1_PIN));
}

ColorValues color_sensor_read(void)
{
    ColorValues readings;
    
    // Read RED
    PORTB &= ~((1 << COLOR_SENSOR_S2_PIN) | (1 << COLOR_SENSOR_S3_PIN));  // S2=0, S3=0
    delay_ms(10);  // Allow reading to stabilize
    readings.red = pulse_duration;
    
    // Read GREEN
    PORTB |= (1 << COLOR_SENSOR_S2_PIN) | (1 << COLOR_SENSOR_S3_PIN);     // S2=1, S3=1
    delay_ms(10);
    readings.green = pulse_duration;
    
    // Read BLUE
    PORTB &= ~(1 << COLOR_SENSOR_S2_PIN);                                 // S2=0
    PORTB |= (1 << COLOR_SENSOR_S3_PIN);                                 // S3=1
    delay_ms(10);
    readings.blue = pulse_duration;
    
    // Read CLEAR
    PORTB |= (1 << COLOR_SENSOR_S2_PIN);                                 // S2=1
    PORTB &= ~(1 << COLOR_SENSOR_S3_PIN);                               // S3=0
    delay_ms(10);
    readings.clear = pulse_duration;
    
    return readings;
}

// Timer1 input capture interrupt handler
interrupt [TIM1_CAPT] void timer1_capt_isr(void)
{
    if(capture_state == WAITING_FOR_RISE) {
        capture_start = ICR1L | (ICR1H << 8);   
        
        // Configure for falling edge detection
        TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | 
                 (0<<CS12) | (0<<CS11) | (1<<CS10); 
        
        capture_state = WAITING_FOR_FALL;
    } else {
        capture_end = ICR1L | (ICR1H << 8);
        pulse_duration = capture_end - capture_start;   
        
        // Configure for rising edge detection
        TCCR1B = (0<<ICNC1) | (1<<ICES1) | (0<<WGM13) | (0<<WGM12) | 
                 (0<<CS12) | (0<<CS11) | (1<<CS10);
        
        capture_state = WAITING_FOR_RISE;
    }              
}

static void display_color_values(const ColorValues* colors)
{
    lcd_clear();
    sprintf(lcd_buffer, "RED=%d", colors->red);
    lcd_puts(lcd_buffer);
    lcd_gotoxy(0, 1);
    sprintf(lcd_buffer, "GREEN=%d", colors->green);
    lcd_puts(lcd_buffer);
    delay_ms(1000);
    lcd_clear();
    sprintf(lcd_buffer, "BLUE=%d", colors->blue);
    lcd_puts(lcd_buffer);
    lcd_gotoxy(0, 1);
    sprintf(lcd_buffer, "CLEAR=%d", colors->clear);
    lcd_puts(lcd_buffer);
    delay_ms(1000);
}

int main(void)
{
    color_sensor_init();
    
    while (1) {
        ColorValues current_reading = color_sensor_read();
        display_color_values(&current_reading);
        delay_ms(100); 
    }
}
