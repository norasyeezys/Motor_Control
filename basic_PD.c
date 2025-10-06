/* basic_PD.c  — ATmega328P @16 MHz
   Map (from your board):
   STEP = PD2 (X), PD4 (Z)
   DIR  = PD5 (X), PD7 (Z)
   EN   = PB0 (active-LOW)
*/

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

/* Pins */
#define X_STEP_BIT  PD2
#define Z_STEP_BIT  PD4
#define X_DIR_BIT   PD5
#define Z_DIR_BIT   PD7
#define EN_BIT      PB0   /* active-LOW */

/* --------- Minimal UART TX @115200 8N1 ---------- */
static void uart_init(void){
  /* 115200 @ 16 MHz: UBRR = 8 (normal speed) */
  uint16_t ubrr = 8;
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)(ubrr & 0xFF);
  UCSR0A = 0x00;                         /* normal speed */
  UCSR0B = (1<<TXEN0);                   /* TX enable */
  UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);      /* 8N1 */
}
static void uart_putc(char c){
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = c;
}
static void uart_print(const char *s){
  while(*s){ uart_putc(*s++); }
}

/* --------- Driver control & stepping ---------- */
static inline void enable_drivers(void){
  DDRB  |= (1<<EN_BIT);
  PORTB &= ~(1<<EN_BIT);   /* EN = LOW (active) */
}
static inline void pulse_step(volatile uint8_t *port, uint8_t bit){
  *port |=  (1<<bit);
  _delay_us(300);                  /* long, opto-friendly pulse */
  *port &= ~(1<<bit);
  _delay_us(300);
}
static void move_axis(volatile uint8_t *s_port, uint8_t s_bit,
                      volatile uint8_t *d_port, uint8_t d_bit,
                      uint16_t n){
  uint16_t i;
  *d_port |=  (1<<d_bit); _delay_ms(5);
  for(i=0;i<n;i++) pulse_step(s_port, s_bit);
  _delay_ms(150);
  *d_port &= ~(1<<d_bit); _delay_ms(5);
  for(i=0;i<n;i++) pulse_step(s_port, s_bit);
  _delay_ms(250);
}

int desired_position = 50;
int Kp = 5;
int Kd = 1;
int current_position = 0;
int previous_error = 0;
int error;
int derivative;
int pd_output;
int move;

int main(void){
  /* Configure only X and Z pins as outputs; leave Y untouched */
  DDRD |= (1<<X_STEP_BIT) | (1<<Z_STEP_BIT) | (1<<X_DIR_BIT) | (1<<Z_DIR_BIT);
  PORTD &= ~((1<<X_STEP_BIT)|(1<<Z_STEP_BIT)|(1<<X_DIR_BIT)|(1<<Z_DIR_BIT));

  enable_drivers();

  uart_init();
  uart_print("\r\nIch baue dir einen Körper, Siegfried. Wenn du in diese Welt zurückkehrst, wirst du ganz sein, auch wenn es nicht aus Fleisch und Blut, sondern aus Maschine und Code besteht.\r\n");

  for(;;){
    error = desired_position - current_position;
    derivative = error - previous_error;

    pd_output =  Kp * error + Kd * derivative;
    move = abs(pd_output); // X and Z get the same abs.

    uart_putc('X'); uart_putc('0'+(move%10)); uart_putc('\n');

    /* Fake out Sensor values */
    current_position = pd_output - desired_position;

    /* Set the direction before moving */
    if (pd_output > 0) {
      PORTD |= (1<<X_DIR_BIT);   // Move in positive direction
    } else {
      PORTD &= ~(1<<X_DIR_BIT);  // Move in negative direction
    }

    if (pd_output < 0) {
      PORTD |= (1<<Z_DIR_BIT);   // Move in positive direction
    } else {
      PORTD &= ~(1<<Z_DIR_BIT);  // Move in negative direction
    }

    move_axis(&PORTD, X_STEP_BIT, &PORTD, X_DIR_BIT, move);
    move_axis(&PORTD, Z_STEP_BIT, &PORTD, Z_DIR_BIT, move);
    _delay_ms(400);
  }
}
