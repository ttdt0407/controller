#include "timer.h"

void timer1_setup()
{
  cli();
   TCCR1A = 0;
   TCCR1B = 0;
   TIMSK1 = 0;
   OCR1A = 0;
   OCR1B = 0;
   TCNT1 = 0;
   ICR1 = 0;
   TCCR1A |=  (1 << COM1A1)| (1 << COM1B1)| (1 << WGM11) | (1 << WGM10);
   TCCR1B |= (1 << ICES1)  | (1 << CS10)| (1 << WGM12);    // prescale = 1
   TIMSK1 = (1 << ICIE1)  |(1 << TOIE1);
   sei();
}

void timer3_setup()
{
  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TIMSK3 = 0;
  TCCR3B |= (1 << WGM33) |(1 << WGM32) | (1 << CS31);
  TCCR3A |= (1 << WGM31) |  (1 << COM3A1)| (1 << COM3B1);
  ICR3 = 39999;
  OCR3A = 0;
  OCR3B = 0;
  TIMSK3 = (1 << TOIE3);
  sei();
}
