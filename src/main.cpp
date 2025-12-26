#include "encoder.h"
#include "timer.h"

/************************************************
 *  Variables
 ************************************************/
typedef struct
{
  float speed;
} main_measure_speed_t;

static float k_value = 255863.5394f;
volatile uint8_t flag_50hz = 0;
static encoder_t channel_a;
static encoder_t channel_b;
static encoder_update_t channel_a_update;
static encoder_update_t channel_b_update;
static encoder_speed_t speed_a;
static encoder_speed_t speed_b;
static main_measure_speed_t measure_speed;

/************************************************
 *  Prototypes
 ************************************************/

void main_init();
void main_count_pulses_A(void);
void main_count_pulses_B(void);
void main_measure_speed(void);
void set_speed(void);
void backward(void);
void forward(void);
void stop(void);

callback main_arr[] = {main_count_pulses_A, main_count_pulses_B};

/************************************************
 *  API
 ************************************************/

void main_init()
{
    memset(&channel_a, 0, sizeof(channel_a));
    memset(&channel_b, 0, sizeof(channel_b));
    memset(&channel_a_update, 0, sizeof(channel_a_update));
    memset(&channel_b_update, 0, sizeof(channel_b_update));
    memset(&speed_a, 0, sizeof(speed_a));
    memset(&speed_b, 0, sizeof(speed_b));
    memset(&measure_speed, 0, sizeof(measure_speed));
}

/**
 * @brief
 *
 */
void main_count_pulses_A(void)
{
  (channel_a.counter) ? (channel_a.last_pulse = TCNT3) : (channel_a.first_pulse = TCNT3);

  channel_a.read_pins = PIND & B00000011;

  switch (channel_a.read_pins)
  {
    case 0: case 3:
      channel_a.counter++;
      break;
    case 1: case 2:
      channel_a.counter--;
      break;
  }
}

/**
 * @brief
 *
 */
void main_count_pulses_B(void)
{

  (channel_b.counter) ? (channel_b.last_pulse = TCNT3) : (channel_b.first_pulse = TCNT3);

  channel_b.read_pins = PIND & B00000011;

  switch (channel_b.read_pins)
  {
    case 0: case 3:
      channel_b.counter--;
      break;
    case 1: case 2:
      channel_b.counter++;
      break;
  }
}

/**
 * @brief
 *
 */
void main_measure_speed()
{
  /* channel A */
  int16_t a_check = channel_a_update.last_pulse_update - channel_a_update.first_pulse_update;

  if (a_check > 0)
  {
    speed_a.num_of_ticks = a_check;
  }
  else
  {
    speed_a.num_of_ticks = 39999 - channel_a_update.first_pulse_update + channel_a_update.last_pulse_update;
  }

  /* Avoiding divided by zero */
  if (speed_a.num_of_ticks == 0) speed_a.num_of_ticks = 1;

  speed_a.rpm = (float)(abs(channel_a_update.counter_update) - 1) / (speed_a.num_of_ticks) * k_value;

  /* channel B */
  int16_t b_check = channel_b_update.last_pulse_update - channel_b_update.first_pulse_update;

  if (b_check > 0)
  {
  speed_b.num_of_ticks = b_check;
  }
  else
  {
  speed_b.num_of_ticks = 39999 - channel_b_update.first_pulse_update + channel_b_update.last_pulse_update;
  }

  speed_b.rpm = (float)(abs(channel_b_update.counter_update) - 1) / (speed_b.num_of_ticks) * k_value;

  /* ensure that speed is valid */
  measure_speed.speed = (speed_a.rpm + speed_b.rpm) / 2.0;

  if (measure_speed.speed > 400)
  {
    measure_speed.speed = 400;
  }
  else if (measure_speed.speed <= 20)
  {
    measure_speed.speed = 20;
  }

  // Serial.print("Measured speed - ");
  // Serial.println(measure_speed.speed);
}

void set_speed()
{
  OCR1A = 500;
}

void backward()
{
  PORTD &= ~(1<<PD7);
  PORTE |= (1<<PE6);
}
void forward()
{
  PORTE &= ~(1<<PE6);
  PORTD |= (1<<PD7);
}
void stop()
{
  PORTE &= ~(1<<PE6);
  PORTD &= ~(1<<PD7);
}


/**
 * @brief Construct a new ISR object
 *
 */
ISR (TIMER3_OVF_vect)
{
  flag_50hz = 1;

  cli();

  channel_a_update.first_pulse_update = channel_a.first_pulse;
  channel_a_update.last_pulse_update = channel_a.last_pulse;
  channel_a_update.counter_update = channel_a.counter;
  channel_a.counter = 0;

  channel_b_update.first_pulse_update = channel_b.first_pulse;
  channel_b_update.last_pulse_update = channel_b.last_pulse;
  channel_b_update.counter_update = channel_b.counter;
  channel_b.counter = 0;

  sei();
}

// ISR (TIMER1_OVF_vect) {
//  TCNT1 = 24;
//  }

void setup() {

  Serial.begin(115200);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);

  encoder_register_callback(main_arr);

  attachInterrupt(digitalPinToInterrupt(2), encoder_count_handler_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encoder_count_handler_b, CHANGE);

  timer1_setup();
  timer3_setup();

  main_init();
}

void loop() {

  if (flag_50hz)
  {
    flag_50hz = 0;
    set_speed();
    backward();
    main_measure_speed();
  }
}
