#ifndef ENCODER_H
#define ENCODER_H

#include "board.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CB_NUMS 2

/************************************************
 *  Encoder Properties
 ************************************************/

typedef void (*callback)(void);

typedef struct
{
    volatile int16_t counter;
    volatile uint16_t first_pulse;
    volatile uint16_t last_pulse;
    volatile uint8_t read_pins;
} encoder_t;

typedef struct
{
    volatile int16_t counter_update;
    volatile uint16_t first_pulse_update;
    volatile uint16_t last_pulse_update;
    volatile uint8_t read_pins_update;
} encoder_update_t;

typedef struct
{
    volatile uint16_t num_of_ticks;
    volatile float rpm;
} encoder_speed_t;

/************************************************
 *  Encoder Behaviors
 ************************************************/

void encoder_init(encoder_t *self, encoder_update_t *update);
void encoder_register_callback(callback *arr);
void encoder_count_handler_a(void);
void encoder_count_handler_b(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */