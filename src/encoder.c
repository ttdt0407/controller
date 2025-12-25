#include "encoder.h"
#include <string.h>


/************************************************
 *  Variables
 ************************************************/
static callback array_callback[CB_NUMS] = {NULL};

/************************************************
 *  Prototypes
 ************************************************/




/************************************************
 *  API
 ************************************************/

 void encoder_init(encoder_t *self, encoder_update_t *update)
 {
     memset(self, 0, sizeof(*self));
     memset(self, 0, sizeof(*update));
 }

void encoder_register_callback(callback *arr)
{
    for (int i = 0; i < CB_NUMS; i++)
    {
        if (arr[i] != NULL)
        {
            array_callback[i] = arr[i];
        }
    }
}

void encoder_count_handler_a(void)
{
    if (array_callback[0] != NULL)
    {
        array_callback[0]();
    }
}

void encoder_count_handler_b(void)
{
    if (array_callback[1] != NULL)
    {
        array_callback[1]();
    }
}
