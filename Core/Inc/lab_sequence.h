#ifndef LAB_SEQUENCE_H
#define LAB_SEQUENCE_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

typedef struct
{
    uint32_t StdId;
    uint8_t Data[8];
} CanMsg_t;

extern QueueHandle_t xCanMsgQueue;

void LabRTOS_Init(void); // Funkcja tworząca taski i obiekty RTOS

#endif