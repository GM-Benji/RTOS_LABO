#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// Definicje ID Twoich serw
#define DYNAMIXEL_TUBE_ID    0x1E // 30
#define DYNAMIXEL_SYRINGE_ID 0x01 // 19

// Instrukcje i adresy rejestrów AX-12A
#define INST_PING           0x01
#define INST_READ_DATA      0x02
#define INST_WRITE_DATA     0x03
#define ADDR_CW_ANGLE_LIMIT 0x06
#define ADDR_LED            0x19
#define ADDR_GOAL_POSITION  0x1E
#define ADDR_MOVING_SPEED   0x20

// Deklaracje funkcji
void AX12_Init(void);
void AX12_SetGoalPosition(uint8_t id, uint16_t position);
void AX12_SetMode_Joint(uint8_t id);
void AX12_SetLED(uint8_t id, uint8_t state);
uint8_t AX12_Ping(uint8_t id);

#endif