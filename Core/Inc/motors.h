#ifndef MOTORS_H
#define MOTORS_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void EmergencyStopMotors(void);
void StopStirrer(void);
int32_t GetDrillPosition(void);
uint8_t IsDrillAtTargetDepth(int32_t target_ticks);
void MotorsControl_Init(void);
void SetDrillSpinSpeed_Talon(int8_t speed_percent);
void SetDrillLoweringSpeed_MC34931(uint16_t speed_pwm, uint8_t direction);
void SetStirrerSpeed_MC34931(uint16_t speed_pwm, uint8_t direction);
uint8_t IsDrillHomed(void);
void ResetDrillEncoder(void); // <--- DODANE
uint8_t IsStartSwitchPressed(void);
void DrillEncoder_OverflowCallback(TIM_HandleTypeDef* htim);

#endif