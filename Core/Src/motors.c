#include "motors.h"
#include "FreeRTOS.h"
#include "stm32f1xx_hal_tim.h"
#include "task.h"

// --- UCHWYTY TIMERÓW ---
extern TIM_HandleTypeDef htim1; // Dla Talon SRX (PB14)
extern TIM_HandleTypeDef htim3; // Dla MC34931 IN1/IN2 (PA6, PA7, PB0, PB1)
extern TIM_HandleTypeDef htim4;
// --- DEFINICJE KANAŁÓW TALON SRX (WIERTŁO NAPĘD) ---
#define TALON_TIM &htim1
#define TALON_CH  TIM_CHANNEL_2 // Dla PB14 używamy TIM1_CH2N (komplementarny)

// --- DEFINICJE KANAŁÓW MC34931 (MIESZADŁO) ---
// PA6 (IN2), PA7 (IN1)
#define STIRRER_IN2_CH TIM_CHANNEL_1
#define STIRRER_IN1_CH TIM_CHANNEL_2

// --- DEFINICJE KANAŁÓW MC34931 (OPUSZCZANIE WIERTŁA) ---
// PB0 (IN2), PB1 (IN1)
#define DRILL_LOWER_IN2_CH TIM_CHANNEL_3
#define DRILL_LOWER_IN1_CH TIM_CHANNEL_4

extern TIM_HandleTypeDef htim4;

volatile int32_t encoder_overflows = 0;
// ==========================================================
// 1. NAPĘD WIERTŁA (TALON SRX - Sygnał RC 50Hz)
// ==========================================================
void SetDrillSpinSpeed_Talon(int8_t speed_percent)
{
    if (speed_percent > 100)
        speed_percent = 100;
    if (speed_percent < -100)
        speed_percent = -100;

    // Przeliczenie procentów na długość impulsu (1000 do 2000)
    // -100% = 1000 (BWD), 0% = 1500 (STOP), 100% = 2000 (FWD)
    uint16_t pulse = 1500 + (speed_percent * 5);

    __HAL_TIM_SET_COMPARE(TALON_TIM, TALON_CH, pulse);
}

// ==========================================================
// 2. OPUSZCZANIE WIERTŁA (MC34931)
// ==========================================================
void SetDrillLoweringSpeed_MC34931(uint16_t speed_pwm, uint8_t direction)
{
    // direction: 0 = STOP, 1 = W DÓŁ, 2 = W GÓRĘ
    if (speed_pwm == 0 || direction == 0)
    {
        // Obie linie na 0 -> STOP
        __HAL_TIM_SET_COMPARE(&htim3, DRILL_LOWER_IN1_CH, 0);
        __HAL_TIM_SET_COMPARE(&htim3, DRILL_LOWER_IN2_CH, 0);
    }
    else if (direction == 1)
    { // W DÓŁ
        // IN1 na 0, IN2 dostaje PWM[cite: 2]
        __HAL_TIM_SET_COMPARE(&htim3, DRILL_LOWER_IN1_CH, 0);
        __HAL_TIM_SET_COMPARE(&htim3, DRILL_LOWER_IN2_CH, speed_pwm);
    }
    else if (direction == 2)
    { // W GÓRĘ
        // IN2 na 0, IN1 dostaje PWM[cite: 2]
        __HAL_TIM_SET_COMPARE(&htim3, DRILL_LOWER_IN2_CH, 0);
        __HAL_TIM_SET_COMPARE(&htim3, DRILL_LOWER_IN1_CH, speed_pwm);
    }
}

// ==========================================================
// 3. MIESZADŁO (MC34931)
// ==========================================================
void SetStirrerSpeed_MC34931(uint16_t speed_pwm, uint8_t direction)
{
    // direction: 0 = STOP, 1 = KIERUNEK A, 2 = KIERUNEK B
    if (speed_pwm == 0 || direction == 0)
    {
        // Obie linie na 0 -> STOP[cite: 2]
        __HAL_TIM_SET_COMPARE(&htim3, STIRRER_IN1_CH, 0);
        __HAL_TIM_SET_COMPARE(&htim3, STIRRER_IN2_CH, 0);
    }
    else if (direction == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim3, STIRRER_IN1_CH, 0);
        __HAL_TIM_SET_COMPARE(&htim3, STIRRER_IN2_CH, speed_pwm);
    }
    else if (direction == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim3, STIRRER_IN2_CH, 0);
        __HAL_TIM_SET_COMPARE(&htim3, STIRRER_IN1_CH, speed_pwm);
    }
}

// ==========================================================
// 4. INICJALIZACJA SYSTEMU SILNIKÓW
// ==========================================================
void MotorsControl_Init(void)
{
    // ------------------------------------------------------
    // A. Konfiguracja pinów włączających MC34931 (GPIO)
    // ------------------------------------------------------
    // Wiertło opuszczanie (D1 = PB2, EN/D2 = PB10)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // D1 = LOW (Disable = False)[cite: 2]
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  // EN/D2 = HIGH (Wake Up)[cite: 2]

    // Mieszadło (D1 = PA3, EN/D2 = PB11)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // D1 = LOW[cite: 2]
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);  // EN/D2 = HIGH[cite: 2]

    // ------------------------------------------------------
    // B. Start kanałów PWM (TIM3 - MC34931)
    // ------------------------------------------------------
    // Upewnij się, że startują z wypełnieniem 0
    SetDrillLoweringSpeed_MC34931(0, 0);
    SetStirrerSpeed_MC34931(0, 0);

    HAL_TIM_PWM_Start(&htim3, STIRRER_IN2_CH);     // PA6
    HAL_TIM_PWM_Start(&htim3, STIRRER_IN1_CH);     // PA7
    HAL_TIM_PWM_Start(&htim3, DRILL_LOWER_IN2_CH); // PB0
    HAL_TIM_PWM_Start(&htim3, DRILL_LOWER_IN1_CH); // PB1

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

    // ------------------------------------------------------
    // C. Start kanału PWM (TIM1 - Talon SRX)
    // ------------------------------------------------------
    SetDrillSpinSpeed_Talon(0); // Sygnał 1.5ms (STOP)
    // Dla PB14 w F103 używamy startu kanału komplementarnego (TIM1_CH2N)
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}

// Deklaracja timera enkodera (np. TIM4, sprawdź w CubeMX czy tak go nazwałeś)

// Zatrzymuje wszystko (SCRAM)
void EmergencyStopMotors(void)
{
    SetDrillSpinSpeed_Talon(0);
    SetDrillLoweringSpeed_MC34931(0, 0);
    SetStirrerSpeed_MC34931(0, 0);
}

// Wrapper do zatrzymania mieszadła
void StopStirrer(void) { SetStirrerSpeed_MC34931(0, 0); }

int32_t GetDrillPosition(void)
{
    int32_t overflows;
    uint32_t current_cnt;

    // Używamy sekcji krytycznej FreeRTOS, aby przerwanie nie wpadło
    // dokładnie pomiędzy odczytem 'overflows' a odczytem 'CNT'.
    // To zapobiega tzw. Race Conditions (błędom jednoczesnego dostępu).
    taskENTER_CRITICAL();
    overflows = encoder_overflows;
    current_cnt = __HAL_TIM_GET_COUNTER(&htim4);
    taskEXIT_CRITICAL();

    // Całkowita pozycja to: (ilość_przepełnień * 65536) + aktualny_stan_licznika
    return (overflows << 16) | current_cnt;
}

void ResetDrillEncoder(void)
{
    taskENTER_CRITICAL();
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    encoder_overflows = 0;
    taskEXIT_CRITICAL();
}

// Własna funkcja obsługi przepełnienia enkodera
void DrillEncoder_OverflowCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM4)
    {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4))
        {
            encoder_overflows--;
        }
        else
        {
            encoder_overflows++;
        }
    }
}

// Sprawdzenie czy wiertło jest na odpowiedniej głębokości
uint8_t IsDrillAtTargetDepth(int32_t target_ticks)
{
    if (GetDrillPosition() >= target_ticks)
    {
        return 1; // Osiągnięto cel
    }
    return 0; // Wiertło nadal jedzie
}

// Krańcówka (Homing) na PB12
uint8_t IsDrillHomed(void)
{
    // Zwraca 1 jeśli krańcówka jest wciśnięta (zakładamy, że zwiera do GND, więc stan LOW = wciśnięta)
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) ? 1 : 0;
}

// Zwraca 1, jeśli przycisk S_SWITCH (PB13) jest wciśnięty (zwarty do masy)
uint8_t IsStartSwitchPressed(void) { return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) ? 1 : 0; }