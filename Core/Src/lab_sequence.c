#include "lab_sequence.h"
#include "FreeRTOS.h"
#include "can.h"
#include "dynamixel.h"
#include "event_groups.h"
#include "motors.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f1xx_hal_can.h"
#include "task.h"
#include <stdlib.h>

typedef enum
{
    LAB_STATE_IDLE,
    LAB_STATE_HOMING,
    LAB_STATE_HOMING_REVOLVERS,
    LAB_STATE_DRILLING,
    LAB_STATE_RETRACT,
    LAB_STATE_TUBE_POS,
    LAB_STATE_FILL_TUBE,
    LAB_STATE_REAGENT_POS,
    LAB_STATE_DOSING,
    LAB_STATE_STIRRER_POS,
    LAB_STATE_STIRRING,
    LAB_STATE_SPECTROMETER_POS,
    LAB_STATE_SPECTROMETER_FLASH,
    LAB_STATE_RESET_READY
} LabState_t;

QueueHandle_t xCanMsgQueue;

#define BIT_SCRAM_ACTIVE  (1 << 0)
#define BIT_MANUAL_MODE   (1 << 1)
#define BIT_DRILL_LOWERED (1 << 2)
#define BIT_START_AUTO    (1 << 3)

// --- PARAMETRY MECHANICZNE REWOLWERÓW ---
#define POS_SAFE_TUBE    1023
#define POS_SAFE_SYRINGE 600
#define TUBE_BASE_POS    790
#define TUBE_SPACING     123

#define SYRINGE_BASE_POS 485
#define SYRINGE_SPACING  132

#define TUBE_STIR_POS 514

#define TUBE_SPECTRO_POS 750 // Pozycja pierwszej probówki pod spektrometrem

// --- NOWA LOGIKA OMIJANIA PROBÓWKI 1 ---
// Fizyczna kolejność gniazd (6 kroków) - zauważ brak jedynki
// Omijamy fizyczne gniazda 1 oraz 3.
// Zostają nam dwie idealne pary robocze!
const uint8_t TUBE_SEQUENCE[] = {6, 4, 2, 0};

static uint8_t current_seq_idx = 0; // Licznik kroków wiercenia (0-5)
static uint8_t current_syringe_index = 0;
static uint8_t active_dosing_seq_idx = 0;       // Krok dla dozownika
static uint8_t tubes_dosed_in_this_cycle = 0;   // Licznik (0-2)
static uint8_t active_stir_seq_idx = 0;         // Krok dla mieszadła
static uint8_t tubes_stirred_in_this_cycle = 0; // Licznik wymieszanych probówek (0-2)
static uint8_t active_spectro_seq_idx = 0;      // Krok dla spektrometru
static uint8_t tubes_spectro_in_this_cycle = 0; // Licznik zbadanych probówek (0-2)

EventGroupHandle_t xSystemEvents;
SemaphoreHandle_t xMotorPowerMutex;
extern SemaphoreHandle_t xUartMutex;
QueueHandle_t xDynamixelQueue;

typedef struct
{
    uint8_t servo_id;
    uint16_t target_position;
} DynamixelCmd_t;

void vTaskCanHandler(void* pvParameters)
{
    CanMsg_t msg;

    // Zmienne śledzące absolutną pozycję serw w trybie manualnym
    int16_t manual_tube_pos = 512;
    int16_t manual_syr_pos = 512;
    uint8_t manual_mode_initialized = 0;

    extern CAN_HandleTypeDef hcan;

    // --- 1. KONFIGURACJA FILTRA CAN (Sprzętowy bramkarz) ---
    CAN_FilterTypeDef canFilterConfig;
    canFilterConfig.FilterBank = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig.FilterIdHigh = 0x0000;
    canFilterConfig.FilterIdLow = 0x0000;
    canFilterConfig.FilterMaskIdHigh = 0x0000;
    canFilterConfig.FilterMaskIdLow = 0x0000;
    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // Kierujemy do FIFO0
    canFilterConfig.FilterActivation = ENABLE;
    canFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &canFilterConfig) != HAL_OK)
    {
        // Błąd konfiguracji filtra (warto tu wstawić np. mignięcie diodą błędu)
    }
    
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    for (;;)
    {
        // Czekamy na ramkę z przerwania (nieskończenie długo)
        if (xQueueReceive(xCanMsgQueue, &msg, portMAX_DELAY) == pdTRUE)
        {
            EventBits_t events = xEventGroupGetBits(xSystemEvents);

            // 1. OBSŁUGA KOMEND SYSTEMOWYCH (Zawsze aktywna)
            if (msg.StdId == 0x095)
            {
                uint8_t sys_cmd = msg.Data[0];

                if (sys_cmd == 0x01)
                {
                    // START AUTO
                    xEventGroupClearBits(xSystemEvents, BIT_MANUAL_MODE | BIT_SCRAM_ACTIVE);
                    xEventGroupSetBits(xSystemEvents, BIT_START_AUTO);
                }
                else if (sys_cmd == 0x02)
                {
                    // SCRAM (Zatrzymanie awaryjne wszystkich procesów)
                    xEventGroupSetBits(xSystemEvents, BIT_SCRAM_ACTIVE);
                    xEventGroupClearBits(xSystemEvents, BIT_START_AUTO);
                }
                else if (sys_cmd == 0x03)
                {
                    // WEJŚCIE W TRYB MANUALNY
                    xEventGroupSetBits(xSystemEvents, BIT_MANUAL_MODE);
                    xEventGroupClearBits(xSystemEvents, BIT_SCRAM_ACTIVE | BIT_START_AUTO);
                }
                else if (sys_cmd == 0x04)
                {
                    // POWRÓT DO IDLE / ZDJĘCIE BLOKAD
                    xEventGroupClearBits(xSystemEvents, BIT_MANUAL_MODE | BIT_SCRAM_ACTIVE);
                }

                continue; // Przetworzyliśmy ramkę systemową, nie idziemy dalej
            }

            // 2. OBSŁUGA RĘCZNA SILNIKÓW (Tylko w trybie MANUAL)
            if (events & BIT_MANUAL_MODE)
            {
                // Inicjalizacja pozycji startowej serw przy pierwszym wejściu w Manual
                if (!manual_mode_initialized)
                {
                    uint16_t current_t = AX12_ReadPosition(DYNAMIXEL_TUBE_ID);
                    uint16_t current_s = AX12_ReadPosition(DYNAMIXEL_SYRINGE_ID);
                    if (current_t != 0xFFFF)
                        manual_tube_pos = current_t;
                    if (current_s != 0xFFFF)
                        manual_syr_pos = current_s;
                    manual_mode_initialized = 1;
                }

                // --- RAMKA 0x096: SERWO 1 I SILNIKI DC ---
                if (msg.StdId == 0x096)
                {
                    // Serwo 1 (Probówki)
                    if (msg.Data[0] != 0)
                    {
                        uint16_t delta = (msg.Data[1] << 8) | msg.Data[2];
                        if (msg.Data[0] == 1)
                            manual_tube_pos += delta;
                        if (msg.Data[0] == 2)
                            manual_tube_pos -= delta;

                        // Kagańce bezpieczeństwa
                        if (manual_tube_pos < 0)
                            manual_tube_pos = 0;
                        if (manual_tube_pos > 1023)
                            manual_tube_pos = 1023;

                        DynamixelCmd_t moveCmd = {DYNAMIXEL_TUBE_ID, (uint16_t)manual_tube_pos};
                        xQueueSend(xDynamixelQueue, &moveCmd, portMAX_DELAY);
                    }

                    // Silniki DC i Wiertło
                    if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                    {
                        uint8_t dc_mode = msg.Data[3];
                        uint16_t dc_speed = msg.Data[4];

                        // Ochrona logiczna - jeśli Tryb = 0, to zatrzymujemy układ MC34931
                        if (dc_mode == 0)
                        {
                            StopStirrer();
                            SetDrillLoweringSpeed_MC34931(0, 0);
                        }
                        else if (dc_mode == 1)
                            SetStirrerSpeed_MC34931((dc_speed * 100) / 255, 1);
                        else if (dc_mode == 2)
                            SetStirrerSpeed_MC34931((dc_speed * 100) / 255, 2);
                        else if (dc_mode == 3)
                            SetDrillLoweringSpeed_MC34931((dc_speed * 1000) / 255, 1);
                        else if (dc_mode == 4)
                            SetDrillLoweringSpeed_MC34931((dc_speed * 1000) / 255, 2);

                        // Wrzeciono Wiertła
                        uint8_t spin_mode = msg.Data[5];
                        int16_t spin_speed = (msg.Data[6] * 100) / 255; // Skalowanie na procenty (-100 do 100)

                        if (spin_mode == 0)
                            SetDrillSpinSpeed_Talon(0);
                        else if (spin_mode == 1)
                            SetDrillSpinSpeed_Talon(spin_speed);
                        else if (spin_mode == 2)
                            SetDrillSpinSpeed_Talon(-spin_speed);

                        xSemaphoreGive(xMotorPowerMutex);
                    }
                }

                // --- RAMKA 0x097: SERWO 2 ---
                else if (msg.StdId == 0x097)
                {
                    if (msg.Data[0] != 0)
                    {
                        uint16_t delta = (msg.Data[1] << 8) | msg.Data[2];
                        if (msg.Data[0] == 1)
                            manual_syr_pos += delta;
                        if (msg.Data[0] == 2)
                            manual_syr_pos -= delta;

                        if (manual_syr_pos < 0)
                            manual_syr_pos = 0;
                        if (manual_syr_pos > 1023)
                            manual_syr_pos = 1023;

                        DynamixelCmd_t moveCmd = {DYNAMIXEL_SYRINGE_ID, (uint16_t)manual_syr_pos};
                        xQueueSend(xDynamixelQueue, &moveCmd, portMAX_DELAY);
                    }
                }
            }
            else
            {
                // Wyczyszczenie flagi, aby po ponownym wejściu w Manual odczytać aktualną pozycję sprzętu
                manual_mode_initialized = 0;
            }
        }
    }
}

void Spectrometer_SetBulb(uint8_t state)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[1];

    // Konfiguracja ramki zgodnie z Twoim odbiornikiem
    TxHeader.StdId = 0x98;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 1; // Wysyłamy tylko 1 bajt
    TxHeader.TransmitGlobalTime = DISABLE;

    TxData[0] = state ? 1 : 0; // 1 = włącz, 0 = wyłącz

    // Wysłanie ramki na magistralę (zmień hcan1 na hcan jeśli tak się u Ciebie nazywa)
    extern CAN_HandleTypeDef hcan;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

uint8_t WaitForServoPosition(uint8_t servo_id, uint16_t target_pos, uint32_t timeout_ms)
{
    uint32_t start_time = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start_time) < timeout_ticks)
    {
        uint16_t current_pos = AX12_ReadPosition(servo_id);

        if (current_pos != 0xFFFF)
        {
            int16_t error = (int16_t)current_pos - (int16_t)target_pos;
            if (abs(error) <= 3)
            {
                return 1;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return 0;
}

void vTaskLabSequence(void* pvParameters)
{
    LabState_t currentState = LAB_STATE_IDLE;
    LabState_t prevState = LAB_STATE_RESET_READY;
    uint8_t state_entry = 0;

    for (;;)
    {
        EventBits_t events = xEventGroupGetBits(xSystemEvents);

        if (events & BIT_SCRAM_ACTIVE)
        {
            EmergencyStopMotors();
            currentState = LAB_STATE_IDLE;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (events & BIT_MANUAL_MODE)
        {
            currentState = LAB_STATE_IDLE;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Ustalenie flagi wejścia do stanu
        if (currentState != prevState)
        {
            state_entry = 1;
            prevState = currentState;
        }
        else
        {
            state_entry = 0;
        }

        switch (currentState)
        {
        case LAB_STATE_IDLE:
            if (IsStartSwitchPressed())
            {
                vTaskDelay(pdMS_TO_TICKS(50));
                if (IsStartSwitchPressed())
                {
                    currentState = LAB_STATE_HOMING;
                }
            }
            else if (events & BIT_START_AUTO)
            {
                xEventGroupClearBits(xSystemEvents, BIT_START_AUTO);
                currentState = LAB_STATE_HOMING;
            }
            break;

        case LAB_STATE_HOMING:
            if (state_entry)
            {
                SetDrillLoweringSpeed_MC34931(800, 2);
            }

            if (IsDrillHomed())
            {
                SetDrillLoweringSpeed_MC34931(0, 0);
                vTaskDelay(pdMS_TO_TICKS(100));

                if (IsDrillHomed())
                {
                    ResetDrillEncoder();
                    xEventGroupClearBits(xSystemEvents, BIT_DRILL_LOWERED);
                    currentState = LAB_STATE_HOMING_REVOLVERS;
                }
            }
            break;

        case LAB_STATE_HOMING_REVOLVERS:
        {
            if (state_entry)
            {
                DynamixelCmd_t moveCmdTube = {DYNAMIXEL_TUBE_ID, POS_SAFE_TUBE};
                DynamixelCmd_t moveCmdSyr = {DYNAMIXEL_SYRINGE_ID, POS_SAFE_SYRINGE};
                xQueueSend(xDynamixelQueue, &moveCmdTube, portMAX_DELAY);
                xQueueSend(xDynamixelQueue, &moveCmdSyr, portMAX_DELAY);
            }

            if (WaitForServoPosition(DYNAMIXEL_TUBE_ID, POS_SAFE_TUBE, 5000) &&
                WaitForServoPosition(DYNAMIXEL_SYRINGE_ID, POS_SAFE_SYRINGE, 5000))
            {
                currentState = LAB_STATE_DRILLING;
            }
            else
            {
                currentState = LAB_STATE_IDLE;
            }
            break;
        }

        case LAB_STATE_DRILLING:
            if (state_entry)
            {
                if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    xEventGroupSetBits(xSystemEvents, BIT_DRILL_LOWERED);
                    SetDrillSpinSpeed_Talon(-80);
                    SetDrillLoweringSpeed_MC34931(800, 1);
                    xSemaphoreGive(xMotorPowerMutex);
                }
            }

            if (IsDrillAtTargetDepth(/* 4 */ 1 * 65535))
            {
                if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    SetDrillLoweringSpeed_MC34931(0, 0);
                    xSemaphoreGive(xMotorPowerMutex);
                }
                currentState = LAB_STATE_RETRACT;
            }
            break;

        case LAB_STATE_RETRACT:
            if (state_entry)
            {
                SetDrillLoweringSpeed_MC34931(800, 2);
            }

            if (IsDrillHomed())
            {
                SetDrillLoweringSpeed_MC34931(0, 0);
                SetDrillSpinSpeed_Talon(0);
                vTaskDelay(pdMS_TO_TICKS(100));

                if (IsDrillHomed())
                {
                    xEventGroupClearBits(xSystemEvents, BIT_DRILL_LOWERED);
                    currentState = LAB_STATE_TUBE_POS;
                }
            }
            break;

        case LAB_STATE_TUBE_POS:
        {
            // ODCZYT FIZYCZNEJ PROBÓWKI Z TABLICY
            uint8_t physical_tube = TUBE_SEQUENCE[current_seq_idx];
            uint16_t calc_pos = TUBE_BASE_POS - (physical_tube * TUBE_SPACING);

            if (calc_pos > 1023)
                calc_pos = 1023;

            if (state_entry)
            {
                DynamixelCmd_t moveCmd = {DYNAMIXEL_TUBE_ID, calc_pos};
                xQueueSend(xDynamixelQueue, &moveCmd, portMAX_DELAY);
            }

            if (WaitForServoPosition(DYNAMIXEL_TUBE_ID, calc_pos, 5000))
            {
                currentState = LAB_STATE_FILL_TUBE;
            }
            else
            {
                EmergencyStopMotors();
                currentState = LAB_STATE_IDLE;
            }
            break;
        }

        case LAB_STATE_FILL_TUBE:
        {
            // Bezpieczne operowanie Mutexem
            if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                SetDrillSpinSpeed_Talon(80);
                xSemaphoreGive(xMotorPowerMutex);
            }

            vTaskDelay(pdMS_TO_TICKS(2000));

            if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                SetDrillSpinSpeed_Talon(0);
                xSemaphoreGive(xMotorPowerMutex);
            }

            // Przechodzimy do następnego indeksu sekwencji
            current_seq_idx++;

            // Logika parowania (1 odwiert = 2 probówki)
            if (current_seq_idx % 2 != 0 && current_seq_idx > 0)
            {
                currentState = LAB_STATE_TUBE_POS;
            }
            else
            {
                active_dosing_seq_idx = current_seq_idx - 2;
                tubes_dosed_in_this_cycle = 0;
                current_syringe_index = 0;
                currentState = LAB_STATE_REAGENT_POS;
            }
            break;
        }

        case LAB_STATE_REAGENT_POS:
        {
            // Tłumaczymy krok maszyny na fizyczną pozycję gniazda
            uint8_t physical_tube = TUBE_SEQUENCE[active_dosing_seq_idx];
            int16_t target_tube_pos = TUBE_STIR_POS + ((6 - physical_tube) * TUBE_SPACING);

            // Poprawka dla probówki 0 (karuzela obróci się za daleko i przebije 1023)
            // Zdejmujemy pełny obrót (ok. 1230 jednostek karuzeli)
            if (target_tube_pos > 1023)
                target_tube_pos -= 1230;
            if (target_tube_pos < 0)
                target_tube_pos = 0;
            if (target_tube_pos > 1023)
                target_tube_pos = 1023;

            int16_t target_syr_pos = SYRINGE_BASE_POS + (current_syringe_index * SYRINGE_SPACING);
            if (target_syr_pos < 0)
                target_syr_pos = 0;
            if (target_syr_pos > 1023)
                target_syr_pos = 1023;

            if (state_entry)
            {
                DynamixelCmd_t moveCmdTube = {DYNAMIXEL_TUBE_ID, (uint16_t)target_tube_pos};
                DynamixelCmd_t moveCmdSyr = {DYNAMIXEL_SYRINGE_ID, (uint16_t)target_syr_pos};
                xQueueSend(xDynamixelQueue, &moveCmdTube, portMAX_DELAY);
                xQueueSend(xDynamixelQueue, &moveCmdSyr, portMAX_DELAY);
            }

            if (WaitForServoPosition(DYNAMIXEL_TUBE_ID, (uint16_t)target_tube_pos, 5000) &&
                WaitForServoPosition(DYNAMIXEL_SYRINGE_ID, (uint16_t)target_syr_pos, 5000))
            {
                currentState = LAB_STATE_DOSING;
            }
            else
            {
                EmergencyStopMotors();
                currentState = LAB_STATE_IDLE;
            }
            break;
        }

        case LAB_STATE_DOSING:
        {
            vTaskDelay(pdMS_TO_TICKS(500));

            current_syringe_index++;

            if (current_syringe_index % 2 != 0)
            {
                currentState = LAB_STATE_REAGENT_POS;
            }
            else
            {
                tubes_dosed_in_this_cycle++;

                if (tubes_dosed_in_this_cycle < 2)
                {
                    active_dosing_seq_idx++;
                    currentState = LAB_STATE_REAGENT_POS;
                }
                else
                {
                    // Koniec dozowania obu probówek.
                    // Przygotowujemy się do mieszania - wracamy do pierwszej probówki z pary!
                    active_stir_seq_idx = current_seq_idx - 2;
                    tubes_stirred_in_this_cycle = 0;
                    currentState = LAB_STATE_STIRRER_POS;
                }
            }
            break;
        }

        case LAB_STATE_STIRRER_POS:
        {
            // Pozycja mieszadła to to samo miejsce co dozowania,
            // więc korzystamy z tej samej bazy (TUBE_STIR_POS).
            uint8_t physical_tube = TUBE_SEQUENCE[active_stir_seq_idx];
            int16_t target_tube_pos = TUBE_STIR_POS + ((6 - physical_tube) * TUBE_SPACING);

            if (target_tube_pos > 1023)
                target_tube_pos -= 1230;
            if (target_tube_pos < 0)
                target_tube_pos = 0;
            if (target_tube_pos > 1023)
                target_tube_pos = 1023;

            if (state_entry)
            {
                // Wysyłamy komendę tylko do dolnego rewolweru. Górny (strzykawki) stoi w miejscu.
                DynamixelCmd_t moveCmdTube = {DYNAMIXEL_TUBE_ID, (uint16_t)target_tube_pos};
                xQueueSend(xDynamixelQueue, &moveCmdTube, portMAX_DELAY);
            }

            if (WaitForServoPosition(DYNAMIXEL_TUBE_ID, (uint16_t)target_tube_pos, 5000))
            {
                currentState = LAB_STATE_STIRRING;
            }
            else
            {
                EmergencyStopMotors();
                currentState = LAB_STATE_IDLE;
            }
            break;
        }

        case LAB_STATE_STIRRING:
        {
            if (state_entry)
            {
                if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    // Uruchamiamy mieszadło
                    SetStirrerSpeed_MC34931(100, 1);
                    xSemaphoreGive(xMotorPowerMutex);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(5000)); // Czas mieszania jednej probówki (5s)

            if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                // Zatrzymujemy mieszadło
                StopStirrer();
                xSemaphoreGive(xMotorPowerMutex);
            }

            // Logika przejść po wymieszaniu
            tubes_stirred_in_this_cycle++;

            if (tubes_stirred_in_this_cycle < 2)
            {
                // Przechodzimy do drugiej probówki z pary
                active_stir_seq_idx++;
                currentState = LAB_STATE_STIRRER_POS;
            }
            else
            {
                // Obie probówki wymieszane, cykl zakończony. Jedziemy do spektrometru!
                // Obie probówki wymieszane!
                // Czas na spektrometr - znowu cofamy się do pierwszej probówki z pary
                active_spectro_seq_idx = current_seq_idx - 2;
                tubes_spectro_in_this_cycle = 0;
                currentState = LAB_STATE_SPECTROMETER_POS;
            }
            break;
        }
        case LAB_STATE_SPECTROMETER_POS:
        {
            // Odczyt z tablicy i wyliczenie pozycji
            uint8_t physical_tube = TUBE_SEQUENCE[active_spectro_seq_idx];

            // UWAGA: Założyłem ten sam kierunek przyrostu obrotu co dla dozownika/mieszadła.
            // Jeśli spektrometr fizycznie jest z innej strony i karuzela musi kręcić się odwrotnie,
            // trzeba będzie zmienić tu znak (np. odjąć offset zamiast dodawać).
            int16_t target_tube_pos = TUBE_SPECTRO_POS + ((6 - physical_tube) * TUBE_SPACING);

            // Poprawka dla pełnego obrotu karuzeli (ominięcie blokady 1023)
            if (target_tube_pos > 1023)
                target_tube_pos -= 1230;
            if (target_tube_pos < 0)
                target_tube_pos = 0;
            if (target_tube_pos > 1023)
                target_tube_pos = 1023;

            if (state_entry)
            {
                DynamixelCmd_t moveCmdTube = {DYNAMIXEL_TUBE_ID, (uint16_t)target_tube_pos};
                xQueueSend(xDynamixelQueue, &moveCmdTube, portMAX_DELAY);
            }

            if (WaitForServoPosition(DYNAMIXEL_TUBE_ID, (uint16_t)target_tube_pos, 5000))
            {
                currentState = LAB_STATE_SPECTROMETER_FLASH;
            }
            else
            {
                EmergencyStopMotors();
                currentState = LAB_STATE_IDLE;
            }
            break;
        }

        case LAB_STATE_SPECTROMETER_FLASH:
        {
            if (state_entry)
            {
                // 1. Włączamy żarówkę (wysyła ramkę CAN z daną 1)
                Spectrometer_SetBulb(1);
            }

            // 2. Czekamy dokładnie 1 sekundę (żarówka świeci)
            vTaskDelay(pdMS_TO_TICKS(1000));

            // 3. Wyłączamy żarówkę (wysyła ramkę CAN z daną 0)
            Spectrometer_SetBulb(0);

            // Logika przejść po zbadaniu probówki
            tubes_spectro_in_this_cycle++;

            if (tubes_spectro_in_this_cycle < 2)
            {
                // Przechodzimy do drugiej probówki z pary
                active_spectro_seq_idx++;
                currentState = LAB_STATE_SPECTROMETER_POS;
            }
            else
            {
                // Obie probówki zbadane i oświetlone!
                currentState = LAB_STATE_RESET_READY;
            }
            break;
        }

        case LAB_STATE_RESET_READY:
            currentState = LAB_STATE_IDLE;
            break;

        default:
            currentState = LAB_STATE_IDLE;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void vTaskDynamixel(void* pvParameters)
{
    AX12_Init();
    DynamixelCmd_t currentCmd;

    for (;;)
    {
        if (xQueueReceive(xDynamixelQueue, &currentCmd, portMAX_DELAY) == pdTRUE)
        {
            EventBits_t events = xEventGroupGetBits(xSystemEvents);

            if (events & BIT_SCRAM_ACTIVE)
                continue;
            if (events & BIT_DRILL_LOWERED)
                continue;

            AX12_SetGoalPosition(currentCmd.servo_id, currentCmd.target_position);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void LabRTOS_Init(void)
{
    xSystemEvents = xEventGroupCreate();
    xMotorPowerMutex = xSemaphoreCreateMutex();
    // Zabezpieczamy muteks dla UART jeśli jeszcze go nie ma
    if (xUartMutex == NULL)
    {
        xUartMutex = xSemaphoreCreateMutex();
    }

    xDynamixelQueue = xQueueCreate(5, sizeof(DynamixelCmd_t));
    xCanMsgQueue = xQueueCreate(5, sizeof(CanMsg_t));



    xTaskCreate(vTaskLabSequence, "AutoSeq", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vTaskDynamixel, "Dynamixel", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(vTaskCanHandler, "CanRx", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
}