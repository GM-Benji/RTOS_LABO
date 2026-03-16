#include "lab_sequence.h"
#include "FreeRTOS.h"
#include "dynamixel.h"
#include "event_groups.h"
#include "motors.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include <stdlib.h>
// ... inne includy RTOS

typedef enum
{
    LAB_STATE_IDLE,
    LAB_STATE_HOMING, // zerowanie wiertła (krańcówka)[cite: 1, 3]
    LAB_STATE_HOMING_REVOLVERS,
    LAB_STATE_DRILLING,           // odwiert na zadaną głębokość[cite: 1, 3]
    LAB_STATE_RETRACT,            // powrót nad probówkę[cite: 1, 3]
    LAB_STATE_TUBE_POS,           // podjazd dolnym rewolwerem[cite: 1, 3]
    LAB_STATE_FILL_TUBE,          // procedura napełniania (wiertło -> rewolwer -> wiertło)[cite: 1, 3]
    LAB_STATE_REAGENT_POS,        // podjazd odpowiednich odczynników[cite: 1, 3]
    LAB_STATE_DOSING,             // dozowniki dozują kolejne odczynniki[cite: 1, 3]
    LAB_STATE_STIRRER_POS,        // podjazd pod mieszadło[cite: 1, 3]
    LAB_STATE_STIRRING,           // mieszanie przez określony czas z określoną szybkością[cite: 1, 3]
    LAB_STATE_SPECTROMETER_POS,   // podjazd pod spektrometr[cite: 1, 3]
    LAB_STATE_SPECTROMETER_FLASH, // błysk ~1s[cite: 1, 3]
    LAB_STATE_RESET_READY         // zerowanie do stanu gotowości do odwiertu[cite: 1, 3]
} LabState_t;

#define BIT_SCRAM_ACTIVE  (1 << 0)
#define BIT_MANUAL_MODE   (1 << 1)
#define BIT_DRILL_LOWERED (1 << 2)
#define BIT_START_AUTO    (1 << 3)

// --- PARAMETRY MECHANICZNE REWOLWERÓW ---
#define POS_SAFE_TUBE    1023 // Bezpieczna pozycja dla rewolweru z probówkami (nie koliduje z wiertłem)
#define POS_SAFE_SYRINGE 600 // Bezpieczna pozycja dla rewolweru ze strzykawką (nie koliduje z wiertłem)
#define TUBE_BASE_POS    790 // Pozycja pierwszej probówki pod wiertłem
#define TUBE_SPACING     123 // Offset 36 stopni (36 / 0.293 st/jednostkę = ~123)

// Zmienna przechowująca numer aktualnej probówki (np. od 0 do 5)
static uint8_t current_tube_index = 0;

EventGroupHandle_t xSystemEvents;
SemaphoreHandle_t xMotorPowerMutex;
QueueHandle_t xDynamixelQueue;

// Struktura komendy dla serwa
typedef struct
{
    uint8_t servo_id;
    uint16_t target_position;
} DynamixelCmd_t;

// Pamiętaj, żeby uchwyt do kolejki był widoczny globalnie w tym pliku
extern QueueHandle_t xDynamixelQueue;

uint8_t WaitForServoPosition(uint8_t servo_id, uint16_t target_pos, uint32_t timeout_ms)
{
    uint32_t start_time = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start_time) < timeout_ticks)
    {

        uint16_t current_pos = AX12_ReadPosition(servo_id);

        if (current_pos != 0xFFFF)
        { // Jeśli odczyt się udał
            int16_t error = (int16_t)current_pos - (int16_t)target_pos;

            // Tolerancja +/- 3 jednostki (~0.8 stopnia)
            // Serwa prawie nigdy nie stają IDEALNIE na punkcie, zawsze lekko drgają.
            if (abs(error) <= 3)
            {
                return 1; // Jesteśmy na miejscu!
            }
        }

        // Zwalniamy procesor na 50ms, żeby inne taski (np. obsługa CAN) mogły działać
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return 0; // Błąd - Timeout minął, serwo się zablokowało!
}

void vTaskLabSequence(void* pvParameters)
{
    LabState_t currentState = LAB_STATE_IDLE;

    // --- KROK TESTOWY 1: Wymuszenie startu od razu po uruchomieniu ---
    // Maszyna stanów od razu przeskoczy do LAB_STATE_HOMING
    // xEventGroupSetBits(xSystemEvents, BIT_START_AUTO);

    for (;;)
    {
        EventBits_t events = xEventGroupGetBits(xSystemEvents);

        // 1. Obsługa SCRAM (Odcięcie awaryjne)
        if (events & BIT_SCRAM_ACTIVE)
        {
            EmergencyStopMotors();
            currentState = LAB_STATE_IDLE;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 2. Obsługa Manual Mode
        if (events & BIT_MANUAL_MODE)
        {
            currentState = LAB_STATE_IDLE;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 3. Maszyna Stanów Trybu Automatycznego
        switch (currentState)
        {
        case LAB_STATE_IDLE:
            // 1. Sprawdzenie fizycznego przycisku S_SWITCH (PB13)
            if (IsStartSwitchPressed())
            {
                // Prosty debouncing (eliminacja drgań styków mechanicznych)
                vTaskDelay(pdMS_TO_TICKS(50));
                if (IsStartSwitchPressed())
                {
                    currentState = LAB_STATE_HOMING;
                }
            }
            // 2. Start z magistrali CAN (jeśli flaga ustawiona np. przez Bena z komputera)
            else if (events & BIT_START_AUTO)
            {
                xEventGroupClearBits(xSystemEvents, BIT_START_AUTO); // Czyścimy flagę, żeby nie wyzwalała się podwójnie
                currentState = LAB_STATE_HOMING;
            }
            break;

        case LAB_STATE_HOMING:
            if (IsDrillHomed())
            {
                // 1. BEZPIECZEŃSTWO: Natychmiast zatrzymaj silnik opuszczania!
                SetDrillLoweringSpeed_MC34931(0, 0);

                // 2. DEBOUNCE: Poczekaj 50ms na ustabilizowanie mechaniczne styków
                vTaskDelay(pdMS_TO_TICKS(100));

                // 3. WERYFIKACJA: Sprawdź, czy krańcówka nadal jest wciśnięta
                if (IsDrillHomed())
                {
                    ResetDrillEncoder();
                    xEventGroupClearBits(xSystemEvents, BIT_DRILL_LOWERED);
                    currentState = LAB_STATE_HOMING_REVOLVERS;
                }
            }
            else
            {
                SetDrillLoweringSpeed_MC34931(800, 2); // 2 = W GÓRĘ
            }
            break;

        case LAB_STATE_HOMING_REVOLVERS:
        {
            DynamixelCmd_t moveCmdTube, moveCmdSyr;

            moveCmdTube.servo_id = DYNAMIXEL_TUBE_ID;
            moveCmdTube.target_position = POS_SAFE_TUBE;
            xQueueSend(xDynamixelQueue, &moveCmdTube, portMAX_DELAY);

            moveCmdSyr.servo_id = DYNAMIXEL_SYRINGE_ID;
            moveCmdSyr.target_position = POS_SAFE_SYRINGE;
            xQueueSend(xDynamixelQueue, &moveCmdSyr, portMAX_DELAY);

            // Czekamy na oba serwa!
            uint8_t tube_ok = WaitForServoPosition(DYNAMIXEL_TUBE_ID, POS_SAFE_TUBE, 5000);
            uint8_t syr_ok = WaitForServoPosition(DYNAMIXEL_SYRINGE_ID, POS_SAFE_SYRINGE, 5000);

            if (tube_ok && syr_ok)
            {
                currentState = LAB_STATE_DRILLING;
            }
            else
            {
                currentState = LAB_STATE_IDLE; // Przerwij z powodu błędu
            }
            break;
        }

        case LAB_STATE_DRILLING:
            if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                xEventGroupSetBits(xSystemEvents, BIT_DRILL_LOWERED);

                // UWAGA: Ponieważ nie mamy fizycznie podpiętego enkodera
                // warunek (pozycja > 15000) nigdy nie zostanie spełniony.
                // Maszyna zablokuje się w tym stanie, co jest idealne do pomiarów!
                if (IsDrillAtTargetDepth(4 * 65535))
                {
                    SetDrillLoweringSpeed_MC34931(0, 0);
                    // SetDrillSpinSpeed_Talon(0);
                    xSemaphoreGive(xMotorPowerMutex);
                    currentState = LAB_STATE_RETRACT;
                }
                else
                {
                    // POMIAR 1: PB0 powinien mieć PWM 80% (ok. 2.6V), PB1 ma 0V.
                    // POMIAR 2: PB14 (Talon) wygeneruje sygnał RC (na multimetrze to ułamek wolta)
                    SetDrillSpinSpeed_Talon(-80);          // Kręcenie (Talon)
                    SetDrillLoweringSpeed_MC34931(800, 1); // 1 = W DÓŁ (MC34931)
                    xSemaphoreGive(xMotorPowerMutex);
                }
            }
            break;

        case LAB_STATE_RETRACT:
            if (IsDrillHomed())
            {
                // 1. BEZPIECZEŃSTWO: Natychmiast zatrzymaj silnik
                SetDrillLoweringSpeed_MC34931(0, 0);
                SetDrillSpinSpeed_Talon(0);

                // 2. DEBOUNCE: 50ms przerwy
                vTaskDelay(pdMS_TO_TICKS(100));

                // 3. WERYFIKACJA
                if (IsDrillHomed())
                {
                    xEventGroupClearBits(xSystemEvents, BIT_DRILL_LOWERED);
                    currentState = LAB_STATE_TUBE_POS;
                }
            }
            else
            {
                SetDrillLoweringSpeed_MC34931(800, 2); // 2 = W GÓRĘ
            }
            break;

        case LAB_STATE_STIRRING:
            if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                // POMIAR: PA6 lub PA7 wygeneruje PWM 50%
                SetStirrerSpeed_MC34931(500, 1);
                vTaskDelay(pdMS_TO_TICKS(5000)); // Mieszaj przez 5 sekund
                StopStirrer();
                xSemaphoreGive(xMotorPowerMutex);
                currentState = LAB_STATE_SPECTROMETER_POS;
            }
            break;

        case LAB_STATE_TUBE_POS:
        {
            DynamixelCmd_t moveCmd;
            moveCmd.servo_id = DYNAMIXEL_TUBE_ID;

            uint16_t calc_pos = TUBE_BASE_POS - (current_tube_index * TUBE_SPACING);
            if (calc_pos > 1023)
                calc_pos = 1023;

            moveCmd.target_position = calc_pos;
            xQueueSend(xDynamixelQueue, &moveCmd, portMAX_DELAY);

            // INTELIGENTNE CZEKANIE (z maksymalnym timeoutem 5 sekund)
            if (WaitForServoPosition(DYNAMIXEL_TUBE_ID, calc_pos, 5000))
            {
                // Serwo dojechało, kontynuujemy cykl

                currentState = LAB_STATE_FILL_TUBE;
            }
            else
            {
                // ERROR! Serwo zablokowane. Zatrzymujemy maszynę!
                // Tutaj w przyszłości można wysłać komunikat po CAN do Bena
                EmergencyStopMotors();
                currentState = LAB_STATE_IDLE;
            }

            break;
        }
        case LAB_STATE_FILL_TUBE:
        {
            // Dla bezpieczeństwa pobieramy muteks
            if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {

                // 1. Zrzucanie gleby: obroty CCW (wartość dodatnia)
                // Możesz tu wpisać odpowiednią moc, np. 80 lub 100
                SetDrillSpinSpeed_Talon(80);

                // 2. Czekamy równo 2 sekundy
                vTaskDelay(pdMS_TO_TICKS(2000));

                // 3. Zatrzymujemy wiertło
                SetDrillSpinSpeed_Talon(0);

                xSemaphoreGive(xMotorPowerMutex);

                // 4. Inkrementujemy licznik napełnionych probówek
                current_tube_index++;

                // Logika: 1 odwiert = 2 probówki
                if (current_tube_index % 2 != 0)
                {
                    // Napełniliśmy nieparzystą liczbę probówek (np. pierwszą z pary)
                    // Wracamy do obrotu rewolwerem pod kolejną probówkę
                    currentState = LAB_STATE_TUBE_POS;
                }
                else
                {
                    // Napełniliśmy parzystą liczbę (np. drugą z pary)
                    // Gleba się skończyła. Przechodzimy do dozowania odczynników!
                    currentState = LAB_STATE_IDLE;
                }
            }
            break;
        }

            // (...) (Tutaj będą inne stany, np. napełnianie, i po nim jazda do rewolweru z chemią)

        case LAB_STATE_REAGENT_POS:
        {
            DynamixelCmd_t moveCmd;
            moveCmd.servo_id = DYNAMIXEL_SYRINGE_ID;
            moveCmd.target_position = 256; // Zmień na właściwą pozycję dla strzykawki

            xQueueSend(xDynamixelQueue, &moveCmd, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(1000));

            currentState = LAB_STATE_DOSING;
            break;
        }
        default:
            currentState = LAB_STATE_IDLE;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void vTaskDynamixel(void* pvParameters)
{
    DynamixelCmd_t currentCmd;

    for (;;)
    {
        // Task zasypia i czeka, aż w kolejce pojawi się komenda
        if (xQueueReceive(xDynamixelQueue, &currentCmd, portMAX_DELAY) == pdTRUE)
        {

            EventBits_t events = xEventGroupGetBits(xSystemEvents);

            // ZABEZPIECZENIE 1: SCRAM
            if (events & BIT_SCRAM_ACTIVE)
            {
                continue; // Ignorujemy komendę
            }

            // ZABEZPIECZENIE 2: Wiertło w dół = blokada rewolwerów!
            if (events & BIT_DRILL_LOWERED)
            {
                // Tutaj w przyszłości możemy wysłać ramkę błędu po CAN do Bena
                continue; // Wiertło jest w dół, odrzucamy komendę obrotu
            }

            // --- WYSYŁANIE RAMKI UART ---
            // Jeśli tu dotarliśmy, jest bezpiecznie. Wysyłamy fizycznie komendę.
            AX12_SetGoalPosition(currentCmd.servo_id, currentCmd.target_position);

            // Mały delay, żeby nie zasypać magistrali UART
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void LabRTOS_Init(void)
{
    xSystemEvents = xEventGroupCreate();
    xMotorPowerMutex = xSemaphoreCreateMutex();
    xUartMutex = xSemaphoreCreateMutex();

    xDynamixelQueue = xQueueCreate(5, sizeof(DynamixelCmd_t));

    // Tworzenie tasków[cite: 3]
    xTaskCreate(vTaskLabSequence, "AutoSeq", 512, NULL, tskIDLE_PRIORITY + 2, NULL);

    xTaskCreate(vTaskDynamixel, "Dynamixel", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
}