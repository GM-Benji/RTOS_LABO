#include "lab_sequence.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "motors.h"
// ... inne includy RTOS

typedef enum {
    LAB_STATE_IDLE,
    LAB_STATE_HOMING,              // zerowanie wiertła (krańcówka)[cite: 1, 3]
    LAB_STATE_DRILLING,            // odwiert na zadaną głębokość[cite: 1, 3]
    LAB_STATE_RETRACT,             // powrót nad probówkę[cite: 1, 3]
    LAB_STATE_TUBE_POS,            // podjazd dolnym rewolwerem[cite: 1, 3]
    LAB_STATE_FILL_TUBE,           // procedura napełniania (wiertło -> rewolwer -> wiertło)[cite: 1, 3]
    LAB_STATE_REAGENT_POS,         // podjazd odpowiednich odczynników[cite: 1, 3]
    LAB_STATE_DOSING,              // dozowniki dozują kolejne odczynniki[cite: 1, 3]
    LAB_STATE_STIRRER_POS,         // podjazd pod mieszadło[cite: 1, 3]
    LAB_STATE_STIRRING,            // mieszanie przez określony czas z określoną szybkością[cite: 1, 3]
    LAB_STATE_SPECTROMETER_POS,    // podjazd pod spektrometr[cite: 1, 3]
    LAB_STATE_SPECTROMETER_FLASH,  // błysk ~1s[cite: 1, 3]
    LAB_STATE_RESET_READY          // zerowanie do stanu gotowości do odwiertu[cite: 1, 3]
} LabState_t;

#define BIT_SCRAM_ACTIVE    ( 1 << 0 )
#define BIT_MANUAL_MODE     ( 1 << 1 )
#define BIT_DRILL_LOWERED   ( 1 << 2 )
#define BIT_START_AUTO      ( 1 << 3 )

EventGroupHandle_t xSystemEvents;
SemaphoreHandle_t xMotorPowerMutex;
QueueHandle_t xDynamixelQueue;

// Usunięto zbędny: extern TIM_HandleTypeDef htim4;

void vTaskLabSequence(void *pvParameters) {
    LabState_t currentState = LAB_STATE_IDLE;
    
    // LINIA DODANA DO TESTOWANIA: Wymuszenie startu sekwencji od razu po włączeniu zasilania
    xEventGroupSetBits(xSystemEvents, BIT_START_AUTO);

    for(;;) {
        EventBits_t events = xEventGroupGetBits(xSystemEvents);

        // 1. Obsługa SCRAM - najwyższy priorytet
        if(events & BIT_SCRAM_ACTIVE) {
            EmergencyStopMotors(); // Szybkie zerowanie PWM i wysłanie stop do Dynamixeli[cite: 3]
            currentState = LAB_STATE_IDLE;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 2. Obsługa Manual Mode[cite: 1, 3]
        if(events & BIT_MANUAL_MODE) {
            // Task sekwencji pauzuje, sterowanie przejmuje vTaskCAN / vTaskManual[cite: 3]
            currentState = LAB_STATE_IDLE;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 3. Maszyna Stanów Trybu Automatycznego
        switch(currentState) {
            case LAB_STATE_IDLE:
                if(events & BIT_START_AUTO) {
                    currentState = LAB_STATE_HOMING;
                }
                break;

            case LAB_STATE_HOMING:
                // Logika dojazdu wiertła do krańcówki[cite: 3]
                if (IsDrillHomed()) {
                    SetDrillLoweringSpeed_MC34931(0, 0); // Zatrzymaj silnik!
                    ResetDrillEncoder(); // <--- Teraz to zadziała, bo dodaliśmy do motors.h
                    xEventGroupClearBits(xSystemEvents, BIT_DRILL_LOWERED);
                    currentState = LAB_STATE_DRILLING;
                } else {
                    SetDrillLoweringSpeed_MC34931(800, 2); // Jedź w górę (kierunek 2)
                }
                break;

            case LAB_STATE_DRILLING:
                // Pobranie muteksa, aby upewnić się, że mieszadło nie działa[cite: 3]
                if(xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    
                    xEventGroupSetBits(xSystemEvents, BIT_DRILL_LOWERED);
                    
                    if (IsDrillAtTargetDepth(15000)) {
                        SetDrillLoweringSpeed_MC34931(0, 0); // Zatrzymanie
                        xSemaphoreGive(xMotorPowerMutex);
                        currentState = LAB_STATE_RETRACT;
                    } else {
                         // Wiertło kręci i opuszcza
                         SetDrillSpinSpeed_Talon(80); 
                         SetDrillLoweringSpeed_MC34931(800, 1); // 1 = W DÓŁ
                         xSemaphoreGive(xMotorPowerMutex); // Zawsze oddajemy mutex!
                    }
                }
                break;

            case LAB_STATE_RETRACT:
                if (IsDrillHomed()) {
                    SetDrillLoweringSpeed_MC34931(0, 0);
                    xEventGroupClearBits(xSystemEvents, BIT_DRILL_LOWERED);
                    currentState = LAB_STATE_TUBE_POS; 
                } else {
                    SetDrillLoweringSpeed_MC34931(800, 2); // 2 = W GÓRĘ
                }
                break;

            // (...) Reszta stanów logicznych

            case LAB_STATE_STIRRING:
                // Znów pobranie muteksa zabezpieczającego jednoczesną pracę wiertła i mieszadła[cite: 3]
                if(xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    // Mieszanie przez określony czas[cite: 1, 3]
                    SetStirrerSpeed_MC34931(500, 1); // Odpalenie mieszadła do testów
                    vTaskDelay(pdMS_TO_TICKS(5000)); // np. 5 sekund[cite: 3]
                    StopStirrer();
                    xSemaphoreGive(xMotorPowerMutex);
                    currentState = LAB_STATE_SPECTROMETER_POS;
                }
                break;

            default:
                currentState = LAB_STATE_IDLE;
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Czas na inne procesy[cite: 3]
    }
}

void LabRTOS_Init(void) {
    xSystemEvents = xEventGroupCreate();
    xMotorPowerMutex = xSemaphoreCreateMutex();
    // Tworzenie tasków[cite: 3]
    xTaskCreate(vTaskLabSequence, "AutoSeq", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
}