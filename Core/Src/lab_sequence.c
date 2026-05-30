#include "lab_sequence.h"
#include "FreeRTOS.h"
#include "can.h"
#include "event_groups.h"
#include "motors.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f1xx_hal_can.h"
#include "task.h"

typedef enum
{
    LAB_STATE_IDLE,
    LAB_STATE_HOMING,
    LAB_STATE_DRILLING_DOWN,
    LAB_STATE_DRILLING_UP,
    LAB_STATE_SPILLING
} LabState_t;

QueueHandle_t xCanMsgQueue;

#define BIT_SCRAM_ACTIVE  (1 << 0)
#define BIT_MANUAL_MODE   (1 << 1)
#define BIT_CMD_HOME      (1 << 2)
#define BIT_CMD_DRILL     (1 << 3)
#define BIT_CMD_SPILL     (1 << 4)
#define BIT_CMD_UV        (1 << 5)

EventGroupHandle_t xSystemEvents;
SemaphoreHandle_t xMotorPowerMutex;
extern SemaphoreHandle_t xUartMutex; 

// ---------------------------------------------------------
// 1. CAN Handler Task
// ---------------------------------------------------------
void vTaskCanHandler(void* pvParameters)
{
    CanMsg_t msg;
    extern CAN_HandleTypeDef hcan;

    // --- BULLETPROOF 32-BIT MASK FILTER ---
    // Accepts ONLY IDs between 192 (0xC0) and 255 (0xFF)
    CAN_FilterTypeDef canFilterConfig;
    canFilterConfig.FilterBank = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // Changed back to Mask Mode
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32-bit mode is safer for ignoring IDE/RTR

    canFilterConfig.FilterIdHigh = 0x0C0 << 5;           // Base ID: 192 (0xC0)
    canFilterConfig.FilterIdLow = 0x0000;                // Ignore lower bits
    
    canFilterConfig.FilterMaskIdHigh = 0x7C0 << 5;       // Mask out the top 5 bits to isolate the 192-255 block
    canFilterConfig.FilterMaskIdLow = 0x0000;            // 0x0000 here tells hardware to IGNORE IDE/RTR bits completely

    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
    canFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &canFilterConfig) != HAL_OK) {}
    
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    for (;;)
    {
        if (xQueueReceive(xCanMsgQueue, &msg, portMAX_DELAY) == pdTRUE)
        {
            EventBits_t events = xEventGroupGetBits(xSystemEvents);

            // SYSTEM COMMANDS (ID 192 / 0xC0)
            if (msg.StdId == 192)
            {
                uint8_t cmd = msg.Data[0];  

                if (cmd == 0x02) {
                    // SCRAM
                    xEventGroupSetBits(xSystemEvents, BIT_SCRAM_ACTIVE);
                    xEventGroupClearBits(xSystemEvents, BIT_CMD_HOME | BIT_CMD_DRILL | BIT_CMD_SPILL);
                }
                else if (cmd == 0x03) {
                    // MANUAL MODE
                    xEventGroupSetBits(xSystemEvents, BIT_MANUAL_MODE);
                    xEventGroupClearBits(xSystemEvents, BIT_SCRAM_ACTIVE | BIT_CMD_HOME | BIT_CMD_DRILL | BIT_CMD_SPILL);
                }
                else if (cmd == 0x04) {
                    // IDLE / CLEAR
                    xEventGroupClearBits(xSystemEvents, BIT_MANUAL_MODE | BIT_SCRAM_ACTIVE);
                }
                else if (cmd == 0x10) {
                    // CMD 1: HOMING
                    xEventGroupClearBits(xSystemEvents, BIT_MANUAL_MODE | BIT_SCRAM_ACTIVE);
                    xEventGroupSetBits(xSystemEvents, BIT_CMD_HOME);
                }
                else if (cmd == 0x20) {
                    // CMD 2: DRILL CYCLE
                    xEventGroupClearBits(xSystemEvents, BIT_MANUAL_MODE | BIT_SCRAM_ACTIVE);
                    xEventGroupSetBits(xSystemEvents, BIT_CMD_DRILL);
                }
                else if (cmd == 0x30) {
                    // CMD 3: SPILL DIRT
                    xEventGroupClearBits(xSystemEvents, BIT_MANUAL_MODE | BIT_SCRAM_ACTIVE);
                    xEventGroupSetBits(xSystemEvents, BIT_CMD_SPILL);
                }
                continue;
            }

            // MANUAL OVERRIDE (ID 193 / 0xC1) - Keeps Drill and Mixer manual control
            if ((events & BIT_MANUAL_MODE) && (msg.StdId == 193))
            {
                if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    uint8_t dc_mode = msg.Data[3];
                    uint16_t dc_speed = msg.Data[4];

                    if (dc_mode == 0) {
                        StopStirrer();
                        SetDrillLoweringSpeed_MC34931(0, 0);
                    }
                    else if (dc_mode == 1) SetStirrerSpeed_MC34931((dc_speed * 100) / 255, 1);
                    else if (dc_mode == 2) SetStirrerSpeed_MC34931((dc_speed * 100) / 255, 2);
                    else if (dc_mode == 3) SetDrillLoweringSpeed_MC34931((dc_speed * 1000) / 255, 1);
                    else if (dc_mode == 4) SetDrillLoweringSpeed_MC34931((dc_speed * 1000) / 255, 2);

                    uint8_t spin_mode = msg.Data[5];
                    int16_t spin_speed = (msg.Data[6] * 100) / 255; 

                    if (spin_mode == 0) SetDrillSpinSpeed_Talon(0);
                    else if (spin_mode == 1) SetDrillSpinSpeed_Talon(spin_speed);
                    else if (spin_mode == 2) SetDrillSpinSpeed_Talon(-spin_speed);

                    xSemaphoreGive(xMotorPowerMutex);
                }
            }
            // KOMENDY DLA DIODY UV (ID 194 / 0xC2)
            if (msg.StdId == 194)
            {
                // Uruchamiamy sekwencję UV (możesz wysłać obojętnie jakie dane, sam fakt ramki 194 to trigger)
                xEventGroupSetBits(xSystemEvents, BIT_CMD_UV);
                continue;
            }
        }
    }
}
// ---------------------------------------------------------
// 2. Main Drill Sequence Task
// ---------------------------------------------------------
void vTaskLabSequence(void* pvParameters)
{
    LabState_t currentState = LAB_STATE_IDLE;
    LabState_t prevState = LAB_STATE_SPILLING; 
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

        if (currentState != prevState) {
            state_entry = 1;
            prevState = currentState;
        } else {
            state_entry = 0;
        }

        switch (currentState)
        {
        case LAB_STATE_IDLE:
            if (events & BIT_CMD_HOME) {
                xEventGroupClearBits(xSystemEvents, BIT_CMD_HOME);
                currentState = LAB_STATE_HOMING;
            } 
            else if (events & BIT_CMD_DRILL) {
                xEventGroupClearBits(xSystemEvents, BIT_CMD_DRILL);
                currentState = LAB_STATE_DRILLING_DOWN;
            }
            else if (events & BIT_CMD_SPILL) {
                xEventGroupClearBits(xSystemEvents, BIT_CMD_SPILL);
                currentState = LAB_STATE_SPILLING;
            }
            break;

        case LAB_STATE_HOMING:
            if (state_entry)
            {
                if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    SetDrillLoweringSpeed_MC34931(800, 1); // Direction 2 (UP)
                    xSemaphoreGive(xMotorPowerMutex);
                }
            }

            if (IsDrillHomed()) // Top Limit (PB12)
            {
                if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    SetDrillLoweringSpeed_MC34931(0, 0); // Stop
                    xSemaphoreGive(xMotorPowerMutex);
                }
                currentState = LAB_STATE_IDLE;
            }
            break;

        case LAB_STATE_DRILLING_DOWN:
            if (state_entry)
            {
                if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    SetDrillSpinSpeed_Talon(TALON_SPEED);    // Spin drill
                    SetDrillLoweringSpeed_MC34931(800, 1);   // Direction 1 (DOWN)
                    xSemaphoreGive(xMotorPowerMutex);
                }
            }

            if (IsDrillAtBottomLimit()) // Bottom Limit (PB13)
            {
                if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    SetDrillLoweringSpeed_MC34931(0, 0); // Stop Going down
                    xSemaphoreGive(xMotorPowerMutex);
                }
                currentState = LAB_STATE_DRILLING_UP;    // Immediately transition to go back up
            }
            break;

        case LAB_STATE_DRILLING_UP:
            if (state_entry)
            {
                if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    SetDrillLoweringSpeed_MC34931(800, 2); // Direction 2 (UP)
                    // (Spindle stays spinning while retracting)
                    SetDrillSpinSpeed_Talon(0);          // Stop spindle
                    xSemaphoreGive(xMotorPowerMutex);
                }
            }

            if (IsDrillHomed()) // Top Limit (PB12)
            {
                if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    SetDrillLoweringSpeed_MC34931(0, 0); // Stop upward movement
                    SetDrillSpinSpeed_Talon(0);          // Stop spindle
                    xSemaphoreGive(xMotorPowerMutex);
                }
                currentState = LAB_STATE_IDLE;
            }
            break;

        case LAB_STATE_SPILLING:
            if (state_entry)
            {
                if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    // Spin drill fast to eject dirt (Adjust TALON_SPEED multiplier as needed)
                    SetDrillSpinSpeed_Talon(-TALON_SPEED * 2); 
                    xSemaphoreGive(xMotorPowerMutex);
                }
            }

            // Wait 3 seconds to spill dirt
            vTaskDelay(pdMS_TO_TICKS(3000));

            if (xSemaphoreTake(xMotorPowerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                SetDrillSpinSpeed_Talon(0); // Stop spinning
                xSemaphoreGive(xMotorPowerMutex);
            }
            currentState = LAB_STATE_IDLE;
            break;

        default:
            currentState = LAB_STATE_IDLE;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
// ---------------------------------------------------------
// Zadanie sterujące diodą UV
// ---------------------------------------------------------
void vTaskUVSequence(void* pvParameters)
{
    // Na starcie upewniamy się, że pin jest w stanie wysokiej impedancji (SET)
    // (Jeśli w CubeMX nazwałeś pin "UV", wygenerowały się makra UV_GPIO_Port i UV_Pin. 
    // Jeśli nie, użyj po prostu GPIOB i GPIO_PIN_15)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

    for (;;)
    {
        // Task "śpi" i czeka na ustawienie flagi BIT_CMD_UV z przerwania CAN.
        // Gdy flaga zostanie ustawiona, przechodzi dalej i automatycznie ją czyści (pdTRUE)
        xEventGroupWaitBits(xSystemEvents, BIT_CMD_UV, pdTRUE, pdFALSE, portMAX_DELAY);

        // --- 1. WŁĄCZENIE (1 zbocze opadające) ---
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // Zwarcie do GND (zbocze opadające)
        vTaskDelay(pdMS_TO_TICKS(50));                         // Krótki impuls 50ms w dole
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // Powrót do wysokiej impedancji
        
        // --- 2. ŚWIECENIE (Czekaj 1 sekundę) ---
        vTaskDelay(pdMS_TO_TICKS(1000));

        // --- 3. WYŁĄCZENIE (3 zbocza opadające co 0.5 sekundy) ---
        for (int i = 0; i < 3; i++)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // Zwarcie do GND
            vTaskDelay(pdMS_TO_TICKS(50));                         // Krótki impuls
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // Powrót do wysokiej impedancji

            if (i < 2) {
                vTaskDelay(pdMS_TO_TICKS(500)); // Przerwa 0.5s między impulsami
            }
        }
    }
}
// ---------------------------------------------------------
// 3. RTOS Initialization
// ---------------------------------------------------------
void LabRTOS_Init(void)
{
    xSystemEvents = xEventGroupCreate();
    xMotorPowerMutex = xSemaphoreCreateMutex();
    
    if (xUartMutex == NULL) {
        xUartMutex = xSemaphoreCreateMutex();
    }

    xCanMsgQueue = xQueueCreate(5, sizeof(CanMsg_t));

    xTaskCreate(vTaskLabSequence, "AutoSeq", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vTaskCanHandler, "CanRx", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(vTaskLabSequence, "AutoSeq", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vTaskCanHandler, "CanRx", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
    
    // DODANE: Rejestracja taska dla obsługi diody UV
    xTaskCreate(vTaskUVSequence, "UVSeq", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
}