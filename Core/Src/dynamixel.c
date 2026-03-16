#include "dynamixel.h"
#include <stdlib.h>

extern UART_HandleTypeDef huart1;    // Zmienna z main.c/usart.c
SemaphoreHandle_t xUartMutex = NULL; // Definicja muteksa

int8_t AX12_CalculateChecksum(uint8_t id, uint8_t len, uint8_t instruction, uint8_t* params, uint8_t params_len)
{
    uint16_t accumulator = id + len + instruction;
    for (int i = 0; i < params_len; i++)
    {
        accumulator += params[i];
    }
    return (uint8_t)(~accumulator);
}

void AX12_SendPacket(uint8_t id, uint8_t instruction, uint8_t* params, uint8_t params_len)
{
    uint8_t buffer[20];
    uint8_t len = params_len + 2;

    buffer[0] = 0xFF;
    buffer[1] = 0xFF;
    buffer[2] = id;
    buffer[3] = len;
    buffer[4] = instruction;

    for (int i = 0; i < params_len; i++)
    {
        buffer[5 + i] = params[i];
    }
    buffer[5 + params_len] = AX12_CalculateChecksum(id, len, instruction, params, params_len);

    // ZABEZPIECZENIE: Czekamy na dostęp do UART (max 100ms)
    if (xUartMutex != NULL && xSemaphoreTake(xUartMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        HAL_HalfDuplex_EnableTransmitter(&huart1);
        HAL_UART_Transmit(&huart1, buffer, 6 + params_len, 10);
        HAL_HalfDuplex_EnableReceiver(&huart1);

        // Zwalniamy dostęp dla innych tasków
        xSemaphoreGive(xUartMutex);
    }
}

uint16_t AX12_ReadPosition(uint8_t id)
{
    uint8_t tx_buffer[8];
    uint8_t rx_buffer[16]; // Zwiększony bufor, by wchłonąć ewentualne "śmieci"
    uint8_t rx_index = 0;

    tx_buffer[0] = 0xFF;
    tx_buffer[1] = 0xFF;
    tx_buffer[2] = id;
    tx_buffer[3] = 0x04;                  // Długość: Instruction(1) + Params(2) + Checksum(1)
    tx_buffer[4] = INST_READ_DATA;        // 0x02
    tx_buffer[5] = ADDR_PRESENT_POSITION; // 0x24 (Rejestr 36)
    tx_buffer[6] = 0x02;                  // Chcemy odczytać 2 bajty (Low i High byte)
    tx_buffer[7] = AX12_CalculateChecksum(id, 0x04, INST_READ_DATA, &tx_buffer[5], 2);

    if (xUartMutex != NULL && xSemaphoreTake(xUartMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {

        // 1. Nadawanie
        HAL_HalfDuplex_EnableTransmitter(&huart1);
        HAL_UART_Transmit(&huart1, tx_buffer, 8, 10);

        // 2. Przełączenie na odbiór
        HAL_HalfDuplex_EnableReceiver(&huart1);

        // 3. HARDWARE FLUSH: Brutalne wyczyszczenie rejestru danych STM32
        // z ewentualnych śmieci po przełączeniu pinu
        __HAL_UART_CLEAR_OREFLAG(&huart1);             // Czyścimy flagę Overrun
        volatile uint32_t dummy = huart1.Instance->SR; // Odczyt Status Register
        dummy = huart1.Instance->DR;                   // Odczyt Data Register
        (void)dummy;                                   // Unikanie warningu kompilatora

        // 4. Odbieranie "Znak po Znaku" z oknem wyszukiwania
        uint32_t start_time = HAL_GetTick();
        while ((HAL_GetTick() - start_time) < 20)
        { // Timeout 20ms

            uint8_t byte;
            // Odczytujemy jeden bajt (z minimalnym timeoutem)
            if (HAL_UART_Receive(&huart1, &byte, 1, 1) == HAL_OK)
            {
                if (rx_index < sizeof(rx_buffer))
                {
                    rx_buffer[rx_index++] = byte;
                }
            }

            // Jeśli mamy już minimum 8 bajtów, zaczynamy szukać ramki
            if (rx_index >= 8)
            {
                // Skanujemy bufor w poszukiwaniu nagłówka (pływające okno)
                for (int i = 0; i <= rx_index - 8; i++)
                {
                    if (rx_buffer[i] == 0xFF && rx_buffer[i + 1] == 0xFF && rx_buffer[i + 2] == id)
                    {

                        // Opcjonalnie: można tu sprawdzić Checksum, ale nagłówek + id zazwyczaj wystarczą
                        uint16_t position = rx_buffer[i + 5] | (rx_buffer[i + 6] << 8);

                        xSemaphoreGive(xUartMutex);
                        return position; // Sukces!
                    }
                }
            }
        }

        // Timeout minął, nie znaleziono ramki
        xSemaphoreGive(xUartMutex);
    }

    return 0xFFFF; // Błąd muteksa lub timeout UART
}

void AX12_SetGoalPosition(uint8_t id, uint16_t position)
{
    if (position > 1023)
        position = 1023; // Limit 0x3FF [cite: 306]

    uint8_t params[3];
    params[0] = ADDR_GOAL_POSITION;
    params[1] = position & 0xFF;
    params[2] = (position >> 8) & 0xFF;

    AX12_SendPacket(id, INST_WRITE_DATA, params, 3);
}

void AX12_SetLED(uint8_t id, uint8_t state)
{
    uint8_t params[2];
    params[0] = ADDR_LED;
    params[1] = state ? 1 : 0;

    AX12_SendPacket(id, INST_WRITE_DATA, params, 2);
}

uint8_t AX12_Ping(uint8_t id)
{
    uint8_t buffer[6];
    uint8_t rx_buffer[6];

    // Konstrukcja pakietu PING
    // Długość = 2 (Instrukcja + Checksum)
    buffer[0] = 0xFF;
    buffer[1] = 0xFF;
    buffer[2] = id;
    buffer[3] = 0x02; // Length
    buffer[4] = 0x01; // Instruction: PING

    // Checksum = ~(ID + Length + Instruction)
    buffer[5] = (uint8_t)(~(id + 0x02 + 0x01));

    // 1. Nadawanie
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, buffer, 6, 10);

    // 2. Odbiór
    // Musimy szybko przełączyć, aby złapać odpowiedź
    HAL_HalfDuplex_EnableReceiver(&huart1);

    // 3. Oczekiwanie na odpowiedź (Status Packet ma 6 bajtów dla PING)
    // Timeout 10ms jest wystarczający (Return Delay Time to zazwyczaj 500us)
    if (HAL_UART_Receive(&huart1, rx_buffer, 6, 10) == HAL_OK)
    {
        // Sprawdzenie nagłówka [cite: 187]
        if (rx_buffer[0] == 0xFF && rx_buffer[1] == 0xFF && rx_buffer[2] == id)
        {
            return 1; // Znaleziono serwo!
        }
    }

    return 0; // Brak odpowiedzi (Timeout)
}

void AX12_ScanBus(void)
{
    // Skanujemy zakres 0-253 [cite: 164]
    for (uint8_t id = 0; id < 254; id++)
    {
        if (AX12_Ping(id))
        {
            // Tutaj możesz wstawić breakpoint lub mignąć LED
            // Znaleziono ID: zmienna 'id'

            // Przykład: Mignij LED tyle razy, jakie jest ID (dla małych ID)
            // Lub po prostu zatrzymaj się w debuggerze, żeby podejrzeć zmienną 'id'

            // Jeśli masz tylko jedno serwo i je znalazłeś, możesz przerwać pętlę:
            // detected_id = id;
            // break;
        }
        HAL_Delay(2); // Krótka przerwa między zapytaniami
    }
}

void AX12_SetMode_Wheel(uint8_t id)
{
    // Uwaga: Parametrów danych jest 4, ale pierwszy bajt w params[] to adres.
    // Funkcja SendPacket (z poprzedniego kodu) bierze tablicę parametrów,
    // gdzie params[0] to adres rejestru.
    // Zmodyfikujmy tablicę, aby pasowała do Twojej funkcji AX12_SendPacket:

    uint8_t data_packet[5];
    data_packet[0] = 0x06; // Adres startowy
    data_packet[1] = 0x00; // CW L
    data_packet[2] = 0x00; // CW H
    data_packet[3] = 0x00; // CCW L
    data_packet[4] = 0x00; // CCW H

    AX12_SendPacket(id, INST_WRITE_DATA, data_packet, 5);
}

void AX12_Wheel_Move(uint8_t id, int16_t speed)
{
    uint16_t speed_value;

    // Ograniczenie zakresu
    if (speed > 1023)
        speed = 1023;
    if (speed < -1023)
        speed = -1023;

    if (speed < 0)
    {
        // Kierunek CW (Bit 10 ustawiony na 1)
        // Wartość absolutna prędkości + 1024 (0x400)
        speed_value = abs(speed) | 0x400;
    }
    else
    {
        // Kierunek CCW (Bit 10 ustawiony na 0)
        speed_value = speed;
    }

    uint8_t params[3];
    params[0] = 0x20;                      // Adres: Moving Speed (L)
    params[1] = speed_value & 0xFF;        // Low Byte
    params[2] = (speed_value >> 8) & 0xFF; // High Byte

    AX12_SendPacket(id, INST_WRITE_DATA, params, 3);
}
void AX12_SetMode_Joint(uint8_t id)
{
    uint8_t params[5];

    // Ustawiamy domyślne limity:
    // CW Limit = 0
    // CCW Limit = 0x3FF (1023) -> 300 stopni

    params[0] = 0x06; // Adres startowy: CW Angle Limit
    params[1] = 0x00; // CW L
    params[2] = 0x00; // CW H
    params[3] = 0xFF; // CCW L (0xFF)
    params[4] = 0x03; // CCW H (0x03) -> 0x03FF = 1023

    AX12_SendPacket(id, INST_WRITE_DATA, params, 5);
}

void AX12_SetMovingSpeed(uint8_t id, uint16_t speed)
{
    if (speed > 1023)
        speed = 1023;

    uint8_t params[3];
    params[0] = ADDR_MOVING_SPEED; // 0x20
    params[1] = speed & 0xFF;
    params[2] = (speed >> 8) & 0xFF;

    AX12_SendPacket(id, INST_WRITE_DATA, params, 3);
}
// Dodajmy tylko prostą funkcję inicjalizacyjną, żeby upewnić się, że serwa są w trybie Joint (pozycjonowania)
void AX12_Init(void)
{
    AX12_SetMode_Joint(DYNAMIXEL_TUBE_ID);
    vTaskDelay(pdMS_TO_TICKS(10));
    AX12_SetMode_Joint(DYNAMIXEL_SYRINGE_ID);
    vTaskDelay(pdMS_TO_TICKS(10));

    AX12_SetMovingSpeed(DYNAMIXEL_TUBE_ID, 100);
    vTaskDelay(pdMS_TO_TICKS(10));
    AX12_SetMovingSpeed(DYNAMIXEL_SYRINGE_ID, 100);
    vTaskDelay(pdMS_TO_TICKS(10));
}