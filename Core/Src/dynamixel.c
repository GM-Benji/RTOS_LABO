#include "dynamixel.h"
#include <stdlib.h>

extern UART_HandleTypeDef huart1; // Zmienna z main.c/usart.c

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
    uint8_t len = params_len + 2; // N parametrów + 2 (Instr + Checksum) [cite: 170]

    buffer[0] = 0xFF;
    buffer[1] = 0xFF; // Start bytes [cite: 162]
    buffer[2] = id;
    buffer[3] = len;
    buffer[4] = instruction;

    for (int i = 0; i < params_len; i++)
    {
        buffer[5 + i] = params[i];
    }

    buffer[5 + params_len] = AX12_CalculateChecksum(id, len, instruction, params, params_len);

    // 1. Włącz nadajnik (STM32 przejmuje linię)
    HAL_HalfDuplex_EnableTransmitter(&huart1);

    // 2. Wyślij dane
    HAL_UART_Transmit(&huart1, buffer, 6 + params_len, 10);

    // 3. Przełącz na odbiór (Zwolnij linię, aby serwo mogło odpowiedzieć - Status Packet)
    // Jest to konieczne nawet jeśli nie czytamy od razu odpowiedzi, aby nie blokować linii
    HAL_HalfDuplex_EnableReceiver(&huart1);
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
// Dodajmy tylko prostą funkcję inicjalizacyjną, żeby upewnić się, że serwa są w trybie Joint (pozycjonowania)
void AX12_Init(void)
{
    AX12_SetMode_Joint(DYNAMIXEL_TUBE_ID);
    HAL_Delay(10);
    AX12_SetMode_Joint(DYNAMIXEL_SYRINGE_ID);
    HAL_Delay(10);
}