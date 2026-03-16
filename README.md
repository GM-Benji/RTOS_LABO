# Automatyczne Laboratorium Glebowe - System Sterowania (STM32 + FreeRTOS)

Projekt oprogramowania wbudowanego dla zrobotyzowanego laboratorium pobierającego i analizującego próbki gleby. System oparty jest na mikrokontrolerze **STM32F103RB** i systemie operacyjnym **FreeRTOS** (czysty port, bez warstwy CMSIS-RTOS), co zapewnia determinizm czasowy i wysoki poziom bezpieczeństwa mechaniki.

## 📌 Architektura Sprzętowa i Pinout (STM32F103RB)

### 1. Napęd Wiertła (Talon SRX - Sygnał RC 50Hz)
* **PB14 (TIM1_CH2N):** PWM generujący sygnał sterujący serwem (1-2 ms).

### 2. Silnik Opuszczania Wiertła (MC34931 - PWM 10kHz)
* **PB0 (TIM3_CH3):** IN2 (Kierunek dół - sygnał PWM)
* **PB1 (TIM3_CH4):** IN1 (Kierunek góra - sygnał PWM)
* **PB2:** D1 (Disable - stan niski)
* **PB10:** EN/D2 (Enable/Sleep - stan wysoki)
* **PB6 (TIM4_CH1):** Enkoder faza A
* **PB7 (TIM4_CH2):** Enkoder faza B

### 3. Silnik Mieszadła (MC34931 - PWM 10kHz)
* **PA6 (TIM3_CH1):** IN2 (Kierunek A - sygnał PWM)
* **PA7 (TIM3_CH2):** IN1 (Kierunek B - sygnał PWM)
* **PA3:** D1 (Disable - stan niski)
* **PB11:** EN/D2 (Enable/Sleep - stan wysoki)

### 4. Rewolwery i Strzykawki (Serwa Dynamixel AX-12A - Protokół 1.0)
* **PA9 (USART1_TX):** Linia DATA (Single-Wire / Half-Duplex). Wymagany zewnętrzny rezystor Pull-up (~4.7kΩ - 10kΩ) do 5V. Baudrate: 1 000 000 bps.
  * **ID 0x1E (30):** Dolny rewolwer (probówki). Baza: 790, Odstęp: 123 (36°). Pozycja bezpieczna: 1023.
  * **ID 0x01 (01):** Górny rewolwer (strzykawki/odczynniki). Pozycja bezpieczna: 600.
* *Uwaga sprzętowa:* Zaimplementowano programowe "pływające okno" odczytu oraz sprzętowe czyszczenie bufora UART (`__HAL_UART_CLEAR_OREFLAG`), aby wyeliminować zakłócenia (glitche) pojawiające się przy przełączaniu pinu w trybie Half-Duplex.

### 5. Wejścia sygnałowe (Czujniki i Przyciski)
* **PB12:** Krańcówka wiertła `DRILL_HOME_SW` (GPIO Input + Pull-Up). Zewrzeć do GND w celu wyzwolenia.
* **PB13:** Przycisk startu `S_SWITCH` (GPIO Input + Pull-Up).

---

## 🏗️ Struktura Oprogramowania (Moduły)

Kod został podzielony na logiczne warstwy, oddzielające sprzęt od logiki operacyjnej:

* `main.c`: Konfiguracja wygenerowana przez STM32CubeMX, inicjalizacja układów i start schedulera RTOS.
* `motors.h` / `motors.c`: Warstwa abstrakcji sprzętu (HAL) dla silników DC. Obsługuje sprzętowe PWM, 32-bitowy enkoder programowy oraz debouncing przycisków.
* `dynamixel.h` / `dynamixel.c`: Niskopoziomowy sterownik dla serw AX-12A. Realizuje zamkniętą pętlę sterowania (odczyt pozycji z rejestru `0x24`), spowolnienie ruchu (rejestr `0x20`) i zabezpiecza komunikację muteksem.
* `lab_sequence.h` / `lab_sequence.c`: Serce systemu. Zawiera zadania (Tasks) FreeRTOS, maszyny stanów, oraz mechanizmy IPC.

---

## ⚙️ FreeRTOS - Konfiguracja i IPC (Inter-Process Communication)

System wykorzystuje natywne API FreeRTOS z następującymi mechanizmami:

### Zadania (Tasks)
1. `vTaskLabSequence` (Priorytet: Normalny+2) - Główna maszyna stanów realizująca cykl pracy laboratorium.
2. `vTaskDynamixel` (Priorytet: Normalny+3) - Task odbierający komendy z kolejki i wysyłający ramki UART do serw (zabezpieczony przed ruchem przy opuszczonym wiertle).

### Bezpieczeństwo i Synchronizacja
* **`xSystemEvents` (Event Group):** Przechowuje globalne flagi stanu systemu (`BIT_SCRAM_ACTIVE`, `BIT_DRILL_LOWERED`).
* **`xMotorPowerMutex` (Mutex):** Gwarantuje wzajemne wykluczanie pracy silnika wiertła i silnika mieszadła.
* **`xUartMutex` (Mutex):** Zapobiega kolizjom ramek na jednoprzewodowej magistrali UART (blokuje nadawanie komend z kolejki, gdy maszyna stanów odpytuje serwo o aktualną pozycję).
* **Closed-Loop Dynamixel:** Maszyna stanów nie przechodzi do kolejnego kroku, dopóki fizycznie nie potwierdzi osiągnięcia zadanej pozycji przez serwa (z marginesem tolerancji i 5-sekundowym timeoutem).

---

## 🚀 Maszyna Stanów (Logika Operacyjna)

Cykl pracy laboratorium przechodzi przez zdefiniowane stany (`LabState_t`):
1. **IDLE:** Oczekiwanie na sygnał z pinu PB13 (lub CAN).
2. **HOMING:** Dojazd wiertła w górę do momentu wciśnięcia krańcówki (PB12). Reset enkodera.
3. **HOMING_REVOLVERS:** Powolny wyjazd obu rewolwerów na pozycje bezpieczne (1023 i 600), aby uniknąć kolizji wiertła z probówkami.
4. **DRILLING:** Włączenie obrotów CW i zjazd w dół na zadaną głębokość na podstawie enkodera.
5. **RETRACT:** Wycofanie wiertła nad rewolwer.
6. **TUBE_POS:** Podjazd dolnego rewolweru na pozycję wyliczoną ze wzoru (Baza 790 - Indeks * 123). Oczekiwanie na potwierdzenie sprzętowe osiągnięcia pozycji.
7. **FILL_TUBE:** Zrzucanie gleby (obroty CCW przez 2 sekundy). Logika zakłada, że jeden zjazd wiertła wystarcza na napełnienie 2 probówek. System zapętla się z punktem 6 dla nieparzystych indeksów.
8. *Kolejne kroki: REAGENT_POS, DOSING, STIRRER_POS, STIRRING, SPECTROMETER_POS, SPECTROMETER_FLASH...*

---

## 🛠️ Ważne notatki środowiskowe (STM32CubeMX)
* FreeRTOS wdrożony metodą "Bare-Metal" (pominięcie CMSIS-V2 z CubeMX). Przerwania `SysTick`, `SVC` i `PendSV` są zmapowane sprzętowo w `FreeRTOSConfig.h`.
* Zegar Timebase Source dla `HAL_Delay` ustawiony na **TIM2**.
* Przerwania włączone dla `TIM2` (Timebase) oraz `TIM4` (Update/Overflow dla enkodera).

---

## 📋 TODO (Następne kroki)
- [ ] Implementacja magistrali CAN.
- [ ] Dokończenie stanów dozowania (obsługa pompek/elektrozaworów w `LAB_STATE_DOSING`).
- [ ] Konfiguracja pinu wyzwalającego błysk dla spektrometru.