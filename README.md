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
  * **ID 0x1E:** Dolny rewolwer (probówki)
  * **ID 0x13:** Górny rewolwer (strzykawki/odczynniki)

### 5. Wejścia sygnałowe (Czujniki i Przyciski)
* **PB12:** Krańcówka wiertła `DRILL_HOME_SW` (GPIO Input + Pull-Up). Zewrzeć do GND w celu wyzwolenia.
* **PB13:** Przycisk startu `S_SWITCH` (GPIO Input + Pull-Up).

---

## 🏗️ Struktura Oprogramowania (Moduły)

Kod został podzielony na logiczne warstwy, oddzielające sprzęt od logiki operacyjnej:

* `main.c`: Konfiguracja wygenerowana przez STM32CubeMX, inicjalizacja układów i start schedulera RTOS.
* `motors.h` / `motors.c`: Warstwa abstrakcji sprzętu (HAL) dla silników DC. Obsługuje sprzętowe PWM, sprzętowy enkoder (wraz z wirtualnym 32-bitowym przepełnieniem w przerwaniu) oraz debouncing przycisków.
* `dynamixel.h` / `dynamixel.c`: Niskopoziomowy sterownik dla serw AX-12A operujący na sprzętowym trybie Half-Duplex USART1.
* `lab_sequence.h` / `lab_sequence.c`: Serce systemu. Zawiera zadania (Tasks) FreeRTOS, maszyny stanów, oraz mechanizmy IPC (Kolejki, Muteksy).

---

## ⚙️ FreeRTOS - Konfiguracja i IPC (Inter-Process Communication)

System wykorzystuje natywne API FreeRTOS z następującymi mechanizmami:

### Zadania (Tasks)
1. `vTaskLabSequence` (Priorytet: Normalny+2) - Główna maszyna stanów realizująca sekwencję (IDLE -> HOMING -> DRILLING -> ...).
2. `vTaskDynamixel` (Priorytet: Normalny+3) - Task dedykowany do odbierania komend z kolejki i bezpiecznego wysyłania ramek UART do serw.

### Bezpieczeństwo i Synchronizacja
* **`xSystemEvents` (Event Group):** Przechowuje globalne flagi stanu systemu, m.in. `BIT_SCRAM_ACTIVE` (Natychmiastowe zatrzymanie maszyn) oraz `BIT_DRILL_LOWERED`.
* **`xMotorPowerMutex` (Mutex):** Gwarantuje wzajemne wykluczanie pracy silnika opuszczania wiertła i silnika mieszadła (zapobiega fizycznym kolizjom i przeciążeniom zasilania).
* **Blokada Dynamixeli:** Task `vTaskDynamixel` natywnie odrzuca komendy obrotu, jeśli flaga `BIT_DRILL_LOWERED` jest podniesiona, co chroni wiertło przed zderzeniem z rewolwerami.
* **Debouncing Krańcówki:** Implementacja programowa (hamowanie przed potwierdzeniem stanu) w `vTaskLabSequence`.

---

## 🚀 Maszyna Stanów (Logika Operacyjna)

Cykl pracy laboratorium przechodzi przez zdefiniowane stany (`LabState_t`):
1. **IDLE:** Oczekiwanie na sygnał z pinu PB13 lub flagę CAN.
2. **HOMING:** Dojazd wiertła w górę do momentu wciśnięcia krańcówki (PB12) z zabezpieczeniem przed drganiami styków. Reset enkodera.
3. **HOMING_REVOLVERS:** Wyjazd rewolwerów na pozycje bezpieczne.
4. **DRILLING:** Włączenie obrotów, zjazd w dół, i odczyt głębokości z 32-bitowego licznika enkodera.
5. **RETRACT:** Wycofanie wiertła nad probówkę.
6. **TUBE_POS:** *(W trakcje wdrażania)* Podjazd odpowiednią probówką na pozycję pod wiertłem.
7. *Kolejne kroki: FILL_TUBE, REAGENT_POS, DOSING, STIRRER_POS, STIRRING, SPECTROMETER_POS, SPECTROMETER_FLASH...*

---

## 🛠️ Ważne notatki środowiskowe (STM32CubeMX)
* FreeRTOS wdrożony metodą "Bare-Metal" (pominięcie CMSIS-V2 z CubeMX). Przerwania `SysTick`, `SVC` i `PendSV` są zmapowane sprzętowo w `FreeRTOSConfig.h`.
* Zegar Timebase Source dla `HAL_Delay` ustawiony na **TIM2**.
* Przerwania włączone dla `TIM2` (Timebase) oraz `TIM4` (Update/Overflow dla enkodera). Przerwania `SysTick` / `SVC` odznaczone w sekcji Code Generation (NVIC).

---

## 📋 TODO (Następne kroki)
- [ ] Kalibracja wartości na sprzęcie (głębokość z enkodera, bezpieczne kąty Dynamixeli).
- [ ] Implementacja magistrali CAN.
- [ ] Dokończenie stanów dozowania (obsługa pompek/elektrozaworów).
- [ ] Konfiguracja sygnału błysku dla spektrometru.