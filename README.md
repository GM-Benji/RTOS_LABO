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
* *Uwaga sprzętowa:* Zaimplementowano programowe "pływające okno" odczytu oraz sprzętowe czyszczenie bufora UART, aby wyeliminować zakłócenia w trybie Half-Duplex. Ze względu na martwą strefę potencjometru serwa (1024-1229), system korzysta z wyselekcjonowanej sekwencji gniazd: `{6, 4, 2, 0}`.

### 5. Komunikacja CAN (Magistrala Systemowa)
* **Baudrate:** 500 kbps (Standard ID, 11-bit, DLC=8).
* **ID 0x095:** Ramka sterowania systemem (0x01 = Start Auto, 0x02 = SCRAM, 0x03 = Tryb Manualny).
* **ID 0x096:** Tryb manualny (sterowanie Serwem 1, Mieszadłem, Wiertłem).
* **ID 0x097:** Tryb manualny (sterowanie Serwem 2).
* **ID 0x098:** Sterowanie modułem spektrometru (TX: 0x01 = błysk 1s).

### 6. Wejścia sygnałowe (Czujniki i Przyciski)
* **PB12:** Krańcówka wiertła `DRILL_HOME_SW` (GPIO Input + Pull-Up). Zewrzeć do GND w celu wyzwolenia.
* **PB13:** Przycisk startu `S_SWITCH` (GPIO Input + Pull-Up).

---

## 🏗️ Struktura Oprogramowania (Moduły)

Kod został podzielony na logiczne warstwy:
* `main.c`: Konfiguracja wygenerowana przez STM32CubeMX, start CAN i schedulera RTOS.
* `motors.h/c`: Warstwa abstrakcji sprzętu (HAL) dla silników DC. Obsługuje sprzętowe PWM i 32-bitowy enkoder programowy.
* `dynamixel.h/c`: Niskopoziomowy sterownik dla serw AX-12A. Realizuje zamkniętą pętlę sterowania i chroni magistralę muteksem.
* `lab_sequence.h/c`: Serce systemu. Zawiera zadania (Tasks) FreeRTOS, maszynę stanów cyklu laboratoryjnego, obsługę poleceń CAN oraz tryb ręczny.

---

## ⚙️ FreeRTOS - Konfiguracja i IPC 

System wykorzystuje natywne API FreeRTOS (Stos: 8192 B) z następującymi mechanizmami:

### Zadania (Tasks)
1. `vTaskLabSequence` (Priorytet: Normalny+2) - Główna maszyna stanów cyklu automatycznego.
2. `vTaskDynamixel` (Priorytet: Normalny+3) - Obsługa sprzętowa UART dla serw.
3. `vTaskCanHandler` (Priorytet: Normalny+3) - Odbiór i parsowanie ramek z magistrali CAN (zarządzanie systemem i tryb manualny).

### Bezpieczeństwo i Synchronizacja
* **`xSystemEvents` (Event Group):** Flagi stanu systemu (`BIT_SCRAM_ACTIVE`, `BIT_MANUAL_MODE`, `BIT_START_AUTO`).
* **`xCanMsgQueue` (Queue):** Bezpieczna kolejka przekazująca odebrane ramki CAN z przerwania `RxFifo0` do taska.
* **`xMotorPowerMutex` & `xUartMutex`:** Zabezpieczają zasoby sprzętowe przed jednoczesnym dostępem z różnych zadań.
* **Przerwania (NVIC):** Przerwanie `CAN RX0` ma wymuszony priorytet `5`, aby bezpiecznie współpracować z funkcjami `...FromISR` systemu FreeRTOS.

---

## 🚀 Maszyna Stanów (Logika Operacyjna)

Cykl pracy został zaprojektowany do obsługi par probówek w celu optymalizacji i ominięcia martwych stref serwomechanizmów:
1. **IDLE:** Oczekiwanie na sygnał z przycisku lub ramkę CAN (ID: 0x095).
2. **HOMING:** Zerowanie układu.
3. **DRILLING & FILL_TUBE:** Odwiert i naprzemienne zasypywanie dwóch probówek z pary ziemią.
4. **REAGENT_POS & DOSING:** Podjazd i zadozowanie odczynników z górnego rewolweru do obu napełnionych probówek.
5. **STIRRER_POS & STIRRING:** Mieszanie każdej z probówek przez 5 sekund.
6. **SPECTROMETER_FLASH:** Oświetlenie żarówką (komenda CAN) każdej probówki pod spektrometrem na 1 sekundę.

---

## 📋 TODO (Następne kroki)
- [x] Implementacja magistrali CAN (tryb automatyczny i ręczny).
- [x] Konfiguracja oświetlenia spektrometru (ramka 0x098).
- [x] Ominięcie martwej strefy serw AX-12A w kodzie.
- [ ] Refaktoryzacja `lab_sequence.c` (wydzielenie obsługi CAN do osobnego pliku).
- [ ] Testy wytrzymałościowe pełnego cyklu mechanicznego z glebą.