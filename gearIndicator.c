#define F_CPU 1000000UL

//#define DEBUG
#define KOSTYL              // КОСТЫЛИЩЕ для борьбы со странностями сохранения в eeprom


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>

// Выводы контроллера, к которым подключены сегменты индикатора, кроме точки
#define LED_SEG_A               PORTA4
#define LED_SEG_B               PORTA5
#define LED_SEG_C               PORTA0
#define LED_SEG_D               PORTA1
#define LED_SEG_E               PORTA6
#define LED_SEG_F               PORTA3
#define LED_SEG_G               PORTA2
#define LED_SEG_PORT            PORTA
#define LED_SEG_DDR             DDRA

#define LED_SEG_DP              PORTB2
#define LED_DP_PORT             PORTB
#define LED_DP_DDR              DDRB

// Вход кнопки
#define BUTTON_PIN              PINB0
#define BUTTON_PORT             PINB
#define BUTTON_DDR              DDRB

// Вход датчика
#define SENSOR_PIN              PINA7
#define SENSOR_PORT             PINA
#define SENSOR_DDR              DDRA

#define LONG_PRESS_TIME         200   // Время длительного нажатия (мс/10), оно же время перехода в режим настроек
#define MIN_V                   10    // Минимальное заначение измеренного напряжения. Ниже этого будет отображаться -
#define CRC_ADDR_1              0x0E  // Адрес EEPROM первой CRC
#define CRC_ADDR_2              0x78  // Адрес EEPROM второй CRC
#define STORE_ADDR              0x10  // Адрес EEPROM первого элемента массива данных
#define BLINK_DELAY1            1000
#define BLINK_DELAY2            250


// Флаги состояни - биты переменной Flag
// 0x01 ==   1 == "00000001"
// 0x02 ==   2 == "00000010"
// 0x04 ==   4 == "00000100"
// 0x08 ==   8 == "00001000"
// 0x10 ==  16 == "00010000"
// 0x20 ==  32 == "00100000"
// 0x40 ==  64 == "01000000"
// 0x80 == 128 == "10000000" 
#define button                  0x1   // Кнопка. 1 - сейчас нажата. 0 - сейчас отпущена 
#define setupMode               0x2   // Режим настройки
#define setupNeeded             0x4   // Требуется перенастройка при ошибке crc данных
#define timer                   0x8   // Состояние таймера. 0 - таймер остановлен, 1 - таймер считает.
#define blink                   0x20  // Включен блинкер


// Макросы
// #define getFlag(x) ((Flag & (x)))
// #define setFlagUp(x) (Flag |= (x))
// #define setFlagDown(x) (Flag &= ~(x))

#define getFlag(x) ((USIDR & (x)))
#define setFlagUp(x) (USIDR |= (x))
#define setFlagDown(x) (USIDR &= ~(x))


#define TIKS_1MS (F_CPU/64/1000)
#define TIKS_10MS (F_CPU/1024/100)


// Знакогенератор    
const uint8_t symbolsGen[12] PROGMEM = {
    (1 << LED_SEG_B) | (1 << LED_SEG_C) | (1 << LED_SEG_E) | (1 << LED_SEG_F) | (1 << LED_SEG_G),                                       //H
    (1 << LED_SEG_B) | (1 << LED_SEG_C),                                                                                                //1
    (1 << LED_SEG_A) | (1 << LED_SEG_B) | (1 << LED_SEG_D) | (1 << LED_SEG_E) | (1 << LED_SEG_G),                                       //2
    (1 << LED_SEG_A) | (1 << LED_SEG_B) | (1 << LED_SEG_C) | (1 << LED_SEG_D) | (1 << LED_SEG_G),                                       //3
    (1 << LED_SEG_B) | (1 << LED_SEG_C) | (1 << LED_SEG_F) | (1 << LED_SEG_G),                                                          //4
    (1 << LED_SEG_A) | (1 << LED_SEG_C) | (1 << LED_SEG_D) | (1 << LED_SEG_F) | (1 << LED_SEG_G),                                       //5
    (1 << LED_SEG_A) | (1 << LED_SEG_C) | (1 << LED_SEG_D) | (1 << LED_SEG_E) | (1 << LED_SEG_F) | (1 << LED_SEG_G),                    //6
    (1 << LED_SEG_G),                                                                                                                   //-
    (1 << LED_SEG_A) | (1 << LED_SEG_D) | (1 << LED_SEG_G),                                                                             //Ξ
    (1 << LED_SEG_A) | (1 << LED_SEG_B) | (1 << LED_SEG_C) | (1 << LED_SEG_D) | (1 << LED_SEG_E) | (1 << LED_SEG_F) | (1 << LED_SEG_G), //8
    ~((1 << LED_SEG_A) | (1 << LED_SEG_B) | (1 << LED_SEG_C) | (1 << LED_SEG_D) | (1 << LED_SEG_E) | (1 << LED_SEG_F) | (1 << LED_SEG_G)),                                                                                                                       //off
    (1 << LED_SEG_A) | (1 << LED_SEG_D) | (1 << LED_SEG_E) | (1 << LED_SEG_F) | (1 << LED_SEG_G)                                        //E
};


__inline__ static void timerDelayMs(uint16_t ms) {
    while(ms--) {
        TCNT0 = 0;
        while(TCNT0 < TIKS_1MS);
    }
}

__inline__ static void startTimer(uint16_t ms) {
    setFlagUp(timer);
    TCNT1 = 65535 - TIKS_10MS * ms;                       // Начальное значение счётчика таймера
    TIMSK1 |= (1 << TOIE1);                               // Прерывание по переполнению таймера 1
    TCCR1B |= (1 << CS10) | (0 << CS11) | (1 << CS12);    // Установка предделителя на 1024
}

__inline__ static void stopTimer(void) {
    setFlagDown(timer);
    TIMSK1 &= ~(1 << TOIE1); // Выключаю прерывания по таймеру 1
    TCCR1B &= ~((0 << CS10) | (0 << CS11) | (0 << CS12)); // Выключаю таймер
    TCNT1 = 0;
}

ISR(TIM1_OVF_vect) {
    setFlagDown(timer);
    TIMSK1 &= ~(1 << TOIE1); // Выключаю прерывания по таймеру 1
    TCCR1B &= ~((0 << CS10) | (0 << CS11) | (0 << CS12)); // Выключаю таймер
    TCNT1 = 0;
}

__inline__ static void indicate(uint8_t sym) {
    LED_SEG_PORT &= pgm_read_byte(&(symbolsGen[10])); // Выключение сегментов
    LED_SEG_PORT |= pgm_read_byte(&(symbolsGen[sym])); 
    LED_DP_PORT = (getFlag(setupMode)) ? LED_DP_PORT | (1 << LED_SEG_DP) : LED_DP_PORT & ~(1 << LED_SEG_DP); // Управление запятой
}

__inline__ static uint8_t getButtonState(void) {   
    uint8_t res = 0;
    if (!getFlag(setupMode)) return 0;
    res = BUTTON_PORT & (1 << BUTTON_PIN);
    timerDelayMs(5);  // Задержка на случай дребезга контактов
    res = res & (BUTTON_PORT & (1 << BUTTON_PIN));
    return res;
}

__inline__ static uint8_t adcRead(void) {
    uint16_t res = 0;   
    for (uint8_t i = 0; i < 3; i++) {
        ADCSRA |= (1 << ADSC); // Старт одного преобразования
        while(ADCSRA & (1 << ADSC)); // Ожидание завершения преобразования
        res += ADCH;
    }
    return res / 3; 
}

__inline__ static void loadFromEeprom(uint8_t *gearsArray, uint8_t size) {
    uint16_t crc1 = 0;
    uint16_t crc2 = 0;
    uint16_t crc  = 0;
  
    crc1 = eeprom_read_word((uint16_t*)CRC_ADDR_1);
    crc2 = eeprom_read_word((uint16_t*)CRC_ADDR_2);
    eeprom_read_block((void*)gearsArray, (const void*)STORE_ADDR, size);
    
    uint8_t i = size;
    do {
        crc = _crc16_update(crc, gearsArray[size - i]);
    } while (--i);
    
    if (crc1 != crc2 || 0 == crc1 || 0xffff == crc1 || crc != crc1)  
        setFlagUp(setupNeeded);   //crc отличаются от расчётной или друг друга или = 0      
}
    
__inline__ static void storeToEeprom(uint8_t *gearsArray, uint8_t size) {
    uint16_t crc = 0;
    uint8_t d = 0;
    for (uint8_t i = 0; i < size; ++i) {
#ifdef KOSTYL
        d = (i != 0) ? gearsArray[i] - 5 : gearsArray[i];   // КОСТЫЛИЩЕ !!!
#else
        d = gearsArray[i];
#endif
        crc = _crc16_update(crc, d);
        eeprom_update_byte((uint8_t*)STORE_ADDR + i, d);
    }
    eeprom_update_word((uint16_t*)CRC_ADDR_1, crc);    
    eeprom_update_word((uint16_t*)CRC_ADDR_2, crc);    
}    

__inline__ static void setupDevice(uint8_t *gearsArray, uint8_t size) {
    uint8_t gear = 10; //init индикатор выключен

    stopTimer();
    setFlagDown(button);
    
    if (getFlag(setupNeeded)) gear = 1;
    
    for(;;) {
        
        uint8_t adcData = adcRead();
        
        if (getFlag(blink))
            startTimer(BLINK_DELAY1);
        
        if (getButtonState()) {                                           // Кнопка нажата
            if (!getFlag(button)) {
                setFlagUp(button);
                setFlagDown(blink);
                stopTimer();
                startTimer(LONG_PRESS_TIME);
            }
            if (getFlag(button) && !getFlag(timer)) {                     // Долгое нажатие
                setFlagDown(setupMode);
                setFlagDown(button);
                return;
            }     
        } else {                                           // Кнопка отжата
            if (getFlag(button)) { 
                switch (gear) {
                    case 10 :
                        gear = 1;       // init -> 1
                        break;
                    case 1  : 
                        gear = 0;       // 1 -> H
                        break;
                    case 0  :
                        gear = 2;       // H -> 2
                        break;
                    case 6  :
                        gear = 1;       // 6 -> 1
                        break;
                    default :
                        gear++;         // 2 -> 3 -> 4 -> 5 -> 6
                }
                setFlagDown(button); 
                setFlagUp(blink);
                stopTimer();
            }
        }
        gearsArray[gear] = adcData;
        if (getFlag(blink))                     // Индикация
            if (getFlag(timer)) {
                timerDelayMs(BLINK_DELAY2);
                indicate(10);
                timerDelayMs(BLINK_DELAY2);
                startTimer(BLINK_DELAY1);
            } ;
        indicate(gear);
    }
}


int main(void) {
    //setup();
    // Режим "выход" для выводов индикатора
    LED_SEG_DDR |=  (1 << LED_SEG_A) | (1 << LED_SEG_B) | (1 << LED_SEG_C) | (1 << LED_SEG_D) | (1 << LED_SEG_E) | (1 << LED_SEG_F) | (1 << LED_SEG_G);
    LED_DP_DDR  |=  (1 << LED_SEG_DP);
    
    // Режим "вход" для выводов кнопки и датчика
    BUTTON_DDR  &= ~(1 << BUTTON_PIN);
    SENSOR_DDR  &= ~(1 << SENSOR_PIN);
    
    // Настройка ЦАП
    ADMUX  &= ~((1 << REFS1) | (1 << REFS0)); // Опорное напряжение = Vcc (REFS[1:0]=0,0)
    ADCSRA |= (1 << ADEN) | (1 << ADPS2); // Предделитель 16 (62.5кГц) 
    ADCSRB |= (1 << ADLAR);
    
    // Начальная калибровка АЦП
    ADMUX |= (1 << MUX5); // вход на землю
    ADCSRA |= (1 << ADSC); // Старт одного преобразования
    while(ADCSRA & (1 << ADSC)); // Ожидание завершения преобразования
    ADMUX &= ~(1 << MUX5);
    // Настройка входа АЦП
    ADMUX  |= (1 << MUX2) | (1 << MUX1) | (1 << MUX0); //  вход ADC7(PA7) MUX[3:0] = 1,1,1

    // Предделитель 64 для таймера 0 
    TCCR0B |= (1 << CS01) | (1 << CS00);

    sei(); // Глобально разрешаю прерывания
    
    indicate(9);
    timerDelayMs(BLINK_DELAY1); 
    
    uint8_t gears[7] = {0, 0, 0, 0, 0, 0, 0};

    loadFromEeprom(gears, sizeof(gears));
    
    if (getFlag(setupNeeded)) {
        indicate(11);      // Показать E 1 и запустить настройку
        timerDelayMs(BLINK_DELAY1 / 2); 
        indicate(1); 
        timerDelayMs(BLINK_DELAY1 / 2); 
        setFlagUp(setupMode);
        setFlagUp(blink);
        setupDevice(gears, sizeof(gears));
//        initArray();
        storeToEeprom(gears, sizeof(gears)); // Сохраняю в EEPROM
        setFlagDown(setupNeeded);
    } else {
        startTimer(LONG_PRESS_TIME);
        setFlagUp(setupMode);
    }
            
    //loop();
    for(;;) {
        /* Обработка нажатия кнопки */
        
        if (getFlag(setupMode)) {
            if (getFlag(timer)) {
                if (getButtonState()) {    
//                    stopTimer();
                    setFlagUp(blink);
                    setupDevice(gears, sizeof(gears));
//                    initArray();
                    storeToEeprom(gears, sizeof(gears)); // Сохраняю в EEPROM                 
                }
            } else setFlagDown(setupMode);
        }
        /* Расчёт номера символа */  
        uint8_t adcData = adcRead();
        uint8_t gear = 0;
        if (adcData > MIN_V) {
            uint8_t i = sizeof(gears);
            while (i--) {
                if ((gears[i] > adcData - adcData / 10) && (gears[i] < adcData + adcData / 10)) {
                    gear = i;
                    break;
                } else gear = 8;
            }
        } else gear = 7;
        indicate(gear);
    }
}


