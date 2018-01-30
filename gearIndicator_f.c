#define F_CPU 1000000UL

#define DEBUG

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/crc16.h>
#include <util/delay.h>


#ifdef DEBUG
#define TX_PIN                  PORTB1
#define TX_PORT                 PORTB
#define TX_DDR                  DDRB
#endif

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
#define STORE_ADDR              0x00  // Адрес EEPROM первого элемента массива данных

//#define KOSTYL              // КОСТЫЛИЩЕ для борьбы со странностями сохранения в eeprom

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
#define timer                   0x8   // // Состояние таймера. 0 - таймер остановлен, 1 - таймер считает.

// Макросы
#define getFlag(x) ((Flag & x))
#define setFlagUp(x) (Flag |= x)
#define setFlagDown(x) (Flag &= ~x)
#define TIKS_1MS (F_CPU/64/1000)
#define TIKS_10MS (F_CPU/1024/100)

// Знакогенератор    
const uint8_t symbolsGen[12] = {
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


// Глобальные переменные
volatile uint8_t Flag = 0;     // Флаги состояния работы
volatile uint8_t symbol = 7;   // Номер символа в таблице знакогенератора. Он будет отображаться при вызовае indicate()
volatile uint8_t adcData = 0;  // Результат последнего измерения данных датчика

//volatile uint8_t gears[7] = {0xEC,0x19,0x26,0x33,0x40,0x4E,0x5B};

volatile uint8_t gears[7] = {0, 0, 0, 0, 0, 0, 0};

void initArray(void) {
    gears[0] = 0xEC;
    gears[1] = 0x19;
    gears[2] = 0x26;
    gears[3] = 0x33;
    gears[4] = 0x40;
    gears[5] = 0x4E;
    gears[6] = 0x5B;
    
}



#ifdef DEBUG
/*
    4800 задержка 208
    9600 задержка 104
*/
void transmitChar(uint8_t data)
{
    data = ~data;
    TX_PORT &= ~TX_PIN;
    _delay_us(208);
    _delay_us(208);
    for ( uint8_t i = 0; i < 8; i++ ) {
        if(data & 1)
            TX_PORT &= ~TX_PIN;
        else
            TX_PORT |= TX_PIN;
        data = data >> 1;
        _delay_us(208);
    }
    TX_PORT |= TX_PIN;
    _delay_us(208);
    _delay_us(208);
    return;
}
#endif


#ifndef DEBUG
__inline__ static void timerDelayMs(uint16_t ms) {
    while(ms--) {
        TCNT0 = 0;
        while(TCNT0 < TIKS_1MS);
    }
}
#endif

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
   stopTimer();
}

__inline__ static void indicate(void) {
    LED_SEG_PORT &= symbolsGen[10]; // Выключение сегментов
    LED_SEG_PORT |= symbolsGen[symbol]; 
    LED_DP_PORT = (getFlag(setupMode)) ? LED_DP_PORT | (1 << LED_SEG_DP) : LED_DP_PORT & ~(1 << LED_SEG_DP); // Управление запятой
}

__inline__ static uint8_t getButtonState(void) {   
    uint8_t res = 0;
    if (!getFlag(setupMode)) return 0;
    res = BUTTON_PORT & (1 << BUTTON_PIN);
#ifndef DEBUG    
    timerDelayMs(5);  // Задержка на случай дребезга контактов
#else
    _delay_ms(5);
#endif
    res = res & (BUTTON_PORT & (1 << BUTTON_PIN));
    return res;
}

__inline__ static void adcRead(void) {
    uint16_t res = 0;   
    for (uint8_t i = 0; i < 3; i++) {
        ADCSRA |= (1 << ADSC); // Старт одного преобразования
        while(ADCSRA & (1 << ADSC)); // Ожидание завершения преобразования
        res += ADCH;
#ifndef DEBUG    
//        timerDelayMs(5);
#else
//        _delay_ms(5);
#endif
    }
    adcData = res / 3; 
}

__inline__ static void loadFromEeprom(void) {
    uint16_t crc1 = 0;
    uint16_t crc2 = 0;
    uint16_t crc  = 0;
  
    crc1 = eeprom_read_word((uint16_t*)CRC_ADDR_1);
    crc2 = eeprom_read_word((uint16_t*)CRC_ADDR_2);
    eeprom_read_block((void*)&gears, (void*)STORE_ADDR, sizeof gears);
    
    for (uint8_t i = 0; i < 7; i++) 
        crc = _crc16_update(crc, gears[i]);
  
    if (crc1 != crc2 || 0 == crc1 || 0xffff == crc1 || crc != crc1)  
        setFlagUp(setupNeeded);   //crc отличаются от расчётной или друг друга или = 0      
}
    
void storeToEeprom(void) {
    uint16_t crc = 0;
    uint8_t d = 0;
    for (uint8_t i = 0; i < 7; ++i) {
#ifdef KOSTYL
        d = (!getFlag(setupNeeded) && i != 0) ? gears[i] - 5 : gears[i];   // КОСТЫЛИЩЕ !!!
#else
        d = gears[i];
#endif
// #ifdef DEBUG
//         transmitChar(0x02);     
//         transmitChar(0x20);
//         transmitChar(d);       
//         transmitChar(0x0A);
// #endif

        crc = _crc16_update(crc, d);
        eeprom_update_byte((uint8_t*)STORE_ADDR + i, d);        
    }

    eeprom_update_word((uint16_t*)CRC_ADDR_1, crc);    
    eeprom_update_word((uint16_t*)CRC_ADDR_2, crc);    
}    

void setupDevice(void) {
    uint8_t gear = 10; //init индикатор выключен

    stopTimer();
    setFlagDown(button);
    
    if (getFlag(setupNeeded)) gear = 1;
    
    while (1) {
        
        adcRead();
        
        if (getButtonState()) {                                           // Кнопка нажата
            if (!getFlag(button)) {
                setFlagUp(button);
                startTimer(LONG_PRESS_TIME);
            }
            if (getFlag(button) && !getFlag(timer)) {                     // Долгое нажатие
//                gears[gear] = adcData;
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
                
// #ifdef DEBUG
//                 transmitChar(0x01);     
//                 transmitChar(0x20);
//                 transmitChar(adcData);       
//                 transmitChar(0x0A);
// #endif
                stopTimer();
            }
        }
        gears[gear] = adcData;
        symbol = gear;
// !!!        
//         if (_seconds % 750) 
//         { 
//             blink(gear);
//         }
// !!!        
        indicate();
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
#ifdef DEBUG
    TX_DDR      |=  (1 << TX_PIN);
#endif
    
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

#ifndef DEBUG
    // Предделитель 64 для таймера 0 
    TCCR0B |= (1 << CS01) | (1 << CS00);
#endif

    sei(); // Глобально разрешаю прерывания
    
    symbol = 9;
    indicate();
#ifndef DEBUG    
    timerDelayMs(600); 
#else
    _delay_ms(600);
#endif
    
    loadFromEeprom();
    
    if (getFlag(setupNeeded)) {
        symbol = 11;
        indicate();      // Показать E 1 и запустить настройку
#ifndef DEBUG    
        timerDelayMs(500); 
#else
        _delay_ms(500);
#endif
        symbol = 1;
        indicate(); 
#ifndef DEBUG    
        timerDelayMs(500); 
#else
        _delay_ms(500);
#endif
        setFlagUp(setupMode);
        setupDevice();
//        initArray();
        storeToEeprom(); // Сохраняю в EEPROM
        setFlagDown(setupNeeded);
    } else {
        startTimer(LONG_PRESS_TIME);
        setFlagUp(setupMode);
    }
        
    symbol = 0;
    uint8_t a = 0, b = 0;    
    
    //loop();
    while(1) {
        /* Обработка нажатия кнопки */
        if (getFlag(setupMode)) {
            if (getFlag(timer)) {
                if (getButtonState()) {    
                    stopTimer();
                    setupDevice();
//                    initArray();
                    storeToEeprom(); // Сохраняю в EEPROM                 
                }
            } else setFlagDown(setupMode);
        }
        /* Расчёт номера символа */  
        adcRead();
        if (adcData > MIN_V) {
            for (uint8_t i = 0; i < 7; ++i) {
                a = adcData - adcData / 10;
                b = (adcData + adcData / 10 >= 0xff) ? 0xff : adcData + adcData / 10;
                if (gears[i] > a && gears[i] < b) {
                    symbol = i;
                    break;
                } else symbol = 8;
            }
        } else symbol = 7;
        /* Индикация */  
        indicate ();
    }
}


