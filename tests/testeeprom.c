#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
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
#define MIN_V                   5     // Минимальное заначение измеренного напряжения. Ниже этого будет отображаться -
#define CRC_ADDR_1              0x0E  // Адрес EEPROM первой CRC
#define CRC_ADDR_2              0x78  // Адрес EEPROM второй CRC
#define STORE_ADDR              0x00  // Адрес EEPROM первого элемента массива данных

#define STORE_TMP               0x10  // Адрес EEPROM первого элемента массива данных

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


volatile const uint8_t symbolsGen[12] = {
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

volatile uint8_t gears[7] = {1, 2, 3, 4, 5, 6, 7};
volatile uint8_t symbol = 7;   // Номер символа в таблице знакогенератора. Он будет отображаться при вызовае indicate()
volatile uint8_t Flag = 0;     // Флаги состояния работы

__inline__ static void indicate(void) {
    LED_SEG_PORT &= symbolsGen[10]; // Выключение сегментов
    LED_SEG_PORT |= symbolsGen[symbol]; 
    LED_DP_PORT = (getFlag(setupMode)) ? LED_DP_PORT | (1 << LED_SEG_DP) : LED_DP_PORT & ~(1 << LED_SEG_DP); // Управление запятой
}

__inline__ static void loadFromEeprom(void) {
    uint16_t crc1 = 0;
    uint16_t crc2 = 0;
    uint16_t crc  = 0;
  
    crc1 = eeprom_read_word((uint16_t*)CRC_ADDR_1);
    crc2 = eeprom_read_word((uint16_t*)CRC_ADDR_2);
    eeprom_read_block((void*)&gears, (void*)STORE_ADDR, sizeof gears);
    
    for (uint8_t i = 0; i < 7; i++) 
        crc = _crc_ccitt_update(crc, gears[i]);
  
    if (crc1 != crc2 || 0 == crc1 || 0xffff == crc1 || crc != crc1)  
        setFlagUp(setupNeeded);   //crc отличаются от расчётной или друг друга или = 0      
}
    
__inline__ static void storeToEeprom(void) {
    uint16_t crc = 0;

    for (uint8_t i = 0; i < 7; i++) 
        crc = _crc_ccitt_update(crc, gears[i]);

    eeprom_update_block((void*)&gears, (void*)STORE_ADDR, sizeof gears);
// test
    eeprom_update_block((void*)&gears, (void*)STORE_TMP, sizeof gears);

    eeprom_update_word((uint16_t*)CRC_ADDR_1, crc);    
    eeprom_update_word((uint16_t*)CRC_ADDR_2, crc);    
}    

__inline__ static uint8_t getButtonState(void) {   
    return BUTTON_PORT & (1 << BUTTON_PIN);
}


int main(void)
{
    // Режим "выход" для выводов индикатора
    LED_SEG_DDR |=  (1 << LED_SEG_A) | (1 << LED_SEG_B) | (1 << LED_SEG_C) | (1 << LED_SEG_D) | (1 << LED_SEG_E) | (1 << LED_SEG_F) | (1 << LED_SEG_G);
    LED_DP_DDR  |=  (1 << LED_SEG_DP);
    
    // Режим "вход" для выводов кнопки и датчика
    BUTTON_DDR  &= ~(1 << BUTTON_PIN);
    SENSOR_DDR  &= ~(1 << SENSOR_PIN);
    
    sei();
    
    loadFromEeprom();
    
    setFlagUp(setupMode); 
    indicate();
    
    if (getFlag(setupNeeded)) {
        symbol = 0;

        for (uint8_t i = 0; i < 7; ++i)
            gears[i] = i;
    } else {
        symbol = gears[0];
        for (uint8_t i = 0; i < 7; ++i)
            gears[i]++;
    }
    
    indicate();
    setFlagDown(setupMode);

    storeToEeprom();
  
    setFlagUp(setupMode);
    indicate();

}

