#include <stdint.h>
#include <stdio.h>



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

uint8_t Flag = 0;



int main() {
    setFlagUp(timer);
    printf("%x\n",Flag);
    Flag ^= timer;
    printf("%x\n",Flag);
    Flag ^= timer;
    printf("%x\n",Flag);

    Flag ^= timer;
    printf("%x\n",Flag);

    Flag ^= timer;
    printf("%x\n",Flag);

    return 0;
}
