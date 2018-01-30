#include <EEPROM.h>
#include <util/crc16.h>

#define _seconds millis()/1000

const byte buttonPin = 11;
const byte analogPin = 0;
const byte minV = 5;
const byte pressTime = 2;
const byte blinkDelay = 250/2;
const byte crcAddr1 = 0x0E;
const byte crcAddr2 = 0x79;
const byte storeAddr = 0x00;

//#define DEBUG

//byte gears[7] = {255, 75, 103, 133, 164, 196, 0};
byte gears[7] = {0, 0, 0, 0, 0, 0, 0};

byte adcData = 0;
byte digit = 7;
byte button = 0;
byte Flag = 1; /* 0 - основной, 1 - первые 3 секунды, 2 - требуется перенастройка */



void indicate(byte num = 7, byte dp = 0) {
   /*
    A      - 3 - PD3
    B      - 2 - PD2
    C      - 7 - PD7
    D      - 8 - PB0
    E      - 9 - PB1
    F      - 4 - PD4
    G      - 5 - PD5
    H (DP) - 6 - PD6
  */
  /*                                    2  3  4  5  6  7  8  9    */
  /*                                    B  A  F  G  H  C  D  E    */
 const byte dig2pins[12][8] PROGMEM = {{1, 0, 1, 1, 0, 1, 0, 1},  //H
                                       {1, 0, 0, 0, 0, 1, 0, 0},  //1
                                       {1, 1, 0, 1, 0, 0, 1, 1},  //2
                                       {1, 1, 0, 1, 0, 1, 1, 0},  //3
                                       {1, 0, 1, 1, 0, 1, 0, 0},  //4
                                       {0, 1, 1, 1, 0, 1, 1, 0},  //5
                                       {0, 1, 1, 1, 0, 1, 1, 1},  //6
                                       {0, 0, 0, 1, 0, 0, 0, 0},  //-
                                       {0, 1, 0, 1, 0, 0, 1, 0},  //Ξ
                                       {1, 1, 1, 1, 0, 1, 1, 1},  //8
                                       {0, 0, 0, 0, 0, 0, 0, 0},  //off
                                       {0, 1, 1, 1, 0, 0, 1, 1}}; //E
                                       
  for (byte i = 2; i < 10; i++ ) {
    digitalWrite(i, dig2pins[num][i - 2]);
  }
  if (1 == dp) digitalWrite(6, HIGH);
}

inline byte adcRead() {
  uint16_t a = 0;
  a = analogRead(analogPin) >> 2;
  delay(10);
  a += analogRead(analogPin) >> 2;
  delay(10);
  a += analogRead(analogPin) >> 2;
  return a/3;
}

void loadFromEeprom() {
  uint16_t crc1 = 0;
  uint16_t crc2 = 0;
  uint16_t crc  = 0;
  
  crc1 = EEPROM[crcAddr1] << 8 | EEPROM[crcAddr1+1];
  crc2 = EEPROM[crcAddr2] << 8 | EEPROM[crcAddr2+1];

  Flag = 1;

  if (crc1 == crc2) { // crc совпадают
    if (0 == crc1) Flag = 2; //crc не была сохранена
    for (byte i = 0; i < 7; i++) {
      gears[i] = EEPROM[storeAddr+i];
      crc = _crc_ccitt_update(crc, gears[i]);
    }
    if (crc != crc1) {
      for (byte i = 0; i < 7; i++)
        gears[i] = 0;
      Flag = 2;
    }
  } else Flag = 2; //crc отличаются     
}

void storeToEeprom() {
  uint16_t crc = 0;
  for (byte i = 0; i < 7; i++) {
    crc = _crc_ccitt_update(crc, gears[i]);
    EEPROM[storeAddr+i] = gears[i];
  }
  EEPROM[crcAddr1] = crc >> 8;
  EEPROM[crcAddr1+1] = crc & 0xFF;
  EEPROM[crcAddr2] = crc >> 8;
  EEPROM[crcAddr2+1] = crc & 0xFF;  
}

void setup() {
  delay(blinkDelay);  
  pinMode(11, INPUT);
  for (byte i = 2; i < 10; i++) 
    pinMode(i, OUTPUT);
  
  indicate(9,1);
  delay(500);

  loadFromEeprom();
  
  if (2 == Flag) {      //Ошибка при чтении eeprom
    indicate(11,1);     //Показать E 1 и запустить настройку
    delay(400);
    indicate(1,1); 
    delay(400);
    setupDevice();
    Flag = 1;
  }
  
#ifdef DEBUG  
  Serial.begin(9600);
#endif
}

void blink(byte num,boolean dp) {
  indicate(num,dp);
  delay(blinkDelay);  
  indicate(10,0);
  delay(blinkDelay);  
}

void setupDevice() {
  byte gear = 10; //init индикатор выключен
  boolean pressed = true;
  byte pressedSeconds = 0;
  byte v = 0;
  while (true) {

    v = adcRead();
    
    if (digitalRead(buttonPin) == HIGH) {        //Кнопка нажата
      
      if (pressed == false) {                    
            pressed = true;
            pressedSeconds = _seconds;
      }
      if (pressed == true && _seconds - pressedSeconds > 1) { //  Долгое нажатие
        gears[gear] = v;
        storeToEeprom();
        return;
      }     
    } else {                                     //Кнопка отжата
      if (pressed == true) { 
        if (10 == gear) { gear = 1; }        // init -> 1
          else if (1 == gear) { gear = 0; }  // 1 -> H     
          else if (0 == gear) { gear = 2; }  // H -> 2
          else if (6 == gear) { gear = 1; }  // 6 -> 1
          else { gear++; };                  // 2 -> 3 -> 4 -> 5 -> 6 
          pressed = false;
          pressedSeconds = 0;
      }
      if (gear != 10) gears[gear] = v;
    }    
    if (_seconds % 750) { 
      blink(gear, 1);
    }
    indicate(gear, 1);
    delay(10); 
  }
}

void loop() {
/* Обработка нажатия кнопки */
  if (1 == Flag) {
    if (digitalRead(buttonPin) == HIGH) {
      setupDevice();
      Flag = 0;
    }
    if (_seconds > pressTime) {
      Flag = 0;
    }
  }
/* Расчёт номера символа */  
  adcData = adcRead();
  if (adcData > minV) {
    for (byte i = 0; i < 7; ++i) {
      if ( gears[i] > adcData-adcData/ 10 && gears[i] < adcData+adcData/10 ) {
        digit = i;
        break;
      }
      else digit = 8;
    }
  }
  else digit = 7;
/* Индикация */  
  indicate (digit, Flag);
  delay(10);
}
