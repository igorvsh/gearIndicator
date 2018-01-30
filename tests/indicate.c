/*
 * A      - 3 - PD3
 * B      - 2 - PD2
 * C      - 7 - PD7
 * D      - 8 - PB0
 * E      - 9 - PB1
 * F      - 4 - PD4
 * G      - 5 - PD5
 * H (DP) - 6 - PD6
 */

#define SEG_A 3 
#define SEG_B 2
#define SEG_C 7
#define SEG_D 0
#define SEG_E 1
#define SEG_F 4
#define SEG_G 5
#define SEG_H 6

void indicate(uint8_t digit,uint8_t dp) {
 uint8_t pb=0,pd=0;

 switch (digit) {
  case 0: 
    pd = (1 << SEG_A) | (1 << SEG_B) | (1 << SEG_C) | (1 << SEG_F); 
    pb = (1 << SEG_D) | (1 << SEG_E);
    break;
  case 1:
    pd = (1 << SEG_B) | (1 << SEG_C);
    break;
  case 2:
    pd = (1 << SEG_A) | (1 << SEG_B) | (1 << SEG_G);
    pb = (1 << SEG_D) | (1 << SEG_E);
    break;
  case 3:
    pd = (1 << SEG_A) | (1 << SEG_B) | (1 << SEG_G) | (1 << SEG_C);
    pb = (1 << SEG_D);
    break;
  case 4:
    pd = (1 << SEG_F) | (1 << SEG_G) | (1 << SEG_B) | (1 << SEG_C);
    break;
  case 5:
    pd = (1 << SEG_A) | (1 << SEG_F) | (1 << SEG_G) | (1 << SEG_C);
    pb = (1 << SEG_D);
    break;
  case 6:
    pd = (1 << SEG_A) | (1 << SEG_F) | (1 << SEG_G) | (1 << SEG_C);
    pb = (1 << SEG_D) | (1 << SEG_E);
    break;
  case 7:
    pd = (1 << SEG_A) | (1 << SEG_B) | (1 << SEG_C);
    break;
  case 8:
    pd = (1 << SEG_A) | (1 << SEG_B) | (1 << SEG_C) | (1 << SEG_F)| (1 << SEG_G);
    pb = (1 << SEG_D) | (1 << SEG_E);
    break;
  case 9:
    pd = (1 << SEG_F) | (1 << SEG_G) | (1 << SEG_A) | (1 << SEG_B) | (1 << SEG_C);
    pb = (1 << SEG_D);
    break;
  case 10: //H
    pd = (1 << SEG_F) | (1 << SEG_G) | (1 << SEG_B) | (1 << SEG_C);
    pb = (1 << SEG_E);
    break;
  case 11: //A
    pd = (1 << SEG_A) | (1 << SEG_B) | (1 << SEG_C) | (1 << SEG_F) | (1 << SEG_G);
    pb = (1 << SEG_E);
    break;
  case 14: //C
    pd = (1 << SEG_A) | (1 << SEG_F);
    pb = (1 << SEG_E) | (1 << SEG_D); 
    break;
  case 13: //d
    pd = (1 << SEG_B) | (1 << SEG_G) | (1 << SEG_C);
    pb = (1 << SEG_E) | (1 << SEG_D);
    break;
  case 12: //b
    pd = (1 << SEG_F) | (1 << SEG_G) | (1 << SEG_C);
    pb = (1 << SEG_E) | (1 << SEG_D);
    break;
  case 15: //c
    pd = (1 << SEG_G);    
    pb = (1 << SEG_E) | (1 << SEG_D);
    break;
  case 16: // rev c
    pd = (1 << SEG_G) | (1 << SEG_C);
    pb = (1 << SEG_D);
    break;
  case 17: // upper squre
    pd = (1 << SEG_F) | (1 << SEG_A) | (1 << SEG_G) | (1 << SEG_B);
    break;
  case 18: // lower squre
    pd = (1 << SEG_G) | (1 << SEG_C);
    pb = (1 << SEG_E) | (1 << SEG_D);
    break;
  default:
    pb = 0;
    pd = 0;
    break;
 }
 if (dp > 1) pd |= (1 << 6);  
 PORTB = (PORTB & 0b11111100) | pb;
 PORTD = (PORTD & 0b00000011) | pd;  
}
