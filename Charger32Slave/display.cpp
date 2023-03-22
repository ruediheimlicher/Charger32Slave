#include <stdio.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdlib.h>

#include "Arduino.h"

#include "display.h"
#include "lcd.h"

#include "text.h"
#include "defines.h"

#include "int2string.h"

#define cursortab0 0
#define cursortab1 6
#define cursortab2 12

#define itemtab0  0
#define itemtab1  1
#define itemtab2  7
#define itemtab3  14

extern volatile uint16_t adctimersekunde;
extern volatile uint8_t adctimerminute;
extern volatile uint8_t adctimerstunde;


extern volatile uint8_t levelwert;
extern volatile uint8_t levelb;

extern volatile unsigned char posx;
extern volatile unsigned char posy;

extern  volatile uint8_t       curr_screen;
extern volatile uint8_t       curr_model; // aktuelles modell
extern volatile uint8_t       curr_cursorzeile; // aktuelle zeile des cursors
extern volatile uint8_t       curr_cursorspalte; // aktuelle colonne des cursors

extern volatile uint8_t       last_cursorzeile; // letzte zeile des cursors
extern volatile uint8_t       last_cursorspalte; // letzte colonne des cursors
extern volatile uint16_t      blink_cursorpos;

extern volatile uint16_t      batteriespannung;

extern uint32_t                    batt_M_Spannung;
extern uint32_t                    batt_O_Spannung;
extern uint32_t                    U_Balance_Mitte;
extern uint32_t                    curr_L_Mitte;
extern uint32_t                    MAX_PWM_A;

extern volatile uint16_t  posregister[4][4]; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

extern volatile uint16_t  cursorpos[4][4]; // Aktueller screen: werte fuer page und daraufliegende col fuer cursor (hex). geladen aus progmem

extern volatile uint16_t              updatecounter; // Zaehler fuer Einschalten



char menubuffer[20];
char titelbuffer[20];


void resetRegister(void)
{
   uint8_t i=0,k=0;
   for(i=0;i<4;i++)
   {
      for (k=0;k<4;k++)
      {
         posregister[i][k]=0xFFFF;
      }
   }
}

void sethomescreen(void)
{
   /*
    // Homescreen
    const char titel0[]  = "Charger Home";
    const char titel1[]  = "A:";
    const char titel2[]  = "B:";
    const char titel3[]  = "PW";
    const char titel4[]  = "PW_M";
    const char titel5[]  = "B";
    const char titel6[]  = "O";
    const char titel7[]  = "M";
    
    
    */
   // Laufzeit
   Serial.printf("sethomescreen start curr_screen: %d\n",curr_screen);
   resetRegister();
   blink_cursorpos=0xFFFF;
   posregister[0][0] = itemtab0 | (0 << 10);// Titel
   posregister[0][1] = itemtab1 | (0 << 10);// Titel
   posregister[0][2] = itemtab2 | (0 << 10);// 
   posregister[0][3] = itemtab3 | (0 << 10);// Zeit
   
   
   posregister[1][0] = itemtab0 | (1 << 10); // Text Motorzeit
   posregister[1][1] = itemtab1 | (1 << 10); // Anzeige Motorzeit
   posregister[1][2] = itemtab2 | (1 << 10); // Anzeige Motorzeit
   posregister[1][3] = itemtab3 | (1 << 10); // Anzeige Motorzeit

   posregister[2][0] = itemtab0 | (2 << 10); // Text Motorzeit
   posregister[2][1] = itemtab1 | (2 << 10); // Anzeige Motorzeit
   posregister[2][2] = itemtab2 | (2 << 10); // Anzeige Motorzeit
   posregister[2][3] = itemtab3 | (2 << 10); // Anzeige Motorzeit
   
   posx = (posregister[0][0] & 0x00FF);
   posy = ((posregister[0][0] & 0xFF00) >> 10);
   
   Serial.printf("sethomescreen posregister[0][0]: %d posx: %d posy: %d\n",posregister[0][0],posx, posy);
   //lcd_gotoxy(posx,posy);
   lcd_puts(TitelTable[0]);
/*
   posx = (posregister[1][0] & 0x00FF);
   posy = ((posregister[1][0] & 0xFF00) >> 10);
   lcd_gotoxy(posx,posy);
   lcd_puts(TitelTable[1]);
   char float_StringM[8]; 
   int_to_floatstr(batt_M_Spannung,float_StringM,2,4);
   lcd_puts(float_StringM);
   
   //lcd_putint12(batt_O_Spannung);
  */ 
   
} // sethomescreen




uint8_t update_screen(uint8_t screencode)
{
   
   uint8_t fehler=0;
   uint16_t cursorposition = cursorpos[curr_cursorzeile][curr_cursorspalte];
   fehler=1;
   //curr_screen = 0;
   //Serial.printf("****************  update_screen: %d screencode: %d\n",curr_screen, screencode);
   
   switch (curr_screen)
   {
      case HOMESCREEN: // homescreen
      {
#pragma mark update HOMESCREEN
         //Serial.printf("****************  update_screen: HOMESCREEN\n");
         fehler=2;
         posx = (posregister[0][3] & 0x00FF)-1;
         posy = ((posregister[0][3] & 0xFF00) >> 10);
         lcd_gotoxy(posx,posy);
         lcd_putint1(adctimerstunde);
         lcd_putc(':');
         lcd_putint2(adctimerminute);
         lcd_putc(':');
         lcd_putint2(adctimersekunde);
          
         posx = (posregister[1][0] & 0x00FF);
         posy = ((posregister[1][0] & 0xFF00) >> 10);
         lcd_gotoxy(posx,posy);
         
         lcd_puts(TitelTable[1]);
         char float_StringM[8]; 
         int_to_floatstr(batt_O_Spannung,float_StringM,2,4);
         lcd_puts(float_StringM);

         posx = (posregister[1][2] & 0x00FF);
         posy = ((posregister[1][2] & 0xFF00) >> 10);
         lcd_gotoxy(posx,posy);
         
         lcd_puts(TitelTable[2]);
         //char float_StringM[8]; 
         int_to_floatstr(batt_M_Spannung,float_StringM,2,4);
         lcd_puts(float_StringM);

         posx = (posregister[1][3] & 0x00FF);
         posy = ((posregister[1][3] & 0xFF00) >> 10);
         lcd_gotoxy(posx,posy);
         
         lcd_puts(TitelTable[5]);
         //char float_StringM[8]; 
         int_to_floatstr(U_Balance_Mitte,float_StringM,2,4);
         lcd_puts(float_StringM);

         // Zeile 2
         posx = (posregister[2][0] & 0x00FF);
         posy = ((posregister[2][0] & 0xFF00) >> 10);
         lcd_gotoxy(posx,posy);
         
         lcd_puts(TitelTable[6]);
         int_to_floatstr(curr_L_Mitte,float_StringM,2,4);
         lcd_puts(float_StringM);

         posx = (posregister[2][2] & 0x00FF);
         posy = ((posregister[2][2] & 0xFF00) >> 10);
         lcd_gotoxy(posx,posy);
         
         //lcd_puts(TitelTable[6]);
         lcd_putc(94);
         lcd_putc(':');
         int_to_floatstr(MAX_PWM_A/10,float_StringM,2,4);
         lcd_puts(float_StringM);
         lcd_putc(' ');
         lcd_putint12(MAX_PWM_A);


         
      }break;
         
         
   }// switch curr screen
   
   
} // update screen
