#include <stdio.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdlib.h>

#ifndef _DISPLAY_H
   #define _DISPLAY_H

#define UPDATESCREEN    5 // Bit in status wird gesetzt wenn eine Taste gedrueckt ist, reset wenn update ausgefuehrt

#define SETTINGWAIT     6  // Bit in status wird gesetzt bis Taste 5 3 * gedrueckt ist


#define MINWAIT         3 // Anzahl loops von loopcount1 bis einschalten

#define HOMESCREEN      0
#define SETTINGSCREEN   1
#define KANALSCREEN     2
#define LEVELSCREEN     3

void resetRegister(void);
void sethomescreen(void);
void setsettingscreen(void);
void setlevelscreen(void);
void setcanalscreen(void);

uint8_t update_screen(uint8_t screencode);


#endif //_DISPLAY_H
