//
//  int2string.cpp
//  Charger32Slave
//
//  Created by Ruedi Heimlicher on 22.03.2023.
//

#include "int2string.h"
#include "Arduino.h"

void int_to_floatstr(uint16_t inum,char *outbuf,int8_t decimalpoint_pos, uint8_t maxdigit)
{
   int8_t i,j,k;
   char chbuf[8];
   itoa(inum,chbuf,10); // convert integer to string
   /*
   Serial.print(F("int_to_dispstr"));
   Serial.print(F(" decimalpoint_pos: "));
   Serial.print(decimalpoint_pos);
   Serial.print(F(" maxdigit: "));
   Serial.print(maxdigit);
  */
   i=strlen(chbuf);
 //  Serial.print(F(" strlen: "));
 //  Serial.println(i);
   
   if (i>(maxdigit-1)) i=maxdigit-1; //overflow protection
   
   /*
   for(k=0;k<maxdigit;k++)
   {
      //strcat(outbuf,' ');
      outbuf[k] = ' ';
   }
  */
   
  // strcat(outbuf,'\0');
   //strcpy(outbuf,"   0"); //decimalpoint_pos==0
   
   if (maxdigit==2) strcpy(outbuf," 0");
   if (maxdigit==3) strcpy(outbuf,"  0");
   if (maxdigit==4) strcpy(outbuf,"   0");
   if (maxdigit==5) strcpy(outbuf,"    0");
   if (maxdigit==6) strcpy(outbuf,"     0");
  
   /*
    if (decimalpoint_pos==1) strcpy(outbuf,"   0.0");
    if (decimalpoint_pos==2) strcpy(outbuf,"  0.00");
    if (decimalpoint_pos==3) strcpy(outbuf," 0.000");
    if (decimalpoint_pos==4) strcpy(outbuf,"0.0000");
    */
   uint8_t l = maxdigit;
   j=l;
   while(i)
   {
      outbuf[j-1]=chbuf[i-1];
      i--;
      j--;
      if (j==l-decimalpoint_pos)
      {
         outbuf[j-1]='.';
         // jump over the pre-set dot
         j--;
      }
   }
}
