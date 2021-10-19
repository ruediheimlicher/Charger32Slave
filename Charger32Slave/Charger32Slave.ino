///
/// @mainpage	Charger32Slave
///
/// @details	LiIon Carger
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @date		15.08.2021 09:40
/// @version	1.1
///
/// @copyright	(c) Ruedi Heimlicher, 2021
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
///


#include "Arduino.h"

#include "gpio_MCP23S17.h"
#include <SPI.h>
// Include application, user and local libraries
#include "lcd.h"
//#include "settings.h"
#include "defines.h"
//#include "adc.h"
#include <EEPROM.h>

#include <ADC.h>
#include <ADC_util.h>

//#include "analog.h"

#include "SPI.h"

// define constants
//#define USB_DATENBREITE 64

#define TEST 0

// Set parameters

ADC *adc = new ADC(); // adc object

// Include application, user and local libraries
// !!! Help http://bit.ly/2CL22Qp




// Define variables and constants
static volatile uint8_t recvbuffer[64]={};
static volatile uint8_t sendbuffer[64]={};

volatile uint8_t           adcstatus=0x00;
volatile uint16_t          adctimercounter = 0;

volatile uint8_t           usbstatus=0x00;

volatile uint8_t           loadstatus=0x00;

volatile uint8_t           senderfolg = 0;
volatile uint8_t           status=0;

volatile uint16_t           PWM_A=0;
volatile uint16_t           PWM_B=0;

uint32_t batt_M_Spannung  = 0;
uint32_t batt_O_Spannung  = 0;

volatile uint16_t           batt_MIN_raw=0;
volatile uint32_t           batt_MAX_raw=0;
volatile uint16_t           batt_OFF_raw=0;

volatile uint16_t       batt_M = 0;
uint16_t[16]   batt_M_Array = {}; // Ringbuffer fuer Mittelwertbildung
uint8_t                 batt_M_pos = 0; // position im ringbuffer
volatile uint16_t       batt_O = 0;
uint16_t[16]   batt_O_Array = {};
uint8_t                 batt_O_pos = 0;
volatile uint16_t       curr_U = 0;
volatile uint16_t       curr_B = 0;

volatile uint16_t       curr_A = 0;

volatile uint16_t       temp_SOURCE = 0;
volatile uint16_t       temp_BATT = 0;

volatile uint16_t       U_Balance = 0;

volatile uint16_t  sekundentimercounter = 0;
volatile uint16_t adctimersekunde = 0;
int8_t r;

uint16_t count=0;

uint16_t loopcount0=0;
uint8_t loopcount1=0;

volatile uint8_t  usbsendcounter = 0;
volatile uint16_t  usbrecvcounter = 0;
volatile uint8_t   packetcount = 0;

volatile uint8_t taskcode;


// Charger
volatile uint16_t                    messungcounter=0; // Anzahl messungen fortlaufend
volatile uint16_t                    strommessungcounter=0;
volatile uint16_t                    blockcounter = 0; // Block, in den gesichert werden soll, mit einem Offset von 1 (Block 0 ist header der SD).

volatile uint16_t                    writedatacounter=0; // Anzahl mmc-writes fortlaufend
volatile uint16_t                    writemmcstartcounter=0; // Anzahl mmc-writes fortlaufend

volatile  uint16_t                  mmcwritecounter=0; // Zaehler fuer Messungen auf SD
static volatile uint8_t             sd_status=0x00; // recvbuffer[1]
volatile uint16_t                   devicecount=0;
volatile uint8_t                    loggerstatus = 0;
volatile uint8_t                    hoststatus = 0;
// copy
volatile uint8_t                    in_taskcounter=0;
volatile uint8_t                    out_taskcounter=0;

volatile uint8_t                    mmcbuffer[SD_DATA_SIZE] = {};


volatile uint16_t                   linecounter=0;

volatile uint16_t                   startblock = 0;


volatile uint8_t                    blockanzahl = 0;

volatile uint8_t                    downloadblocknummer = 0;
volatile uint16_t                    downloaddatacounter = 0;

// counter fuer Mess-Intervall
volatile uint16_t                    intervallcounter=0;

// mess-Intervall
volatile uint16_t                    intervall=1; // defaultwert, 1s

volatile uint8_t  kanalstatusarray[8] = {0};
volatile int devicenummer =0;

// eeprom
uint16_t eeprom_intervall  = 1; // default intervall
uint16_t eeprom_startblock  = 1; // default startblock
uint16_t eeprom_blockcount  = 0; // Anzahl geschriebene Blocks
uint16_t eeprom_messungcount  = 0; // Anzahl geschriebene Messungen
uint8_t eeprom_kanalbyte0 = 0;
uint8_t eeprom_kanalbyte1 = 0;
uint8_t eeprom_kanalbyte2 = 0;
uint8_t eeprom_kanalbyte3 = 0;
uint8_t readerr=0;
uint8_t writeerr=0;

volatile uint16_t saveSDposition = 0;
volatile uint16_t blockoffset = 1;
volatile uint16_t blockdatacounter = 0; //  Anzahl gespeicherter Messungen auf aktuellem Block. Fuer rescue verwendet

volatile uint16_t teensydatacounter = 0; //  Anzahl gespeicherter Messungen auf aktuellem Block. Fuer rescue verwendet

//declare an eeprom array
uint8_t   eeprom_kanalstatus_array[4];

volatile uint16_t          startminute = 0;
IntervalTimer              adcTimer;
//volatile uint16_t          timerintervall = TIMERINTERVALL;
volatile uint16_t         adctimerintervall = 1000;
IntervalTimer              stromTimer;


// constants

// bits von hoststatus
#define LADUNG_RUN         7
#define MESSUNG_RUN        6
#define MESSUNG_OK         5
#define DOWNLOAD_OK        4
#define USB_READ_OK        3
#define TEENSY_ADC_OK      2
#define TEENSY_MMC_OK      1
#define MANUELL_OK         0

        



// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa


// Utilities


// Functions

void slaveinit(void)
{
   pinMode(LOOPLED, OUTPUT);
   
   pinMode(OSZI_PULS_A, OUTPUT);
   pinMode(OSZI_PULS_B, OUTPUT);
   
   pinMode(ADC_M, INPUT);
   pinMode(ADC_O, INPUT);

   pinMode(ADC_SHUNT, INPUT);
   pinMode(ADC_AAA, INPUT);

   pinMode(ADC_TEMP_SOURCE, INPUT);
   pinMode(ADC_TEMP_BATT, INPUT);
   
   pinMode(ADC_BALANCE, INPUT);
   
   //LCD
   pinMode(LCD_RSDS_PIN, OUTPUT);
   pinMode(LCD_ENABLE_PIN, OUTPUT);
   pinMode(LCD_CLOCK_PIN, OUTPUT);

  // pinMode(LADESTROM_PWM_A, OUTPUT);
  // pinMode(LADESTROM_PWM_B, OUTPUT);
   
   pinMode(LOAD_START, INPUT); // Laden START manuell
   pinMode(LOAD_STOP, INPUT); // Laden STOP manuell
   
//   pinMode(25, OUTPUT);// OC1A
//   pinMode(26, OUTPUT);// OC1A
   
 //  pinMode(24, OUTPUT);// OC2A
   
   // SPI
   pinMode(10, OUTPUT);// CS
   pinMode(11, OUTPUT);// MOSI
   pinMode(12, INPUT);// MISO
   pinMode(13, OUTPUT);// SCK
}

void ADC_init(void) 
{
   
   
   adc->adc0->setAveraging(4); // set number of averages 
   adc->adc0->setResolution(10); // set bits of resolution
   adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
   adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
   adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
  // adc->adc0->enableInterrupts(ADC_0);
   
   
//   delay(100);
   
   
}


void clear_sendbuffer(void)
{
   for (int i=0;i<USB_PACKETSIZE;i++)
   {
      sendbuffer[i] = 0;
   }
}

// debounce
volatile uint8_t tipptastenstatus = 0;
#define MAX_CHECKS 8
volatile uint8_t last_debounced_state = 0;
volatile uint8_t debounced_state = 0;
volatile uint8_t state[MAX_CHECKS] = {0};

volatile uint8_t debounceindex = 0;
volatile uint8_t SPItastenstatus = 0;
volatile uint8_t SPIcheck=0;

typedef struct
{
   uint8_t pin = 0xFF;
   uint16_t tasten_history;
   uint8_t pressed;
   long lastDebounceTime;
}tastenstatus;

//long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 20;    // the debounce time; increase if the output flickers

tastenstatus tastenstatusarray[8] = {}; 

uint8_t tastenbitstatus = 0; // bits fuer tasten

volatile uint8_t tastencode = 0;


uint8_t readTaste(uint8_t taste)
{
   return (digitalReadFast(taste) == 0);
}

void update_button(uint8_t taste, uint16_t *button_history)
{
   *button_history = *button_history << 1;
   *button_history |= readTaste(taste);
}

uint8_t is_button_pressed(uint8_t button_history)
{
   return (button_history == 0b01111111);
}

uint8_t is_button_released(uint8_t button_history){
   return (button_history == 0b10000000);
}

uint8_t test_for_press_only(uint8_t pin)
{   
   static uint16_t button_history = 0;
   uint8_t pressed = 0;    
   
   button_history = button_history << 1;
   button_history |= readTaste(pin);
   if ((button_history & 0b11000111) == 0b00000111)
   { 
      pressed = 1;
      button_history = 0b11111111;
   }
   return pressed;
}



uint8_t checktasten(void)
{
   uint8_t count = 0; // Anzahl aktivierter Tasten
   uint8_t i=0;
   uint8_t tastencode = 0;
   while (i<8)
   {
      uint8_t pressed = 0;
      if (tastenstatusarray[i].pin < 0xFF)
      {
         count++;
         tastenstatusarray[i].tasten_history = tastenstatusarray[i].tasten_history << 1; // shift left
 
         tastenstatusarray[i].tasten_history |= readTaste(tastenstatusarray[i].pin); // status, pin-nummer von $element i
         if ((tastenstatusarray[i].tasten_history & 0b11000111) == 0b00000111)
         {
            pressed = 1;
            tipptastenstatus |= (1<<i);
            tastenbitstatus |= (1<<i);
            tastenstatusarray[i].tasten_history = 0b11111111;
            tastenstatusarray[i].pressed = pressed;
         }
         
      }// i < 0xFF
      
      i++;
   }
   // tastenstatusarray
   //return tastencode;
   return tipptastenstatus ;
}



void debounce_ISR(void)
{
   //digitalWriteFast(OSZIA,LOW);
   uint8_t old_tipptastenstatus = tipptastenstatus;
   tipptastenstatus = checktasten();
   //SPIcheck  = checkSPItasten(); 
   //digitalWriteFast(OSZIA,HIGH);
   
}


//MARK: checkSPItasten
uint8_t checkSPItasten() // MCP23S17 abrufen // Takt ca. 300us
{
   uint8_t count = 0; // Anzahl aktivierter Tasten
   uint8_t i=0;
   //uint8_t tastencode = 0;
   uint8_t check=0;
   //digitalWriteFast(OSZIB,LOW); // 
   
   //tastencode = 0xFF - mcp0.gpioReadPortB(); // 8 us active taste ist LO > invertieren
     
   //digitalWriteFast(OSZIB,HIGH);
   //digitalWriteFast(OSZIB,LOW);
   while (i<8) // 1us
   {
      uint8_t pressed = 0;
      if (tastenstatusarray[i].pin < 0xFF)
      {
         count++;
         tastenstatusarray[i].tasten_history = tastenstatusarray[i].tasten_history << 1;
      
         uint8_t pinnummer = tastenstatusarray[i].pin;
         tastenstatusarray[i].tasten_history |= ((tastencode & (1<<pinnummer)) > 0);
         if ((tastenstatusarray[i].tasten_history & 0b11000111) == 0b00000111)
         {
            pressed = 1;
            SPItastenstatus |= (1<<i);
            tastenbitstatus |= (1<<i);
            tastenstatusarray[i].tasten_history = 0b11111111;
            tastenstatusarray[i].pressed = pressed;
         }
      }// i < 0xFF
      i++;
   }
   //controllooperrcounterD = count;
   //digitalWriteFast(OSZIB,HIGH); // 9us
   return SPItastenstatus ;
}

void prellcheck(void) // 30us debounce mit ganssle-funktion
{
   //digitalWriteFast(OSZIB,LOW);
      
    
   // MCP lesen
 /*  
   regB = bereichpos;
//   digitalWriteFast(OSZIB,LOW);
   uint8_t regBB = (regB & 0x07)<< 5;
   mcp0.gpioWritePortA((regA | regBB)); // output
  //digitalWriteFast(OSZIB,HIGH);
   tastencode = 0xFF-mcp0.gpioReadPortB(); // input 8us . PORTB invertieren, 1 ist aktiv
    //digitalWriteFast(OSZIB,HIGH);
      
   //digitalWriteFast(OSZIB,LOW);
 */  
   //debounce_switch(tastencode); // 6us
   debounced_state = checkSPItasten();
   
   
   //controllooperrcounterB = debounced_state;
      
   // end test
   //   controllooperrcounterC = ausgabestatus;
  // digitalWriteFast(OSZIB,HIGH);

}

// end debounce
void OSZIATOGG()
{
   digitalWriteFast(OSZI_PULS_A, !digitalReadFast(OSZI_PULS_A));
}

void OSZIBTOGG()
{
   digitalWriteFast(OSZI_PULS_B, !digitalReadFast(OSZI_PULS_B));
}

void init_analog(void) 
{
    analogWriteFrequency(4, 375000);
   
   adc->adc0->setAveraging(16); // set number of averages 
   adc->adc0->setResolution(12); // set bits of resolution
   adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
   adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
   adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
   
   
   adc->adc0->enableInterrupts(ADC_0);
    
    
   adc->startSynchronizedContinuous(ADC_M, ADC_O); // 
   delay(100);
   
   
 }




void adctimerfunction()
{
   OSZIBTOGG();
   sekundentimercounter++;
   if (sekundentimercounter == 50)
   {
   }
   if (sekundentimercounter >= 1000)
   {
      intervallcounter++;
      if(intervallcounter >= intervall)
      {
         intervallcounter = 0;
         hoststatus |= (1<<MESSUNG_OK); // Messung ausloesen
         adcstatus |= (1<<ADC_U_BIT);
         adcstatus |= (1<<ADC_I_BIT);
        
      }
      sekundentimercounter=0;
      adctimersekunde++;
      //digitalWriteFast(OSZI_PULS_A, !digitalReadFast(OSZI_PULS_A));
      OSZIATOGG();
   }
}

// Add setup code
void setup()
{
   //Serial.begin(9600);
   Serial.println(F("RawHID H0"));
   slaveinit();
   /* initialize the LCD */
   _delay_ms(100);
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   _delay_ms(100);
   lcd_puts("Guten Tag Charger");
   _delay_ms(500);
   adcTimer.begin(adctimerfunction,adctimerintervall); // 1ms
   lcd_clr_line(0);
   ADC_init();
//   analogWriteResolution(10);
   pinMode(A14,OUTPUT);
   pinMode(13,OUTPUT);
   analogWriteResolution(10);

//   SPI.begin();
   
//   batt_MAX_raw = (U_MAX <<4) / (TEENSYVREF * 100)/ADC_U_FAKTOR;
   volatile uint16_t           batt_MAX_raw=0;
   volatile uint16_t           batt_OFF_raw=0;
   
   // Tasten zuteilen
   tastenstatusarray[0].pin = LOAD_START; // Taste Laden Start
   tastenstatusarray[1].pin = LOAD_STOP; // Taste Laden Start
   pinMode(BLINKLED,OUTPUT);

}

void f_to_a(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
   Serial.print(F("f_to_a ipart: "));
   Serial.print(ipart);
    // Extract floating part
    float fpart = n - (float)ipart;
   
   Serial.print(F(" fpart: "));
   Serial.print(fpart);

    // convert integer part to string
    int i = itoa(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        itoa((int)fpart, res + i + 1, afterpoint);
    }
}
void int_to_dispstr_tux(uint16_t inum,char *outbuf,int8_t decimalpoint_pos)
{
   // Convert an integer which is representing a float into a string.
   // Our display is always 4 digits long (including one
   // decimal point position). decimalpoint_pos defines
   // after how many positions from the right we set the decimal point.
   // The resulting string is fixed width and padded with leading space.
   //
   // decimalpoint_pos=2 sets the decimal point after 2 pos from the right:
   // e.g 74 becomes "0.74"
   // The integer should not be larger than 999.
   // The integer must be a positive number.
   // decimalpoint_pos can be 0, 1 or 2
   
   int8_t i,j;
   char chbuf[8];
   itoa(inum,chbuf,10); // convert integer to string
 //  Serial.print(F("int_to_dispstr"));
   // Serial.print(F(" chbuf: "));
   //Serial.println(chbuf);
   i=strlen(chbuf);
 //  Serial.print(F(" strlen: "));
 //  Serial.println(i);
   
   if (i>3) i=3; //overflow protection
   strcpy(outbuf,"   0"); //decimalpoint_pos==0
   if (decimalpoint_pos==1) strcpy(outbuf," 0.0");
   if (decimalpoint_pos==2) strcpy(outbuf,"0.00");
   uint8_t l = 4;
   j=l;
   while(i){
      outbuf[j-1]=chbuf[i-1];
      i--;
      j--;
      if (j==l-decimalpoint_pos)
      {
         // jump over the pre-set dot
         j--;
      }
   }
}

void int_to_dispstr(uint16_t inum,char *outbuf,int8_t decimalpoint_pos)
{
        int8_t i,j;
        char chbuf[10];
        itoa(inum,chbuf,10); // convert integer to string
  // Serial.print(F("int_to_dispstr"));
  // Serial.print(F(" chbuf: "));
   //Serial.println(chbuf);
        i=strlen(chbuf);
//   Serial.print(F(" strlen: "));
//   Serial.println(i);
  
        if (i>4) i=4; //overflow protection
   
        strcpy(outbuf,"    0"); //decimalpoint_pos==0
        if (decimalpoint_pos==1) strcpy(outbuf,"  0.0");
        if (decimalpoint_pos==2) strcpy(outbuf," 0.00");
         if (decimalpoint_pos==3) strcpy(outbuf,"0.000");
         uint8_t l = 5;
        j=l;
        while(i){
                outbuf[j-1]=chbuf[i-1];
                i--;
                j--;
                if (j==l-decimalpoint_pos)
                {
                        // jump over the pre-set dot
                        j--;
                }
        }
}

void load_start(uint8_t manuell)
{
   hoststatus |= (1<<SEND_OK); 
 
   hoststatus |= (1<<MESSUNG_RUN); 
   hoststatus |= (1<<LADUNG_RUN); 
//            clear_sendbuffer();
   cli();
   Serial.print(F("MESSUNG_START"));
   //uint8_t ee = eeprom_read_word(&eeprom_intervall);
   //lcd_gotoxy(12,3);
   //lcd_putint(ee);
   hoststatus |= (1<<USB_READ_OK);
   
   adcstatus |= (1<<FIRSTRUN);
   
   messungcounter = 0;
   blockcounter = 0;
   sendbuffer[0] = MESSUNG_START;
   blockdatacounter = 0;            // Zaehler fuer Data auf dem aktuellen Block
   writedatacounter = 0;
   linecounter=0;
   intervallcounter=0;
   
   usbstatus = taskcode;
   
   sd_status = recvbuffer[1]; 
    
   mmcwritecounter = 0; // Zaehler fuer Messungen auf MMC
   
   saveSDposition = 0; // Start der Messung immer am Anfang des Blocks
   
   // intervall
   intervall = recvbuffer[TAKT_LO_BYTE] | (recvbuffer[TAKT_HI_BYTE]<<8);
   Serial.print(F("intervall "));
   Serial.print(intervall);
    
   //               abschnittnummer = recvbuffer[ABSCHNITT_BYTE]; // Abschnitt,
   
   blockcounter = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8);
   Serial.print(F(" blockcounter "));
   Serial.println(blockcounter);

   startminute  = recvbuffer[STARTMINUTELO_BYTE] | (recvbuffer[STARTMINUTEHI_BYTE]<<8); // in SD-Header einsetzen
   
   // Kanalstatus lesen. Wird beim Start der Messungen uebergeben
   
   // nicht verwendet
    uint8_t kan = 0;
//     lcd_gotoxy(8,2);
   for(kan = 0;kan < 4;kan++)
   {
//        lcd_putint2(kanalstatusarray[kan]);
   }
//       _delay_ms(100);          
   for(kan = 0;kan < 4;kan++)
   {
      kanalstatusarray[kan] = recvbuffer[KANAL_BYTE + kan];
   }
   
//     lcd_gotoxy(8,2);
   for(kan = 0;kan < 4;kan++)
   {
//       lcd_putint2(kanalstatusarray[kan]);
   
   }
   PWM_A = STROM_LO_RAW;
   analogWrite(A14,PWM_A);
   
   sendbuffer[1] = sd_status; // rueckmeldung 
   
   sendbuffer[USB_PACKETSIZE-1] = 76;
   saveSDposition = 0; // erste Messung sind header
   sei();
   
   // control
   /*
    #define STROM_HI  1000 // mA
    #define STROM_LO  50   // mA
    #define STROM_REP  100 // Reparaturstrom bei Unterspannung

    */
    // _delay_ms(1000);
   //uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
   
}
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



elapsedMillis sinceRecv;
elapsedMillis sincePrellcheck;
elapsedMillis sinceCheckTaste;

// Add loop code
void loop()
{
   /*
   if (sincePrellcheck > 5)
   {
      sincePrellcheck = 0;
      debounce_ISR();
   }
   if (sinceCheckTaste > 100) // Tastenstatus abfragen
   {
   // Taste[0]
   if (tastenstatusarray[LOAD_START].pressed) // Taste gedrueckt Load Start
   {
      lcd_gotoxy(14,1);
      lcd_puts("Start");
      
      tastenstatusarray[LOAD_START].pressed = 0;
   }
   else
   {
      lcd_gotoxy(14,1);
      lcd_putc(' ');
   }
   }
   */
   loopcount0+=1;
   if (loopcount0==0x0FFF)
   {
      digitalWrite(BLINKLED, !digitalRead(BLINKLED));
      loopcount0=0;
      loopcount1+=1;
      if (loopcount1 > 10)
      {
         //digitalWrite(LOOPLED, !digitalRead(LOOPLED));
digitalWrite(13, !digitalRead(13));
         loopcount1 = 0;
         lcd_gotoxy(0,0);
         /*
         lcd_puthex(recvbuffer[STROM_A_H_BYTE]);
         lcd_putc(' ');
         lcd_puthex(recvbuffer[STROM_A_L_BYTE]);           
         lcd_putc(' ');
          */
         lcd_putint12(PWM_A); 
         lcd_putc(' ');
         lcd_puthex(recvbuffer[TASK_BYTE]);           

         //Spannung = wert * TEENSYVREF / 1024
         
         lcd_gotoxy(16,0);
         lcd_putint12(adctimersekunde);

         lcd_gotoxy(0,1);
         lcd_putc('O');
         lcd_putc(':');
         lcd_putint12(batt_O);
         lcd_putc(' ');
         lcd_putc('M');
         lcd_putc(':');
         lcd_putint12(batt_M);
         lcd_putc(' ');
         lcd_putc('T');
         lcd_puthex(taskcode);
         
         lcd_gotoxy(0,2);
         lcd_putc('S');
         lcd_putc(':');
         lcd_putint12(curr_A);
         lcd_putc(' ');
         /*
         lcd_putc('B');
         lcd_putc(':');
         lcd_putint12(curr_B);
         lcd_putc(' ');
         */
/*
         lcd_putc('T');
         lcd_putc(':');
         lcd_putint12(temp_SOURCE);
 */        
         lcd_putc('B');
         lcd_putc(':');
         lcd_putint12(U_Balance);
         
         lcd_putc(' ');
         lcd_putc('L');
         lcd_putc(':');
         lcd_puthex(loadstatus);

         
         
     //    lcd_clr_line(3);
         uint16_t TEENSYVREF_Int = TEENSYVREF*100;
         lcd_gotoxy(0,3);
 //        lcd_putint12(TEENSYVREF_Int);
 //        lcd_putc(' ');

  //       lcd_putint12(batt_M);
  //       lcd_putc(' ');
         batt_M_Spannung = (((batt_M  * TEENSYVREF_Int) )* ADC_U_FAKTOR )>>10 ;
         Serial.print(F("\nbatt_M: "));
         Serial.print(batt_M);
         Serial.print(F(" batt_M_Spannung: "));
         Serial.println(batt_M_Spannung);
 
         char battbufM[8];
         itoa(batt_M_Spannung,battbufM,10); // convert integer to string
   //      lcd_puts(battbufM);
   //      lcd_putc(' ');
         char batt_StringM[8]; 
         int_to_dispstr(batt_M_Spannung,batt_StringM,2);
     //    Serial.print(F("int_to_dispstr M: "));
      //   Serial.println(batt_StringM);
  
         //lcd_puts(batt_String);
         
         char float_StringM[8]; 
         int_to_floatstr(batt_M_Spannung,float_StringM,2,4);
         
         //Serial.print(F("int_to_floatstr O: "));
         //Serial.println(float_StringM);

         lcd_puts("UA:");
         lcd_puts(float_StringM);
         //lcd_putc('*');
 
         lcd_putc(' ');
         // Batt O
         batt_O_Spannung = (((batt_O  * TEENSYVREF_Int) )* ADC_U_FAKTOR )>>10 ; // ADC_U_FAKTOR: Umrechnung aus ADC 10bit, Teensy ref 3.3
         //Serial.print(F("\nbatt_O: "));
         //Serial.print(batt_O);
         //Serial.print(F("  batt_O_Spannung: "));
         //Serial.println(batt_O_Spannung);
 
         char battbufO[8];
         itoa(batt_M_Spannung,battbufO,10); // convert integer to string
   //      lcd_puts(battbufM);
   //      lcd_putc(' ');
         char batt_StringO[8]; 
         int_to_dispstr(batt_O_Spannung,batt_StringO,2);
         //Serial.print(F("int_to_dispstr O: "));
         //Serial.println(batt_StringO);
  
         //lcd_puts(batt_String);
         
         char float_StringO[8]; 
         int_to_floatstr(batt_O_Spannung,float_StringO,2,4);
         //Serial.print(F("int_to_floatstr O: "));
         //Serial.println(float_StringO);

         lcd_puts("UO:");
         lcd_puts(float_StringO);
         //lcd_putc('*');
 
         
         
    //     lcd_putint16(batt_M_Spannung);
    //     lcd_putc(' ');
    //     lcd_puthex(senderfolg);
   //      lcd_putc(' ');
   //      lcd_putint12(usbsendcounter);

      
      
      }
      
   }// LOOPLED
   
   // MARK: MESSUNG_OK  
   if (hoststatus & (1<<MESSUNG_OK) ) // Messung ausloesen 
   {
     
      noInterrupts();
      
      hoststatus &= ~(1<<MESSUNG_OK);
      
      sendbuffer[0] = TEENSY_DATA;
 
      batt_M =  adc->analogRead(ADC_M);
      
      uint16_t TEENSYVREF_Int = TEENSYVREF*100;
      
      // Batt M
      batt_M_Spannung = (batt_M  * TEENSYVREF_Int) ;
      
      sendbuffer[U_M_L_BYTE + DATA_START_BYTE] = batt_M & 0x00FF;
      sendbuffer[U_M_H_BYTE + DATA_START_BYTE] = (batt_M & 0xFF00)>>8;
      sendbuffer[DEVICE_BYTE + DATA_START_BYTE] |= (1<<SPANNUNG_ID);
      
      // Batt O
      batt_O = analogRead(ADC_O);
      
      sendbuffer[U_O_L_BYTE + DATA_START_BYTE] = batt_O & 0x00FF;
      sendbuffer[U_O_H_BYTE + DATA_START_BYTE] = (batt_O & 0xFF00)>>8;
      
       
      // Strom Charger
      curr_A = adc->analogRead(ADC_SHUNT); // normiert auf Masse
      
      // Strom Balancer
      curr_B = analogRead(ADC_AAA) - SHUNT_OFFSET;
      
      
      
      
      //    Serial.print(F(" ADC usbsendcounter: "));
      //     Serial.print(usbsendcounter);
      sendbuffer[STROM_A_L_BYTE + DATA_START_BYTE] = curr_A & 0x00FF;
      sendbuffer[STROM_A_H_BYTE + DATA_START_BYTE] = (curr_A & 0xFF00)>>8;

      // Strom B  // test    
      sendbuffer[STROM_B_L_BYTE + DATA_START_BYTE] = curr_B & 0x00FF;
      sendbuffer[STROM_B_H_BYTE + DATA_START_BYTE] = (curr_B & 0xFF00)>>8;
     
      
      sendbuffer[DEVICE_BYTE + DATA_START_BYTE] |= (1<<STROM_ID);
      //    sendbuffer[I_SHUNT_O_L_BYTE + DATA_START_BYTE] = curr_B & 0x00FF;
      //    sendbuffer[I_SHUNT_O_H_BYTE + DATA_START_BYTE] = (curr_B & 0xFF00)>>8;
      
      temp_SOURCE = analogRead(ADC_TEMP_SOURCE);
      temp_BATT = analogRead(ADC_TEMP_BATT);
      
      sendbuffer[TEMP_SOURCE_L_BYTE + DATA_START_BYTE] = temp_SOURCE & 0x00FF;
      sendbuffer[TEMP_SOURCE_H_BYTE + DATA_START_BYTE] = (temp_SOURCE & 0xFF00)>>8;
      sendbuffer[TEMP_BATT_L_BYTE + DATA_START_BYTE] = temp_BATT & 0x00FF;
      sendbuffer[TEMP_BATT_H_BYTE + DATA_START_BYTE] = (temp_BATT & 0xFF00)>>8;
      sendbuffer[DEVICE_BYTE + DATA_START_BYTE] |= (1<<TEMP_ID);
      
      U_Balance = analogRead(ADC_BALANCE);
      sendbuffer[BALANCE_L_BYTE + DATA_START_BYTE] = U_Balance & 0x00FF;
      sendbuffer[BALANCE_H_BYTE + DATA_START_BYTE] = (U_Balance & 0xFF00)>>8;
    
      
      interrupts();
      
      Serial.print(F("ADC batt_M "));
      Serial.print(batt_M);
      
      Serial.print(F(" ADC batt_O "));
      Serial.print(batt_O);
      
      Serial.print(F(" PWM_A: "));
      Serial.print(PWM_A);

      Serial.print(F(" curr_A: ")); // shunt
      Serial.println(curr_A);
      Serial.print(F("hoststatus: "));
      Serial.println(hoststatus);

      if (hoststatus & (1<<MESSUNG_RUN))
      {
         Serial.print(F("hoststatus OK "));
         Serial.print(F(" batt_M "));
         Serial.print(batt_M);
         
         Serial.print(F(" batt_O "));
         Serial.println(batt_O);

         Serial.print(F(" BATT_MIN_RAW "));
         Serial.print(BATT_MIN_RAW);
         
         Serial.print(F(" BATT_MAX_RAW "));
         Serial.println(BATT_MAX_RAW);

         //loadstatus &= ~(1<<BATT_DOWN_BIT);
         
         
         if ((batt_M < BATT_MIN_RAW) || (batt_O < BATT_MIN_RAW))
         {
            Serial.print(F("STROM_REP_RAW: "));
            //hoststatus |= (1<<LADUNG_RUN); 
            Serial.println(curr_A);
            
            PWM_A = STROM_LO_RAW;
            
            analogWrite(A14,PWM_A);
         }
         else if (((batt_M >= BATT_MIN_RAW) && (batt_O >= BATT_MIN_RAW)) && ((batt_M < BATT_MAX_RAW) || (batt_O < BATT_MAX_RAW)))
         {
            Serial.print(F(" < STROM_HI_RAW: "));
            Serial.println(curr_A);
            
            if ((PWM_A < STROM_HI_RAW) && (hoststatus & (1<<LADUNG_RUN)) && (!(loadstatus & (1<<BATT_DOWN_BIT))))
            {
               PWM_A += 4;
            
               analogWrite(A14,PWM_A);
            }
         }
         
         if ((batt_M > BATT_MAX_RAW) || (batt_O > BATT_MAX_RAW))
         {
            Serial.print(F("< STROM_REP_RAW: "));
            //hoststatus &= ~(1<<LADUNG_RUN);  // Ladungszyklus beenden
            loadstatus |= (1<<BATT_DOWN_BIT); // Strom absenken
            Serial.println(curr_A);
            if ((PWM_A > STROM_REP_RAW) )
            {
               //PWM_A -= 16;
               //analogWrite(A14,PWM_A);
            }
            //PWM_A = STROM_REP_RAW;
            //analogWrite(A14,PWM_A);
            
          
         }
         if (loadstatus &(1<<BATT_DOWN_BIT))
         {
            if (PWM_A > (STROM_REP_RAW + 8) )
            {
               PWM_A -= 8;
               analogWrite(A14,PWM_A);
               
            }// Strom absenken
            
            Serial.println(curr_A);
            
         }

      }
       
      
      if ((batt_M > BATT_OFF_RAW) || (batt_O > BATT_OFF_RAW))
      {
         Serial.print(F("STROM_OFF_RAW: "));
         Serial.println(curr_A);

         PWM_A = 0;
         analogWrite(A14,PWM_A);
        
      }

      
      if (curr_A < 20)
      {
         Serial.print(F(" ***"));
      }
      else 
      {
         Serial.print(F("    "));
      }
      
       
      
      if(adcstatus & (1<<FIRSTRUN))
      {
         Serial.println(F(" FIRSTRUN "));
         messungcounter = 0;
         strommessungcounter = 0;
         sendbuffer[DATACOUNT_LO_BYTE] = (messungcounter & 0x00FF);
         sendbuffer[DATACOUNT_HI_BYTE] = ((messungcounter & 0xFF00)>>8);
         
         adcstatus &= ~(1<<FIRSTRUN);
      }
      if (hoststatus & (1<<MESSUNG_RUN))
      {
         messungcounter++;
         sendbuffer[DATACOUNT_LO_BYTE] = (messungcounter & 0x00FF);
         sendbuffer[DATACOUNT_HI_BYTE] = ((messungcounter & 0xFF00)>>8);
         hoststatus |= (1<<SEND_OK);
      }
      // hoststatus |= (1<<SEND_OK);
      // messungcounter uebergeben
      Serial.print(F(" strommessungcounter: "));
      Serial.print(strommessungcounter);
      Serial.print(F(" messungcounter: "));
      Serial.println(messungcounter);
      
      
      strommessungcounter++;
      
      sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF; // Byte 3, 4
      sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi
      
      //   if (batt_M > 
      
      /*    
       if (hoststatus & (1<<SEND_OK))
       {
       hoststatus &= ~(1<<SEND_OK);
       senderfolg = RawHID.send((void*)sendbuffer, 100);
       if (senderfolg > 0) 
       {
       //Serial.print(F(" ADC packet "));
       //Serial.println(packetcount);
       packetcount = packetcount + 1;
       
       } else {
       Serial.println(F("Unable to transmit packet"));
       }
       Serial.print(F("***  senderfolg: "));
       Serial.println(senderfolg);
       usbsendcounter++;
       }
       */   
   } // if MESSUNG_OK
   
   
   if (hoststatus & (1<<SEND_OK))
   {
      hoststatus &= ~(1<<SEND_OK);
      senderfolg = RawHID.send((void*)sendbuffer, 100);
      if (senderfolg > 0) 
      {
         Serial.print(F(" ADC packet "));
         Serial.println(packetcount);
         packetcount = packetcount + 1;
         
      } else {
         Serial.println(F("Unable to transmit packet"));
      }
  //    Serial.print(F("***  senderfolg: "));
  //    Serial.println(senderfolg);
      usbsendcounter++;
   }

   
   
   
   
   if (sinceRecv > 2)
   {
       
      //
      sinceRecv = 0;
      recvbuffer[0] = 0; // nichts senden, wenn kein taskcode
      noInterrupts();
      r = RawHID.recv(recvbuffer, 0); 
      usbrecvcounter++;
      if (r > 0)
      {
         taskcode = recvbuffer[0];
         usbrecvcounter++;
         switch (taskcode)
         {
               
            case STROM_SET:
            {
               // MARK: STROM_SET  
               Serial.print(("STROM_SET  0x88 "));
               PWM_A = ((recvbuffer[STROM_A_H_BYTE]) << 8) | recvbuffer[STROM_A_L_BYTE] ;
               Serial.print("STROM PWM_A:");
               Serial.println(PWM_A); 
               //analogWrite(23,PWM_A);
               analogWrite(A14,PWM_A);
               
               
            }break;
                 
               //   *********************************************************************            
               // MARK: READ_START  
               //   *********************************************************************                
            case READ_START:
            {
               Serial.print(F("READ_START "));
               Serial.println(packetcount);

               lcd_gotoxy(16,2);
               lcd_putc('R');
               sendbuffer[0] = READ_START;
               messungcounter = 0;
               blockcounter = 0;
               //    hoststatus |= (1<<USB_READ_OK);
               sendbuffer[30] = 73;
               sendbuffer[DEVICECOUNT_BYTE] = devicecount;
               uint8_t ind=0;
               adcstatus |= (1<<FIRSTRUN);
               
               hoststatus |= (1<<SEND_OK); 
            }break;
               
               //   *********************************************************************    
               // MARK: MESSUNG_START
               //   ********************************************************************* 
            case MESSUNG_START:
            {
               hoststatus |= (1<<SEND_OK); 
               hoststatus |= (1<<MESSUNG_RUN); 
               hoststatus |= (1<<LADUNG_RUN); 
               
               loadstatus &= ~(1<<BATT_DOWN_BIT);
               
 //            clear_sendbuffer();
               cli();
               Serial.print(F("MESSUNG_START"));
               //uint8_t ee = eeprom_read_word(&eeprom_intervall);
               //lcd_gotoxy(12,3);
               //lcd_putint(ee);
               hoststatus |= (1<<USB_READ_OK);
               
               adcstatus |= (1<<FIRSTRUN);
               
               messungcounter = 0;
               blockcounter = 0;
               sendbuffer[0] = MESSUNG_START;
               blockdatacounter = 0;            // Zaehler fuer Data auf dem aktuellen Block
               writedatacounter = 0;
               linecounter=0;
               intervallcounter=0;
               
               usbstatus = taskcode;
               
               sd_status = recvbuffer[1]; 
                
               mmcwritecounter = 0; // Zaehler fuer Messungen auf MMC
               
               saveSDposition = 0; // Start der Messung immer am Anfang des Blocks
               
               // intervall
               intervall = recvbuffer[TAKT_LO_BYTE] | (recvbuffer[TAKT_HI_BYTE]<<8);
               Serial.print(F("intervall "));
               Serial.print(intervall);
                
               //               abschnittnummer = recvbuffer[ABSCHNITT_BYTE]; // Abschnitt,
               
               blockcounter = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8);
               Serial.print(F(" blockcounter "));
               Serial.println(blockcounter);
            
               startminute  = recvbuffer[STARTMINUTELO_BYTE] | (recvbuffer[STARTMINUTEHI_BYTE]<<8); // in SD-Header einsetzen
               
               // Kanalstatus lesen. Wird beim Start der Messungen uebergeben
               
               // nicht verwendet
                uint8_t kan = 0;
          //     lcd_gotoxy(8,2);
               for(kan = 0;kan < 4;kan++)
               {
          //        lcd_putint2(kanalstatusarray[kan]);
               }
        //       _delay_ms(100);          
               for(kan = 0;kan < 4;kan++)
               {
                  kanalstatusarray[kan] = recvbuffer[KANAL_BYTE + kan];
               }
               
          //     lcd_gotoxy(8,2);
               for(kan = 0;kan < 4;kan++)
               {
           //       lcd_putint2(kanalstatusarray[kan]);
               
               }
               PWM_A = STROM_LO_RAW;
               analogWrite(A14,PWM_A);
               
               sendbuffer[1] = sd_status; // rueckmeldung 
               
               sendbuffer[USB_PACKETSIZE-1] = 76;
               saveSDposition = 0; // erste Messung sind header
               sei();
               
               // control
               /*
                #define STROM_HI  1000 // mA
                #define STROM_LO  50   // mA
                #define STROM_REP  100 // Reparaturstrom bei Unterspannung

                */
                // _delay_ms(1000);
               //uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               
            }break;
               
               //   *********************************************************************                
               // MARK: MESSUNG_STOP
               //   *********************************************************************                
            case MESSUNG_STOP:
            {
               Serial.println(F("MESSUNG_STOP "));
               cli();
               hoststatus &= ~(1<<MESSUNG_RUN);
               hoststatus &= ~(1<<LADUNG_RUN); 
               hoststatus |= (1<<SEND_OK); 
               clear_sendbuffer();
               sendbuffer[0] = MESSUNG_STOP;
               hoststatus &= ~(1<<USB_READ_OK);
               
               hoststatus  |= (1<<TEENSY_MMC_OK);
               // lcd_clr_line(1);
               //lcd_gotoxy(12,0);
               //lcd_putc('h');
               //lcd_putc(':');
               //lcd_puthex(code); // code
               //lcd_putc('*');
               usbstatus = taskcode;
               sd_status = recvbuffer[1]; // code fuer SD: schreiben, stoppen
               
               sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF;
               sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8;
               
               sendbuffer[DATACOUNT_LO_BYTE] = messungcounter & 0x00FF;
               sendbuffer[DATACOUNT_HI_BYTE] = (messungcounter & 0xFF00)>>8;
               
               
               
               sendbuffer[USB_PACKETSIZE-1] = 79;
               
               lcd_gotoxy(6,3);
               lcd_putint(blockcounter & 0x00FF);

               eeprom_update_word(&eeprom_blockcount,blockcounter);
               eeprom_update_word(&eeprom_messungcount,messungcounter);
               
               //          lcd_gotoxy(19,1);
               //         lcd_putc('+');
               //         lcd_gotoxy(12,1);
               //          lcd_puts("m stop");
               
               //uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               sei();
               // Haltestrom einschalten
               PWM_A = STROM_LO_RAW;
               
               analogWrite(A14,PWM_A);

            }break;
               

               //   ********************************************************************* 
               // MARK: LOGGER_START
               //   *********************************************************************                
            case LOGGER_START:
            {
               clear_sendbuffer();
               hoststatus |= (1<<SEND_OK);
               loggerstatus = 0;
               hoststatus &= ~(1<<USB_READ_OK);
               hoststatus &= ~(1<<MESSUNG_OK);
               hoststatus &= ~(1<<TEENSY_ADC_OK);
               hoststatus |= (1<<DOWNLOAD_OK); // Download von SD, Messungen unterbrechen
               hoststatus &= ~(1<<MANUELL_OK);
               lcd_clr_line(1);
               lcd_gotoxy(16,1);
               lcd_putc('T');
               lcd_putc(':');
               lcd_puthex(taskcode);
               
               sendbuffer[0] = LOGGER_START;
               
               writemmcstartcounter = 0;
               //code = LOGGER_START; // read block starten
               //lcd_putc('c');
               //lcd_puthex(code); // packetcount
               
               // Block lesen
               /*
                lcd_putc('l');
                lcd_puthex(recvbuffer[BLOCKOFFSETLO_BYTE]); // startblock lo
                lcd_putc('h');
                lcd_puthex(recvbuffer[BLOCKOFFSETHI_BYTE]); // startblock hi
                lcd_putc('*');
                lcd_puthex(recvbuffer[PACKETCOUNT_BYTE]); // packetcount
                */
               // old
               
               startblock = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8); // zu lesender Block auf mmc
               
               uint8_t paketindex = 0;
               
               // old
               //packetcount = recvbuffer[DEVICECOUNT_BYTE] ;// laufender Index Paket, beim Start 0
               
               packetcount = recvbuffer[PACKETCOUNT_BYTE] ;// laufender Index Paket, beim Start 0
               //               packetcount=0;
               // lcd_gotoxy(12,1);
               // lcd_puts(">mmc");
               lcd_gotoxy(18,1);
               lcd_puthex(startblock);
               
               blockanzahl = recvbuffer[BLOCK_ANZAHL_BYTE] ;// laufender Index Paket, beim Start 0
               
               downloaddatacounter = recvbuffer[DATACOUNT_LO_BYTE] | (recvbuffer[DATACOUNT_HI_BYTE]<<8); // 
               
               /*
               lcd_gotoxy(6,2);
               lcd_puts("load ");
               lcd_putint2(blockanzahl);
               lcd_putc(' ');
               lcd_putint12(downloaddatacounter);
               */
                
               uint16_t logger_blockcount = eeprom_read_word(&eeprom_blockcount);
               sendbuffer[23] = '*';
               sendbuffer[24] = logger_blockcount & 0x00FF;
               sendbuffer[25] = (logger_blockcount & 0xFF00)>>8;
               
               lcd_gotoxy(6,2);
               lcd_puts("     ");

               lcd_gotoxy(6,2);
               lcd_putc('*');
               lcd_putint(logger_blockcount & 0x00FF);
               lcd_putc('*');
               uint16_t logger_messungcount = eeprom_read_word(&eeprom_messungcount);
               sendbuffer[26] = logger_messungcount & 0x00FF;
               sendbuffer[27] = (logger_messungcount & 0xff00)>>8;
               sendbuffer[28] = '*';                  

               //lcd_gotoxy(7,3);
               //lcd_puthex(downloaddatacounter);
               
               
               // Beim Start Block aus SD lesen
   //            uint8_t readerr = mmc_disk_read((void*)mmcbuffer,1+ startblock,1);
               EEPROM.read(1+ startblock);
               
               if (readerr == 0)
               {
                  lcd_gotoxy(15,1);
                  lcd_puts(">OK ");
                  if (TEST == 0)
                  {
                     sendbuffer[DATA_START_BYTE -1] = 111;
                     sendbuffer[USB_PACKETSIZE-1] = 87;
                  }
                  
                  // Header uebertragen, HEADER_SIZE bytes nach BLOCK_SIZE im Block. Schreiben nach DATA_STARTBYTE + HEADER_OFFSET
                  
                  uint8_t headerindex=0;
                  for (headerindex = 0;headerindex < 10;headerindex++)
                  {
                     sendbuffer[DATA_START_BYTE + HEADER_OFFSET + headerindex] = mmcbuffer[BLOCK_SIZE  + headerindex];
                  }
                  //#define DATA_START_BYTE    8    // erstes byte fuer Data auf USB
                  //#define HEADER_OFFSET      4     // Erstes Byte im Block nach BLOCK_SIZE: Daten, die bei LOGGER_NEXT uebergeben werden

                  
                  
                  
               } // if readerr==0
               else
               {
                  lcd_gotoxy(14,1);
                  lcd_puts(">err");
               }
               
               sendbuffer[PACKETCOUNT_BYTE] = 0; //
               sendbuffer[1] = readerr;
               
               if (TEST == 1)
               {
                  sendbuffer[16] = 57;
                  sendbuffer[17] = startblock;
                  sendbuffer[18] = 58;
                  sendbuffer[19] = blockanzahl;
                  sendbuffer[20] = 59;                  
               }
            }break;
               
               //   *********************************************************************                
               // MARK: LOGGER_CONT 
               //   *********************************************************************                            
            case LOGGER_CONT: // weiteres Paket lesen (Datenzeile)
            {
               clear_sendbuffer();
               hoststatus |= (1<<SEND_OK);
               if (TEST)
               {
                  sendbuffer[0] = LOGGER_CONT;
                  sd_status = recvbuffer[1];
                  packetcount = recvbuffer[PACKETCOUNT_BYTE];
                  
                  
                  uint8_t paketindex = 0;
                  if (downloadblocknummer == 0 && packetcount == 0) // Start Logging
                  {
                     // Daten in mmcbuffer ab index (packetcount*PACKET_SIZE) lesen    
                     for (paketindex=0;paketindex< PACKET_SIZE;paketindex++) // PACKET_SIZE: 24 bytes fuer sendbuffer
                     {
                        
                        //lcd_gotoxy(18,3);
                        //lcd_putc(65); // Kontrolle: A
                        // Header ist schon geladen, Breite ist HEADER_SIZE (8)
                        //sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[HEADER_SIZE + (packetcount*PACKET_SIZE)+paketindex];
                        sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[(packetcount*PACKET_SIZE)+paketindex];
                     }
                  }
                  else
                  {
                     // Daten in mmcbuffer ab index (packetcount*PACKET_SIZE) lesen
                     for (paketindex=0;paketindex< PACKET_SIZE;paketindex++) // 24 bytes fuer sendbuffer
                     {
                        // Breite ist HEADER_SIZE (8)
                        sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[(packetcount*PACKET_SIZE)+paketindex];
                     }
                  }
               }
               else        
               {
                  
                  // lcd_clr_line(1);
                  //lcd_gotoxy(0,3);
                  //lcd_putc('c');
                  // lcd_putc(':');
                  sendbuffer[0] = LOGGER_CONT;
                   sd_status = recvbuffer[1];
                  packetcount = recvbuffer[PACKETCOUNT_BYTE];
                  uint8_t paketindex = 0;
                  
                  if (downloadblocknummer == 0 && packetcount == 0) // Start Logging
                  {
                     writedatacounter = 0;
                     // writemmcstartcounter++;
                     //lcd_gotoxy(13,3);
                     //lcd_puthex(writemmcstartcounter);
                      // Daten in mmcbuffer ab index (packetcount*PACKET_SIZE) lesen    
                     for (paketindex=0;paketindex< PACKET_SIZE;paketindex++) // PACKET_SIZE: 24 bytes fuer sendbuffer
                     {
                        
                        //                        lcd_gotoxy(18,3);
                        //                       lcd_putc(65); // Kontrolle: A
                         sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[(packetcount*PACKET_SIZE)+paketindex];
                     }
                     
                  }
                  else
                  {
                     writedatacounter++;
                     //                    lcd_gotoxy(15,3);
                     //                    lcd_puthex(writedatacounter);
                     
                     
                     
                     //                     lcd_gotoxy(19,3);
                     //                    lcd_putc(97); // Kontrolle: a
                     
                     // Daten in mmcbuffer ab index (packetcount*PACKET_SIZE) lesen
                     for (paketindex=0;paketindex< PACKET_SIZE;paketindex++) // 24 bytes fuer sendbuffer
                     {
                       
                        sendbuffer[DATA_START_BYTE + paketindex] = mmcbuffer[(packetcount*PACKET_SIZE)+paketindex];
                     }
                  }
                  
                  // packetcount mitgeben und incrementieren
                  sendbuffer[PACKETCOUNT_BYTE] = ++packetcount; // Byte 2
                  
                  sendbuffer[USB_PACKETSIZE-1] = 74;
                  sendbuffer[USB_PACKETSIZE-2] = writedatacounter;
                  //              uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
                  
                  //lcd_gotoxy(18,2);
                  //lcd_puthex(usberfolg);
               }
               
            }break;
               //   *********************************************************************                
               // MARK: LOGGER_NEXT
               //   *********************************************************************                
            case LOGGER_NEXT: // Block an startblock + downloadblocknummer lesen
            {
               for (int i=0;i<USB_PACKETSIZE;i++)
               {
                  sendbuffer[i] = 0;
               }
               //clear_sendbuffer();
               //lcd_clr_line(1);
               //lcd_gotoxy(0,1);
               //lcd_putc('n');
               //lcd_putc(':');
               //lcd_puthex(code); // code
               sendbuffer[0] = LOGGER_NEXT;
               sendbuffer[USB_PACKETSIZE-1] = 71;
               //startblock = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8); // zu lesender Block auf mmc
               uint8_t paketindex = 0;
               //lcd_gotoxy(0,3);
               //lcd_puthex(startblock);
               
               downloadblocknummer  = recvbuffer[DOWNLOADBLOCKNUMMER_BYTE] ;// nummer des next blocks, Byte 10
               
               lcd_gotoxy(18,3);
               lcd_puthex(downloadblocknummer);
                              
               // Beim Start Block aus SD lesen
   //            uint8_t  readerr = mmc_disk_read((void*)mmcbuffer,1+ startblock + downloadblocknummer,1);
               if (readerr == 0)
               {
                  lcd_gotoxy(19,1);
                  lcd_putc('+');
                  
                  // Header uebertragen, letzte HEADER_SIZE bytes im Block
                  
                  uint8_t headerindex=0;
                  for (headerindex = 0;headerindex < HEADER_SIZE;headerindex++)
                  {
                     sendbuffer[DATA_START_BYTE + HEADER_OFFSET + headerindex] = mmcbuffer[BLOCK_SIZE + headerindex];
                  }
               } // if readerr==0
               else
               {
                  lcd_gotoxy(19,1);
                  lcd_putc('-');
               }
               uint8_t delta = 0;
               uint8_t offset = 4;
               
               //Daten am Ende des letzten Blocks lesen
               /*
                sendbuffer[DATA_START_BYTE + offset + delta] = mmcbuffer[BLOCK_SIZE + delta];// blockcounter lo
                delta++;
                sendbuffer[DATA_START_BYTE + offset + delta] = mmcbuffer[BLOCK_SIZE + delta]; // blockcounter hi
                delta++;
                sendbuffer[DATA_START_BYTE + offset + delta] = mmcbuffer[BLOCK_SIZE + delta]; // blockdatacounter lo
                delta++;
                sendbuffer[DATA_START_BYTE + offset + delta] = mmcbuffer[BLOCK_SIZE + delta]; // blockdatacounter hi
                delta++;
                sendbuffer[DATA_START_BYTE + offset + delta] = mmcbuffer[BLOCK_SIZE + delta]; // messungcounter lo
                
                delta++;
                sendbuffer[DATA_START_BYTE + offset + delta] = mmcbuffer[BLOCK_SIZE + delta]; // messungcounter hi
              */  
               
               //            uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               
               //lcd_gotoxy(18,2);
               //lcd_puthex(usberfolg);
            }break; // LOGGER_NEXT
               
               //   *********************************************************************                
               // MARK: LOGGER_STOP
               //   ********************************************************************* 
            case LOGGER_STOP: // 0xAF
            {
               hoststatus &= ~(1<<DOWNLOAD_OK); // Download von SD beendet, 
               //lcd_clr_line(1);
               lcd_clr_line(1);
               lcd_gotoxy(0,1);
               lcd_putc('s');
               lcd_putc(':');
               lcd_puthex(taskcode); // code
               
               mmcwritecounter = 0;
               usbstatus = 0;
               sendbuffer[0] = LOGGER_STOP;
               sendbuffer[PACKETCOUNT_BYTE] = 0; // packetcount
               sendbuffer[BLOCKOFFSETLO_BYTE] = downloadblocknummer & 0x00FF;
               sendbuffer[BLOCKOFFSETHI_BYTE] = (downloadblocknummer & 0xFF00)>>8;
               hoststatus |= (1<<SEND_OK);

            }break;
         }//switch taskcode
         interrupts();
         if (sendbuffer[0]) // nur senden, wenn code gesetzt ist. Bsp SERVO_OUT: kein code
         {
            writemmcstartcounter++;
            //  lcd_gotoxy(11,3);
            //  lcd_puthex(loggerstatus);
            if (TEST == 1)
            {
               lcd_gotoxy(13,3);
               lcd_puthex(writemmcstartcounter);
               sendbuffer[21] = sendbuffer[0];
               sendbuffer[22] = writemmcstartcounter;
               sendbuffer[23] = loggerstatus;
            }
            
  // send
            if (hoststatus & (1<<SEND_OK))
            {
               hoststatus &= ~(1<<SEND_OK);
               senderfolg = RawHID.send((void*)sendbuffer, 100);
               if (senderfolg > 0) 
               {
                  Serial.print(F(" ADC messungcounter "));
                  Serial.print(messungcounter);
                  Serial.print(F(" ADC packet "));
                  Serial.println(packetcount);
                  packetcount = packetcount + 1;
                  
               } else {
                  Serial.println(F("Unable to transmit packet"));
               }
               Serial.print(F("*** *** senderfolg: "));
               Serial.println(senderfolg);
               usbsendcounter++;
            }

            
            // end send
            sendbuffer[0] = 0;
         } // if sendbuffer[0] > 0

         
      } // r>0
      else     // Betrieb ohne host, Messungen vornehmen
      {
         
         
      }

   }
   
}
