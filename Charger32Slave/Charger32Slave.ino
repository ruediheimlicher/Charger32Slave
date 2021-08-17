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

// define constants
//#define USB_DATENBREITE 64

#define TEST 0

// Set parameters


// Include application, user and local libraries
// !!! Help http://bit.ly/2CL22Qp


// Define structures and classes
ADC *adc = new ADC(); // adc object


// Define variables and constants
static volatile uint8_t recvbuffer[64]={};
static volatile uint8_t sendbuffer[64]={};

volatile uint8_t           adcstatus=0x00;
volatile uint16_t          adctimercounter = 0;

volatile uint8_t           usbstatus=0x00;

volatile uint8_t          senderfolg = 0;
volatile uint8_t status=0;

volatile uint16_t           PWM_A=0;
volatile uint16_t           PWM_B=0;

volatile uint16_t       batt_M = 0;
volatile uint16_t       batt_O = 0;
volatile uint16_t       curr_U = 0;
volatile uint16_t       curr_O = 0;

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
volatile uint16_t                     blockcounter = 0; // Block, in den gesichert werden soll, mit einem Offset von 1 (Block 0 ist header der SD).

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

volatile uint8_t mmcbuffer[SD_DATA_SIZE] = {};


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


IntervalTimer              adcTimer;
//volatile uint16_t          timerintervall = TIMERINTERVALL;
volatile uint16_t         adctimerintervall = 1000;
IntervalTimer              stromTimer;


// constants

// bits von hoststatus
#define TEENSYPRESENT      7
#define MESSUNG_OK         6
#define DOWNLOAD_OK        5
#define USB_READ_OK        4
#define TEENSY_ADC_OK      3
#define TEENSY_MMC_OK      2
#define MANUELL_OK         1




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

   //LCD
   pinMode(LCD_RSDS_PIN, OUTPUT);
   pinMode(LCD_ENABLE_PIN, OUTPUT);
   pinMode(LCD_CLOCK_PIN, OUTPUT);

   pinMode(LADESTROM_PWM_A, OUTPUT);
   pinMode(LADESTROM_PWM_A, OUTPUT);
   
   
//   pinMode(25, OUTPUT);// OC1A
//   pinMode(26, OUTPUT);// OC1A
   
 //  pinMode(24, OUTPUT);// OC2A
}

void clear_sendbuffer(void)
{
   for (int i=0;i<USB_PACKETSIZE;i++)
   {
      sendbuffer[i] = 0;
   }
}

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
   
   adc->adc0->setAveraging(2); // set number of averages 
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
   Serial.begin(9600);
   Serial.println(F("RawHID H0"));
   slaveinit();
   /* initialize the LCD */
   _delay_ms(100);
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   _delay_ms(100);
   lcd_puts("Guten Tag Charger");
   _delay_ms(100);
   adcTimer.begin(adctimerfunction,adctimerintervall); // 1ms
   lcd_clr_line(0);
   analogWriteResolution(10);
   pinMode(A14,OUTPUT);
   

}
elapsedMillis sinceRecv;
// Add loop code
void loop()
{
   loopcount0+=1;
   if (loopcount0==0x0FFF)
   {
      loopcount0=0;
      loopcount1+=1;
      if (loopcount1 > 10)
      {
         digitalWrite(LOOPLED, !digitalRead(LOOPLED));
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
         
     //    lcd_clr_line(3);
    //     lcd_gotoxy(0,3);
    //     lcd_putint12(usbrecvcounter);
    //     lcd_putc(' ');
    //     lcd_puthex(senderfolg);
   //      lcd_putc(' ');
   //      lcd_putint12(usbsendcounter);

      
      
      }
      
   }// LOOPLED
   
   // MARK: MESSUNG_OK  
   if (hoststatus & (1<<MESSUNG_OK) ) // Messung ausloesen
   {
      hoststatus &= ~(1<<MESSUNG_OK);
      
      sendbuffer[0] = TEENSY_DATA;
      adcstatus &= ~(1<<ADC_U_BIT);
      noInterrupts();
      //batt_M = readKanal(ADC_M);
      batt_M = analogRead(ADC_M);
      interrupts();
      Serial.print(F("ADC batt_M "));
      Serial.print(batt_M);
      sendbuffer[U_M_L_BYTE + DATA_START_BYTE] = batt_M & 0x00FF;
      sendbuffer[U_M_H_BYTE + DATA_START_BYTE] = (batt_M & 0xFF00)>>8;
      
      batt_O = analogRead(ADC_O);
      Serial.print(F(" ADC batt_O "));
      Serial.print(batt_O);

      sendbuffer[U_O_L_BYTE + DATA_START_BYTE] = batt_O & 0x00FF;
      sendbuffer[U_O_H_BYTE + DATA_START_BYTE] = (batt_O & 0xFF00)>>8;
      
      adcstatus &= ~(1<<ADC_I_BIT);
      curr_U = analogRead(ADC_SHUNT_U);
      curr_O = analogRead(ADC_SHUNT_O);
      Serial.print(F(" ADC usbsendcounter: "));
        Serial.print(usbsendcounter);
      sendbuffer[I_SHUNT_U_L_BYTE + DATA_START_BYTE] = curr_U & 0x00FF;
      sendbuffer[I_SHUNT_U_H_BYTE + DATA_START_BYTE] = (curr_U & 0xFF00)>>8;
      sendbuffer[I_SHUNT_O_L_BYTE + DATA_START_BYTE] = curr_O & 0x00FF;
      sendbuffer[I_SHUNT_O_H_BYTE + DATA_START_BYTE] = (curr_O & 0xFF00)>>8;
      
      if(adcstatus & (1<<FIRSTRUN))
      {
         Serial.println(F(" FIRSTRUN "));

         messungcounter = 0;
         adcstatus &= ~(1<<FIRSTRUN);
         
      }
      // messungcounter uebergeben
      Serial.print(F(" messungcounter: "));
        Serial.println(messungcounter);

      sendbuffer[DATACOUNT_LO_BYTE] = (messungcounter & 0x00FF);
      sendbuffer[DATACOUNT_HI_BYTE] = ((messungcounter & 0xFF00)>>8);
      
      messungcounter++;
      
      sendbuffer[BLOCKOFFSETLO_BYTE] = blockcounter & 0x00FF; // Byte 3, 4
      sendbuffer[BLOCKOFFSETHI_BYTE] = (blockcounter & 0xFF00)>>8; // Nummer des geschriebenen Blocks hi

      if (hoststatus & (1<<SEND_OK))
      {
         senderfolg = RawHID.send((void*)sendbuffer, 100);
         if (senderfolg > 0) 
         {
            Serial.print(F(" ADC packet "));
            Serial.println(packetcount);
            packetcount = packetcount + 1;
            
         } else {
            Serial.println(F("Unable to transmit packet"));
         }
         Serial.print(F("***  senderfolg: "));
         Serial.println(senderfolg);
         usbsendcounter++;
      }
   } // if adcstatus

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
               //Serial.println(("STROM_SET  0x88 "));
               PWM_A = ((recvbuffer[STROM_A_H_BYTE]) << 8) | recvbuffer[STROM_A_L_BYTE] ;
               Serial.print("STROM PWM_A:");
               Serial.println(PWM_A); 
               analogWrite(23,PWM_A);
               analogWrite(A14,PWM_A);
               
            }break;
            case DEFAULT: // 
            {
               //lcd_clr_line(2);
               sendbuffer[0] = DEFAULT;
               //sendbuffer[3] = 0;
               mmcwritecounter = 0;
               //            code = WRITE_MMC_TEST;
               //lcd_putc('c');
               //lcd_puthex(code); // code
               sd_status = recvbuffer[1]; // bit 0: sd mit testdaten beschreiben
               //lcd_putc('-');
               //lcd_puthex(sd_status); // code
               //uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               //lcd_gotoxy(18,2);
               //lcd_puthex(usberfolg);
               // PWM fuer Channel A
               
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
 //             clear_sendbuffer();
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
               
               
               /*
                lcd_clr_line(1);
                lcd_gotoxy(0,1);
                lcd_putc('m');
                lcd_putc(':');
                lcd_puthex(code); // code
                */
               usbstatus = taskcode;
               sd_status = recvbuffer[1]; 
               
               
               mmcwritecounter = 0; // Zaehler fuer Messungen auf MMC
               
               saveSDposition = 0; // Start der Messung immer am Anfang des Blocks
               
               // intervall
               intervall = recvbuffer[TAKT_LO_BYTE] | (recvbuffer[TAKT_HI_BYTE]<<8);
               Serial.print(F("intervall "));
               Serial.println(intervall);
               //lcd_gotoxy(14,2);
               //lcd_putc('i');
               //lcd_putc(':');
               //lcd_putint(intervall);
               
               
               //               abschnittnummer = recvbuffer[ABSCHNITT_BYTE]; // Abschnitt,
               
               blockcounter = recvbuffer[BLOCKOFFSETLO_BYTE] | (recvbuffer[BLOCKOFFSETHI_BYTE]<<8);
               Serial.print(F("blockcounter "));
               Serial.println(blockcounter);
            
               // startminute  = recvbuffer[STARTMINUTELO_BYTE] | (recvbuffer[STARTMINUTEHI_BYTE]<<8); // in SD-Header einsetzen
               
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
          
               
               /*
                lcd_putc(' ');
                lcd_puthex(blockcounter);
                
                lcd_putc(' ');
                lcd_puthex(sd_status);
                lcd_putc(' ');
                lcd_puthex(saveSDposition);
                */
         //      lcd_gotoxy(12,1);
         //      lcd_puts("start ");
               sendbuffer[1] = sd_status; // rueckmeldung 
               //sendbuffer[2] = wl_callback_status;
               //               sendbuffer[5] = 18;//recvbuffer[STARTMINUTELO_BYTE];;
               //               sendbuffer[6] = 19;//recvbuffer[STARTMINUTEHI_BYTE];;
               //sendbuffer[7] = 21;
               
               sendbuffer[USB_PACKETSIZE-1] = 76;
               saveSDposition = 0; // erste Messung sind header
               sei();
               
               // _delay_ms(1000);
               //uint8_t usberfolg = usb_rawhid_send((void*)sendbuffer, 50);
               
            }break;
               
               //   *********************************************************************                
               // MARK: MESSUNG_STOP
               //   *********************************************************************                
            case MESSUNG_STOP:
            {
               hoststatus &= ~(1<<SEND_OK); 
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
               
            }break;
               

               //   ********************************************************************* 
               // MARK: LOGGER_START
               //   *********************************************************************                
            case LOGGER_START:
            {
               clear_sendbuffer();
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
            
  //          uint8_t usberfolg = RawHID.send((void*)sendbuffer, 100);
            sendbuffer[0] = 0;
         } // if sendbuffer[0] > 0

         
      } // r>0
      else     // Betrieb ohne host, Messungen vornehmen
      {
         
         
      }

   }
   
}
