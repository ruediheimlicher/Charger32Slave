//
//  text.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 22.11.2013.
//
//

#ifndef DOG_LCD_text_h
#define DOG_LCD_text_h


// Homescreen
 const char titel0[]  = "Charger Home";
 const char titel1[]  = "O:";
 const char titel2[]  = "M:";
 const char titel3[]  = "PW";
 const char titel4[]  = "PW_M";
 const char titel5[]  = "B:";
 const char titel6[]  = "I:";
 const char titel7[]  = "M";


 const char *TitelTable[]  = {titel0, titel1, titel2, titel3, titel4, titel5, titel6, titel7};

// Modelle
const char model0[]  = "E-Segler A  ";
const char model1[]  = "E-Segler B  ";
const char model2[]  = "Hangsegler A";
const char model3[]  = "Motor A     ";
const char model4[]  = "Motor B     ";
const char model5[]  = "AA\0        ";
const char model6[]  = "BB\0        ";
const char model7[]  = "CC\0        ";

char *ModelTable[]  = {model0, model1, model2, model3, model4, model5, model6, model7};





// Settingscreen
const char menutitel[]  = "Settings";
const char model[]  = "Modell";
const char setting[]  = "Set";
const char kanal[]  = "Kanal";

const char mix[]  = "Mix";
const char zuteilung[]  = "Zuteilung";
const char ausgang[]  = "Ausgang";


const char *SettingTable[]  = {menutitel, model, setting, kanal,  mix, zuteilung,ausgang};


// Kanalscreen
const char kanaltitel[]  = "Kan:";
const char richtung[]  = "Ri:";
const char funktion[]  = "Fkt:";
const char level[]  = "Level";
const char expo[]  = "Expo";

const char seitea[]  = "A:";
const char seiteb[]  = "B:";
const char kanaltyp[]  = "Typ:";

const char *KanalTable[]  = {kanaltitel, richtung, funktion, level, expo, seitea, seiteb,kanaltyp};


// Kanaltyp
const char pitchtyp[]  = "Pitch";
const char schiebertyp[]  = "Schieber";
const char schaltertyp[]  = "Schalter";

const char *KanalTypTable[]  = {pitchtyp,schiebertyp,schaltertyp};


// Mix
const char mixtitel[]  = "Mixing";
const char MixTable[]  = {mixtitel};

// Mixtyp
const char nada[]  = "OFF ";
const char vmix[]  = "V-Mix";
const char butterfly[]  = "B-fly";
const char A[]  = "ABCD ";

const char *MixTypTable[]  = {nada,vmix,butterfly,A};

// Zuteilung
const char zuteilungtitel[]  = "Zuteilung";
const char ZuteilungTable[]  = {zuteilungtitel};

// Sichern
const char frage[]  = "Aenderungen sichern";
const char sichern[]  = "SICHERN";
const char abbrechen[]  = "Ignorieren";

const char* SichernTable[]  = {frage,abbrechen,sichern};

// Funktion

const char funktion0[]  = "Seite";
const char funktion1[]  = "Hoehe";
const char funktion2[]  = "Quer ";
const char funktion3[]  = "Motor";
const char funktion4[]  = "QuerL";
const char funktion5[]  = "QuerR";
const char funktion6[]  = "Lande";
const char funktion7[]  = "Aux  ";

const char *FunktionTable[]  = {funktion0, funktion1, funktion2, funktion3, funktion4, funktion5, funktion6, funktion7};

// Ausgang

const char ausgang0[]  = "Imp";
const char ausgang1[]  = "Kan";
const char ausgang2[]  = "Dev";
const char ausgang3[]  = "Fkt";
const char ausgang4[]  = " ";
const char ausgang5[]  = "Quer R\0";
const char ausgang6[]  = "Lande \0";
const char ausgang7[]  = "Aux    \0";

const char *AusgangTable[]  = {ausgang0, ausgang1, ausgang2, ausgang3, ausgang4, ausgang5, ausgang6, ausgang7};


// Zuteilung an device auf dem Sender
const char device0[]  = "L-H\0"; // Pitch links horizontal
const char device1[]  = "L-V\0"; // Pitch links vertikal
const char device2[]  = "R-H\0";
const char device3[]  = "R-V\0";
const char device4[]  = "S-L\0"; // Schieber links
const char device5[]  = "S-R\0"; // Schieber rechts
const char device6[]  = "Sch\0"; // Schalter
const char device7[]  = "Aux\0";

const char *DeviceTable[]  = {device0, device1, device2, device3, device4, device5, device6, device7};


#endif


