//  ======inisialisasi============== //
#include <Fuzzy.h>
#include <FuzzyComposition.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzyOutput.h>
#include <FuzzyRule.h>
#include <FuzzyRuleAntecedent.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzySet.h>
#define ESP32_RTOS  
#include "OTA.h
//insilisasilibray untuk ph
#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"

//mengatur accespoint yang digunakan
#define mySSID"Citizencouncil"
#define myPASSWORD"bnjkio7890"

// LCD 16X2
#include <LiquidCrystal_I2C.h>
int lcdColumns = 16; int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows); //address untuk I2C_lCD

//ULTRASONIC
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701
 uint32_t entry;
 const int trigPin = 17; const int echoPin = 16; //inisialisasi pin triger&echo
 long duration;
 float distanceCm;
 float distanceInch;

//Relay
  const int relay_pumpin = 27;  //pompa pengisian
  const int relay_pumpout = 32; //pompa pembuangan
   const int relay_EC = 28; //EC
  
  //fuzzy MS 
Fuzzy*fuzzy =new Fuzzy();
//member function
//pH
 FuzzySet* AM=new FuzzySet(0,0,3.3,6);
 FuzzySet* OP=new FuzzySet(5.8,7,7,9.2);
 FuzzySet* BS=new FuzzySet(9,11,14,14);

//TSS
 FuzzySet* BN=new FuzzySet(0,0,5.8,59.89);
 FuzzySet* PK=new FuzzySet(5.88,59.89,59.89,100);
 FuzzySet* SPK=new FuzzySet(59.89,100,100,111);

//Output
  FuzzySet* PROSES=new FuzzySet(0,1021,1021,2050);
  FuzzySet* BUANG=new FuzzySet(2050,3075,3075,4096); 

//ph
#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"

DFRobot_ESP_PH ph;
#define ESPADC 4096.0   //the esp Analog Digital Convertion value
#define ESPVOLTAGE 3300 //the esp voltage supply value
#define PH_PIN 35    //the esp gpio data pin number
float voltage, phValue, temperature = 25;
 

void setup() {
  Serial.begin(115200);
    setupOTA("esp32code", mySSID, myPASSWORD);
  //LCD
  lcd.backlight();

  //PINMODE RELAY ,ULTRASONIC 
  pinMode(relay_pumpin ,OUTPUT);
  pinMode(relay_pumpout ,OUTPUT);
//pinMode(relay_EC ,OUTPUT);
  pinMode(trigPin, OUTPUT); // Ultrasonic Sets the trigPin as an Output
  pinMode(echoPin, INPUT); //  UltrasonicSets the echoPin as an Input
  
  //rule fuzzy
    // put your setup code here, to run once:
  //fuzzy Input pH meter
  FuzzyInput* pH=new FuzzyInput(1);
  pH->addFuzzySet(AM);
  pH->addFuzzySet(OP);
  pH->addFuzzySet(BS);
  fuzzy->addFuzzyInput(pH);
 
  //fuzzy Input TSS meter
  FuzzyInput* TSS=new FuzzyInput(2);
  TSS->addFuzzySet(BN);
  TSS->addFuzzySet(PK);
  TSS->addFuzzySet(SPK);
  fuzzy->addFuzzyInput(TSS);

  //Fuzzy output
   FuzzyOutput* OUT=new FuzzyOutput(1);
  OUT->addFuzzySet(PROSES);
  OUT->addFuzzySet(BUANG);
  fuzzy->addFuzzyOutput(OUT);

  //rule 1
  FuzzyRuleAntecedent*ifpHAMAndTSSBN = new FuzzyRuleAntecedent ();
  ifpHAMAndTSSBN->joinWithAND(AM,BN);
  FuzzyRuleConsequent*thenPROSES = new FuzzyRuleConsequent();
  thenPROSES->addOutput(PROSES);
  FuzzyRule* fuzzyRule01 = new FuzzyRule(1,ifpHAMAndTSSBN,thenPROSES);
  fuzzy->addFuzzyRule(fuzzyRule01);

  //rule2
  FuzzyRuleAntecedent*ifpHOPAndTSSBN = new FuzzyRuleAntecedent ();
  ifpHOPAndTSSBN->joinWithAND(OP,BN);
  FuzzyRuleConsequent*thenBUANG = new FuzzyRuleConsequent();
  thenBUANG->addOutput(BUANG);
  FuzzyRule* fuzzyRule02 = new FuzzyRule(2,ifpHOPAndTSSBN,thenBUANG);
  fuzzy->addFuzzyRule(fuzzyRule02);

  //rule3
  FuzzyRuleAntecedent*ifpHBSAndTSSBN = new FuzzyRuleAntecedent ();
  ifpHOPAndTSSBN->joinWithAND(BS,BN);
  FuzzyRuleConsequent*thenBUANG2 = new FuzzyRuleConsequent();
  thenBUANG2->addOutput(BUANG);
  FuzzyRule* fuzzyRule03 = new FuzzyRule(3,ifpHBSAndTSSBN,thenBUANG2);
  fuzzy->addFuzzyRule(fuzzyRule03);

  //rule4
  FuzzyRuleAntecedent*ifpHAMAndTSSPK = new FuzzyRuleAntecedent ();
  ifpHAMAndTSSPK->joinWithAND(AM,PK);
  FuzzyRuleConsequent*thenPROSES2 = new FuzzyRuleConsequent();
  thenPROSES2->addOutput(PROSES);
  FuzzyRule* fuzzyRule04 = new FuzzyRule(4,ifpHAMAndTSSPK,thenPROSES2);
  fuzzy->addFuzzyRule(fuzzyRule04);

  //rule5
  FuzzyRuleAntecedent*ifpHOPAndTSSPK = new FuzzyRuleAntecedent ();
  ifpHOPAndTSSPK->joinWithAND(OP,PK);
  FuzzyRuleConsequent*thenPROSES3 = new FuzzyRuleConsequent();
  thenPROSES3->addOutput(PROSES);
  FuzzyRule* fuzzyRule05 = new FuzzyRule(4,ifpHOPAndTSSPK,thenPROSES3);
  fuzzy->addFuzzyRule(fuzzyRule05);

  //rule6
  FuzzyRuleAntecedent*ifpHBSAndTSSPK = new FuzzyRuleAntecedent ();
  ifpHBSAndTSSPK->joinWithAND(BS,PK);
  FuzzyRuleConsequent*thenPROSES4 = new FuzzyRuleConsequent();
  thenPROSES4->addOutput(PROSES);
  FuzzyRule* fuzzyRule06 = new FuzzyRule(6,ifpHBSAndTSSPK,thenPROSES4);
  fuzzy->addFuzzyRule(fuzzyRule06);

  //rule7
   FuzzyRuleAntecedent*ifpHAMAndTSSSPK = new FuzzyRuleAntecedent ();
  ifpHAMAndTSSSPK->joinWithAND(AM,SPK);
  FuzzyRuleConsequent*thenPROSES5 = new FuzzyRuleConsequent();
  thenPROSES5->addOutput(PROSES);
  FuzzyRule* fuzzyRule07 = new FuzzyRule(7,ifpHAMAndTSSSPK,thenPROSES5);
  fuzzy->addFuzzyRule(fuzzyRule07);
  //rule8
   FuzzyRuleAntecedent*ifpHOPAndTSSSPK = new FuzzyRuleAntecedent ();
  ifpHAMAndTSSSPK->joinWithAND(OP,SPK);
  FuzzyRuleConsequent*thenPROSES6 = new FuzzyRuleConsequent();
  thenPROSES6->addOutput(PROSES);
  FuzzyRule* fuzzyRule08 = new FuzzyRule(8,ifpHAMAndTSSSPK,thenPROSES6);
  fuzzy->addFuzzyRule(fuzzyRule08);
  //rule9
   FuzzyRuleAntecedent*ifpHBSAndTSSSPK = new FuzzyRuleAntecedent ();
  ifpHAMAndTSSSPK->joinWithAND(BS,SPK);
  FuzzyRuleConsequent*thenPROSES7 = new FuzzyRuleConsequent();
  thenPROSES6->addOutput(PROSES);
  FuzzyRule* fuzzyRule09 = new FuzzyRule(9,ifpHBSAndTSSSPK,thenPROSES7);
  fuzzy->addFuzzyRule(fuzzyRule09);
  //ph
    EEPROM.begin(32);//needed to permit storage of calibration value in eeprom
  ph.begin();
}

void loop() {
	#ifdef defined(ESP32_RTOS) && defined(ESP32)
	#else // If you do not use FreeRTOS, you have to regulary call the handle method.
  ArduinoOTA.handle();
	#endif
   //ucapan pembuka pada LCD
	lcd.setCursor(0,0); lcd.print("==AUTOMASI SISTEM FUZZY==");
	lcd.setCursor(0,1); lcd.print("ELEKTROKOAGULASI"); 
	delay(3000);
	lcd.clear();
	
	proses1:

  //proses pengecekan bak pengisian 
   // Clears the trigPin
  lcd.clear();
   
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
    // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  // Convert to inches
  distanceInch = distanceCm * CM_TO_INCH;
  
  //jika bak kosong akan disisi 
  if (distanceCm < 4)
  {digitalWrite(relay_pumpin, LOW);
   digitalWrite(relay_pumpout, HIGH);
  lcd.setCursor(0,0); lcd.print("PROSES pengisian");;}
  goto proses1;
  
  //jika bak kosong akan dibuang
  else if (distanceCm =13.46)
  {digitalWrite(relay_pumpin, HIGH);
  digitalWrite(relay_pumpout, LOW  );
  lcd.setCursor(0,0); lcd.print("PROSES Pembuangan");
  }
   delay(500);
  
  fuzzy:
  

 

}
