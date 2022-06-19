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
#include "OTA.h"
#include <FirebaseESP32.h>
//insilisasilibray untuk ph
#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"
#include "time.h"


//string
String (USs);


//untuk pengiriman data ke firebase
FirebaseData firebaseData;
FirebaseJson json;


//ntp

// Variable to save current epoch time
unsigned long epochTime; 
const char* ntpServer = "pool.ntp.org";

//get curent epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}


//timer
int x=500;

//mengatur accespoint yang digunakan
#define mySSID "Citizencouncil"
#define myPASSWORD "bnjkio7890"

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
//ph 
int val = analogRead(34);

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
float voltage, phValue,temperature = 25;
 


float volt;

void setup() {
  Serial.begin(115200);
  
  //auth firebsae
  Firebase.begin("https://ujicova-b614a-default-rtdb.firebaseio.com/","l3NmeFUtle9tV2IrpdiMPTpasdPlKo8dUHOGNshO");
    (Firebase.setString(firebaseData,"Sensor/Sw","0"));
    setupOTA("esp32code", mySSID, myPASSWORD);

   //int time
 configTime(0, 0, ntpServer);
  
  //LCD
  lcd.backlight();

  //PINMODE RELAY ,ULTRASONIC 
  pinMode(relay_pumpin ,OUTPUT);
  pinMode(relay_pumpout ,OUTPUT);
  pinMode(relay_EC ,OUTPUT);
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
void sensorph(){
	 static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) //time interval: 1s
  {
    timepoint = millis();
    //voltage = rawPinValue / esp32ADC * esp32Vin
    voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // read the voltage
    Serial.print("voltage:");
    Serial.println(voltage, 4);
    
    //temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
    Serial.print("temperature:");
    Serial.print(temperature, 1);
    Serial.println("^C");

    phValue = ph.readPH(voltage, temperature); // convert voltage to pH with temperature compensation
    Serial.print("pH:");
    Serial.println(phValue, 4);
  }
  ph.calibration(voltage, temperature); // calibration process by Serail CMD
	
}

void sensorTSS(){

	
	
}

void loop() {
//	#ifdef defined(ESP32_RTOS) && defined(ESP32)
	//#else // If you do not use FreeRTOS, you have to regulary call the handle method.
  ArduinoOTA.handle();
	//endif
   epochTime= getTime();
  if(Firebase.set(firebaseData,"/Sensor/time",epochTime));
   //ucapan pembuka pada LCD
	lcd.setCursor(0,0); lcd.print("==AUTOMASI SISTEM FUZZY==");
	lcd.setCursor(0,1); lcd.print("ELEKTROKOAGULASI"); 
	delay(3000);
	lcd.clear();
	
	proses1:

  //proses pengecekan bak pengisian 

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
   USs="Penuh";
  lcd.setCursor(0,0); lcd.print("PROSES pengisian");;}


  //jika bak terisi akan dibuang
  else if (distanceCm =13.46)
  {digitalWrite(relay_pumpin, HIGH);
  digitalWrite(relay_pumpout, LOW  );
  USs="Kosong";
  lcd.setCursor(0,0); lcd.print("PROSES Pembuangan");
  }
   delay(500);
  
  fuzzy:
  //proses 2 pentuan kondisi untuk diproses atau di buang 
  
 

  //sensor ph

	int sensorph1=phValue;
  
  //sensor tss
	double ntu1 = (val *(-0.067)) + 146.65;
	double ntu =constrain(ntu1, 1, 100);
	Serial.println("- Kekeruhan : ");
	Serial.println(" NTU");
	Serial.println(ntu);
  delay (x);
  int ntus=ntu;
  
  //variable yang digunakn untuk intput fuzzy
	fuzzy->setInput(1, ntus);
	fuzzy->setInput(2, sensorph1);
	
	//serial print nilai fuzzifikasi tiap member function
	  fuzzy->fuzzify();
	  Serial.print(ntus);
	  Serial.print("   kepekatan: ");
	  Serial.print(SPK->getPertinence());
	  Serial.print(", ");
	  Serial.print(PK->getPertinence());
	  Serial.print(", ");
	  Serial.print(SPK->getPertinence());
	  Serial.print("  ");

	  Serial.print(sensorph1);
	  Serial.print("   keasaman: ");
	  Serial.print(AM->getPertinence());
	  Serial.print(", ");
	  Serial.print(OP->getPertinence());
	  Serial.print(", ");
	  Serial.print(BS->getPertinence());
	  Serial.print("  ");
	  
	// hasil defuzzy 
		float output1 = fuzzy->defuzzify(1);
		// serial print defuzzi
	  Serial.print("Defuzzy: ");
	  Serial.println(output1);
	  
	//penentuan aksi 
	//kondisi proses
	if(output1 >= 0 && output1 <=2049 ){ 
		lcd.clear();
		digitalWrite(relay_pumpin, LOW);
		digitalWrite(relay_pumpout, LOW  );
		lcd.setCursor(0,0);lcd.print("F=");
		lcd.setCursor(2,0);lcd.print(output1);
		lcd.setCursor(9,0);lcd.print("PROSES");
		bool b=1;
		lcd.setCursor(0,b);lcd.write(1);lcd.setCursor(1,b);
		lcd.print("=");lcd.setCursor(2,b);lcd.print(ntus);
		lcd.setCursor(6,b);lcd.write(2);lcd.setCursor(7,b);
		lcd.print("=");lcd.setCursor(8,b);lcd.print(sensorph1);

		delay(x);
		goto fuzzy;
	  }
	  
	 //kondisi BUANG
  else if(output1 > 2050 && output1 <= 4096){
    lcd.clear();
    digitalWrite(relay_pumpin, HIGH);
    digitalWrite(relay_pumpout, LOW  );
    lcd.setCursor(0,0);lcd.print("F=");
    lcd.setCursor(2,0);lcd.print(output1);
    lcd.setCursor(9,0);lcd.print("BUANG");
    
 	bool b=1;
		lcd.setCursor(0,b);lcd.write(1);lcd.setCursor(1,b);
		lcd.print("=");lcd.setCursor(2,b);lcd.print(ntus);
		lcd.setCursor(6,b);lcd.write(2);lcd.setCursor(7,b);
		lcd.print("=");lcd.setCursor(8,b);lcd.print(sensorph1);

    
  
    goto fuzzy;
  }

   //virtual switch
  if (Firebase.getString(firebaseData, "/Sensor/Sw")) { //misal database diberikan nama relay1
    if  (firebaseData.dataType() == "string") 
    {
      String FBStatus = firebaseData.to<String>();
      if (FBStatus == "1") {   
      Serial.println(" manual mode");  
      
 if (Firebase.getString(firebaseData, "/Sensor/pompa")) { //misal database diberikan nama relay1
    if  (firebaseData.dataType() == "string") 
    {
      String FBStatus = firebaseData.to<String>();
      if (FBStatus == "1") {                                                         
      Serial.println("pompa On");                         
      digitalWrite(relay_pumpin, HIGH); 
      }
          else if (FBStatus == "0") {                                                  
          Serial.println("Pompa Off");
          digitalWrite(relay_pumpin, LOW);                                                
              }
      else {Serial.println("Salah kode! isi dengan data On/Off");}
    }
  }


 if (Firebase.getString(firebaseData, "/Sensor/Statuskran")) { //misal database diberikan nama relay1
    if  (firebaseData.dataType() == "string") 
    {
      String FBStatus = firebaseData.to<String>();
      if (FBStatus == "2") {                                                         
      Serial.println("pompa2 On");                         
      digitalWrite(relay_pumpout, HIGH); 
      }
          else if (FBStatus == "3") {                                                  
          Serial.println("Pompa2 Off");
          digitalWrite(relay_pumpout, LOW);                                                
              }
      else {Serial.println("Salah kode! isi dengan data On/Off");}
    }
  }

 if (Firebase.getString(firebaseData, "/Sensor/StatusEC")) { //misal database diberikan nama relay1
    if  (firebaseData.dataType() == "string") 
    {
      String FBStatus = firebaseData.to<String>();
      if (FBStatus == "4") {                                                         
      Serial.println("Ec On");                         
      digitalWrite(relay_EC, HIGH); 
      }
          else if (FBStatus == "5") {                                                  
          Serial.println("Ec Off");
          digitalWrite(relay_EC, LOW);                                                
              }
      else {Serial.println("Salah kode! isi dengan data On/Off");}
    }
  }
      }
          else if (FBStatus== "0") {                                                  
          Serial.println("otomatis mode");
          if(Firebase.setFloat(firebaseData,"Sensor/TSS",ntus)){
      Serial.println("send tss sucsessfully"); 
    }else{
      Serial.println(" tss not send"); 
      Serial.println("casue :"+firebaseData.errorReason()); 
    }
    if (Firebase.setFloat(firebaseData,"Sensor/pH",sensorph1)){
      Serial.println("send ph sucsessfully"); 
    }else{
      Serial.println("  ph not send"); 
      Serial.println("casue :"+firebaseData.errorReason());
    }
	//for list below database has configure but sensor on esp32 not configure propperly (pompa,statuskran,level air,level ec)
	
	if(Firebase.setString(firebaseData,"Sensor/Levelair",USs)){
      Serial.println("send pompa sucsessfully"); 
    }else{
      Serial.println(" Wlevel not send"); 
      Serial.println("casue :"+firebaseData.errorReason()); 
    }
              }
      else {Serial.println("Salah kode! isi dengan data On/Off");}
    }
  }
  //conversion 
	//ultrasonic
		
  
   if(Firebase.setFloat(firebaseData,"Sensor/TSS",ntus)){
      Serial.println("send tss sucsessfully"); 
    }else{
      Serial.println(" tss not send"); 
      Serial.println("casue :"+firebaseData.errorReason()); 
    }
    if (Firebase.setFloat(firebaseData,"Sensor/pH",sensorph1)){
      Serial.println("send ph sucsessfully"); 
    }else{
      Serial.println("  ph not send"); 
      Serial.println("casue :"+firebaseData.errorReason());
    }
	//for list below database has configure but sensor on esp32 not configure propperly (pompa,statuskran,level air,level ec)
	
	if(Firebase.setString(firebaseData,"Sensor/Levelair",USs)){
      Serial.println("send pompa sucsessfully"); 
    }else{
      Serial.println(" Wlevel not send"); 
      Serial.println("casue :"+firebaseData.errorReason()); 
    }
  
	
    delay(800);
}
