/*
   Copyright 2016 Alessandro Pasqualini

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
   
   @author         Alessandro Pasqualini <alessandro.pasqualini.1105@gmail.com>
   @url            https://github.com/alessandro1105
*/

//---LIBRERIE---

//RTC
#include <Wire.h>
#include <RTClib.h>

//FlashString
#include <Flash.h>

//Jack
#include <Jack.h>
#include <SoftwareSerialJack.h>
#include <SoftwareSerial.h>


//---COSTANTI--

//VREF ADC
#define VREF 1.1 //tensione di riferimento per il ADC

//LM35
#define LM35_ENABLE_PIN 4 //abilitazione
#define LM35_PIN A0 //lettura

//GSR
#define GSR_ENABLE_PIN 5 //abilitazione
#define GSR_PIN A1 //lettura
#define GSR_VCC_PIN A2 //lettura ddp applicata
#define GSR_NOISE 40 //rimozione del rumore

//HM-10
#define HM10_TX 8 //TX 
#define HM10_RX 9 //RX
#define HM10_BAUDRATE 9600 //baudrate (da verificare)
//#define HM10_BT_NAME "LW2v0" //nome del bluetooth

//DATA COLLECT
#define INTERVAL_BETWEEN_DATA_COLLECT 10000 //DEBUG 1 sec
//#define INTERVAL_BETWEEN_DATA_COLLECT 300000 //intervallo tra un data collect e un altro

//JACK
#define TIMER_SEND_MESSAGE 5000 //intervallo tra l'invio dei messaggi
#define TIMER_POLLING 1000 //intervallo tra un polling e l'altro del mezzo di trasmissione

//CHIAVI PER IL MESSAGGIO
#define TIMESTAMP_KEY "TMP" //chiave per timestamp (TiMetamP)
#define GSR_KEY "GSR" //chiave per gsr (GSR)
#define TEMPERATURE_KEY "TME" //chiave per temperatura (TeMperaturE)


//costante per il debug su seriale
#define DEBUG 1


//---VARIABILI---

//stato dei sensori
typedef enum lwSensorState {
  LW_SENSOR_SLEEP, //sensori addormentati
  LW_SENSOR_WAKE //sensori svegli
};

lwSensorState sensorState;

//RTC
RTC_DS1307 RTC;

//DATA COLLECT
long timeLastDataCollect;

//JACK
SoftwareSerial bluetooth(HM10_TX, HM10_RX); //seriale per il modulo HM-10
SoftwareSerialJack mmJTM(bluetooth); //Mezzo di trasmissione per Jack
Jack jack(mmJTM, &onReceive, &onReceiveAck, &getTimestamp, TIMER_SEND_MESSAGE, TIMER_POLLING); //Jack



//---HANDLER JACK---
void onReceive(JData &message) {} //handler per messaggi dati in entrata
void onReceiveAck(long id) {} //handler per ricezione ack


//---GET DATA FROM SENSORS FUNCTIONS---

//legge e coverte in percentuale la lettura del sensore GSR
uint8_t getGSR() {
  
  int gsr = analogRead(GSR_PIN); //prelevo la lettura dal sensore

  //rimozione del rumore
  if (gsr > GSR_NOISE) {
    gsr -= GSR_NOISE;
  } else {
    gsr = 0;
  }

  int vcc = analogRead(GSR_VCC_PIN); //leggo la vcc applicata al sensore

  #ifdef DEBUG
    Serial.print(F("\nGSR READ: "));
    Serial.println(gsr);
    Serial.print(F("\nVCC READ: "));
    Serial.println(gsr);
  #endif

  return (uint8_t) (100.0 * gsr / vcc); //ritorno la lettura in percentuale
  
}

//legge e converte in gradi C la lettura del sensore LM35
double getTemperature() {
  
  double temp = analogRead(LM35_PIN); //prelevo la lettura dal sensore
  temp = (temp * VREF / 1023.0) * 100.0; //converto la temperatura letta in gradi
  
  double decimalPart = temp - floor(temp); //ricavo la parte decimale
  temp = floor(temp) + (floor(decimalPart * 10) / 10); //sommo la parte intera e una cifra dopo la virgola
  
  #ifdef DEBUG
    Serial.print(F("\nTEMPERATURE READ: "));
    Serial.println(temp);
  #endif
  
  return temp; //restituisco la temperatura con un decimale
  
}

//ottiene il timestamp da RTC
long getTimestamp() {

  long timestamp = RTC.now().unixtime();
  
  #ifdef DEBUG
    Serial.print(F("\nTIMESTAMP: "));   
    Serial.println(timestamp);  
  #endif
  
  return timestamp; //get unix timestamp;
  
}


//---SETUP/SLEEP/WAKEUP SENSORS FUNCTIONS---

//setup sensor
void setupSensor() {

  analogReference(INTERNAL); //imposto VREF a 1.1V

  //preparo i pin di abilitazione
  pinMode(LM35_ENABLE_PIN, OUTPUT);
  pinMode(GSR_ENABLE_PIN, OUTPUT);

  //spengo i sensori
  sleepSensor();

  //setup RTC
  Wire.begin(); //wire
  RTC.begin(); //rtc

  //se RTC non è partito imposto l'ora
  if (!RTC.isrunning()) {

    #ifdef DEBUG
      Serial.println(F("\nRTC is NOT running!\n"));
    #endif
    
    RTC.adjust(DateTime(__DATE__, __TIME__)); //setto RTC con il data e ora di compilazione dello sketch
  }

  #ifdef DEBUG
    Serial.print(F("\nSENSORI SETTATI\n"));
  #endif
  
}

//sveglia i sensori
void wakeupSensor() {

  //sveglio i sensori
  digitalWrite(LM35_ENABLE_PIN, HIGH);
  digitalWrite(GSR_ENABLE_PIN, HIGH);

  //imposto lo stato dei sensori
  sensorState = LW_SENSOR_WAKE;

  #ifdef DEBUG
    Serial.print(F("\nSENSORI SVEGLIATI\n"));
  #endif
  
}

//addormenta i sensori per risparmiare energia
void sleepSensor() {

  //addormento i sensori
  digitalWrite(LM35_ENABLE_PIN, LOW);
  digitalWrite(GSR_ENABLE_PIN, LOW);

  //imposto lo stato dei sensori
  sensorState = LW_SENSOR_SLEEP;

  #ifdef DEBUG
    Serial.print(F("\nSENSORI ADDORMENTATI\n"));
  #endif
  
}

//ritorna lo stato corrente dei sensori (svegli, addormentati)
lwSensorState getSensorState() {
  return sensorState;
}


//---SETUP BLUETOOTH---
void setupBluetooth() {
  //avvio la seriale
  bluetooth.begin(HM10_BAUDRATE);
  
}


//---DATA COLLECT FUNCTION---

//preleva i dati dai sensori e li invia
void collectData() {

  //verifico che i sensori non siano addormentati (ovvero siano svegli)
  if (getSensorState() == LW_SENSOR_SLEEP) {
   
    #ifdef DEBUG
      Serial.println(F("\n\nIMPOSSIBILE PRELEVARE I DATI SEI SENSORI PERCHE' SONO ADDORMENTATI\n\n"));
    #endif

    //i sensori sono addormentati non posso prelevare i dati
    return;
  }

  //prelevo i dati dai sensori insieme al timestamp
  long timestamp = getTimestamp();
  uint8_t gsr = getGSR();
  double temperature = getTemperature();

  #ifdef DEBUG
    Serial.println(F("\n\n---------LETTURE DEI SENSORI (collectData())---------\n"));
    Serial.print(F("TIMESTAMP: "));
    Serial.print(timestamp);
    Serial.print(F("\nTEMPERATURA: "));
    Serial.print(temperature);
    Serial.print(F("\nGSR: "));
    Serial.print(gsr);
    Serial.println(F("\n------------------\n\n"));
  #endif

  //creo il contenitore del messaggio
  JData message;

  //aggiungo i dati
  message.add(TIMESTAMP_KEY, timestamp);
  message.add(GSR_KEY, gsr);
  message.add(TEMPERATURE_KEY, temperature);

  //invio il messaggio
  jack.send(message);
  
}


//---SETUP FUNCTION---
void setup() {

  #ifdef DEBUG
    Serial.begin(9600);
  #endif

  //inizializzo i sensori
  setupSensor();

  //inizializzo il bluetooth
  setupBluetooth();

  //avvio jack
  jack.start();

}


//---LOOP FUNCTION---
void loop() {

  //loop jack
  jack.loop();

  //prelevo il tempo passato dall'inizio dell'esecuzione
  long now = millis();

  //se l'intervallo di data collect è stato raggiunto
  if (now - timeLastDataCollect >= INTERVAL_BETWEEN_DATA_COLLECT) {

    //prelevo i dati dai sensori
    collectData();

    //addormento i sensori
    sleepSensor();

    //salvo il tempo passato dall'inizio dell'esecuzione all'ultimo data collect
    timeLastDataCollect = now;

  //se è stato raggiunta la metà dell'intervallo di invio
  } else if (getSensorState() == LW_SENSOR_SLEEP && now - timeLastDataCollect >= (int) (INTERVAL_BETWEEN_DATA_COLLECT / 2)) {

    //sveglio i sensori
    wakeupSensor();
    
  }

}
