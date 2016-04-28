//---LIBRERIE---

//RTC
#include <Wire.h>
#include <RTClib.h>


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
#define HM10_TX 8 //TX (da verificare) 
#define HM10_RX 9 //RX (da verificare)
#define HM10_BAUDRATE 9600 //baudrate (da verificare)

//costante per il debug su seriale
#define DEBUG 1

//---VARIABILI---

//stato dei sensori
enum lwSensorState {
  LW_SENSOR_SLEEP,
  LW_SENSOR_WAKE
};

lwSensorState sensorState;

RTC_DS1307 RTC;

//---funzioni per ottenere la lettura dei sensori---

//legge e coverte in percentuale la lettura del sensore GSR
long getGSR() {
  
  int gsr = analogRead(GSR_PIN); //prelevo la lettura dal sensore

  //rimozione del rumore
  if (gsr > GSR_NOISE) {
    gsr -= GSR_NOISE;
  } else {
    gsr = 0;
  }

  int vcc = analogRead(GSR_VCC_PIN); //leggo la vcc applicata al sensore

  #ifdef DEBUG
    Serial.print("GSR READ: ");
    Serial.println(gsr);
    Serial.print("VCC READ: ");
    Serial.println(gsr);
  #endif

  return (int) (100.0 * gsr / vcc); //ritorno la lettura in percentuale
  
}

//legge e converte in gradi C la lettura del sensore LM35
double getTemperature() {
  
  double temp = analogRead(LM35_PIN); //prelevo la lettura dal sensore
  temp = (temp * VREF / 1023.0) * 100.0; //converto la temperatura letta in gradi
  
  double decimalPart = temp - floor(temp); //ricavo la parte decimale
  temp = floor(temp) + (floor(decimalPart * 10) / 10); //sommo la parte intera e una cifra dopo la virgola
  
  #ifdef DEBUG
    Serial.print("TEMPERATURE READ: ");
    Serial.println(temp);
  #endif
  
  return temp; //restituisco la temperatura con un decimale
  
}

//ottiene il timestamp da RTC
long getTimespamp() {

  long timestamp = RTC.now().unixtime();
  
  #ifdef DEBUG
    Serial.print("TIMESTAMP: ");   
    Serial.println(timestamp);  
  #endif
  
  return timestamp; //get unix timestamp;
  
}


//---setup/sleep/wakeup function---

//setup sensor
void setupSensor() {

  analogReference(INTERNAL); //imposto VREF a 1.1V

  //preparo i pin di abilitazione
  pinMode(LM35_ENABLE_PIN, OUTPUT);
  pinMode(GSR_ENABLE_PIN, OUTPUT);

  //spengo i sensori
  sleepSensor();

  #ifdef DEBUG
    Serial.print("SENSORI SETTATI");
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
    Serial.print("SENSORI SVEGLIATI");
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
    Serial.print("SENSORI ADDORMENTATI");
  #endif
  
}

//ritorna lo stato corrente dei sensori (svegli, addormentati)
uint8_t getSensorState() {
  return sensorState;
}







void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
