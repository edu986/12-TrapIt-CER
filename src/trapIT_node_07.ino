/*
Last version, 11/01/17 everithing working
Intalled version for Telefonica

Node IDs and Channel keys on  https://docs.google.com/spreadsheets/d/18ahhKX5Bcw9BbATKLpYCgIWZweLfMB-y8I8mF4U9v_8/edit?usp=sharing

*/
// CONFIGURATION START

char thingspeak_ch[17] = "D2Z5FUVJTW3C73PZ";      // thingspeak channel
#define node_addr         7                      // NODE ADDRESS

#define   RechargePeriod  60                    // Period in seconds to recharge the capacitor
unsigned int idlePeriodInMin = 15;                // TIME IN MINUTES BETWEEN 2 READING & TRANSMISSION

// CONFIGURATION END


#include <SPI.h>
#include "SX1272.h"               // Include the Congduc modified SX1272  http://cpham.perso.univ-pau.fr/
#include <stdlib.h>               // to be able to use the dtostrf function
#include <TH02_dev.h>             // TH02 lib from seeed studio driver lib
#include <SPIFlash.h>             // Flash memory spi lib, https://github.com/LowPowerLab/SPIFlash                                  // writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <Wire.h>                 // I2C lib
#include <OneWire.h>              // one wire bus lib for soil temp sensor
#include <DallasTemperature.h>    // soil temp sensor lib https://github.com/milesburton/Arduino-Temperature-Control-Library


#define ANARDUINO               // Set the board type manually, since the Mini wireless board type does not define this
#define RADIO_RFM92_95          // radio is an HopeRF RFM95W
#define BAND900                 // //#define BAND433, BAND868
#define DEBUG

#define PRINTLN                   Serial.println("")
#define PRINT_CSTSTR(fmt,param)   Serial.print(F(param))
#define PRINT_STR(fmt,param)      Serial.print(param)
#define PRINT_VALUE(fmt,param)    Serial.print(param)
#define FLUSHOUTPUT               Serial.flush();

///////////////////////////////////////////////////////////////////
// COMMENT OR UNCOMMENT TO CHANGE FEATURES.
// ONLY IF YOU KNOW WHAT YOU ARE DOING!!! OTHERWISE LEAVE AS IT IS
#define WITH_APPKEY
#define FLOAT_DATA
#define NEW_DATA_FIELD
#define LOW_POWER
#define WITH_ACK

#define LORAMODE          1            // THE LORA MODE

#define DEFAULT_DEST_ADDR 1
#ifdef WITH_ACK
  #define NB_RETRIES      2
#endif


#ifdef WITH_APPKEY
  ///////////////////////////////////////////////////////////////////
  // CHANGE HERE THE APPKEY, BUT IF GW CHECKS FOR APPKEY, MUST BE
  // IN THE APPKEY LIST MAINTAINED BY GW.
  uint8_t my_appKey[4]={5, 6, 7, 8};
#endif




// data message format:
// \!OJ4BUPW28QPJCXLA##/SM1/4.60/SM2/9.67/SM3/1.02/SM4/0.01/V/3.28/T/6.84/H/5.69/ST/25.38

#define LOW_POWER_PERIOD 8
#include "LowPower.h"   // LowPower library from RocketScream,  https://github.com/rocketscream/Low-Power
unsigned int nCycle = idlePeriodInMin*60/LOW_POWER_PERIOD;
unsigned int RechargeCycle = RechargePeriod/LOW_POWER_PERIOD;

unsigned long lastTransmissionTime = 0;
unsigned long delayBeforeTransmit = 0;
char float_str[10];    // used to be 20 in simple temp example
uint8_t message[100];  // the message array to be sent over radio
int loraMode=LORAMODE;



// PIN DEFINITIONS
#define THPWR             A3   // TH02 power pin, turn TH02 sensor on or off 3.3
#define DSWITCH           8    //selected pin to turn on/off the digital switch
#define MOTH_D            3    //selected pin when for the IRQ interruption

#define SEND_DATA_PERIOD  8    //Period of time to send data

#define FLASH_SS          5    // MW has FLASH SS on D5, Moteino has FLASH SS on D8
#define BATT_MONITOR_EN   A1   //enables battery voltage divider to get a reading from a battery, disable it to save power
#define BATT_MONITOR      A0   //through 1Meg+330Kohm and 0.1uF cap from battery VCC - this ratio divides the voltage to bring it below 3.3V where it is scaled to a readable range
#define BATT_CYCLES       1    //defines the frecuency of battery measurements, every X times it will actually read the battery voltage
#define LED               9

char Tstr[8];
char RHstr[8];
char TSoilstr[8];
char BATstr[5];
char MCstr[8];
char MEstr[8];

SPIFlash flash(FLASH_SS);           // Initiate the flash memory

// VARIABLE DEFINITIONS

volatile bool flag1 = false;               //flag to indicates that an IRQ interrupt has been produced
volatile bool flag2 = false;               //flag to indicates that an IRQ interrupt has been produced
volatile unsigned long countSeconds = 0;   //
volatile unsigned long countTimeSD = 0;    //counts the time to send data
float event;
float temper;
float humidity;
float vbat;
float mothCount = 0;
int counter1 = 0;                         // counter to save the value of i when interrupt is called
int counter2 = 0;                         // counter to save the value of i when interrupt is called
unsigned int i=0;                         // main counter of sleep loop

float batteryVolts;
int cycleCount = 0;

float checkBattery();

void setup() {

  pinMode(DSWITCH, OUTPUT);
  pinMode(MOTH_D, INPUT_PULLUP);     //Was INPUT_PULLUP
  pinMode(BATT_MONITOR_EN, OUTPUT);
  pinMode(BATT_MONITOR, INPUT);

  pinMode(THPWR, OUTPUT);
  pinMode(LED, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(MOTH_D), detectionIRQ, FALLING);     //attach a IRQ interrupt to the MOTH_D pin
  //attachInterrupt(MOTH_D, detectionIRQ, FALLING) ;

  //START SERIAL COMINICATION if DEBUG is defined, todo, clan up serial prints
  #ifdef DEBUG
      Serial.begin(38400);
      while (!Serial) ; // Wait for serial port to be available
      Serial.flush();
      Serial.println(F("TrapIT_revEA_v4.ino"));

  #endif


/*
 //Hangs here if the one wire temperature sensor is on DIO5
  // START AND SLEEP OF FLASH MEMORY
  // ---------------------------------------------------------------------------
  if (flash.initialize()){
    flash.sleep();
    #ifdef DEBUG
      Serial.println(F("Flash is sleeping"));
    #endif
  }
  else{
    #ifdef DEBUG
      Serial.println(F("Flash Init failed"));
    #endif
  }
*/



#ifndef DEBUG
    Serial.end();           // If not debug mode, turn off the serial interface
#endif

int e;
sx1272.ON();                      // Power ON the radio module
e = sx1272.setMode(loraMode);     // Set transmission mode and print the result
PRINT_CSTSTR("%s","Setting Mode: state ");
PRINT_VALUE("%d", e);
PRINTLN;

sx1272._enableCarrierSense=true;   // enable carrier sense, this makes it hang sometimes when waiting for proper config to be met
#ifdef LOW_POWER
// TODO: with low power, when setting the radio module in sleep mode
// there seem to be some issue with RSSI reading
sx1272._RSSIonSend=false;
#endif

#ifdef BAND868
  // Select frequency channel
  e = sx1272.setChannel(CH_10_868);
#elif defined BAND900
  // Select frequency channel
  e = sx1272.setChannel(CH_05_900);
#elif defined BAND433
  // Select frequency channel
  e = sx1272.setChannel(CH_00_433);
#endif

PRINT_CSTSTR("%s","Setting Channel: state ");
PRINT_VALUE("%d", e);
PRINTLN;

// Select output power (Max, High or Low)
#if defined RADIO_RFM92_95 || defined RADIO_INAIR9B
e = sx1272.setPower('X');   // Max power is X, 20dBm, 100mW. x is 14dBm
#else
e = sx1272.setPower('M');
#endif

PRINT_CSTSTR("%s","Setting Power: state ");
PRINT_VALUE("%d", e);
PRINTLN;

// Set the node address and print the result
e = sx1272.setNodeAddress(node_addr);
PRINT_CSTSTR("%s","Setting node addr: state ");
PRINT_VALUE("%d", e);
PRINTLN;

// Print a success message
PRINT_CSTSTR("%s","Sending to ThingSpeak Channel:");
PRINT_VALUE("%s", thingspeak_ch);
PRINTLN;

delay(500);
Read_TH02();
flag1 = false;

cycleCount = BATT_CYCLES;
vbat = checkBattery();
cycleCount = 0;

digitalWrite(LED, HIGH);
delay(250);
digitalWrite(LED, LOW);
delay(100);
digitalWrite(LED, HIGH);
delay(250);
digitalWrite(LED, LOW);
Recharge_Fnc();


}


// START OF MAIN LOOP
// -----------------------------------------------------------------------------

void loop() {

  unsigned long startSend;
  unsigned long endSend;
  uint8_t app_key_offset = 0;
  int e;

  // use app_key_offset to make space for the app key at the beginning of the message
  app_key_offset = sizeof(my_appKey);
  memcpy(message,my_appKey,app_key_offset);  // insert the app key in the payload

  // Insert the cloud key in the message, just after the app key
  //  uint8_t cloud_key_offset = 2+sizeof(thingspeak_ch);
  uint8_t cloud_key_offset = 2+16+2;   // 2 for the \!, 16 for the thing speak channel, 2 for the two hashes
  sprintf((char*)message+app_key_offset,"\\!%s##", thingspeak_ch);
  Serial.print(F("cloud_key_offset: ")); Serial.println(cloud_key_offset);

  if (flag1){                 // FROM INTERUPT, MOTH DETECTED

    delay(200);              //delay to avoid count the oscilations when the discharge happen
    noInterrupts();
    flag1=0;                  // Clear the flag
    mothCount +=1;            // add one moth
    event = 1;                // indicates an interrupt event
    i = counter1;             // restore value for sleep counter after interrupt

    interrupts();
    digitalWrite(LED, HIGH);
    delay(2000);
    digitalWrite(LED, LOW);
  }

  else{

    Read_TH02();              // Read TH02 sensor
    vbat = checkBattery();       // Read battery voltage
    i = 0;                    // reset the value of sleep loop
    event=0;

  }
  Recharge_Fnc();
  int dataoffset = 0;
  while (message[dataoffset] != '\0') dataoffset ++;  // find the length of the data so far.
  // Add the measuered supporting data to the message
  sprintf((char*)message+dataoffset, "V/%s/T/%s/H/%s/MC/%s/ME/%s",
                                      dtostrf(vbat, 4,2, BATstr),
                                      dtostrf(temper, 4,2, Tstr),
                                      dtostrf(humidity, 4,2, RHstr),
                                      dtostrf(mothCount, 4,2, MCstr),
                                      dtostrf(event, 4,2, MEstr));


// Set up the send
// -----------------------------------------------------------------------------
int pl=0;  // the payload length, assuming ascii char message, 8 bits per char.
while (message[pl] != '\0') pl ++;  // find the length of the data so far.

PRINT_CSTSTR("%s","Sending: ");
PRINT_STR("%s",(char*)(message+app_key_offset));
PRINTLN;

PRINT_CSTSTR("%s","Real payload size is: ");
PRINT_VALUE("%d", pl);
PRINTLN;

sx1272.CarrierSense();
startSend=millis();

#ifdef WITH_APPKEY
      // indicate that we have an appkey
      sx1272.setPacketType(PKT_TYPE_DATA | PKT_FLAG_DATA_WAPPKEY);
#else
      // just a simple data packet
      sx1272.setPacketType(PKT_TYPE_DATA);
#endif

// Send message to the gateway and print the result
// with the app key if this feature is enabled
PRINT_CSTSTR("%s","About to send...\n");

#ifdef WITH_ACK
      int n_retry=NB_RETRIES;

      do {
        e = sx1272.sendPacketTimeoutACK(DEFAULT_DEST_ADDR, message, pl);

        if (e==3)
          PRINT_CSTSTR("%s","No ACK..");

        n_retry--;

        if (n_retry)
          PRINT_CSTSTR("%s","Retry..");
        else
          PRINT_CSTSTR("%s","Abort..");

      } while (e && n_retry);
#else
      e = sx1272.sendPacketTimeout(DEFAULT_DEST_ADDR, message, pl);
#endif
      endSend=millis();


PRINT_CSTSTR("%s","LoRa pkt size ");
PRINT_VALUE("%d", pl);
PRINTLN;

PRINT_CSTSTR("%s","LoRa pkt seq ");
PRINT_VALUE("%d", sx1272.packet_sent.packnum);
PRINTLN;

PRINT_CSTSTR("%s","LoRa Sent in ");
PRINT_VALUE("%ld", endSend-startSend);
PRINTLN;

PRINT_CSTSTR("%s","LoRa Sent w/CAD in ");
PRINT_VALUE("%ld", endSend-sx1272._startDoCad);
PRINTLN;

PRINT_CSTSTR("%s","Packet sent, state ");
PRINT_VALUE("%d", e);
PRINTLN;

#ifdef LOW_POWER
    PRINT_CSTSTR("%s","Switch to power saving mode\n");
    e = sx1272.setSleepMode();
    if (!e)
      PRINT_CSTSTR("%s","Successfully switch LoRa module in sleep mode\n");
    else
      PRINT_CSTSTR("%s","Could not switch LoRa module in sleep mode\n");
    FLUSHOUTPUT
    delay(50);

    PRINT_VALUE("%d", i);
    delay(200);

    for (i; i<nCycle; i++) {
      counter1 = i;

      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      if (counter2 >= RechargeCycle){
        Recharge_Fnc();
        counter2 = 0;
      }
      PRINT_CSTSTR("%s",".");
      counter2 += 1;
      FLUSHOUTPUT;
      delay(10);

    }

#endif


  Serial.println(F("END of LOOP.."));

}//END MAIN LOOP


void detectionIRQ(void)
{
  delayMicroseconds(16383);
  delayMicroseconds(16383);
  flag1 = true;                 //an IRQ interrupt has been produced
  i = nCycle;                   // change the value of the sleep loop to exit just after the interrupt
//  Serial.println("IM HERE!!!!!!!!");
//  delayMicroseconds(16383);
//  delayMicroseconds(16383);

}

void Recharge_Fnc() {

    digitalWrite(DSWITCH, HIGH);  //Turns ON the digital switch
  //  Serial.println(F("Recharging.."));
    delay(1500);

    digitalWrite(DSWITCH,LOW);    //Turns OFF the digital switch

}


void Read_TH02(){
   // READ HUM AND TEMP (TH02)
   digitalWrite(THPWR, HIGH); // Turn on sensor power pin
   delay(150);                // Wait for sensor to initiate
   TH02.begin();
   temper   = TH02.ReadTemperature();
   humidity = TH02.ReadHumidity();
   //Serial.print(F("Air Temperature: ")); Serial.println(temper);
   //Serial.print(F("Air Humidity: ")); Serial.println(humidity);
   Wire.endTransmission(1);
   delay(150);
   digitalWrite(THPWR, LOW); // turn off sensor
   if (temper < -50.0) temper = -50.0;   // sensor neg min range
   if (humidity < 0.0) humidity = 0.0;   // can not be neg humidity

  }


  float checkBattery(){

    if (cycleCount == BATT_CYCLES) //only read battery every BATT_CYCLES sleep cycles
    {
      unsigned int reading=0;
      //Serial.println(" ");
      //enable battery monitor
      digitalWrite(BATT_MONITOR_EN, HIGH);
      delay(500);  // wait 500 ms to saturate the transistors
      for (byte i=0; i<10; i++)
        reading += analogRead(BATT_MONITOR);

      //disable battery monitor
      digitalWrite(BATT_MONITOR_EN, LOW);

      batteryVolts = (3.12*map((reading/10),0,1023,0,3300)/1000)+0.08; // 3.12 is the ratio between the resistors
      //Serial.print("battery voltage is : "); Serial.println(batteryVolts);
      //Serial.print("voltage is: "); Serial.println(map(batteryVolts, 0,1023,0,3300));

      cycleCount = 0;
      return batteryVolts;
    }
    else{
      cycleCount += 1;
      return batteryVolts;
    }
  }
