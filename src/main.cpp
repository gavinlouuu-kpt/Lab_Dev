#include "Arduino.h"
#include "PCF8574.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_BusIO_Register.h>

// Create two instances of Adafruit_ADS1115 with different addresses
Adafruit_ADS1115 ads1;  /* First ADS1115 at address 0x48 */
Adafruit_ADS1115 ads2;  /* Second ADS1115 at address 0x49 */

// Set i2c address
PCF8574 pcf8574_25(0x25);
PCF8574 pcf8574_27(0x27);

unsigned long lastPCF8574Toggle = 0;   // Variable to track the last time the PCF8574 was toggled
bool pcf8574State = HIGH;              // Variable to track the current state of the PCF8574


void setup()
{
	Serial.begin(115200);
	delay(1000);
  

	// Set pinMode to OUTPUT
	pcf8574_25.pinMode(P1, OUTPUT);

  pcf8574_27.pinMode(P0, OUTPUT);
  pcf8574_27.pinMode(P1, OUTPUT);
  pcf8574_27.pinMode(P2, OUTPUT);
  pcf8574_27.pinMode(P3, OUTPUT);
  pcf8574_27.pinMode(P4, OUTPUT);
  pcf8574_27.pinMode(P5, OUTPUT);

  pinMode(25,OUTPUT);
  digitalWrite(25,HIGH);

	Serial.print("Init pcf8574...");
	if (pcf8574_25.begin()){
		Serial.println("OK");
	}else{
		Serial.println("KO");
	}
	if (pcf8574_27.begin()){
		Serial.println("OK");
	}else{
		Serial.println("KO");
	}

  pcf8574_27.digitalWrite(P0,LOW);
  pcf8574_27.digitalWrite(P1,LOW);
  pcf8574_27.digitalWrite(P2,LOW);
  pcf8574_27.digitalWrite(P3,LOW);
  pcf8574_27.digitalWrite(P4,LOW);
  pcf8574_27.digitalWrite(P5,LOW);

  ads1.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  if (!ads1.begin(0x48)) {
    Serial.println("Failed to initialize ADS at 0x48.");
    while (1);
  }
  ads2.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  if (!ads2.begin(0x49)) {
    Serial.println("Failed to initialize ADS at 0x49.");
    while (1);
  }
}

void loop()
{

  // ------------------------------------------------------------------------------------
  // ADS1115 testing zone
  // ------------------------------------------------------------------------------------

  int16_t adc0_1, adc1_1, adc2_1, adc3_1, adc0_2, adc1_2, adc2_2, adc3_2;

  adc0_1 = ads1.readADC_SingleEnded(0);
  adc1_1 = ads1.readADC_SingleEnded(1);
  adc2_1 = ads1.readADC_SingleEnded(2);
  adc3_1 = ads1.readADC_SingleEnded(3);

  adc0_2 = ads2.readADC_SingleEnded(0);
  adc1_2 = ads2.readADC_SingleEnded(1);
  adc2_2 = ads2.readADC_SingleEnded(2);
  adc3_2 = ads2.readADC_SingleEnded(3);

  Serial.print(adc0_1); Serial.print(",");
  Serial.print(adc1_1); Serial.print(",");
  Serial.print(adc2_1); Serial.print(",");
  Serial.print(adc3_1); Serial.print(",");
  Serial.print(0); Serial.print(",");
  Serial.print(adc0_2); Serial.print(",");
  Serial.print(adc1_2); Serial.print(",");
  Serial.print(adc2_2); Serial.print(",");
  Serial.println(adc3_2);

  // delay(500);

  // ------------------------------------------------------------------------------------
  // ADS1115 testing zone
  // ------------------------------------------------------------------------------------



  // ------------------------------------------------------------------------------------
  // PCF8574 testing zone
  // ------------------------------------------------------------------------------------

	// pcf8574_25.digitalWrite(P1, HIGH); // I2C driven GPIO control for solenoid valve on off, needs 1K pull up resistor at output 
	// delay(1000);
	// pcf8574.digitalWrite(P1, LOW);
	// delay(1000);

  // Non-blocking control for PCF8574 using millis()
  unsigned long currentMillis = millis();

  if (currentMillis - lastPCF8574Toggle >= 1000) {  // Check if 1 second has passed
    pcf8574_25.digitalWrite(P1, pcf8574State);     // Set the PCF8574 state
    pcf8574State = !pcf8574State;                  // Toggle the state for the next loop
    lastPCF8574Toggle = currentMillis;             // Update the last toggle time
  }
  // ------------------------------------------------------------------------------------
  // PCF8574 testing zone
  // ------------------------------------------------------------------------------------

  // ------------------------------------------------------------------------------------
  // DC-DC output test zone
  // ------------------------------------------------------------------------------------  



}
