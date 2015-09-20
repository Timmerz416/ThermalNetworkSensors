#include <Wire.h>
#include <XBee.h>
#include <AltSoftSerial.h>
#include <limits.h>

#include "MPL3115A2.h"
#include "LumSensor.h"

// XBee variables
AltSoftSerial xbeeSerial;  // The software serial port for communicating with the Xbee (TX Pin 9, RX Pin 8)
XBee localRadio = XBee();  // The connection for the local coordinating radio

// XBee data codes
const uint8_t TEMPERATURE_CODE = 1;
const uint8_t LUMINOSITY_CODE = 2;
const uint8_t PRESSURE_CODE = 3;
const uint8_t HUMIDITY_CODE = 4;
const uint8_t POWER_CODE = 5;
const uint8_t LUX_CODE = 6;
const uint8_t HEATING_CODE = 7;
const uint8_t THERMOSTAT_CODE = 8;
const uint8_t TEMP_12BYTE_CODE = 9;

// Determine the maximum unsigned long
const unsigned long MAX_LONG = ULONG_MAX;

// Timing variables
const unsigned long TEMP_PERIOD = 600000;  // The period between temperature measurements (ms).
unsigned long prevTime = 0;  // Tracks the current time for determining when to update the local temperature
unsigned long curTime = 0;  // Track the time for the last local temperature update

// Create the instance of the luminosity sensor
AutoLightSensor luxSensor;

// Create the instance of the pressure sensor
MPL3115A2 pressureSensor;

// Union for conversion of numbers to byte arrays
union FloatConverter {
  float f;
  uint8_t b[sizeof(float)];
};

//-------------------------------------------------------------------------
void setup(){
  // Create the software serial port and connect the XBee
  xbeeSerial.begin(9600);
  localRadio.setSerial(xbeeSerial);
  
  // Start the hardware serial port
  Serial.begin(9600);
  
  // Start the I2C bus through the lux sensor
  luxSensor.begin();
  
  // Set the pressure sensor modes
  pressureSensor.begin();
  pressureSensor.setModeBarometer();
  pressureSensor.setOversampleRate(7);
  pressureSensor.enableEventFlags();
}

//-------------------------------------------------------------------------

void loop(){
  // Determine the delta time since the last local temperature measurement
  curTime = millis();
  unsigned long diffTime = (curTime > prevTime) ? curTime - prevTime : (MAX_LONG - prevTime) + curTime + 1;

  if(diffTime >= TEMP_PERIOD) {
    // Get a temperature reading
    FloatConverter Temperature;
    Temperature.f = pressureSensor.readTemp();
    
    // Get the pressure reading
    FloatConverter Pressure;
    Pressure.f = pressureSensor.readPressure();

	// Get the luminosity reading
    FloatConverter Luminosity;
    float luminosity = luxSensor.getLuminosity();
	Luminosity.f = luminosity;
    
	// Set the power
	FloatConverter Power;
	Power.f = 5.0;
	  
    // Create the byte array to pass through the XBee
    size_t floatBytes = sizeof(float);
    uint8_t package[4*(1+floatBytes)];
    package[0] = TEMPERATURE_CODE;
    package[floatBytes+1] = LUX_CODE;
    package[2*(floatBytes+1)] = PRESSURE_CODE;
    package[3*(floatBytes+1)] = POWER_CODE;
    for(int i = 0; i < floatBytes; i++) {
      package[i+1] = Temperature.b[i];
      package[(floatBytes+1)+(i+1)] = Luminosity.b[i];
      package[2*(floatBytes+1)+(i+1)] = Pressure.b[i];
      package[3*(floatBytes+1)+(i+1)] = Power.b[i];
    }
    
    // Send the data package to the coordinator
    XBeeAddress64 address = XBeeAddress64(0x00000000, 0x00000000);
    ZBTxRequest zbTX = ZBTxRequest(address, package, sizeof(package));
    localRadio.send(zbTX);
    
    // Print message to the serial port
    Serial.print("Sent the following message (");
    Serial.print(Temperature.f);
    Serial.print(",");
    Serial.print(Luminosity.f);
    Serial.print(",");
    Serial.print(Pressure.f);
    Serial.print(",");
    Serial.print(Power.f);
    Serial.print("): ");
    for(int i = 0; i < sizeof(package); i++) {
      if(i != 0) Serial.print("-");
      Serial.print(package[i], HEX);
    }
    Serial.println("");
    
    // Update the time tracker
    prevTime = curTime;
  }
}
