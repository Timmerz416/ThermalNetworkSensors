#include <Wire.h>
#include <XBee.h>
#include <AltSoftSerial.h>

// XBee variables
AltSoftSerial xbeeSerial;  // The software serial port for communicating with the Xbee (TX Pin 9, RX Pin 8)
XBee localRadio = XBee();  // The connection for the local coordinating radio

// XBee command codes
const uint8_t CMD_SENSOR_DATA = 4;

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
const uint8_t BATTERY_SOC_CODE = 10;

// Timing variables
const unsigned long SENSOR_DELAY = 600000;  // The period between temperature measurements (ms).

// Local pins
const int LOCAL_LUMINOSITY_PIN = A0;
const int LOCAL_POWER_PIN = A1;

// Create the instance of the pressure sensor
const int TempAddress = 0x48;

// Union for conversion of numbers to byte arrays
union FloatConverter {
  float f;
  uint8_t b[sizeof(float)];
};

//-------------------------------------------------------------------------
void setup(){
  // Start the hardware serial port
  Serial.begin(9600);
  Serial.println("Started Serial...");

  // Create the software serial port and connect the XBee
  Serial.print("Starting XBee Connection...");
  xbeeSerial.begin(9600);
  localRadio.setSerial(xbeeSerial);
  Serial.println("FINISHED");
  
  // Start the I2C bus
  Wire.begin();
  
  // Set the pins
//  analogReference(EXTERNAL);  // Use the 3.3V hooked up to the AREF pin
}

//-------------------------------------------------------------------------

void loop(){
  // Get a temperature reading
  FloatConverter Temperature;
  Temperature.f = getTemperature();
  
  // Get a power reading
  FloatConverter Power;
  int powerSignal = analogRead(LOCAL_POWER_PIN);
  Power.f = 5.0*powerSignal/1023.0;

  // Create the byte array to pass through the XBee
  size_t floatBytes = sizeof(float);
  uint8_t package[1 + 2*(1+floatBytes)];
  package[0] = CMD_SENSOR_DATA;
  package[1] = TEMPERATURE_CODE;
  package[1 + floatBytes+1] = POWER_CODE;
  for(int i = 0; i < floatBytes; i++) {
    package[i+2] = Temperature.b[i];
    package[(floatBytes+1)+(i+2)] = Power.b[i];
  }
  
  // Send the data package to the coordinator
  XBeeAddress64 address = XBeeAddress64(0x00000000, 0x00000000);
  ZBTxRequest zbTX = ZBTxRequest(address, package, sizeof(package));
  localRadio.send(zbTX);
  
  // Print message to the serial port
  Serial.print("Sent the following message (");
  Serial.print(Temperature.f);
  Serial.print(",");
  Serial.print(Power.f);
  Serial.print("): ");
  for(int i = 0; i < sizeof(package); i++) {
    if(i != 0) Serial.print("-");
    Serial.print(package[i], HEX);
  }
  Serial.println("");

  // Transmission delay
  delay(SENSOR_DELAY);
}

//-------------------------------------------------------------------------
float getTemperature(){
  // Make the temperature sensor take a reading
  Wire.requestFrom(TempAddress,2); 

  // Read the 16-bit data
  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //it's a 12 bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 

  // Calculate the temperature
  float Celsius = 0.0625*TemperatureSum;
  return Celsius;
}

