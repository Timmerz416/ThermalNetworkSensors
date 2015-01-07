// Living Room Controller
//
// This code is for a thermostat relay that gets its signals to turn on and off from a xbee.

#include <Wire.h>
#include <XBee.h>
#include <AltSoftSerial.h>
#include <limits.h>

#include "DSClock.h"
#include "LumSensor.h"
#include "HTU21D.h"

// XBee variables
AltSoftSerial xbeeSerial;
XBee xbee = XBee();
ZBTxRequest xData = ZBTxRequest();

// XBee data codes
const uint8_t TEMPERATURE_CODE = 1;
const uint8_t LUMINOSITY_CODE = 2;
const uint8_t PRESSURE_CODE = 4;
const uint8_t HUMIDITY_CODE = 8;
const uint8_t POWER_CODE = 16;
const uint8_t LUX_CODE = 32;
const uint8_t HEATING_CODE = 64;
const uint8_t THERMOSTAT_CODE = 128;

// Thermostat switch constants
boolean thermoOn = false;    // Keeps track of whether the thermostat is on or off
boolean relayOn = false;    // Keeps track of whether the relay is on or off
const int RELAY_ON = 6;      // Digital pin to turn on the relay
const int RELAY_OFF = 7;     // Digital pint to turn off the relay
const int RELAY_DELAY = 10;  // Time to cycle the relay switch
const float MIN_TEMPERATURE = 16.0;  // Below this, the thermostat turns on despite what is programmed
const float MAX_TEMPERATURE = 25.0;  // Above this, the thermostat turns off despite what is programmed

// Thermostat rule variables
unsigned char num_rules;	// To keep track of the number of rules created
TemperatureRule rules[10];	// Allow the user up to 10 rules
const int RELAY_SIGNAL_PIN = 5;	// The digital pin indicating relay status (high = on; low = off)
const float bufferLimit = 0.5;  // The range on the target temperature used for buffering

// LiPo Shield parameters
float supplyVoltage = 5.0;  // Tracks the voltage on the battery/power supply

// Determine the maximum unsigned long
const unsigned long MAX_LONG = ULONG_MAX;

// Timing variables
const unsigned long CONTROL_PERIOD = 300000;  // The period between resetting the relay (ms).
const unsigned long SENSOR_PERIODS = 2;  // The number of control periods per sensor period.
unsigned long prevTime = 0;  // Tracks the current time for determining when to update the local temperature
unsigned long curTime = 0;  // Track the time for the last local temperature update
byte curLoop = 0;  // Tracks the number of loops without sending a sensor measurement

// Local analog pins
const int POWER_STATE_PIN = A0;  // Tracks whether the controller is on (high) or off (low)
//const int LOCAL_POWER_PIN = A1;       // Not currently used

// Set the I2C Addresses
const int POWER_ADDRESS = 0x36;

// Electronic components
DSClock clock;
AutoLightSensor luxSensor;
HTU21D tempSensor;

// Union for conversion of numbers to byte arrays
union FloatConverter {
  float f;
  uint8_t b[sizeof(float)];
};

void setup() {
  // Set the digital pins for relay control (initially turn relay off)
  pinMode(RELAY_ON, OUTPUT);
  pinMode(RELAY_OFF, OUTPUT);
  pinMode(RELAY_SIGNAL_PIN, OUTPUT);
  digitalWrite(RELAY_ON, LOW);
  digitalWrite(RELAY_OFF, HIGH);
  delay(RELAY_DELAY);
  digitalWrite(RELAY_OFF, LOW);
  digitalWrite(RELAY_SIGNAL_PIN, LOW);

  // Setup the xbee to read from the software serial port
  xbeeSerial.begin(9600);
  xbee.setSerial(xbeeSerial);

  // Create default rules
  num_rules = 6;
  rules[0].setRule(Weekdays, 23.5, 19.0);
  rules[1].setRule(Weekdays, 16.5, 22.0);
  rules[2].setRule(Weekdays, 9.0, 18.0);
  rules[3].setRule(Weekdays, 7.0, 22.0);
  rules[4].setRule(Weekends, 23.5, 19.0);
  rules[5].setRule(Weekends, 7.5, 22.0);

  // Start the wire interface through the luminosity sensor
  luxSensor.begin();

  // Start the serial port
  Serial.begin(9600);
  Serial.println("Starting Thermostat Controller");
}

void loop() {
  //-------------------------------------------------------------------
  // TIMING CALCULATION AND TEMPERATURE READING
  //-------------------------------------------------------------------
  // Determine the delta time since the last local temperature measurement
  curTime = millis();
  unsigned long diffTime = (curTime > prevTime) ? curTime - prevTime : (MAX_LONG - prevTime) + curTime + 1;

  // Read the clock and take a temperature measurement
  clock.readClock();
  float temperature = tempSensor.readTemperature();

  //-------------------------------------------------------------------
  // THERMOSTAT CONTROL CODE
  //-------------------------------------------------------------------
  // Get thermostat power level in volts (5 V = on; 0 V = off)
  float power_level = 5.0*analogRead(POWER_STATE_PIN)/1023;

  // Evaluate the relay control based on current thermostat power level
  if((power_level > 2.0) && !thermoOn) {  // Turn on the thermostat if previously off, check to see if relay should be on or off
    thermoOn = true;
    setRelay(false);	// Turn off the relay be default, ensuring it will only come on if below the rule temperature
    Serial.print("Thermostat turned ON at ");  formatDateTime();  Serial.print("; ");
    evaluateRules(temperature, true);  // Evaluate relay status based on rules and temperature, this calls sendXbeeMessage
    Serial.println();
  } else if((power_level < 2.0) && thermoOn) {  // Indicate that thermostat is off, and open relay
    thermoOn = false;
    Serial.print("Thermostat turned OFF at ");  formatDateTime();  Serial.println();
    setRelay(true);  // Open the relay
    sendXbeeMessage(temperature);	// Dispatch change of thermostat & relay state
  }

  //-------------------------------------------------------------------
  // SENSOR MEASUREMENT CODE
  //-------------------------------------------------------------------
  if(diffTime >= CONTROL_PERIOD) {
    // Print time and temperature to the serial port
    Serial.print("At ");
    formatDateTime();
    Serial.print(" - Temperature = ");
    Serial.print(temperature);

    //-------------------------------------------------------------------
    // RELAY CONTROL EVALUATION
    //-------------------------------------------------------------------
	// Only perform this evaluation is the thermostat is turned on
	if(thermoOn) {
		// Check to see if the temperature limits have been reached
		if(temperature < MIN_TEMPERATURE) {  // Temperature below limit
		  setRelay(true);	// Turn on relay
		  sendXbeeMessage(temperature);	// Dispatch change of relay state

		  Serial.println("; Relay turned ON due to low temperature");
		} 
		else if(temperature >= MAX_TEMPERATURE) {  // Temperature above limit
		  setRelay(false);	// Turn off relay
		  sendXbeeMessage(temperature);	// Dispatch change of relay state

		  Serial.print("; Relay turned OFF due to high temperature");
		} 
		else {  // Temperature within limits, evaluate status based on rules if thermostat is on
		  Serial.print("; ");
		  evaluateRules(temperature, false);	// Determine relay state based on rules
		}
		Serial.println();  // Move to new line
	}

    //-------------------------------------------------------------------
    // Data logging evaluation
    //-------------------------------------------------------------------
    ++curLoop;  // Increment the loop counter
    if(curLoop == SENSOR_PERIODS) {
      // Send the data through the XBee
      sendXbeeMessage(temperature);
      
      // Reset the loop counter
      curLoop = 0;
    }

    // Update the time tracker
    prevTime = curTime;
  }

  // Delay for a period of time to lower power usage
  // delay(CONTROL_PERIOD/10);
}

//-------------------------------------------------------------------
// Function to output datetime
//-------------------------------------------------------------------
void formatDateTime() {
  Serial.print(clock.year);
  Serial.print('-');
  Serial.print(clock.month);
  Serial.print('-');
  Serial.print(clock.day);
  Serial.print(' ');
  Serial.print(clock.hour);
  Serial.print(':');
  Serial.print(clock.minute);
  Serial.print(':');
  Serial.print(clock.second);
}


//-------------------------------------------------------------------
// RELAY HARDWARE CONTROL FUNCTION
//-------------------------------------------------------------------
// 
void setRelay(boolean turnOn) {
  if(turnOn == true && relayOn == false) {  // Turn on relay when it's off
    // Turn on thermostat
    digitalWrite(RELAY_ON, HIGH);
    delay(RELAY_DELAY);
    digitalWrite(RELAY_ON, LOW);
    digitalWrite(RELAY_SIGNAL_PIN, HIGH);

    relayOn = true;  // Set master flag
  } 
  else if(turnOn == false && relayOn == true) {  // Turn off relay when it's on
    // Turn off thermostat
    digitalWrite(RELAY_OFF, HIGH);
    delay(RELAY_DELAY);
    digitalWrite(RELAY_OFF, LOW);
    digitalWrite(RELAY_SIGNAL_PIN, LOW);

    relayOn = false;	// Set master flag
  }
}

//-------------------------------------------------------------------
// THERMOSTAT PROGRAMMING LOGIC FUNCTION
//-------------------------------------------------------------------
void evaluateRules(float cur_temperature, boolean forceXbee) {
  // Determine the weekday and decimal time from the clock
  DayType cur_day = static_cast<DayType>(clock.weekDay);
  float cur_time = (float) clock.hour + (float) clock.minute/60.0;
  bool rule_found = false;	// Indicates that the rule was found

  // Print out the current time being evaluated
  Serial.print("Evaluating day ");
  Serial.print(cur_day);
  Serial.print(" at ");
  Serial.print(cur_time);
  Serial.print(" hours - ");

  // Iterate through the rules
  while(!rule_found) {
    // Run through the individual rules to see if one applies
    for(register int i = 0; i < num_rules; i++) {
      // Check to see if the rule applies
      if(applyRule(rules[i], cur_day, cur_time)) {
        // Control the relay, as necessary
        if((cur_temperature > (rules[i].temperature + bufferLimit)) && relayOn) {  // Temperature higher than rule+buffer and relay on
          setRelay(false);	// Turn off the relay
          sendXbeeMessage(cur_temperature);	// Dispatch the change to the relay status

          // Debugging output to the serial port
          Serial.print("Relay turned OFF since temperature (");
          Serial.print(cur_temperature);
          Serial.print(") is greater than rule temperature (");
          Serial.print(rules[i].temperature);
          Serial.print(") plus buffer - ");
        } 
        else if((cur_temperature < (rules[i].temperature - bufferLimit)) && !relayOn) {  // Temperature lower than rule-buffer and relay off
          setRelay(true);	// Turn on the relay
          sendXbeeMessage(cur_temperature);	// Dispatch the change to the relay status

          // Debugging output to the serial port
          Serial.print("Relay turned ON since temperature (");
          Serial.print(cur_temperature);
          Serial.print(") is less than rule temperature (");
          Serial.print(rules[i].temperature);
          Serial.print(") minus buffer - ");
        }
        else {  // No change to relay status, so check to see if XBee update is forced and output debug info
          if(forceXbee) sendXbeeMessage(cur_temperature);

          // Debugging output to the serial port
          Serial.print("Relay is ");
          Serial.print(relayOn ? "ON" : "OFF");
        }

        rule_found = true;	// Indicate that a matching rule has been found
        break;	// Break from the for loop
      }
    }

    // If a rule was not found, need to check against the previous day
    if(!rule_found) {
      // Decrease the day, but add the day to the time
      if(cur_day == Sunday) cur_day = Saturday;
      else cur_day = static_cast<DayType>(cur_day - 1);
      cur_time += 24.0;
    }
  }
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
bool applyRule(TemperatureRule rule, DayType check_day, float check_time) {
  // First, check that the time is later than the rule time
  if(check_time >= rule.time) {
    // The specific day doesn't matter
    if(rule.days == Everyday) return true;

    // A specific rule day has been met
    if(check_day == rule.days) return true;

    // The rule applies to weekdays and this is met
    if((rule.days == Weekdays) && ((check_day >= Monday) && (check_day <= Friday))) return true;

    // The rule applies to weekends and this is met
    if((rule.days == Weekends) && ((check_day == Saturday) || (check_day == Sunday))) return true;
  }

  // By default, the rule does not apply and return false
  return false;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
void sendXbeeMessage(float measured_temp) {
    // Get humidity and temperature readings
    float luminosity = luxSensor.getLuminosity();
    float humidity = tempSensor.readHumidity();

    // Print measurements to the screen
    Serial.print("Measurements for Luminosity = ");
    Serial.print(luminosity);
    Serial.print(", Humidity = ");
    Serial.print(humidity);
    Serial.println("%");

    // Create the conversion unions
    FloatConverter tempConv, lumConv, humConv, powConv, heatingConv, thermoConv;
    tempConv.f = measured_temp;
    lumConv.f = luminosity;
    humConv.f = humidity;
    powConv.f = supplyVoltage;
    heatingConv.f = relayOn ? 1.0 : 0.0;
	thermoConv.f = thermoOn ? 1.0 : 0.0;

    // Create the byte array to pass through the XBee
    size_t floatBytes = sizeof(float);
    uint8_t package[6*(1+floatBytes)];
    package[0] = TEMPERATURE_CODE;
    package[floatBytes+1] = LUX_CODE;
    package[2*(floatBytes+1)] = HUMIDITY_CODE;
    package[3*(floatBytes+1)] = POWER_CODE;
    package[4*(floatBytes+1)] = HEATING_CODE;
	package[5*(floatBytes+1)] = THERMOSTAT_CODE;
    for(int i = 0; i < floatBytes; i++) {
      package[i+1] = tempConv.b[i];
      package[(floatBytes+1)+(i+1)] = lumConv.b[i];
      package[2*(floatBytes+1)+(i+1)] = humConv.b[i];
      package[3*(floatBytes+1)+(i+1)] = powConv.b[i];
      package[4*(floatBytes+1)+(i+1)] = heatingConv.b[i];
	  package[5*(floatBytes+1)+(i+1)] = thermoConv.b[i];
    }

    // Send the data package to the coordinator
    XBeeAddress64 address = XBeeAddress64(0x00000000, 0x00000000);
    ZBTxRequest zbTX = ZBTxRequest(address, package, sizeof(package));
    xbee.send(zbTX);

    // Print message to the serial port
    Serial.print("Sent the following message (");
    Serial.print(tempConv.f);
    Serial.print(",");
    Serial.print(lumConv.f);
    Serial.print(",");
    Serial.print(humConv.f);
    Serial.print(",");
    Serial.print(powConv.f);
    Serial.print(",");
    Serial.print(heatingConv.f);
	Serial.print(",");
	Serial.print(thermoConv.f);
    Serial.print("): ");
    for(int i = 0; i < sizeof(package); i++) {
      if(i != 0) Serial.print("-");
      Serial.print(package[i], HEX);
    }
    Serial.println("");
}

/*
//-------------------------------------------------------------------
 // Listen for an XBee transmission
 //-------------------------------------------------------------------
 xbee.readPacket(500);
 
 // Check to see if there was something interesting
 if(xbee.getResponse().isAvailable()) {
 if(xbee.getResponse().getApiId() == ZB_TX_REQUEST) {  // Check for a TX
 uint8_t *payload = xbee.getResponse().getFrameData();  // The data itself
 
 // Get and convert the data
 FloatConverter data;
 data.b[0] = payload[13];
 data.b[1] = payload[14];
 data.b[2] = payload[15];
 data.b[3] = payload[16];
 
 Serial.print("Received transmission to turn ");
 
 // Operate the switch
 if(data.f > 0.0) {
 digitalWrite(RELAY_ON, HIGH);
 delay(RELAY_DELAY);
 digitalWrite(RELAY_ON, LOW);
 Serial.print("ON");
 } else {
 digitalWrite(RELAY_OFF, HIGH);
 delay(RELAY_DELAY);
 digitalWrite(RELAY_OFF, LOW);
 Serial.print("OFF");
 }
 Serial.println(" the relay.");
 }
 }
 */


