/*
MCP3424.cpp
Library for using the mcp3424 adc to I2C converter
	
	
written by jeroen cappaert (c) for nanosatisfi August 2012

*/

#include <Arduino.h>
#include <Wire.h>
#include "MCP3424.h"

// constructor takes I2C address, required channel, gain and resolution and configures chip.
MCP3424::MCP3424(uint8_t address_, uint8_t gain_, uint8_t resolution_) :
    address(address_),
    resolution(resolution_),
    gain(gain_)
{ 
}

void MCP3424::begin() {
    // Configure the device for one-shot mode
    // Resolution and gain don't matter here.
    uint8_t adcConfig =
        MCP342X_RESOLUTION(resolution)
        | MCP342X_GAIN(gain);

    MCP3424WriteConfig(adcConfig);
}

// calculate and return mV devisor from gain and resolution.
int MCP3424::getMvDivisor()
{
    int mvDivisor = 1 << (gain + 2*resolution);
    return mvDivisor;
}

// Write byte to register on MCP3424
void MCP3424::MCP3424WriteConfig(uint8_t value)
{
    Wire.beginTransmission(address);
    Wire.write(value);
    Wire.endTransmission(); 
}

void MCP3424::startMeasurement(uint8_t channel) {
    uint8_t adcConfig =
        MCP342X_START
        | MCP342X_CHANNEL(channel)
        | MCP342X_RESOLUTION(resolution)
        | MCP342X_GAIN(gain);

    MCP3424WriteConfig(adcConfig);
}


bool MCP3424::measurementReady() {
    // Assume <18-bit mode. Note that this needs to handle 
    // 4-byte responses to work with 18-bit mode.
    int16_t rawValue = 0;

    // Ask for 3 bytes from the sensor
    Wire.requestFrom(address, 3);
    if (Wire.available() != 3) {
        return false;
    }

    // Read the first two bytes into the correct positions
    // for a uint16_t
    for (uint8_t i = 0; i < 2; i++) {
        rawValue = (rawValue << 8) | Wire.read();
    }

    // read config/status byte
    uint8_t status = Wire.read();

/*
    Serial.print("Reading: ");
    Serial.print(rawValue, HEX);
    Serial.print(" ");
    Serial.print(status, HEX);
    Serial.print("\n");
*/

    if (status & MCP342X_BUSY) {
        return false;
    }

    value = (double)rawValue/getMvDivisor();

    return true;
}

double MCP3424::getMeasurement() {
    return value;
}
