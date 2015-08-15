/*
MCP3424.h
Library for working with the MCP3424 ADC to I2C converter

written by jeroen cappaert (c) for nanosatisfi August 2012

Make an instance of MCP3423 by:
MCP3424 ADC1(I2Caddress, gain, resolution)
- gain values 0-3 represent {x1,x2,x4,x8}
- resolution 0-3 represents {12bits, 14bits, 16bits, 18bits}

User functions:
 - setChannel(channelnr.) - for channel 1-4, enter a value 0-1
 - getMvDivisor() - get the conversion value between the ADC value and mV
 - readData() - read data from the I2C channel
 - getChannelmV(channelnr.) - get data in mV directly from the I2C channel chosen by channelnr.

*/

//#ifndef MCP3424_h
//#define MCP3424_h

#include <Arduino.h>

#define MCP3424_ADDRESS 0X69

// Define configuration register bits and addresses
#define MCP342X_GAIN_X1    0X00 // PGA gain X1
#define MCP342X_GAIN_X2    0X01 // PGA gain X2
#define MCP342X_GAIN_X4    0X02 // PGA gain X4
#define MCP342X_GAIN_X8    0X03 // PGA gain X8

#define MCP342X_12_BIT     0X00 // 12-bit 240 SPS
#define MCP342X_14_BIT     0X01 // 14-bit 60 SPS
#define MCP342X_16_BIT     0X02 // 16-bit 15 SPS
//#define MCP342X_18_BIT     0X03 // 18-bit 3.75 SPS
// NOTE: Need to extend the measurement routine to support 18-bit operations
#define MCP342X_RES_FIELD  0x03 // Resolution bitfield

#define MCP342X_CONTINUOUS 0X10 // 1 = continuous, 0 = one-shot

#define MCP342X_CHANNEL(n)      (uint8_t)(((n) & 0x03) << 5)
#define MCP342X_RESOLUTION(n)   (uint8_t)(((n) & 0x03) << 2)
#define MCP342X_GAIN(n)         (uint8_t)(((n) & 0x03) << 0)

#define MCP342X_START      0X80 // write: start a conversion
#define MCP342X_BUSY       0X80 // read: output not ready

#define ERROR_NO_ERROR          0
#define ERROR_READ_TIMEOUT      1


class MCP3424
{
public:
    // @param address (input) I2C bus address of the device
    // @param gain (input) Measurement gain multiplier (1,2,4,8)
    // @param resolution (input) ADC resolution (0,1,2,4)
    MCP3424(uint8_t address, uint8_t gain, uint8_t resolution);

    // Initialize the MCP3424
    void begin();

    // Start a one-shot channel measurement
    // @param channel (input) Analog input to read (1-4)
    void startMeasurement(uint8_t channel);

    // Test if a measurement is ready. Use getMeasurement() to return
    // the value of the result.
    // @return True if measurement was ready, false otherwise
    bool measurementReady();

    // Get the result of the last channel measurement
    // @param value (output) Measured value of channel, in mV
    int getMeasurementUv();

private:
    // Update the configuration register on the MCP3424
    // @param message (output) byte to write to the 
    void MCP3424WriteConfig(uint8_t value);

    // Get the divisor for converting a measurement into mV
    int getMvDivisor();

    const int address;      // I2C bus address
    const uint8_t resolution;   // ADC resolution
    const uint8_t gain;         // Gain
    int value;                  // Last measurement result, in uV
};

//#endif

