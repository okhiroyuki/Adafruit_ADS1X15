/**************************************************************************/
/*!
    @file     Adafruit_ADS1015.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    Driver for the ADS1015/ADS1115 ADC

    This is a library for the Adafruit MPL115A2 breakout
    ----> https://www.adafruit.com/products/???

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <I2C.h>
#include "Adafruit_ADS1015.h"

/**************************************************************************/
/*!
    @brief  Writes 16-bits to the specified destination register
*/
/**************************************************************************/
static void writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value) {
  char buf[2];
  buf[0] = value>>8;
  buf[1] = value & 0xFF;
  I2c.write(i2cAddress,(uint8_t)reg,buf);
}

/**************************************************************************/
/*!
    @brief  Writes 16-bits to the specified destination register
*/
/**************************************************************************/
static uint16_t readRegister(uint8_t i2cAddress, uint8_t reg) {
  I2c.read(i2cAddress,(uint8_t)ADS1015_REG_POINTER_CONVERT,(uint8_t)2);
  return((I2c.receive() << 8) | I2c.receive());
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADS1015 class w/appropriate properties
*/
/**************************************************************************/
Adafruit_ADS1015::Adafruit_ADS1015(uint8_t i2cAddress) 
{
   m_i2cAddress = i2cAddress;
   m_device = 0;
   m_conversionDelay = ADS1015_CONVERSIONDELAY;
   m_mode = ADS1015_REG_CONFIG_MODE_SINGLE;
   m_datarate = ADS1015_REG_CONFIG_DR_1600SPS; // 1600 samples per second (default)
   m_bitShift = 4;
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADS1115 class w/appropriate properties
*/
/**************************************************************************/
Adafruit_ADS1115::Adafruit_ADS1115(uint8_t i2cAddress)
{
   m_i2cAddress = i2cAddress;
   m_device = 1;
   m_conversionDelay = ADS1115_CONVERSIONDELAY;
   m_mode = ADS1015_REG_CONFIG_MODE_SINGLE;
   m_datarate = ADS1115_REG_CONFIG_DR_128SPS; // 1600 samples per second (default)
   m_bitShift = 0;
}

void Adafruit_ADS1015::setAddress(uint8_t i2cAddress)
{
  m_i2cAddress = i2cAddress;
}

void Adafruit_ADS1015::setContinuosMode()
{
  m_mode = ADS1015_REG_CONFIG_MODE_CONTIN;  
}

void Adafruit_ADS1015::setDataRate(int value)
{
  switch(value)
  {
    case 0:
      if(m_device == 0) m_datarate = ADS1015_REG_CONFIG_DR_128SPS;
      else              m_datarate = ADS1115_REG_CONFIG_DR_8SPS;
      break;
    case 1:
      if(m_device == 0) m_datarate = ADS1015_REG_CONFIG_DR_250SPS;
      else              m_datarate = ADS1115_REG_CONFIG_DR_16SPS;
      break;
    case 2:
      if(m_device == 0) m_datarate = ADS1015_REG_CONFIG_DR_490SPS;
      else              m_datarate = ADS1115_REG_CONFIG_DR_32SPS;
      break;
    case 3:
      if(m_device == 0) m_datarate = ADS1015_REG_CONFIG_DR_920SPS;
      else              m_datarate = ADS1115_REG_CONFIG_DR_64SPS;
      break;
    case 4:
      if(m_device == 0) m_datarate = ADS1015_REG_CONFIG_DR_1600SPS;
      else              m_datarate = ADS1115_REG_CONFIG_DR_128SPS;
      break;
    case 5:
      if(m_device == 0) m_datarate = ADS1015_REG_CONFIG_DR_2400SPS;
      else              m_datarate = ADS1115_REG_CONFIG_DR_250SPS;
      break;
    case 6:
      if(m_device == 0) m_datarate = ADS1015_REG_CONFIG_DR_3300SPS;
      else              m_datarate = ADS1115_REG_CONFIG_DR_475SPS;
      break;
    case 7:
      if(m_device == 0) m_datarate = ADS1015_REG_CONFIG_DR_3300SPS;
      else              m_datarate = ADS1115_REG_CONFIG_DR_860SPS;
      break;
  }
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
void Adafruit_ADS1015::begin() {
  I2c.begin();
  I2c.setSpeed(1);  //400Hz
}

/**************************************************************************/
/*!
    @brief  Gets a single-ended ADC reading from the specified channel
*/
/**************************************************************************/
uint16_t Adafruit_ADS1015::readADC_SingleEnded(uint8_t channel) {
  if (channel > 3)
  {
    return 0;
  }
  
  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   ; // Traditional comparator (default val)

  //Set Conversion mode
  config |= m_mode;

  //Set DataRate
  config |= m_datarate;

  // Set PGA/voltage range
  config |= ADS1015_REG_CONFIG_PGA_6_144V;            // +/- 6.144V range (limited to VDD +0.3V max!)

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set 'start single-conversion' bit
  if(m_mode == ADS1015_REG_CONFIG_MODE_SINGLE) config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  if(m_mode == ADS1015_REG_CONFIG_MODE_SINGLE) delay(m_conversionDelay);

  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
  return readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;  
}

/**************************************************************************/
/*! 
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN0) and N (AIN1) input.  Generates
            a signed value since the difference can be either
            positive or negative.
*/
/**************************************************************************/
int16_t Adafruit_ADS1015::readADC_Differential_0_1() {
  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   ; // Traditional comparator (default val)

  //Set Conversion mode
  config |= m_mode;

  //Set DataRate
  config |= m_datarate;

  // Set PGA/voltage range
  config |= ADS1015_REG_CONFIG_PGA_6_144V;            // +/- 6.144V range (limited to VDD +0.3V max!)

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_0_1;          // AIN0 = P, AIN1 = N

  // Set 'start single-conversion' bit
  if(m_mode == ADS1015_REG_CONFIG_MODE_SINGLE) config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  if(m_mode == ADS1015_REG_CONFIG_MODE_SINGLE) delay(m_conversionDelay);

  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
  return (int16_t)(readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift);  
}

/**************************************************************************/
/*! 
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN2) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative.
*/
/**************************************************************************/
int16_t Adafruit_ADS1015::readADC_Differential_2_3() {
  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   ; // Traditional comparator (default val)

  //Set Conversion mode
  config |= m_mode;

  //Set DataRate
  config |= m_datarate;

  // Set PGA/voltage range
  config |= ADS1015_REG_CONFIG_PGA_6_144V;            // +/- 6.144V range (limited to VDD +0.3V max!)

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_2_3;          // AIN2 = P, AIN3 = N

  // Set 'start single-conversion' bit
  if(m_mode == ADS1015_REG_CONFIG_MODE_SINGLE) config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  if(m_mode == ADS1015_REG_CONFIG_MODE_SINGLE) delay(m_conversionDelay);

  // Shift 12-bit results right 4 bits for the ADS1015
  return (int16_t)(readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift);  
}

/**************************************************************************/
/*!
    @brief  Sets up the comparator to operate in basic mode, causing the
            ALERT/RDY pin to assert (go from high to low) when the ADC
            value exceeds the specified threshold.

            This will also set the ADC in continuous conversion mode.
*/
/**************************************************************************/
void Adafruit_ADS1015::startComparator_SingleEnded(uint8_t channel, int16_t threshold)
{
  uint16_t value;

  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_1CONV   | // Comparator enabled and asserts on 1 match
                    ADS1015_REG_CONFIG_CLAT_LATCH   | // Latching mode
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_MODE_CONTIN  | // Continuous conversion mode
                    ADS1015_REG_CONFIG_PGA_6_144V   | // +/- 6.144V range (limited to VDD +0.3V max!)
                    ADS1015_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  //Set DataRate
  config |= m_datarate;

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1015
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_HITHRESH, threshold << m_bitShift);

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);
}

/**************************************************************************/
/*!
    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value.
*/
/**************************************************************************/
int16_t Adafruit_ADS1015::getLastConversionResults()
{
  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
  return (int16_t)(readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift);
}

