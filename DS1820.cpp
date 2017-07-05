/*
DS1820 Arduino Library
*/

#include "DS1820.h"

/** 
  * @mainpage
  * This is an Arduino-compatible library
  * for use with the MAXIM DS18x20 Digital Thermometers.\n
  * DS18S20\n
  * DS18B20\n
  * For example,
  * <a href="http://www.adafruit.com/products/642">Adafruit High Temp Waterproof DS18B20 Digital temperature sensor</a>
  * 
  * Inherits the OneWire library.\n
  * The latest version of this library may be found at:
  * http://www.pjrc.com/teensy/td_libs_OneWire.html\n
  * DS1820 was tested with OneWire version 2.1.\n
  * Many thanks to all who contributed to OneWire!
  * 
  * Read the datasheets! All will be made clear.\n
  * DS18S20:
  * http://www.maxim-ic.com/datasheet/index.mvp/id/2815\n
  * DS18B20:
  * http://www.maxim-ic.com/datasheet/index.mvp/id/2812\n
  * App Note comparing the two:
  * http://www.maxim-ic.com/app-notes/index.mvp/id/4377\n
  * Thank you to the folks at MAXIM!
  * 
  * Note regarding read/write operations...\n
  * After a reset, a ROM address for the desired device
  * on the 1-Wire bus or a skip command (to access all 
  * devices) must be sent before executing most commands.
  * The addressing is valid until the next reset.
  * Currently, addressing is skipped.
  *
  * ---------------------------------------------
  * DS1820 version 0.2\n
  * 2012.07.26\n
  * Added calcTemperature() and calcTemperatureHiRes()
  * methods. These split the calculation and convert
  * operations in the getTemperature() methods.\n
  * Added wait/no wait argument to convertTemperature()
  * (defaults to wait).
  *
  * DS1820 version 0.1\n
  * 2012.06.30\n
  * Assumes only one device on the bus.\n
  * No support for ROM-addressing multiple devices.\n
  * No CRC comparisons are made.\n
  * No support for parasite power.\n
  * (The OneWire library provides basic support for ROM addressing,
  * parasite power and CRCs.)
  * 
  * ---------------------------------------------
  * Michael Damkier\n
  * Hamburg, Germany\n
  * (michael@vondervotteimittiss.com)
  * 
  * You are free to use this as is, change it, whatever.
  * You may cut-and-paste without attribution.
  * However, I make no guarantee that this will work and
  * I am not responsible for any consequences resulting
  * from its use.
  */

/**
  * @param pinDQ The Arduino pin for the device 1-Wire communication (DQ) pin.
  * (Initially set as an input in the OneWire constructor.)
  */
DS1820::DS1820( uint8_t pinDQ )
  : OneWire( pinDQ )
{
  // set device family by reading the ROM
  reset();
  write( CMD_READ_ROM );
  _deviceFamily = read();
  // read the scratchpad contents into this instance
  readScratchpad();
}

/** 
  * The other constructor will set the device family by reading the device ROM
  * and is the preferred one.
  * 
  * @param pinDQ The Arduino pin for the device 1-Wire communication (DQ) pin.
  * (Initially set as an input in the OneWire constructor.)
  * 
  * @param deviceFamily The DS18x20 device family code.
  * (0x10 for DS18S20, 0x28 for DS18B20) 
  */
DS1820::DS1820( uint8_t pinDQ, uint8_t deviceFamily )
  : OneWire( pinDQ ), _deviceFamily( deviceFamily )
{
  // read the scratchpad contents into this instance
  readScratchpad();
}

/**
  * Directs the device to do a temperature measurement
  * The result is stored in the device scratchpad.
  * Use readScratchpad() to get the device scratchpad into
  * this instance and readTemp() to get the temperature value.
  *
  * For the B_FAMILY, the time it takes to do a conversion
  * varies based on the resolution setting (~610mS for 12-bit
  * and ~85mS for 9-bit resolution).
  *
  * For the S_FAMILY, the time can be up to one second.
  *
  * @param waitForIt Defaults to true to wait for the conversion
  * before returning. If false, the convert command is sent to the
  * device and the method immdeiately returns. It is then up to
  * application to wait an appropriate time before reading the
  * device scratchpad.
  */
void DS1820::convertTemperature( boolean waitForIt )
{
  reset();
  skip();
  write( CMD_CONVERT_T );
  if ( waitForIt ) while ( ! read_bit() ); // read-slot returns 1 when done
}

/**
  * Reads the device scratchpad to the instance _scratchpad
  * (readScratchpad()) and then reads the temperature value
  * (readTemp()). Calculates a lo-res temperature value
  * (9-bit).
  *
  * Returns the temp as an integer with 0 or 5 as the
  * least significant digit. For example, 19.0 is 190,
  * 21.5 is 215, etc.
  */
int16_t DS1820::calcTemperature()
{
  readScratchpad();
  int16_t temp_read = readTemp();
  // For B_FAMILY, the resolution is 12-bits, for S_FAMILY,
  // 9-bits, so lose the extra bits if B.
  if ( _deviceFamily == B_FAMILY )
    temp_read >>= 3;
      // the shift operator, above, depends on it being an arithmetic right shift
      // if the compiler does a logical right shift this will break for negative temperatures
  return temp_read * 5;
    // think of temp_read as being the number of half-degrees. So, multiplying
    // by 5 gives the desired 'decimal' value, i.e., ending in 0 or 5 
}

/**
  * Reads the device scratchpad to the instance _scratchpad
  * (readScratchpad()) and then reads the temperature value
  * (readTemp()). Calculates a hi-res temperature value.
  *
  * Returns the temp as a floating-point value. For B_FAMILY devices,
  * the resolution is determined by the resolution setting.
  * Refer to setResolution(). S_Family devices use values in the count registers
  * to calculate additional resolution. Refer to the datasheets for info.
  */
float DS1820::calcTemperatureHiRes()
{
  readScratchpad();
  int16_t temp_read = readTemp();
  float temp = 0.0;
  if ( _deviceFamily == S_FAMILY )
  {
    // refer to the DS18S20 Datasheet for the S_FAMILY HiRes algorithm
    float count_remain = _scratchpad[ BYTE_COUNT_REMAIN ];
    float count_per_degree = _scratchpad[ BYTE_COUNT_PER_C ];
    temp = ((float) temp_read) / 2 - 0.25 + ((count_per_degree - count_remain) / count_per_degree);
  }
  else if ( _deviceFamily == B_FAMILY )
  {
    // for resolutions < 12 the applicable lower-order bits are undefined...
    uint8_t shift = 12 - readResolution();
    // ...so shift everything to get the right answer
    temp = ((float) (temp_read >> shift)) / (16 >> shift);
      // the shift operator, above, depends on it being an arithmetic right shift
      // if the compiler does a logical right shift this will break for negative temperatures
  }
  return temp;
}

/**
  * Get the temperature to .5 degree C resolution.
  * Initiates a temperature conversion by calling
  * convertTemperature() and calculates the lo-res
  * value with calcTemperature().
  *
  * For a high-resolution temperature reading use
  * getTemperatureHiRes().
  *
  * (If you only use this routine and don't need
  * high-resolution, consider reducing the resolution
  * of B_FAMILY devices with setResolution().)
  */
int16_t DS1820::getTemperature()
{
  convertTemperature();
  return calcTemperature();
}

/**
  * Perform a hi-resolution temperature read.
  * Initiates a temperature conversion by calling
  * convertTemperature() and calculates the hi-res
  * value with calcTemperatureHiRes().
  */
float DS1820::getTemperatureHiRes()
{
  convertTemperature();
  return calcTemperatureHiRes();
}

/**
  * Writes the high and low alarm limits to the
  * device scratchpad (and instance _scratchpad).
  * This does not write the new values to EEPROM.
  * To update the EEPROM with the new values,
  * call updateEEPROM().
  * @param upperLimit,lowerLimit The limits are signed byte (8-bit) values, they
  * don't have the decimal bits.
  */
void DS1820::setAlarmLimits( int8_t upperLimit, int8_t lowerLimit )
{
  reset();
  skip();
  write( CMD_WRITE_SCRATCHPAD );
  write( upperLimit );
  write( lowerLimit );
  if ( _deviceFamily == B_FAMILY ) write( _scratchpad[ BYTE_CONFIG_REGISTER ] );
  _scratchpad[ BYTE_ALARM_TH ] = upperLimit;
  _scratchpad[ BYTE_ALARM_TL ] = lowerLimit;
}

/**
  * Writes the resolution for HiRes temperature
  * conversions to the device scratchpad
  * (and instance _scratchpad).
  * This only applies to the B_FAMILY.
  * This does not write the new value to EEPROM.
  * To update the EEPROM with the new value,
  * call updateEEPROM().
  * @param resolution The resolution value is 9, 10, 11 or 12
  * referring to the number of bits in the temperature result.
  * (The integral temperature is the 8 MSBits. So, 9 is .5 deg.
  * resolution, 10 is .25, 11 .125 and 12 .0625 deg,)
  */
void DS1820::setResolution( uint8_t resolution )
{
  if ( _deviceFamily != B_FAMILY ) return;
  uint8_t configByte = (resolution - 9) << 5;
  reset();
  skip();
  write( CMD_WRITE_SCRATCHPAD );
  write( _scratchpad[ BYTE_ALARM_TH ] ); // must first write the two
  write( _scratchpad[ BYTE_ALARM_TL ] ); // alarm limit values
  write( configByte );
  _scratchpad[ BYTE_CONFIG_REGISTER ] = configByte;
}

/**
  * Reads the resolution from the instance _scratchpad.
  * The value is 9, 10, 11, or 12.
  * (For the S_FAMILY the resolution is 9. Refer to the
  * DS18B20 Datasheet for information about B_FAMILY resolution.)
  */
uint8_t DS1820::readResolution()
{
  uint8_t res = 9;
  if ( _deviceFamily == B_FAMILY )
    res += (_scratchpad[ BYTE_CONFIG_REGISTER ] & CFG_RES) >> 5;
  return res;
}

/**
  * Reads the raw temperature value from the instance _scratchpad.
  * For B_FAMILY devices this is a 12-bit value, for S_FAMILY, 9-bit.
  * (Refer to the appropriate datasheet for information about format.)
  */
int16_t DS1820::readTemp()
{
  int16_t temp_read = _scratchpad[ BYTE_TEMP_MSB ];
  temp_read <<= 8;
  temp_read |= _scratchpad[ BYTE_TEMP_LSB ];
  return temp_read;
}

/**
  * Reads the alarm upper-limit value from the
  * instance _scratchpad.
  */
int8_t DS1820::readAlarmUpperLimit()
{
  return _scratchpad[ BYTE_ALARM_TH ];
}

/**
  * Reads the alarm lower-limit value from the
  * instance _scratchpad.
  */
int8_t DS1820::readAlarmLowerLimit()
{
  return _scratchpad[ BYTE_ALARM_TL ];
}

/**
  * This reads the 9-byte device scratchpad and stores the
  * byte values in this instance (in _scratchpad).
  */
void DS1820::readScratchpad() {
  reset();
  skip();
  write( CMD_READ_SCRATCHPAD );
  read_bytes( _scratchpad, 9 );
}

/**
  * Copies the current device scratchpad values of the alarm limits
  * and configuration register (B_FAMILY) to the EEPROM.
  */
void DS1820::updateEEPROM()
{
  reset();
  skip();
  write( CMD_COPY_SCRATCHPAD );
  while ( ! read_bit() ); // read-slot returns 1 when done
}

/**
  * Reads the current EEPROM values to the device scratchpad.
  * To read the scratchpad values into this instance use readScratchpad().
  * (Since the device scratchpad is intialized from the EEPROM on
  * power-up, this is only useful for verifying an updateEEPROM() call.)
  */
void DS1820::recallEEPROM()
{
  reset();
  skip();
  write( CMD_RECALL_E2 );
  while ( ! read_bit() ); // read-slot returns 1 when done
  readScratchpad();
}  

