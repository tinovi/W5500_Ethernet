/*
DS1820 Arduino Library
*/

#ifndef DS1820_h
#define DS1820_h

#include <OneWire.h>

class DS1820 : public OneWire
{
  public:

  DS1820( uint8_t pinDQ );
  DS1820( uint8_t pinDQ, uint8_t deviceFamily );
  
  void convertTemperature( boolean waitForIt = CONVERT_WAIT );
  int16_t calcTemperature();
  float calcTemperatureHiRes();
  int16_t getTemperature();
  float getTemperatureHiRes();

  void setAlarmLimits( int8_t upperLimit, int8_t lowerLimit );
  void setResolution( uint8_t resolution );
  
  int16_t readTemp();
  uint8_t readResolution();
  int8_t readAlarmUpperLimit();
  int8_t readAlarmLowerLimit();

  void readScratchpad();
  void updateEEPROM();
  void recallEEPROM();
  
  uint8_t deviceFamily()
  {
    return _deviceFamily;
  }
  
  // DS18x20 device family values
  static const uint8_t S_FAMILY = 0x10;
  static const uint8_t B_FAMILY = 0x28;
  
  // temperature conversion wait/no wait constants
  static const boolean CONVERT_WAIT = true;
  static const boolean CONVERT_NO_WAIT = false;

  private:

  uint8_t _deviceFamily;
  uint8_t _scratchpad[ 9 ];
  
  // DS18x20 function commands
  static const uint8_t CMD_CONVERT_T        = 0x44; // Initiates temperature conversion
  static const uint8_t CMD_READ_SCRATCHPAD  = 0xBE; // Reads the 9 scratchpad bytes
  static const uint8_t CMD_WRITE_SCRATCHPAD = 0x4E; // Write the TH, TL, config data to the scratchpad
  static const uint8_t CMD_COPY_SCRATCHPAD  = 0x48; // Copies TH, TL, config data from the scratchpad to EEPROM
  static const uint8_t CMD_RECALL_E2        = 0xB8; // Recalls TH, TL, config data from EEPROM to the scratchpad
  static const uint8_t CMD_READ_POWER       = 0xB4; // Signals DS18x20 power supply mode to the master
  // DS18x20 ROM commands
  static const uint8_t CMD_SEARCH_ROM       = 0xF0;
  static const uint8_t CMD_READ_ROM         = 0x33;
  static const uint8_t CMD_MATCH_ROM        = 0x55;
  static const uint8_t CMD_SKIP_ROM         = 0xCC;
  static const uint8_t CMD_ALARM_SEARCH     = 0xEC;
  // DS18x20 scratchpad bytes
  // (byte 5 reserved, refer to datasheet)
  static const uint8_t BYTE_TEMP_LSB        = 0;
  static const uint8_t BYTE_TEMP_MSB        = 1;
  static const uint8_t BYTE_ALARM_TH        = 2;
  static const uint8_t BYTE_ALARM_TL        = 3;
  static const uint8_t BYTE_CONFIG_REGISTER = 4; // B_FAMILY only
  static const uint8_t BYTE_COUNT_REMAIN    = 6; // S_FAMILY only
  static const uint8_t BYTE_COUNT_PER_C     = 7; // S_FAMILY only
  static const uint8_t BYTE_CRC             = 8;
  // B_FAMILY (DS18B20) configuration register bits
  // (bits 0-4 and 7 are reserved, refer to datasheet)
  static const uint8_t CFG_BIT_R1           = B01000000; // resolution, high bit
  static const uint8_t CFG_BIT_R0           = B00100000; // resolution, low bit
  static const uint8_t CFG_RES              = CFG_BIT_R1 | CFG_BIT_R0;
  
};

#endif // DS1820_h
