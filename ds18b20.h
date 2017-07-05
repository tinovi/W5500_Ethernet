#ifndef DS18B20_H
#define DS18B20_H

#include <Arduino/Arduino.h>
#include <OneWire.h>

class DS18B20 {
 public:
  typedef enum {
    OK = 0,
	NO_DEVICE,
	CRC_FAILED
  } Result;

  DS18B20(int pin) : ds_(pin), got_addr_(false) {
    for (int i = 0; i < 8; ++i) {
      temperature_addr_[i] = 0;
    }
    got_addr_ = searchForTempSensor(temperature_addr_) == OK;
  }

  DS18B20(int pin, byte temperature_addr[8]) : ds_(pin), got_addr_(true) {
    for (int i = 0; i < 8; ++i) {
      temperature_addr_[i] = temperature_addr[i];
    }
  }

  Result searchForTempSensor(byte temperature_addr[8]) {
    if (!ds_.search(temperature_addr)) {
      ds_.reset_search();
      return NO_DEVICE;
    } else if (OneWire::crc8(temperature_addr, 7) != temperature_addr[7]) {
      return CRC_FAILED;
    }
    return OK;
  }

  Result getTemperature(int &value, bool centigrade = true) {
    byte data[9];
    byte HighByte, LowByte, SignBit;

    Result r = read(data);
    if (r != OK) {
      return r;
    }

    LowByte = data[0];
    HighByte = data[1];

    // Set undefined bits to 0 if needed; bits 5 and 6 of byte 4 determine
    // resolution
    int resolution = (data[4] & 0x60) >> 5;
    switch (resolution) {
      case 0:  // 9 bits
        LowByte &= 0xFC;
      break;
      case 1:  // 10 bits
        LowByte &= 0xFD;
      break;
      case 2:  // 11 bits
        LowByte &= 0xFE;
      break;
      case 3:  // 12 bits
        // nothing to do here, all bits valid.
      break;
      default:
        // this shouldn't happen?
        return CRC_FAILED;
    }

    int TReading = (HighByte << 8) + LowByte;
    SignBit = HighByte & 0x80;  // test most sig bit
    if (SignBit) // negative
    {
    	value = ((TReading ^ 0xffff) + 1)*-10/16; // 2's comp
    }else{
    	value = TReading * 10 / 16;
    }
    ds_.depower();
    return OK;
  }

  byte* getAddress() {
    if (! got_addr_) {
      got_addr_ = searchForTempSensor(temperature_addr_) == OK;
    }
    return temperature_addr_;
  }

  Result getConfig(byte &config) {
    byte data[9];
    Result r = read(data);
    if (r != OK) {
      return r;
    }
    config = data[4];
    return OK;
  }

 private:
  Result read(byte data[9]) {
    if (!got_addr_) {
      searchForTempSensor(temperature_addr_);
      if (!got_addr_) {
        return NO_DEVICE;
      }
    }
    ds_.reset();
    ds_.select(temperature_addr_);
    // start conversion, with parasite power on at the end
    ds_.write(0x44, 1);

    ds_.reset();
    ds_.select(temperature_addr_);
    ds_.write(0xBE);         // Read Scratchpad

    for (int i = 0; i < 9; i++) {  // we need 9 bytes
      data[i] = ds_.read();
    }
    // Verify CRC
    if (OneWire::crc8(data, 8) != data[8]) {
      return CRC_FAILED;
    }
    return OK;
  }

  OneWire ds_;
  byte temperature_addr_[8];
  bool got_addr_;
};

#endif  // DS18B20_H
