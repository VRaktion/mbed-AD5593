#ifndef AD5593_H
#define AD5593_H

#include "mbed.h"

enum ExpPinName {
  exp_p0 = 0,
  exp_p1,
  exp_p2,
  exp_p3,
  exp_p4,
  exp_p5,
  exp_p6,
  exp_p7,
};

struct AD5593Register {
  static const uint8_t NOOP = 0x0;
  static const uint8_t DAC_READBACK = 0x1;
  static const uint8_t ADC_SEQ = 0x2;
  static const uint8_t CTRL = 0x3;
  static const uint8_t ADC_EN = 0x4;
  static const uint8_t DAC_EN = 0x5;
  static const uint8_t PULLDOWN = 0x6;
  static const uint8_t LDAC = 0x7;
  static const uint8_t GPIO_OUT_EN = 0x8;
  static const uint8_t GPIO_SET = 0x9;
  static const uint8_t GPIO_IN_EN = 0xA;
  static const uint8_t PD = 0xB;
  static const uint8_t OPEN_DRAIN = 0xC;
  static const uint8_t TRISTATE = 0xD;
  static const uint8_t RESET = 0xF;
};

struct AD5593Mode {
  static const uint8_t CONF = (0 << 4);
  static const uint8_t DAC_WRITE = (1 << 4);
  static const uint8_t ADC_READBACK = (4 << 4);
  static const uint8_t DAC_READBACK = (5 << 4);
  static const uint8_t GPIO_READBACK = (6 << 4);
  static const uint8_t REG_READBACK = (7 << 4);
};

class AD5593 {
public:
  AD5593(I2C *i2c, int address);
  AD5593(PinName sda, PinName scl, int address);

  void init();
  void reset();
  void reference_internal();
  void reference_range_high();

  void i2c_write(unsigned char addr, unsigned short value);
  unsigned short i2c_read(unsigned char addr);

  void setAnalogOutPin(ExpPinName pin);
  void setAnalogInPin(ExpPinName pin);
  void setAnalogInOutPin(ExpPinName pin);
  void setDigitalOutPin(ExpPinName pin);
  void setDigitalInPin(ExpPinName pin);

  void setUnusedLowPin(ExpPinName pin);
  void setUnusedHighPin(ExpPinName pin);
  void setUnusedTristatePin(ExpPinName pin);
  void setUnusedPulldownPin(ExpPinName pin);

  void writeDigitalPin(ExpPinName pin, int value);
  int readDigitalPin(ExpPinName pin);
  void writeAnalogPin(ExpPinName pin, float value);
  float readAnalogPin(ExpPinName pin);

  void configureAllPins();

private:
  // helpers
  bool isBitSet(unsigned short mask, unsigned char pin);
  unsigned short shortIsBitSet(unsigned short mask, unsigned char pin);
  void setBit(unsigned short &mask, unsigned char pin);
  void clearBit(unsigned short &mask, unsigned char pin);
  void setOrClear(unsigned short &mask, unsigned char pin, bool set);

  unsigned short gpio_out;
  unsigned short gpio_in;
  unsigned short gpio_set;
  unsigned short adc_en;
  unsigned short dac_en;
  unsigned short tristate;
  unsigned short pulldown;

  int address;
  I2C *i2c;
};

class AD5593Pin {
public:
  AD5593Pin(AD5593 *_device, ExpPinName _pin);
//   virtual void vWrite(int value);
//   virtual int read();
  //#ifdef MBED_OPERATORS
  AD5593Pin &operator=(int value);
  operator int();
  //#endif
protected:
  ExpPinName pin;
  AD5593 *device;
};

class AD5593DigitalOut : public AD5593Pin {
public:
  AD5593DigitalOut(AD5593 *_device, ExpPinName _pin);
  void vWrite(int value);
  int read();
};

class AD5593DigitalIn : public AD5593Pin {
public:
  AD5593DigitalIn(AD5593 *_device, ExpPinName _pin);
  void vWrite(int value);
  int read();
};

class AD5593AnalogOut : public AD5593Pin {
public:
  AD5593AnalogOut(AD5593 *_device, ExpPinName _pin);
  void vWrite(float value);
  int read();
};

class AD5593AnalogIn : public AD5593Pin {
public:
  AD5593AnalogIn(AD5593 *_device, ExpPinName _pin);
  void vWrite(int value);
  float read();
};

#endif