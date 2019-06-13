#include "AD5593.h"

Serial pcd(p13, p12, "debug", 115200);

AD5593::AD5593(I2C *i2c, int address) : i2c(i2c), address(address) {
  this->init();
}

AD5593::AD5593(PinName sda, PinName scl, int address)
    : i2c(new I2C(sda, scl)), address(address << 1) {
  this->init();
}

void AD5593::i2c_write(unsigned char addr, unsigned short value) {
  char i2CWriteBuffer[3];

  i2CWriteBuffer[0] = addr;
  i2CWriteBuffer[1] = (unsigned char)(value >> 8);
  i2CWriteBuffer[2] = (unsigned char)(value & 0xFF);

  int res = i2c->write(this->address, i2CWriteBuffer, 3);
  pcd.printf("ad5593 write:%d\n\r", res);
}

unsigned short AD5593::i2c_read(unsigned char addr) {
  char i2CReadBuffer[2];
  char i2CWriteBuffer[1];

  if (!(addr & 0xF0))                 // 11110000
    addr |= AD5593Mode::REG_READBACK; // 7<<4 .. 111 << 4 .. 1110000

  i2CWriteBuffer[0] = addr;

  int res;
  res = i2c->write(this->address, i2CWriteBuffer, sizeof(i2CWriteBuffer));
  pcd.printf("ad5593 read w:%d\n\r", res);
  res = i2c->read(this->address, i2CReadBuffer, sizeof(i2CReadBuffer));
  pcd.printf("ad5593 read r:%d\n\r", res);

  return (unsigned short)(i2CReadBuffer[0] << 8 | i2CReadBuffer[1]);
}

void AD5593::init() {
  //   this->gpio_out = 0;
  //   this->gpio_in = 0;
  //   this->gpio_set = 0;
  //   this->adc_en = 0;
  //   this->dac_en = 0;
  //   this->tristate = 0;
  //   this->pulldown = 0xFF;

  pcd.printf("addr ad5593 %x \n\r", this->address);

    this->i2c->frequency(400000);
  this->pulldown = this->i2c_read(AD5593Register::PULLDOWN);
  this->tristate = this->i2c_read(AD5593Register::TRISTATE);
  this->dac_en = this->i2c_read(AD5593Register::DAC_EN);
  this->adc_en = this->i2c_read(AD5593Register::ADC_EN);
  this->gpio_set = this->i2c_read(AD5593Register::GPIO_SET);
  this->gpio_out = this->i2c_read(AD5593Register::GPIO_OUT_EN);
  this->gpio_in = this->i2c_read(AD5593Register::GPIO_IN_EN);
}

void AD5593::reset() { i2c_write(AD5593Register::RESET, 0xdac); }

void AD5593::reference_internal() { i2c_write(AD5593Register::PD, 1 << 9); }

void AD5593::reference_range_high() {
  i2c_write(AD5593Register::CTRL, (1 << 4) | (1 << 5));
}

void AD5593::setAnalogOutPin(ExpPinName pin) {
  this->clearBit(this->pulldown, pin);
  this->clearBit(this->tristate, pin);
  this->setBit(this->dac_en, pin);
  this->clearBit(this->gpio_out, pin);
  this->clearBit(this->gpio_in, pin);
}

void AD5593::setAnalogInPin(ExpPinName pin) {
  this->clearBit(this->pulldown, pin);
  this->clearBit(this->tristate, pin);
  this->clearBit(this->dac_en, pin);
  this->setBit(this->adc_en, pin);
  this->clearBit(this->gpio_out, pin);
  this->clearBit(this->gpio_in, pin);
}

void AD5593::setAnalogInOutPin(ExpPinName pin) {
  this->clearBit(this->pulldown, pin);
  this->clearBit(this->tristate, pin);
  this->setBit(this->dac_en, pin);
  this->setBit(this->adc_en, pin);
  this->clearBit(this->gpio_out, pin);
  this->clearBit(this->gpio_in, pin);
}

void AD5593::setDigitalOutPin(ExpPinName pin) {
  this->clearBit(this->pulldown, pin);
  this->clearBit(this->tristate, pin);
  this->clearBit(this->dac_en, pin);
  this->clearBit(this->adc_en, pin);
  this->setBit(this->gpio_out, pin);
  this->clearBit(this->gpio_in, pin);
}

void AD5593::setDigitalInPin(ExpPinName pin) {
  this->clearBit(this->pulldown, pin);
  this->clearBit(this->tristate, pin);
  this->clearBit(this->dac_en, pin);
  this->clearBit(this->adc_en, pin);
  this->clearBit(this->gpio_out, pin);
  this->setBit(this->gpio_in, pin);
}

void AD5593::setUnusedLowPin(ExpPinName pin) {
  this->clearBit(this->pulldown, pin);
  this->clearBit(this->tristate, pin);
  this->clearBit(this->dac_en, pin);
  this->clearBit(this->adc_en, pin);
  this->clearBit(this->gpio_set, pin);
  this->setBit(this->gpio_out, pin);
  this->clearBit(this->gpio_in, pin);
}

void AD5593::setUnusedHighPin(ExpPinName pin) {
  this->clearBit(this->pulldown, pin);
  this->clearBit(this->tristate, pin);
  this->clearBit(this->dac_en, pin);
  this->clearBit(this->adc_en, pin);
  this->setBit(this->gpio_set, pin);
  this->setBit(this->gpio_out, pin);
  this->clearBit(this->gpio_in, pin);
}

void AD5593::setUnusedTristatePin(ExpPinName pin) {
  this->clearBit(this->dac_en, pin);
  this->clearBit(this->adc_en, pin);
  this->clearBit(this->gpio_out, pin);
  this->clearBit(this->gpio_in, pin);
  this->setBit(this->tristate, pin);
}

void AD5593::setUnusedPulldownPin(ExpPinName pin) {
  this->clearBit(this->dac_en, pin);
  this->clearBit(this->adc_en, pin);
  this->clearBit(this->gpio_out, pin);
  this->clearBit(this->gpio_in, pin);
  this->setBit(this->pulldown, pin);
}

void AD5593::configureAllPins() {
  this->i2c_write(AD5593Register::DAC_EN, this->dac_en);
  this->i2c_write(AD5593Register::ADC_EN, this->adc_en);
  this->i2c_write(AD5593Register::GPIO_SET, this->gpio_set);
  this->i2c_write(AD5593Register::GPIO_OUT_EN, this->gpio_out);
  this->i2c_write(AD5593Register::GPIO_IN_EN, this->gpio_in);
  this->i2c_write(AD5593Register::GPIO_SET, this->pulldown);
  this->i2c_write(AD5593Register::GPIO_SET, this->tristate);
}

bool AD5593::isBitSet(unsigned short mask, unsigned char pin) {
  return (mask & (1 << pin)) != 0;
}

unsigned short AD5593::shortIsBitSet(unsigned short mask, unsigned char pin) {
  return (mask & (1 << pin)) != 0;
}

void AD5593::setBit(unsigned short &mask, unsigned char pin) {
  mask |= (1 << pin);
}

void AD5593::clearBit(unsigned short &mask, unsigned char pin) {
  mask &= ~(1 << pin);
}

void AD5593::setOrClear(unsigned short &mask, unsigned char pin, bool set) {
  if (set)
    setBit(mask, pin);
  else
    clearBit(mask, pin);
}

void AD5593::writeDigitalPin(ExpPinName pin, int value) {
  // this->gpio_set = this->i2c_read(AD5593Register::GPIO_SET);
  this->setOrClear(gpio_set, pin, value > 0);
  this->i2c_write(AD5593Register::GPIO_SET, gpio_set);
}

int AD5593::readDigitalPin(ExpPinName pin) {
  // this->gpio_out = this->i2c_read(AD5593Register::GPIO_OUT_EN);
  if (this->isBitSet(this->gpio_out, pin)) {
    // this->gpio_set = this->i2c_read(AD5593Register::GPIO_SET);
    return this->shortIsBitSet(this->gpio_set, pin);
  } else {
    return this->shortIsBitSet(this->i2c_read(AD5593Mode::GPIO_READBACK), pin);
  }
}

void AD5593::writeAnalogPin(ExpPinName pin, float value) {
  this->i2c_write((unsigned char)(AD5593Mode::DAC_WRITE | (unsigned char)pin),
                  (unsigned char)(value * 255.0)); // 12 Bit?
}

float AD5593::readAnalogPin(ExpPinName pin) {
  // this->adc_en = this->i2c_read(AD5593Register::ADC_EN);
  if (this->shortIsBitSet(this->gpio_set, pin)) {
    this->i2c_write(AD5593Mode::CONF | AD5593Register::ADC_SEQ,
                    (unsigned short)(1 << pin));
    unsigned short res =
        (unsigned short)((int)this->i2c_read(
                             (unsigned char)AD5593Mode::ADC_READBACK) &
                         0xFFF);
    return (float)res / 4096.0; // 12 Bit?
  } else {
    return this->i2c_read(
        (float)(AD5593Register::DAC_READBACK | (unsigned char)pin));
  }
}

//////Pin Base Class/////////

AD5593Pin::AD5593Pin(AD5593 *_device, ExpPinName _pin)
    : device(_device), pin(_pin) {}

// AD5593Pin &AD5593Pin::operator=(int value) {
//   this->vWrite(value);
// }

// AD5593Pin::operator int() { return (this->read()); }

//////Pin Base Class/////////

/////Digital Out//////////

AD5593DigitalOut::AD5593DigitalOut(AD5593 *_device, ExpPinName _pin)
    : AD5593Pin(_device, _pin) {
  this->device->setDigitalOutPin(this->pin);
}

void AD5593DigitalOut::vWrite(int value) {
  this->device->writeDigitalPin(this->pin, value);
  return;
}

int AD5593DigitalOut::read() { return this->device->readDigitalPin(this->pin); }

/////Digital Out//////////

/////Digital In//////////

AD5593DigitalIn::AD5593DigitalIn(AD5593 *_device, ExpPinName _pin)
    : AD5593Pin(_device, _pin) {
  this->device->setDigitalInPin(this->pin);
}

void AD5593DigitalIn::vWrite(int value) {
  this->device->writeDigitalPin(this->pin, value);
  return;
}

int AD5593DigitalIn::read() { return this->device->readDigitalPin(this->pin); }

/////Digital In//////////

/////Analog Out//////////

AD5593AnalogOut::AD5593AnalogOut(AD5593 *_device, ExpPinName _pin)
    : AD5593Pin(_device, _pin) {
  this->device->setAnalogOutPin(this->pin);
}

void AD5593AnalogOut::vWrite(float value) {
  this->device->writeAnalogPin(this->pin, value);
  return;
}

int AD5593AnalogOut::read() { return this->device->readAnalogPin(this->pin); }

/////Analog Out//////////

/////Analog In//////////

AD5593AnalogIn::AD5593AnalogIn(AD5593 *_device, ExpPinName _pin)
    : AD5593Pin(_device, _pin) {
  this->device->setAnalogInPin(this->pin);
}

void AD5593AnalogIn::vWrite(int value) {
  this->device->writeAnalogPin(this->pin, value);
  return;
}

float AD5593AnalogIn::read() { return this->device->readAnalogPin(this->pin); }

/////Analog In//////////