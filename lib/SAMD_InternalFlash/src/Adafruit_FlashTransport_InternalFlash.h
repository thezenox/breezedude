/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 hathach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef ADAFRUIT_FLASHTRANSPORT_INTERNALFLASH_H_
#define ADAFRUIT_FLASHTRANSPORT_INTERNALFLASH_H_

#include "Arduino.h"
#include "SPI.h"
#include "flash_devices.h"
#include <Adafruit_FlashTransport.h>

class InternalFlash;
class Adafruit_InternalFlash_Wrapper;

class Adafruit_FlashTransport_InternalFlash : public Adafruit_FlashTransport {
  friend Adafruit_InternalFlash_Wrapper;

private:
  InternalFlash *_flash;
  SPIFlash_Device_t _flash_device;

public:
  Adafruit_FlashTransport_InternalFlash(InternalFlash *_flash);

  virtual void begin(void);
  virtual void end(void);

  virtual bool supportQuadMode(void);

  virtual void setClockSpeed(uint32_t write_hz, uint32_t read_hz);

  virtual bool runCommand(uint8_t command);
  virtual bool readCommand(uint8_t command, uint8_t *response, uint32_t len);
  virtual bool writeCommand(uint8_t command, uint8_t const *data, uint32_t len);
  virtual bool eraseCommand(uint8_t command, uint32_t addr);

  virtual bool readMemory(uint32_t addr, uint8_t *data, uint32_t len);
  virtual bool writeMemory(uint32_t addr, uint8_t const *data, uint32_t len);

  // Flash device is already detected and configured, get the pointer without
  // go through initial sequence
  SPIFlash_Device_t *getFlashDevice(void);
};

#endif /* ADAFRUIT_FLASHTRANSPORT_INTERNALFLASH_H_ */
