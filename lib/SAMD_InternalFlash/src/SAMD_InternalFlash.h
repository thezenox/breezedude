/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Written by Cristian Maglie

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SAMD_INTERNALFLASH_H_
#define SAMD_INTERNALFLASH_H_

#pragma once

#include <Arduino.h>
#include <FlashStorage.h>

#include "Adafruit_FlashTransport_InternalFlash.h"
#include "Adafruit_InternalFlash_Wrapper.h"



class InternalFlash {
public:
  InternalFlash();

  void write(uint32_t offset, const void *data, uint32_t size);
  void erase(uint32_t offset, uint32_t size);
  void read(uint32_t offset, void *data, uint32_t size);

  uint32_t get_flash_size() const { return _flash_size;}
  void *get_flash_address() const { return (void*)_flash_address;}

  void flush_buffer();

private:
  volatile uint8_t *_flash_address;
  uint32_t _flash_size;

  FlashClass fl;

#if defined(__SAMD51__)
  uint8_t _buff[8192];
  uint32_t _buff_addr;
  bool _buff_in_used;
#endif
};

#endif	// SAMD_INTERNALFLASH_H_

