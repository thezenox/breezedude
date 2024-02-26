#include "Adafruit_InternalFlash_Wrapper.h"
#include "Adafruit_FlashTransport_InternalFlash.h"
#include "SAMD_InternalFlash.h"


#if SPIFLASH_DEBUG
#define SPIFLASH_LOG(_block, _count)                                           \
  do {                                                                         \
    Serial.print(__FUNCTION__);                                                \
    Serial.print(": lba = ");                                                  \
    Serial.print(_block);                                                      \
    if (_count) {                                                              \
      Serial.print(" count = ");                                               \
      Serial.print(_count);                                                    \
    }                                                                          \
    Serial.println();                                                          \
  } while (0)
#else
#define SPIFLASH_LOG(_sector, _count)
#endif

Adafruit_InternalFlash_Wrapper::Adafruit_InternalFlash_Wrapper() : Adafruit_SPIFlash() 
{
  _cache = NULL;
}

Adafruit_InternalFlash_Wrapper::Adafruit_InternalFlash_Wrapper(Adafruit_FlashTransport_InternalFlash *transport)
    : Adafruit_SPIFlash(transport) 
{
  _cache = NULL;
}

bool Adafruit_InternalFlash_Wrapper::begin(SPIFlash_Device_t const *flash_devs,
                              size_t count) 
{
  if (_trans == NULL)
    return false;
  _trans->begin();

  _flash_dev = ((Adafruit_FlashTransport_InternalFlash*)_trans)->getFlashDevice();

  // Use cache
  if (_flash_dev) {
    // new cache object if not already
    if (!_cache) {
      _cache = new Adafruit_FlashCache();
    }
  }

  if (((Adafruit_FlashTransport_InternalFlash*)_trans)->_flash==NULL || 
      ((Adafruit_FlashTransport_InternalFlash*)_trans)->_flash->get_flash_size()<8192)
  {
    // The internal flash should be allocated and provided and its size 
    // should not be smaller than 8 kB. This is required for the Fat system
    return false;
  }

  return true;
}

//--------------------------------------------------------------------+
// SdFat BaseBlockDRiver API
// A block is 512 bytes
//--------------------------------------------------------------------+
bool Adafruit_InternalFlash_Wrapper::readBlock(uint32_t block, uint8_t *dst) 
{
  SPIFLASH_LOG(block, 1);
  return _cache->read(this, block * 512, dst, 512);
}

bool Adafruit_InternalFlash_Wrapper::syncBlocks() {
  SPIFLASH_LOG(0, 0);
  return _cache->sync(this);
}

bool Adafruit_InternalFlash_Wrapper::writeBlock(uint32_t block, const uint8_t *src) {
  SPIFLASH_LOG(block, 1);
  return _cache->write(this, block * 512, src, 512);
}

bool Adafruit_InternalFlash_Wrapper::readBlocks(uint32_t block, uint8_t *dst, size_t nb) {
  SPIFLASH_LOG(block, nb);
  return _cache->read(this, block * 512, dst, 512 * nb);
}

bool Adafruit_InternalFlash_Wrapper::writeBlocks(uint32_t block, const uint8_t *src,
                                    size_t nb) {
  SPIFLASH_LOG(block, nb);
  return _cache->write(this, block * 512, src, 512 * nb);
}

