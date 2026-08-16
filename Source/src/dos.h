// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// DOS filesystem viewer

#pragma once
#include "config.h"

// used by FATFS of no-OS-FatFS-SD-SDIO-SPI-RPi-Pico (glue.c)
extern "C"
{
  unsigned int dosDiskRead(uint8_t pdrv, uint8_t* buff, uint64_t sec, unsigned int count);
  unsigned int dosDiskWrite(uint8_t pdrv, uint8_t* buff, uint64_t sec, unsigned int count);
  unsigned int dosIoctl(uint8_t pdrv, uint8_t cmd, uint8_t *buff);  
}

uint16_t dosGetSectorSize();
uint32_t dosGetTotalSectorCount();
void dosConvertLogicalSectorToCHS(const uint32_t& logical, uint16_t& cylinder, uint8_t& head, uint8_t& sector);

void commandDos(LLF* format);
