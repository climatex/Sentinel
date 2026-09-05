// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Low level format base class

#include "config.h"

bool LLF::readSectorMicrostep(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  // call derived readSector implementations, do automatic microstep beforehand
  bool onNoAddressMark;
  bool onCRCError;
  hdd.getMicrostepping(onNoAddressMark, onCRCError);
  
  uint8_t attempts = 1; // 1st attempt: regular read
  if (onNoAddressMark || onCRCError)
  {
    attempts += RECOVERY_MODE_MICROSTEPS;
  }
  
  bool success = false;
  uint8_t attempt = 0;  
  for (; attempt < attempts; attempt++)
  {
    if (attempt)
    {
      hdd.microStep(true); // past regular read
    }
    
    if (readSector(sector, overrideCyl, overrideHead))
    {
      success = true;
      break;
    }
    
    // failed, determine why
    const uint8_t result = hdd.getLastResult();
    
    if ((attempts == 1) || // microstep not enabled at all
        (result < HDD_STATUS_NO_SECTOR_ID) || // timeout, invalid arguments, not ready, write fault
        ((result == HDD_STATUS_NO_SECTOR_ID) && !onNoAddressMark) || // ID address mark not found, microstep not enabled
        ((result == HDD_STATUS_NO_DATA_ID) && !onNoAddressMark) || // data address mark not found, microstep not enabled
        ((result == HDD_STATUS_DATA_ERROR) && !onCRCError)) // uncorrected CRC error, microstep not enabled
    {
      break;
    }
  }
  
  // leave recovery mode if turned on
  if (attempt)
  {
    hdd.microStep(false);
  }
  
  return success;
}

void LLF::calculateInterleave(std::vector<uint8_t>& sectors, uint8_t& sectorsPerTrack, uint8_t& startSector, uint8_t& interleave)
{
  // sectors vector: raw reads from scanID(); will be aligned to start with startSector
  
  // presumed unknown
  sectorsPerTrack = 0;
  startSector = 0;
  interleave = 0;
  if (sectors.empty())
  {
    return;
  }
  
  // determine starting sector number and sectors per track by accounting for any defects in the table
  uint8_t gaps = 0;
  bool gapsDetect[256] = {false};
  
  startSector = (uint8_t)-1;
  uint8_t maxSecNumber = 0;
  for (const uint8_t& sector : sectors)
  {
    gapsDetect[sector] = true;
    if (sector < startSector)
    {
      startSector = sector;
    }
    if (sector > maxSecNumber)
    {
      maxSecNumber = sector;
    }
  }
  for (uint8_t test = startSector; test < maxSecNumber; test++)
  {
    if (!gapsDetect[test])
    {
      gaps++;
    }
  }
  sectorsPerTrack = maxSecNumber-startSector+1 - gaps;
  
  // adjust so that the vector begins with startSector, then resize it to sectorsPerTrack
  auto it = std::find(sectors.begin(), sectors.end(), startSector);
  if (it != sectors.end())
  {
    std::rotate(sectors.begin(), it, sectors.end());
  }
  if (sectors.size() > sectorsPerTrack)
  {
    sectors.resize(sectorsPerTrack);
  }
  
  // try to calculate interleave
  if (sectorsPerTrack < 3)
  {
    interleave = 1;
    return;
  }
  
  for (uint8_t i = 1; i < sectorsPerTrack; i++)
  {
    if (sectors[i] == (startSector + 1))
    {
      interleave = i;
      break;
    }
  }
  
  // verify
  if (interleave)
  {
    bool verified[256] = {false};
    verified[0] = true; // startSector
    
    uint8_t pos = 0;
    uint8_t expected = startSector + 1;
    for (uint8_t i = 1; i < sectorsPerTrack; i++)
    {
      pos += interleave;
      if (pos >= sectorsPerTrack)
      {
        pos -= sectorsPerTrack;
      }        

      while (verified[pos])
      {
        pos++;
        if (pos >= sectorsPerTrack)
        {
          pos = 0; // table wraps around from start
        }
      }
      
      if (sectors[pos] != expected)
      {
        interleave = 0;
        return;
      }  
      
      verified[pos] = true;
      expected++;
    }
  }  
}

// the reverse of above
void LLF::getInterleaveTable(uint8_t sectorsPerTrack, uint8_t startSector, uint8_t interleave, std::vector<uint8_t>& output)
{
  // sanity check
  if (interleave >= sectorsPerTrack)
  {
    interleave = 1;
  }
  
  uint8_t pos = 0;
  uint8_t currentSector = 1;
  output.resize(sectorsPerTrack, 0);
  
  while (currentSector <= sectorsPerTrack)
  {
    output[pos] = currentSector++;
    pos += interleave;
    
    if (pos >= sectorsPerTrack)
    {
      pos %= sectorsPerTrack;
      while ((pos < sectorsPerTrack) && (output[pos] != 0))
      {
        pos++;
      }
    }
  }

  // adjust table for starting sector
  for (uint8_t& sector : output)
  {
    if (startSector == 0)
    {
      sector--;
    }
    else if (startSector > 1)
    {
      sector += startSector - 1;
    }
  }
}