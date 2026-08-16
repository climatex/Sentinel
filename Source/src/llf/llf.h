// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Low level format base class

#pragma once
#include "config.h"

class LLF
{
public:

  enum FormatType
  {
    WD,
    OMTI,
    XebecAdaptec,
    HDC9224,
    SM1040
  };

  virtual ~LLF() {}
  
  virtual FormatType getType() = 0;  
  virtual bool analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave) = 0;
  virtual bool scanID(uint16_t* cylinder, uint8_t* head, uint8_t* sector, uint16_t* reserved1 = NULL, uint16_t* reserved2 = NULL) = 0;
  virtual bool readSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL) = 0;
  virtual bool writeSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL) = 0;
  virtual bool formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields = NULL, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL) = 0; 
  virtual uint8_t* getSectorBuffer() = 0;
  
  // generic helpers
  inline static void calculateInterleave(std::vector<uint8_t>& sectors, uint8_t& sectorsPerTrack, uint8_t& startSector, uint8_t& interleave);
  inline static void getInterleaveTable(uint8_t sectorsPerTrack, uint8_t startSector, uint8_t interleave, std::vector<uint8_t>& output);
};

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
    uint8_t pos = 0;
    uint8_t expected = startSector + 1;
    for (; expected < startSector + sectorsPerTrack; expected++)
    {
      pos += interleave;
      if (pos >= sectorsPerTrack)
      {
        pos -= sectorsPerTrack;
      }        

      if (sectors[pos] != expected)
      {
        interleave = 0;
        return;
      }  
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