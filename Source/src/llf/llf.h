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
    Seagate,
    OMTI,
    XebecAdaptec,
    HDC9224,
    SM1040,
    ADT
  };

  virtual ~LLF() {}
  
  virtual FormatType getType() = 0;  
  virtual bool analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave) = 0;
  virtual bool scanID(uint16_t* cylinder, uint8_t* head, uint8_t* sector, uint16_t* reserved1 = NULL, uint16_t* reserved2 = NULL) = 0;
  virtual bool readSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL) = 0;
  virtual bool writeSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL) = 0;
  virtual bool formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields = NULL, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL) = 0; 
  virtual uint8_t* getSectorBuffer() = 0;
  
  // readSector with automatic microstep on errors, if enabled
  bool readSectorMicrostep(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  
  // generic helpers
  static void calculateInterleave(std::vector<uint8_t>& sectors, uint8_t& sectorsPerTrack, uint8_t& startSector, uint8_t& interleave);
  static void getInterleaveTable(uint8_t sectorsPerTrack, uint8_t startSector, uint8_t interleave, std::vector<uint8_t>& output);
};
