// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Standard Microsystems HDC9224 (512 byte sectors)

#pragma once
#include "config.h"

class HDC9224 : public LLF
{
public:  
  
  // overrides
  FormatType getType() { return FormatType::HDC9224; }
  bool analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave);
  bool scanID(uint16_t* cylinder, uint8_t* head, uint8_t* sector, uint16_t* keyword = NULL, uint16_t* cylinderCount = NULL);
  bool readSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  bool writeSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  bool formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields = NULL, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  uint8_t* getSectorBuffer();
  
  HDC9224();
  void getCustomAnalyzeTrackResults(bool& cylNumberMismatch, bool& hdNumberMismatch, uint16_t& actualCylNumber, uint8_t& actualHdNumber);
  
private:
  bool m_AnalyzeCylNumberMismatch;
  bool m_AnalyzeHdNumberMismatch;
  uint16_t m_AnalyzeActualCylNumber;
  uint8_t m_AnalyzeActualHdNumber;
  
  std::vector<uint8_t> m_SectorBuffer;
};