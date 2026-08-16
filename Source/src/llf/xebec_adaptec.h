// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Xebec ("IBM Fixed Disk Adapter"), 512-byte format used in the XT
// Also works with the Adaptec (ACB-2010A) XT controller
// ID and data field 32bit CRC; headselect always 3 bits

#pragma once
#include "config.h"

class XebecAdaptec : public LLF
{
public:  
  
  // overrides
  FormatType getType() { return FormatType::XebecAdaptec; }
  bool analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave);
  bool scanID(uint16_t* cylinder, uint8_t* head, uint8_t* sector, uint16_t* reserved1 = NULL, uint16_t* reserved2 = NULL);
  bool readSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  bool writeSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  bool formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields = NULL, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  uint8_t* getSectorBuffer();
  
  XebecAdaptec();
  void getCustomAnalyzeTrackResults(bool& cylNumberMismatch, bool& hdNumberMismatch, uint16_t& actualCylNumber, uint8_t& actualHdNumber);
  bool getWriteModeAdaptec() { return m_WriteModeAdaptec; }
  void setWriteModeAdaptec(bool set) { m_WriteModeAdaptec = set; }
  
private:    
  bool m_AnalyzeCylNumberMismatch;
  bool m_AnalyzeHdNumberMismatch;
  uint16_t m_AnalyzeActualCylNumber;
  uint8_t m_AnalyzeActualHdNumber;
  uint16_t m_AnalyzeSkew;
  uint8_t m_AnalyzeSectorsPerTrack;
  uint8_t m_AnalyzeStartSector;
  
  bool m_WriteModeAdaptec;
  
  std::vector<uint8_t> m_SectorBuffer;
};