// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Generic Western Digital MFM/RLL format
// Sector sizes supported: 128B 256B 512B 1024B
// Data field CRC: 16-bit, 32-bit, 56-bit; head select in ID fields 3 or 4bit

#pragma once
#include "config.h"

class WD : public LLF
{
public:  
  
  // overrides
  FormatType getType() { return FormatType::WD; }
  bool analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave);
  bool scanID(uint16_t* cylinder, uint8_t* sdh, uint8_t* sector, uint16_t* reserved1 = NULL, uint16_t* reserved2 = NULL);
  bool readSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  bool writeSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  bool formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields = NULL, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  uint8_t* getSectorBuffer();
  
  WD();
  
  void setWorkingSectorSizeBytes(uint16_t sectorSizeBytes);
  void setDataCrcBits(uint8_t dataCrcBits);
  void setSdh4Bit(bool fourBit) { m_Sdh4Bit = fourBit; }
  void getCustomAnalyzeTrackResults(bool& cylNumberMismatch, bool& hdNumberMismatch, bool& variableSectorSize, uint16_t& actualCylNumber, uint8_t& actualHdNumber);
  
private:
  uint8_t getIdent(uint16_t cylinder);
  uint16_t getSectorSizeFromSDH(uint8_t sdh);
  uint8_t getSDHFromSectorSize(uint16_t sectorSizeBytes);
  
  bool m_Sdh4Bit;
  uint8_t m_DataCrcBits;
  uint16_t m_WorkingSectorSizeBytes;
  
  bool m_AnalyzeCylNumberMismatch;
  bool m_AnalyzeHdNumberMismatch;
  bool m_AnalyzeVariableSectorSize;
  uint16_t m_AnalyzeActualCylNumber;
  uint8_t m_AnalyzeActualHdNumber;
  
  std::vector<uint8_t> m_SectorBuffer;
};