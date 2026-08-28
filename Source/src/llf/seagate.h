// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Seagate ST21/ST22 MFM format, PC/AT (TODO: RLL)

#pragma once
#include "config.h"

class Seagate : public LLF
{
public:  
  
  // overrides
  FormatType getType() { return FormatType::Seagate; }
  bool analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave);
  bool scanID(uint16_t* cylinder, uint8_t* sdh, uint8_t* sector, uint16_t* reserved1 = NULL, uint16_t* reserved2 = NULL);
  bool readSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  bool writeSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  bool formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields = NULL, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  uint8_t* getSectorBuffer();
  
  Seagate();
  
  bool formatWriteReservedCylinder(uint8_t head, uint8_t interleave); 
  void getCustomAnalyzeTrackResults(bool& cylNumberMismatch, bool& hdNumberMismatch, bool& useSpareSector, uint16_t& actualCylNumber, uint8_t& actualHdNumber);
  void getAnalyzeSectorsTable(std::vector<uint8_t>& table) { table = m_AnalyzeSectorsTable; }
  bool isTrackRelocated() { return m_TrackIsRelocated; }
  void getRelocation(uint16_t& cylinder, uint8_t& head) { cylinder = m_RelocationCyl; head = m_RelocationHd; }
  
private:
  std::vector<uint8_t> m_AnalyzeSectorsTable;
  bool m_AnalyzeCylNumberMismatch;
  bool m_AnalyzeHdNumberMismatch;
  bool m_AnalyzeUseSpareSector;
  uint16_t m_AnalyzeActualCylNumber;
  uint8_t m_AnalyzeActualHdNumber;
    
  bool m_TrackIsRelocated;
  uint16_t m_RelocationCyl;
  uint8_t m_RelocationHd;
  
  std::vector<uint8_t> m_SectorBuffer;
};