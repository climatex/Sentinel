// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// VUVT SMEP SM 1040, an Iron Curtain RK07 emulator that used MFM drives for storage
// Each byte pair in sector data fields is recorded swapped: presumably, it processed them in 16bit big endian words

#pragma once
#include "config.h"

class SM1040 : public LLF
{
public:  
  
  // overrides
  FormatType getType() { return FormatType::SM1040; }
  bool analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave);
  bool scanID(uint16_t* cylinder, uint8_t* head, uint8_t* sector, uint16_t* keyword = NULL, uint16_t* cylinderCount = NULL);
  bool readSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  bool writeSector(uint8_t sector, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  bool formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields = NULL, uint16_t* overrideCyl = NULL, uint8_t* overrideHead = NULL);
  uint8_t* getSectorBuffer();
  
  SM1040();
  
  void getCustomAnalyzeTrackResults(bool& cylNumberMismatch, bool& hdNumberMismatch, uint16_t& actualCylNumber, uint8_t& actualHdNumber, uint16_t& logDriveCylCount);
  bool isDriveTypeRK06() { return m_DriveTypeRK06; }
  void setDriveTypeRK06(bool rk06) { m_DriveTypeRK06 = rk06; }  
  bool isTrackRelocated() { return m_TrackIsRelocated; }
  void getRelocation(uint16_t& cylinder, uint8_t& head) { cylinder = m_RelocationCyl; head = m_RelocationHd; }
  
private:
  void whipperSnapperBufferSwapper();
  
  bool m_AnalyzeCylNumberMismatch;
  bool m_AnalyzeHdNumberMismatch;
  uint16_t m_AnalyzeActualCylNumber;
  uint8_t m_AnalyzeActualHdNumber;
  uint16_t m_AnalyzeLogDriveCylCount;
  
  bool m_TrackIsRelocated;
  uint16_t m_RelocationCyl;
  uint8_t m_RelocationHd;

  bool m_DriveTypeRK06;
  
  std::vector<uint8_t> m_SectorBuffer;
};