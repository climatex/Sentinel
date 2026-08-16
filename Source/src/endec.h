// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Encoder/decoder and synchronization for MFM and RLL

#pragma once 
#include "config.h"

class ENDEC
{
public:
  
  // precompensation types
  enum Precomp
  {
    PRECOMP_NOMINAL = 0,
    PRECOMP_EARLY,
    PRECOMP_LATE
  };
  
  // MFM precompensation table
  static constexpr Precomp MFM_PRECOMP[] =
  {
    // bits: prev2, prev1, comp, next
    [ 0 /* 0 0 0 0 */] = PRECOMP_NOMINAL,
    [ 1 /* 0 0 0 1 */] = PRECOMP_EARLY,
    [ 2 /* 0 0 1 0 */] = PRECOMP_NOMINAL,
    [ 3 /* 0 0 1 1 */] = PRECOMP_LATE,
    [ 4 /* 0 1 0 0 */] = PRECOMP_NOMINAL,
    [ 5 /* 0 1 0 1 */] = PRECOMP_NOMINAL,
    [ 6 /* 0 1 1 0 */] = PRECOMP_EARLY,
    [ 7 /* 0 1 1 1 */] = PRECOMP_NOMINAL,
    [ 8 /* 1 0 0 0 */] = PRECOMP_LATE,
    [ 9 /* 1 0 0 1 */] = PRECOMP_NOMINAL,
    [10 /* 1 0 1 0 */] = PRECOMP_NOMINAL,
    [11 /* 1 0 1 1 */] = PRECOMP_LATE,
    [12 /* 1 1 0 0 */] = PRECOMP_NOMINAL,
    [13 /* 1 1 0 1 */] = PRECOMP_NOMINAL,
    [14 /* 1 1 1 0 */] = PRECOMP_EARLY,
    [15 /* 1 1 1 1 */] = PRECOMP_NOMINAL
  };
  
  // RLL conversion
  struct RLLtoNRZ
  {
    uint8_t rllBits;
    uint8_t rllPattern;
    uint8_t nrzBits;
    uint8_t nrzPattern;
  };
  
  // RLL 2,7 coding: Western Digital 
  static constexpr RLLtoNRZ RLL_CODING_TABLE_WD[] =
  {
    { 4, 0b0100,     2, 0b10   },
    { 4, 0b1000,     2, 0b11   },
    { 6, 0b100100,   3, 0b000  }, // NRZ 000 is RLL 000100 for IBM/Seagate
    { 6, 0b000100,   3, 0b010  }, // NRZ 010 is RLL 100100 for IBM/Seagate
    { 6, 0b001000,   3, 0b011  },
    { 8, 0b00100100, 4, 0b0010 },
    { 8, 0b00001000, 4, 0b0011 },
  };

  // RLL 2,7 coding: Seagate, IBM
  static constexpr RLLtoNRZ RLL_CODING_TABLE_ST[] =
  {
    { 4, 0b0100,     2, 0b10   },
    { 4, 0b1000,     2, 0b11   },
    { 6, 0b000100,   3, 0b000  },
    { 6, 0b100100,   3, 0b010  },
    { 6, 0b001000,   3, 0b011  },
    { 8, 0b00100100, 4, 0b0010 },
    { 8, 0b00001000, 4, 0b0011 },
  };
  
  // coding types
  enum RLLCoding
  {
    WD = 0,
    SeagateIBM
  };
    
  ENDEC();
    
  // reading
  bool lockPLL(uint32_t timeout_us);
  uint8_t findSync(uint16_t pattern, uint16_t& partial, uint8_t& bitShift);
  bool decodeMFM(uint8_t* out, size_t& count, uint16_t& partial, const uint8_t& bitShift, CRC* crc = NULL);
  bool decodeRLL(uint8_t* out, size_t& count, const uint16_t& partial, const uint8_t& bitShift, CRC* crc = NULL, bool ignoreCodingErrors = false);  
  void setReadGate(bool on);
  
  // writing
  void encodeMFM(const uint8_t* input, size_t len, const std::vector<size_t>& dropClockBitOffsets, std::vector<uint32_t>& output);  
  void encodeRLL(const uint8_t* input, size_t len, const std::vector<size_t>& insertSyncByteOffsets, std::vector<uint32_t>& output);
  void prepareWriteDMA(const void* buffer, size_t words);
  bool writeWholeTrack();
  void setWriteGate(bool on);
  
  // misc
  void setRLLCoding(RLLCoding type) { m_RLLCoding = type; }  
  bool waitForTrackStart();
  uint16_t getMaximumTrackBytes();
  uint8_t getMaximumSectorCountFor(LLF* format, uint16_t sectorSizeBytes);
  bool readFifo16(uint16_t& out);
  bool getBit(const uint8_t* buffer, size_t len, size_t bit);  
  uint8_t getPrecompNibble(bool wdata, Precomp precomp);
  
private:
  INLINE uint8_t extractMFMData(uint16_t r);
  INLINE uint8_t extractMFMClock(uint16_t r);  
  uint8_t applyMFMPrecomp(bool& prev2, bool& prev1, bool& comp, bool next, bool clockAbsent);
  INLINE bool extractRLLByte(uint32_t& buffer, int& bufferBits, uint8_t& output, uint8_t& bitCount, bool ignoreCodingErrors);
  
  RLLCoding m_RLLCoding;
};

// pre-compiled decode tables for all 256 byte values for both WD and ST codings
typedef std::array<ENDEC::RLLtoNRZ, 256> RLLBytesTable;
template<size_t N>
static constexpr RLLBytesTable makeBytesTable(const ENDEC::RLLtoNRZ (&table)[N])
{
  RLLBytesTable lut{};
  
  for (size_t byte = 0; byte < 256; byte++)
  {
    lut[byte] = { 0, 0, 0, 0 }; 
    
    for (const auto& entry : table)
    {
      if ((byte >> (8 - entry.rllBits)) == entry.rllPattern)
      {
        lut[byte] = entry;
        break;
      }
    } 
  }
  
  return lut;
}
static constexpr RLLBytesTable RLL_BYTES_TABLE_WD = makeBytesTable(ENDEC::RLL_CODING_TABLE_WD);
static constexpr RLLBytesTable RLL_BYTES_TABLE_ST = makeBytesTable(ENDEC::RLL_CODING_TABLE_ST);

// complementary: pre-compiled tables, indexed by 4-bit NRZ lookahead
// each resolves to exactly one RLL codeword
typedef std::array<ENDEC::RLLtoNRZ, 16> RLLLookaheadTable;
template<size_t N>
static constexpr RLLLookaheadTable makeLookaheadTable(const ENDEC::RLLtoNRZ (&table)[N])
{
  RLLLookaheadTable lut{};
  
  // 16 (2^n) max lookahead bits; n=4
  for (uint8_t lookAhead = 0; lookAhead < 16; lookAhead++)
  {
    lut[lookAhead] = {0, 0, 0, 0};
    
    for (const auto& entry : table)
    {
      if ((lookAhead >> (4 - entry.nrzBits)) == entry.nrzPattern)
      {
        lut[lookAhead] = entry;
        break;
      }
    }
  }
  
  return lut;
}
static constexpr RLLLookaheadTable RLL_LOOKAHEAD_TABLE_WD = makeLookaheadTable(ENDEC::RLL_CODING_TABLE_WD);
static constexpr RLLLookaheadTable RLL_LOOKAHEAD_TABLE_ST = makeLookaheadTable(ENDEC::RLL_CODING_TABLE_ST);