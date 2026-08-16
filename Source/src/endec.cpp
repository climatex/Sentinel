// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Encoder/decoder and synchronization for MFM and RLL

#include "config.h"

// defined in main.cpp
extern volatile int g_IndexCount;
extern const uint g_PioSamplerOffset;
extern const uint g_PioWriterOffset;
extern const int g_PioWriterDma;
extern float g_WclockRate;

// disk reads (sampler.pio) use an 8-word RX FIFO, autopush every 16 bits, read manually through readFifo16()
// disk writes (writer.pio) use an 8-word TX FIFO, autopull every 32 bits, with DMA transfer prepared beforehand

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////// shared functions /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ENDEC::ENDEC()
{
  m_RLLCoding = RLLCoding::WD; // default
}

void ENDEC::setReadGate(bool on)
{
  // RGATE set to 1 on state machine start
  pio_sm_set_enabled(pio0, 0, on);
  
  if (!on)
  {
    // bring RGATE down
    pio_sm_set_pins_with_mask(pio0, 0, 0, 8);
    
    // restart SM and reset program counter
    pio_sm_restart(pio0, 0);   
    pio_sm_clear_fifos(pio0, 0);
    pio_sm_exec(pio0, 0, pio_encode_jmp(g_PioSamplerOffset));
  }
}

void ENDEC::setWriteGate(bool on)
{
  // both write buffer and (paused) DMA transfer must be already prepared
  if (on)
  {
    dma_channel_start(g_PioWriterDma);
  }
  
  // WGATE set to 1 on state machine start
  pio_sm_set_enabled(pio0, 1, on);
  
  if (!on)
  {
    // bring WGATE down
    pio_sm_set_pins_with_mask(pio0, 1, 0, 0x80);
    
    // stop DMA if need be: RP2350 - clear the enable bit prior to abort
    hw_clear_bits(&dma_hw->ch[g_PioWriterDma].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    dma_channel_abort(g_PioWriterDma);
    dma_channel_acknowledge_irq0(g_PioWriterDma);
    
    // restart SM and reset program counter
    pio_sm_restart(pio0, 1);   
    pio_sm_clear_fifos(pio0, 1);
    pio_sm_exec(pio0, 1, pio_encode_jmp(g_PioWriterOffset));
  }
}

bool ENDEC::getBit(const uint8_t* buffer, size_t len, size_t bit)
{
  // value of a bit in a given MSB bitstream; 0 when out of bounds
  return (bit < len*8) ? ((buffer[bit >> 3] >> (7 - (bit & 7))) & 1) : false;
}

bool ENDEC::lockPLL(uint32_t timeout_us)
{
  // assert RGATE and begin reading once DRUN circuit detects preambles (zeros)
  const absolute_time_t deadline = make_timeout_time_us(timeout_us);
  while (!gpio_get(2)) // DRUN
  {
    if (time_reached(deadline))
    {
      return false;
    }
  }
  
  setReadGate(true);  
  return true;
}

bool ENDEC::readFifo16(uint16_t& out)
{
  const absolute_time_t deadline = make_timeout_time_us(TIMEOUT_FIFO_READS_US);
  while (pio_sm_is_rx_fifo_empty(pio0, 0))
  {
    if (time_reached(deadline))
    {
      return false;
    }
  }
  
  out = (uint16_t)pio_sm_get(pio0, 0);
  return true;
}

bool ENDEC::waitForTrackStart()
{
  // wait for the beginning of the track for /INDEX to go low
  absolute_time_t index_deadline = make_timeout_time_us(TIMEOUT_DISK_ROTATION_US);
  while (gpio_get(6))
  {
    if (time_reached(index_deadline))
    {
      hdd.setLastResult(HDD_STATUS_NOT_READY);
      return false;
    }
  }
  
  return true;
}

void ENDEC::prepareWriteDMA(const void* buffer, size_t words)
{
  // read addr: MFM/RLL encoded buffer, length: in 32-bit words, write addr: PIO0 SM1 TX FIFO
  dma_channel_config cfg = dma_channel_get_default_config(g_PioWriterDma);
  
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
  channel_config_set_read_increment(&cfg, true);
  channel_config_set_write_increment(&cfg, false);
  channel_config_set_dreq(&cfg, pio_get_dreq(pio0, 1, true));
  
  dma_channel_configure(g_PioWriterDma, &cfg, &pio0->txf[1], buffer, words, false);
}

bool ENDEC::writeWholeTrack()
{
  // check /READY and /WFAULT before commencing
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  // wait for /INDEX to go low - start of the track
  bool startOfTrack = true;
  if (!waitForTrackStart())
  {
    return false;
  }
  
  // write
  // abort on /READY high, /WFAULT low, PIO transfer done or /INDEX low (end-of-track)  
  setWriteGate(true);  
  while (!gpio_get(15) &&
         gpio_get(20) &&
         (startOfTrack || gpio_get(6)) &&
         !pio_sm_is_tx_fifo_empty(pio0, 1))
  {
    if (gpio_get(6)) // /INDEX went high, reset start of track flag
    {
      startOfTrack = false;
    }
  }
  setWriteGate(false);
  
  // aborted due to these two?
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  hdd.setLastResult(HDD_STATUS_OK);
  return true;
}

uint8_t ENDEC::getPrecompNibble(bool wdata, Precomp precomp)
{
  // prepare a 4-bit nibble containing write and precompensation information
  // bits 3-0: /EARLY, /LATE, unused, WDATA -> to be routed to GPIO 11, 10, 9, 8 in this order
  
  if (!wdata) // transition
  {
    return (1 << 3) | (1 << 2);     // /EARLY and /LATE high, WDATA low
  }
  
  switch(precomp)
  {
  case PRECOMP_NOMINAL:
  default:
    return (1 << 3) | (1 << 2) | 1; // /EARLY, /LATE, WDATA high
  case PRECOMP_EARLY:
    return (1 << 2) | 1;            // /EARLY low, /LATE, WDATA high
  case PRECOMP_LATE:
    return (1 << 3) | 1;            // /EARLY high, /LATE low, WDATA high
  }
};

uint8_t ENDEC::findSync(uint16_t pattern, uint16_t& partial, uint8_t& bitShift)
{
  // find a unique 16bit MFM/RLL encoded sync pattern in a 32bit sliding window
  // returns HDD_STATUS_...
  // RGATE is on, data incoming
  uint16_t word1;
  if (!readFifo16(word1))
  {
    return HDD_STATUS_TIMEOUT;
  }
  
  uint16_t word2;
  if (!readFifo16(word2))
  {
    return HDD_STATUS_TIMEOUT;
  }
  
  // maximum of how many 16bit FIFO words to obtain before failing
  for (uint16_t attempt = 0; attempt < MAX_SYNC_SEARCH_WORDS; attempt++)
  {
    // low word (word2): most recently obtained bits
    uint32_t window = ((uint32_t)word1 << 16) | word2;
    
    // try all 16 shifts to the right
    for (bitShift = 0; bitShift <= 16; bitShift++)
    {
      if (((window >> bitShift) & 0xFFFF) == pattern)
      {
        if (bitShift) // shifted
        {
          // partial: synchronized next valid data - shift them to MSB so they can be OR'ed with more incoming bits
          partial = (uint16_t)((word2 & ((1 << bitShift) - 1)) << (16 - bitShift));
        }
        else // exact word boundary match
        {
          partial = 0; // subsequent readFifo calls are aligned
        }

        return HDD_STATUS_OK;
      }
    }

    // no match, drop the oldest and pull new word
    word1 = word2;
    if (!readFifo16(word2))
    {
      return HDD_STATUS_TIMEOUT;
    }
  }
  
  // sync pattern not found
  return HDD_STATUS_NO_SECTOR_ID;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// MFM ///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

INLINE uint8_t ENDEC::extractMFMData(uint16_t r)
{
  // data bits 7-0 at positions 14,12,10,8,6,4,2,0
  uint8_t d = 0;
  for (int i = 7; i >= 0; i--)
      d = (uint8_t)((d << 1) | ((r >> (i * 2)) & 1));
  return d;
}

INLINE uint8_t ENDEC::extractMFMClock(uint16_t r)
{
  // clock bits 7-0 at positions 15,13,11,9,7,5,3,1
  uint8_t c = 0;
  for (int i = 7; i >= 0; i--)
      c = (uint8_t)((c << 1) | ((r >> (i * 2 + 1)) & 1));
  return c;
}

bool ENDEC::decodeMFM(uint8_t* out, size_t& count, uint16_t& partial, const uint8_t& bitShift, CRC* crc)
{
  // decode subsequent MFM data right after a successful sync from findSync()
  // partial: first valid data, shifted left bitShift times, or 0 if FIFO data is right on
  // RGATE is on, data incoming
  for (size_t idx = 0; idx < count; idx++)
  {
    uint16_t word;
    if (!readFifo16(word))
    {
      count = idx; // byte count that were successfully decoded prior to error
      return false;
    }
    
    // if not on exact boundary: OR with previous valid data; update window
    const uint16_t raw16 = (bitShift == 0) ? word : (partial | (word >> bitShift));
    if (bitShift)
    {
      partial = (uint16_t)((word & ((1 << bitShift) - 1)) << (16 - bitShift));  
    }
    
    // ignore 8 clock bits and extract 8 data bits
    out[idx] = extractMFMData(raw16);
    if (crc)
    {
      crc->add(out[idx]); // compute CRC on-the-fly
    }
  }
  
  return true;
}

uint8_t ENDEC::applyMFMPrecomp(bool& prev2, bool& prev1, bool& comp, bool next, bool clockAbsent)
{
  const Precomp precomp = MFM_PRECOMP[(((uint8_t)prev2) << 3) | (((uint8_t)prev1) << 2) | (((uint8_t)comp) << 1) | (uint8_t)next];
  const bool clock = clockAbsent ? false : (!prev1 && !comp); // MFM clock rules, forced false if dropping clock
  const bool data = comp;
     
  // slide the window 1 bit
  prev2 = prev1;
  prev1 = comp;
  comp = next;
  
  // to be fed into PIO:
  // bits 7-4: /EARLY, /LATE, unused, WDATA for CkN
  // bits 3-0: /EARLY, /LATE, unused, WDATA for DN
  return (getPrecompNibble(clock, precomp) << 4) | getPrecompNibble(data, precomp);
}

void ENDEC::encodeMFM(const uint8_t* input, size_t len, const std::vector<size_t>& dropClockBitOffsets, std::vector<uint32_t>& output)
{
  output.clear();
  if (!len)
  {
    return;
  }
    
  // dropClockBitOffsets: data buffer BIT OFFSETS where a clock bit should be dropped
  // must be sorted ascending
  auto sync = dropClockBitOffsets.begin();
  auto nextSyncFunc = [&]() -> size_t
  {
    // next bit or SIZE_MAX if there's none
    return (sync != dropClockBitOffsets.end()) ? (*sync) : SIZE_MAX;
  };  
  size_t nextSync = nextSyncFunc();
    
  // MFM sliding window initial values
  bool prev2 = false;
  bool prev1 = false;
  bool comp = getBit(input, len, 0);
  bool next = false;
  
  const size_t totalBits = len * 8;
  output.reserve(len * 2);
  
  size_t bit = 0; 
  while (bit < totalBits)
  {
    uint32_t fifoWord = 0;
    
    // 4 bytes per 32bit FIFO word
    for (uint8_t byte = 0; byte < 4; ++byte)
    {
      const size_t pos = bit + byte; // comp
      const bool clockAbsent = (pos == nextSync);
      if (clockAbsent)
      {
        ++sync;
        nextSync = nextSyncFunc();
      }
      
      // each NRZ bit occupies a byte in the FIFO word: two 4-bit nibbles for CkN, DN containing transition and precomp bits
      next = getBit(input, len, pos + 1); // lookahead for applyMFMPrecomp, 0 if no bits follow
      fifoWord = (fifoWord << 8) | applyMFMPrecomp(prev2, prev1, comp, next, clockAbsent);
    }
    
    output.push_back(fifoWord);
    bit += 4;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// RLL ///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

INLINE bool ENDEC::extractRLLByte(uint32_t& buffer, int& bufferBits, uint8_t& output, uint8_t& bitCount, bool ignoreCodingErrors)
{
  // decode 1 RLL codeword to NRZ output and bitCount
  while (bufferBits < 8) // keep topped up to maximum RLL codeword length (8)
  {
    uint16_t word;
    if (!readFifo16(word))
    {
      return false;
    }
 
    // next bit(s) to be decoded MSB aligned, bufferBits: how many top bits are valid
    buffer |= ((uint32_t)word) << (16 - bufferBits);
    bufferBits += 16;
  }
  
  const RLLBytesTable& table = (m_RLLCoding == RLLCoding::WD) ? RLL_BYTES_TABLE_WD : RLL_BYTES_TABLE_ST; 
  const RLLtoNRZ& entry = table[buffer >> 24]; // codeword might be 4, 6 or 8 bits long - take whole byte
  
  // no match
  if (entry.nrzBits == 0)
  {
    if (!ignoreCodingErrors)
    {
      return false;  
    }

    // slide the window by 1 and return 0 bits read; function fails only if FIFO read fails
    buffer <<= 1;
    bufferBits -= 1;    
    bitCount = 0;
    output = 0;
    return true;  
  }
 
  // slide the window by exact number of bits consumed
  buffer <<= entry.rllBits;
  bufferBits -= entry.rllBits;
 
  output = entry.nrzPattern;
  bitCount = entry.nrzBits;
  return true;
}

bool ENDEC::decodeRLL(uint8_t* out, size_t& count, const uint16_t& partial, const uint8_t& bitShift, CRC* crc, bool ignoreCodingErrors)
{
  uint32_t buffer = ((uint32_t)partial) << 16; // returned from findSync()
  int bufferBits = bitShift;
  
  // each valid RLL codeword may produce a variable number of decoded NRZ bits (2, 3 or 4)
  // accumulate for full 8 bits at least
  uint32_t bitsAccumulated = 0;
  uint8_t bitsAvailable = 0;

  for (size_t idx = 0; idx < count; idx++)
  {
    while (bitsAvailable < 8)
    {
      uint8_t decodedByte;
      uint8_t decodedBits;
      
      // "ignore coding errors" used for dump/debug purposes
      if (!extractRLLByte(buffer, bufferBits, decodedByte, decodedBits, ignoreCodingErrors))
      {
        count = idx;  // coding errors: returns count of bytes successfully read
        return false;
      }
      if (!decodedBits)
      {
        continue; // coding errors ignored: buffer is consumed 1 bit
      }        
      
      bitsAccumulated = (bitsAccumulated << decodedBits) | decodedByte;
      bitsAvailable += decodedBits;
    }
        
    // whole byte ready at least
    bitsAvailable -= 8;
    out[idx] = (uint8_t)(bitsAccumulated >> bitsAvailable);
    if (crc)
    {
      crc->add(out[idx]); // compute CRC on-the-fly
    }
    
    // discard processed byte
    bitsAccumulated &= (bitsAvailable == 0) ? 0 : (1 << bitsAvailable) - 1;
  }
  
  return true;
}

void ENDEC::encodeRLL(const uint8_t* input, size_t len, const std::vector<size_t>& insertSyncByteOffsets, std::vector<uint32_t>& output)
{
  output.clear();
  if (!len)
  {
    return;
  }
  
  // phase 1: convert input data to RLL bitstream (+ sync marks as necessary), and pack into bytes
  // phase 2: then go through them and output words for the PIO FIFO with write precompensation
  std::vector<uint8_t> rllBytes;  
  rllBytes.reserve(len * 2);
  
  size_t rllBitCount = 0; 
  auto pushRLLBit = [&](bool bit)
  {
    if ((rllBitCount & 7) == 0)
    {
      rllBytes.push_back(0);
    }
    if (bit)
    {
      rllBytes.back() |= (uint8_t)(1 << (7 - (rllBitCount & 7)));
    }
    rllBitCount++;
  };
 
  // insertSyncByteOffsets: data buffer BYTE OFFSETS before which RLL 1000000010010000 should be inserted
  // must be sorted ascending
  static const uint8_t SYNC_BITS[] = { 1,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0 };
  auto sync = insertSyncByteOffsets.begin();
  auto nextSyncFunc = [&]() -> size_t
  {
    return (sync != insertSyncByteOffsets.end()) ? (*sync) * 8 : SIZE_MAX;
  };  
  size_t nextSync = nextSyncFunc();
  
  const RLLLookaheadTable& table = (m_RLLCoding == RLLCoding::WD) ? RLL_LOOKAHEAD_TABLE_WD : RLL_LOOKAHEAD_TABLE_ST; 
   
  const size_t totalNrzBits = len * 8;
  size_t pos = 0;
  while (pos < totalNrzBits)
  {
    // insert RLL sync mark
    if (pos >= nextSync)
    {
      ++sync;
      nextSync = nextSyncFunc();
      
      for (uint8_t b : SYNC_BITS)
      {
        pushRLLBit(b);
      }
    }
    
    // resolve RLL codeword from up to 4 lookahead NRZ bits
    uint8_t lookAhead = 0;
    for (uint8_t k = 0; k < 4; k++)
    {
      lookAhead = (uint8_t)(lookAhead << 1) | getBit(input, len, pos + k);
    }
    
    const RLLtoNRZ& entry = table[lookAhead];
    if ((pos + entry.nrzBits) <= nextSync) // codeword must not cross into a pending sync mark
    {
      for (int8_t b = entry.rllBits - 1; b >= 0; b--)
      {
        pushRLLBit((entry.rllPattern >> b) & 1);
      }
      
      pos += entry.nrzBits;
    }
    
    // try if there are remainders right before a sync mark
    else
    {
      bool isZeroPreamble = (nextSync != SIZE_MAX);
      for (size_t p = pos; isZeroPreamble && p < nextSync; p++)
      {
        if (getBit(input, len, p))
        {
          isZeroPreamble = false;
        }
      }
 
      if (isZeroPreamble)
      {
        pos = nextSync; // jump to the sync boundary
        continue;
      }
 
      // RLL encoding failure; not adjacent to a pending sync mark
      break;
    }
  }
  
  // classify precomp and output one 4bit nibble per one RLL bit
  uint32_t word = 0;
  uint8_t nibblesInWord = 0;
  output.reserve((rllBitCount / 8) + 1); // in uint32_t's; eight nibbles in a 32bit word + 1 word padding
     
  for (size_t i = 0; i < rllBitCount; i++)
  {
    const bool wdata = getBit(rllBytes.data(), rllBytes.size(), i);
    Precomp precomp = PRECOMP_NOMINAL;
 
    if (wdata)
    {
      // precompensation applied if there's a transition
      // EARLY: four preceding bits of transition are 0100, four following 0000
      // LATE: four preceding bits 0000, four following 0010 (the other way around)
      // NOMINAL in all other cases
      
      uint8_t precedingRun = 4;
      if (i >= 3 && getBit(rllBytes.data(), rllBytes.size(), i - 3)) precedingRun = 2;
      else if (i >= 4 && getBit(rllBytes.data(), rllBytes.size(), i - 4)) precedingRun = 3;
 
      uint8_t followingRun = 4;
      if (i + 3 < rllBitCount && getBit(rllBytes.data(), rllBytes.size(), i + 3)) followingRun = 2;
      else if (i + 4 < rllBitCount && getBit(rllBytes.data(), rllBytes.size(), i + 4)) followingRun = 3;
 
      if ((precedingRun == 2) && (followingRun >= 4))
      {
        precomp = PRECOMP_EARLY;
      }
      else if ((precedingRun >= 4) && (followingRun == 2))
      {
        precomp = PRECOMP_LATE;
      }
    }
 
    word = (word << 4) | getPrecompNibble(wdata, precomp);
    if (++nibblesInWord == 8) // pushed when full
    {
      output.push_back(word);
      word = 0;
      nibblesInWord = 0;
    }
  }
 
  // pad to full 32 bits for PIO TX
  while (nibblesInWord > 0 && nibblesInWord < 8)
  {
    word = (word << 4) | getPrecompNibble(0, PRECOMP_NOMINAL);
    nibblesInWord++;
  }
  if (nibblesInWord == 8)
  {
    output.push_back(word);
  }
}

uint16_t ENDEC::getMaximumTrackBytes()
{
  // for buffer allocations; absolute max at nominal 3600RPM + 5% (disks should be within +- 2%)
  return (uint16_t)((g_WclockRate * 17500.0) / 8.0);
}

uint8_t ENDEC::getMaximumSectorCountFor(LLF* format, uint16_t sectorSizeBytes)
{
  const double nominalTrackBytes = (uint16_t)((g_WclockRate * 16666.67) / 8.0);
  
  // the length of ID and DATA field preambles, ID field contents + CRC, data field AM + CRC and one intersector gap
  // see LLF::formatWriteTrack()
  uint16_t overhead = 0;
  switch(format->getType())
  {
  case LLF::FormatType::WD: // 128, 256, 512, 1024B sectors, gap formula source see WD::formatWriteTrack()
  default:
    overhead = 46 /* ID and DATA field overhead */ + round(0.02*sectorSizeBytes) + 18; /* intersector gap */
    break;
  case LLF::FormatType::OMTI: // 512B sectors
    overhead = 46 /* ID and DATA field overhead */ + 36; /* intersector gap */
    break;
  case LLF::FormatType::HDC9224: // 512B sectors
    overhead = 44 /* ID and DATA field overhead */ + 38; /* intersector gap */
    break;
  case LLF::FormatType::XebecAdaptec: // 512B sectors
    overhead = 59 /* ID and DATA field overhead */ + 34; /* intersector gap */
    break;
  case LLF::FormatType::SM1040: // 512B sectors
    overhead = 53 /* ID and DATA field overhead */ + 32; /* intersector gap */
    break;
  }
  
  size_t result = (nominalTrackBytes / ((overhead*1.0) + sectorSizeBytes));
  if (result > MAX_SPT_LIMIT) // 128B sectors etc
  {
    result = MAX_SPT_LIMIT;
  }
  
  return (uint8_t)result;
}