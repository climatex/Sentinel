// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// VUVT SMEP SM 1040, an Iron Curtain RK07 emulator that used MFM drives for storage
// Each byte pair in sector data fields is recorded swapped: presumably, it processed them in 16bit big endian words
// Sector size: 512 bytes

#include "config.h"

// defined in main.cpp
extern volatile int g_IndexCount;

SM1040::SM1040()
{
  // one sector buffer 512B + 1 byte data address mark + 4 bytes ECC
  m_SectorBuffer.resize(517, 0);
  
  // default: RK07
  m_DriveTypeRK06 = false;
    
  // custom analyzeTrack() results
  m_AnalyzeCylNumberMismatch = false;
  m_AnalyzeHdNumberMismatch = false;
  m_AnalyzeActualCylNumber = 0;
  m_AnalyzeActualHdNumber = 0;
  m_AnalyzeLogDriveCylCount = 0;
  
  // scanID(): is track relocated?
  m_TrackIsRelocated = false;
  m_RelocationCyl = 0;
  m_RelocationHd = 0;
}

uint8_t* SM1040::getSectorBuffer()
{ 
  // single sector buffer of a data field, 512 bytes
  return &m_SectorBuffer[1]; // F8 data address mark ignored during data field read
}

bool SM1040::analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave)
{
  m_AnalyzeCylNumberMismatch = false;
  m_AnalyzeHdNumberMismatch = false;
  m_AnalyzeActualCylNumber = 0;
  m_AnalyzeActualHdNumber = 0;
  m_AnalyzeLogDriveCylCount = 0;
  
  startSector = 0;
  sectorSizeBytes = 0;
  interleave = 0;
  
  std::vector<uint8_t> sectors;
  sectors.reserve(idSamples);
  
  if (printOut)
  {
    printf(str_CHInfo, hdd.getPhysicalCylinder(), hdd.getPhysicalHead());
  }
  
  // sample from the beginning of the track
  if (!endec.waitForTrackStart())
  {
    if (printOut)
    {
      printf(hdd.getLastResultMessage());  
    }    
    return false;
  }
  
  for (uint8_t sample = 0; sample < idSamples; sample++)
  {
    uint16_t cylinder;
    uint8_t head;
    uint8_t sector;
    uint16_t cylinderCount;
    
    if (!scanID(&cylinder, &head, &sector, NULL, &cylinderCount))
    {
      if (printOut)
      {
        if (hdd.getLastResult() == HDD_STATUS_NO_SECTOR_ID)
        {
          // no sectors? track might be relocated
          if (m_TrackIsRelocated)
          {
            printf(str_AnalyzeRelocated, m_RelocationCyl, m_RelocationHd);
          }
          else
          {
            printf(str_AnalyzeNoSectors);  
          }          
        }
        else
        {
          printf(hdd.getLastResultMessage());          
        }
      }
      return false;
    }
    
    m_AnalyzeCylNumberMismatch |= (cylinder != hdd.getPhysicalCylinder());
    m_AnalyzeHdNumberMismatch |= (head != hdd.getPhysicalHead());
    
    m_AnalyzeActualCylNumber = cylinder;
    m_AnalyzeActualHdNumber = head;
    m_AnalyzeLogDriveCylCount = cylinderCount & 0x3FFF; // see below in scanID()
    sectors.push_back(sector);
  }
  
  // sector IDs obtained
  sectorSizeBytes = 512;
  calculateInterleave(sectors, sectorsPerTrack, startSector, interleave);
  
  // results per track
  if (printOut)
  {
    printf(str_AnalyzeSpt, sectorsPerTrack);
    printf(str_AnalyzeSectorSize, sectorSizeBytes);
    
    if (interleave)
    {
      printf(str_AnalyzeInterleave, interleave);
    }
    else
    {
      printf(str_AnalyzeBadInterleave);
    }
    
    if (m_AnalyzeCylNumberMismatch)
    {
      printf("*");
    }
    if (m_AnalyzeHdNumberMismatch)
    {
      printf("@");
    }
    
    printf(str_AnalyzeSectorOrder);
    for (const uint8_t& sector : sectors)
    {
      printf("%u ", sector);
    }    
  }
  
  return true;
}

void SM1040::getCustomAnalyzeTrackResults(bool& cylNumberMismatch, bool& hdNumberMismatch, uint16_t& actualCylNumber, uint8_t& actualHdNumber, uint16_t& logDriveCylCount)
{
  cylNumberMismatch = m_AnalyzeCylNumberMismatch;
  hdNumberMismatch = m_AnalyzeHdNumberMismatch;
  actualCylNumber = m_AnalyzeActualCylNumber;
  actualHdNumber = m_AnalyzeActualHdNumber;
  logDriveCylCount = m_AnalyzeLogDriveCylCount;
}

void SM1040::whipperSnapperBufferSwapper()
{
  // each byte pair is recorded swapped on disk
  for (uint16_t idx = 1; idx < 513; idx += 2)
  {
    const uint8_t first = m_SectorBuffer[idx];
    m_SectorBuffer[idx] = m_SectorBuffer[idx+1];
    m_SectorBuffer[idx+1] = first;
  }
}

bool SM1040::scanID(uint16_t* cylinder, uint8_t* head, uint8_t* sector, uint16_t* keyword, uint16_t* cylinderCount)
{ 
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  g_IndexCount = 0;  
  while (g_IndexCount < 2)
  {
    CRC32 crc(CRC::Type::SM1040_ID);
    
    if (!endec.lockPLL(TIMEOUT_DISK_ROTATION_US))
    {
      hdd.setLastResult(HDD_STATUS_NO_SECTOR_ID);
      return false;
    }

    uint16_t partial;
    uint8_t bitShift;
    const uint8_t status = endec.findSync(DEFAULT_MFM_SYNC_PATTERN, partial, bitShift);

    if (status == HDD_STATUS_TIMEOUT)
    {
      endec.setReadGate(false);
      if (hdd.checkReadyWriteFault()) // potential reason for timeout
      {
        hdd.setLastResult(HDD_STATUS_TIMEOUT);
      }
      return false;
    }
    else if (status == HDD_STATUS_NO_SECTOR_ID)
    {
      endec.setReadGate(false);
      continue;
    }
    
    crc.add(0xA1); // consumed by findSync() and not part of the read
    
    size_t count = 13;
    uint8_t idField[13]; // [FE][KEYWORD_HI][KEYWORD_LO][CYL_HI][CYL_LO][HEAD][SECTOR][CYLCOUNT_HI][CYLCOUNT_LO] + 4 bytes CRC
    const bool success = endec.decodeMFM(idField, count, partial, bitShift, &crc);
    endec.setReadGate(false);  // read gate can be deasserted now
   
    if (!success || (crc.get() != 0) || (idField[0] != 0xFE))
    {
      continue;
    }    
    
    // extract sector info
    uint8_t* ptr;
    
    // keyword (16 bits)
    // bits 0-3:   number of heads on drive, counted from 0 (0-15)
    // bits 4-7:   logical drive type: 0=RK06, 1=RK07
    // bit 8:      1=reduced write current (RWC) enabled here
    // bit 9:      1=write precompensation enabled here
    // bits 10-11: seek type (0: fast buffered, 1: slow, 2: fast algorithm, 3: fast buffered with 1 disk rotation wait after seeking)
    // bit 12:     1=relocation mode, see below
    // bit 13:     1=last sector on track
    // bit 14:     1=last sector on cylinder
    // bit 15:     1=last sector on logical drive
    if ((idField[1] & 0x10) || (idField[3] & 0x80)) // if relocation mode is on, or bad track bit is set, store relocation information - see below
    {
      m_TrackIsRelocated = true;
      m_RelocationCyl = ((uint16_t)(idField[5] & 0xF) << 8) | idField[6];
      m_RelocationHd = idField[5] >> 4;
      continue; // will return HDD_STATUS_NO_SECTOR_ID for this track
    }
    else
    {
      m_TrackIsRelocated = false;      
    }
    
    m_DriveTypeRK06 = (idField[2] & 0xF0) == 0;
    
    if (keyword)
    {
      ptr = (uint8_t*)keyword;
      ptr[1] = idField[1];
      ptr[0] = idField[2];
    }

    // cylinder number
    // bits 0-11: cylinder number (0-4095)
    // bit 15: 1=bad track - RELOCATION MODE    
    if (cylinder)
    {
      ptr = (uint8_t*)cylinder;
      ptr[1] = idField[3] & 0xF;
      ptr[0] = idField[4];
    }
    
    // head number  
    // NORMAL MODE: bits 0-3: head number (0-15)
    // RELOCATION MODE: if cylinder bit 15=1 (bad track):
    //              bits 0-3: MSB of relocation cylinder number
    //              bits 4-7: relocation head number
    if (head)
    {
      *head = idField[5] & 0xF;
    }
    
    // sector number
    // NORMAL MODE: bits 0-5: sector number (0-63)
    //                     6: 1=bad sector flag
    //                     7: 1=this is the last sector on track
    // RELOCATION MODE: if cylinder bit 15=1 (bad track):
    //              bits 0-7: LSB of relocation cylinder number
    if (sector)
    {
      *sector = idField[6] & 0x3F;
    }
    
    // cylinder count
    // bits: 0-11: cylinder count of a logical drive, counted from 1
    //      14-15: number of logical drives, counted from 0 (0: one,... 0-3)
    if (cylinderCount)
    {
      ptr = (uint8_t*)cylinderCount;
      ptr[1] = idField[7];
      ptr[0] = idField[8];
    }

    hdd.setLastResult(HDD_STATUS_OK);
    return true;
  }
  
  // no sector IDs whatsoever
  endec.setReadGate(false);
  if (hdd.checkReadyWriteFault())
  {
    hdd.setLastResult(HDD_STATUS_NO_SECTOR_ID);
  }
  return false;
}

bool SM1040::readSector(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  uint16_t cylinder = overrideCyl ? *overrideCyl : hdd.getPhysicalCylinder();
  uint8_t head = overrideHead ? *overrideHead : hdd.getPhysicalHead();
  
  // A1 consumed by findSync(), ident F8, 512 bytes, 4 byte CRC
  size_t dataFieldCount = 517;  
      
  for (uint8_t readAttempt = 0; readAttempt < READ_SECTOR_ATTEMPTS; readAttempt++)
  {
    CRC32 crc(CRC::Type::SM1040_DATA);
    bool found = false;
    
    // reseek on last attempt
    if (hdd.getParams()->ReseekOnSectorErrors && (READ_SECTOR_ATTEMPTS > 1) && (readAttempt == READ_SECTOR_ATTEMPTS-1))
    {
      const uint16_t cyl = hdd.getPhysicalCylinder();
      const uint8_t hd = hdd.getPhysicalHead();
      hdd.seekDrive(0, 0);
      hdd.seekDrive(cyl, hd);        
    }
    
    for (uint8_t locateAttempt = 0; locateAttempt < MAX_SPT_LIMIT; locateAttempt++)
    {
      uint16_t scanCyl;
      uint8_t scanHead;
      uint8_t scanSector;
      
      if (!scanID(&scanCyl, &scanHead, &scanSector))
      {
        return false; // no sector IDs whatsoever
      }
       
      if ((scanCyl == cylinder) && (scanHead == head) && (scanSector == sector))
      {
        found = true;
        break;
      }
    }
    
    if (!found)
    {          
      if (hdd.checkReadyWriteFault())
      {
        hdd.setLastResult(HDD_STATUS_NO_SECTOR_ID);
      }
      return false;
    }
    
    if (!endec.lockPLL(TIMEOUT_DATA_PREAMBLE_US))
    {
      continue;
    }
    
    uint16_t partial;
    uint8_t bitShift;
    uint8_t status = endec.findSync(DEFAULT_MFM_SYNC_PATTERN, partial, bitShift); 
    if (status == HDD_STATUS_TIMEOUT)
    {
      endec.setReadGate(false);
      if (hdd.checkReadyWriteFault())
      {
        hdd.setLastResult(HDD_STATUS_TIMEOUT);
      }
      return false;
    }
    else if (status == HDD_STATUS_NO_SECTOR_ID)
    {
      endec.setReadGate(false);
      continue;
    }
    
    crc.add(0xA1); // part of computation
    const bool success = endec.decodeMFM(m_SectorBuffer.data(), dataFieldCount, partial, bitShift, &crc);    
    endec.setReadGate(false); // read gate can be deasserted now
    
    // data address mark must be F8
    if (!success || (m_SectorBuffer[0] != 0xF8))
    {
      continue;
    }

    if (crc.get() != 0)
    {
      // last, try computing correction
      if (readAttempt == READ_SECTOR_ATTEMPTS-1)
      {
        if (!crc.tryComputeCorrection(m_SectorBuffer.data(), dataFieldCount))
        {
          hdd.setLastResult(HDD_STATUS_DATA_ERROR);
          return false;
        }
        
        hdd.setLastResult(HDD_STATUS_DATA_CORRECTED);
        return true;
      }
      
      continue;      
    }
    
    whipperSnapperBufferSwapper();
    hdd.setLastResult(HDD_STATUS_OK);
    return true;
  }
  
  endec.setReadGate(false);
  if (hdd.checkReadyWriteFault())
  {
    hdd.setLastResult(HDD_STATUS_NO_DATA_ID);
  }
  return false;
}

bool SM1040::writeSector(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  std::vector<uint8_t> data;
  std::vector<size_t> clockBits;
  
  // reserve 13 bytes zero preamble, A1 (dropped clock), F8 and sector data, followed by CRC (4 bytes) and a zero byte
  data.reserve(532);
  
  // prepare CRC counter
  CRC32 crc(CRC::Type::SM1040_DATA);
    
  // append 13 bytes preamble and info where to insert MFM/RLL sync
  data.insert(data.end(), 13, 0); 
  
  // insert 0xA1, drop Ck2
  clockBits.push_back(data.size() * 8 + 5);  
  data.push_back(0xA1); 
  crc.add(0xA1); // used with CRC computation  

  // ID part of CRC computation
  crc.add(0xF8);
  data.push_back(0xF8);
  
  // sector data, bytes swapped
  for (size_t i = 0; i < 512; i++)
  {
    const uint16_t swapper = (i % 2) ? i-1 : i+1;
    const uint8_t byte = getSectorBuffer()[swapper];
    data.push_back(byte);
    crc.add(byte);
  }
  
  // store data field CRC
  const uint32_t dataCrcVal = crc.get();
  uint8_t* crcPtr = (uint8_t*)&dataCrcVal;
  data.push_back(crcPtr[3]);
  data.push_back(crcPtr[2]);  
  data.push_back(crcPtr[1]);
  data.push_back(crcPtr[0]);
  
  // 0
  data.push_back(0);
      
  std::vector<uint32_t> dmaBuffer;
  endec.encodeMFM(data.data(), data.size(), clockBits, dmaBuffer);
  endec.prepareWriteDMA(dmaBuffer.data(), dmaBuffer.size());  
  
  uint16_t cylinder = overrideCyl ? *overrideCyl : hdd.getPhysicalCylinder();
  uint8_t head = overrideHead ? *overrideHead : hdd.getPhysicalHead();  

  bool found = false;  
  for (uint8_t locateAttempt = 0; locateAttempt < MAX_SPT_LIMIT; locateAttempt++)
  {
    uint16_t scanCyl;
    uint8_t scanHead;
    uint8_t scanSector;
    
    if (!scanID(&scanCyl, &scanHead, &scanSector))
    {
      return false; // no sector IDs whatsoever
    }
    
    if ((scanCyl == cylinder) && (scanHead == head) && (scanSector == sector))
    {
      found = true;
      break;
    }
  }
  
  if (!found)
  {         
    if (hdd.checkReadyWriteFault())
    {
      hdd.setLastResult(HDD_STATUS_NO_SECTOR_ID);
    }
    return false;
  }
   
  // write
  // abort on /READY high, /WFAULT low, PIO transfer done or /INDEX low
  endec.setWriteGate(true);  
  while (!gpio_get(15) &&
         gpio_get(20) &&
         gpio_get(6) &&
         !pio_sm_is_tx_fifo_empty(pio0, 1))
  {
    tight_loop_contents();
  }
  endec.setWriteGate(false);
  
  // aborted due to these two?
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  hdd.setLastResult(HDD_STATUS_OK);
  return true;
}

bool SM1040::formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  // dataFields: pointer to buffer containing data for the data fields in sequential sector order
  // dataFields null: format the track with FF's
  if (interleave.empty())
  {
    hdd.setLastResult(HDD_STATUS_INVALID_ARGS);
    return false;
  }
  
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  const uint8_t startSector = interleave[0];
  const uint8_t sectorCount = interleave.size();
  uint16_t cylinder = overrideCyl ? *overrideCyl : hdd.getPhysicalCylinder();
  uint8_t head = overrideHead ? *overrideHead : hdd.getPhysicalHead();  
   
  // leave some slack (next /INDEX stops write)
  std::vector<uint8_t> track;
  const uint16_t maxTrackBytes = endec.getMaximumTrackBytes();
  track.resize(maxTrackBytes, 0); // fill with gap byte 0
  
  // bit offsets in the track buffer where to drop clock bits
  std::vector<size_t> clockBits;  
   
  // skip initial gap, at least 32 bytes (+ own latency) gap byte  
  uint8_t* data = track.data();
  size_t offset = 32;  
  for (size_t sec = 0; sec < sectorCount; sec++)
  {
    // safety margin
    if (offset+600 >= maxTrackBytes)
    {
      break;
    }
    
    // ID field preamble: 16 bytes zeros
    memset(data+offset, 0, 16);
    offset += 16;
    
    CRC32 idFieldCrc(CRC::Type::SM1040_ID);
    
    // 0xA1 with dropped Ck2 follows
    idFieldCrc.add(0xA1); // used with CRC computation
    clockBits.push_back(offset * 8 + 5);    
    data[offset++] = 0xA1;
    
    // IDENT 0xFE
    data[offset++] = 0xFE;
    idFieldCrc.add(0xFE);
    
    // KEYWORD_HI
    uint8_t keywordHi = 0;
    // default zeros used for: track relocation none, 1 logical drive on disk
    // bit0=1: reduced write current on track
    if (hdd.getParams()->UseReduceWriteCurrent && (hdd.getPhysicalCylinder() >= hdd.getParams()->RWCStartCyl))
    {
      keywordHi |= 1;
    }
    // bit1=1: write precompensation used on track
    if (hdd.getParams()->UseWritePrecomp && (hdd.getPhysicalCylinder() >= hdd.getParams()->WritePrecompStartCyl))
    {
      keywordHi |= 2;
    }
    // bits2-3: seek type - fast buffered or slow; skip algorithm seek and "special" (fast buffered with 1 rotation wait)
    // bit2=0: fast buffered, 1: slow, like a floppy drive
    if (hdd.getParams()->SlowSeek)
    {
      keywordHi |= 4;
    }
    // bit5=1: last (physical ?) sector on track
    if (sec == sectorCount-1)
    {
      keywordHi |= 0x20;
      
      // bit6=1: also last sector on cylinder
      if (hdd.getPhysicalHead() == hdd.getParams()->Heads-1)
      {
        keywordHi |= 0x40;
      }
      // bit7=1: also last sector on logical drive (1 logical drive on disk: last sector on disk)
      if (hdd.getPhysicalCylinder() == hdd.getParams()->Cylinders-1)
      {
        keywordHi |= 0x80;
      }
    }
    data[offset++] = keywordHi;
    idFieldCrc.add(keywordHi);
    
    // KEYWORD_LO
    // bits0-3: number of heads starting from 0
    uint8_t keywordLo = (hdd.getParams()->Heads-1) & 0xF;
    // bit4=0: RK06, 1: RK07
    if (!m_DriveTypeRK06)
    {
      keywordLo |= 0x10;
    }
    data[offset++] = keywordLo;
    idFieldCrc.add(keywordLo);
    
    // CYL_HI
    // bits 0-3: MSB of cylinder number 0-4095
    // bit7=1: bad track, relocation mode (ignored)
    uint8_t cylHi = (cylinder & 0xFFF) >> 8;
    data[offset++] = cylHi;
    idFieldCrc.add(cylHi);
    
    // CYL_LO
    data[offset++] = (uint8_t)cylinder;
    idFieldCrc.add((uint8_t)cylinder);
    
    // HEAD
    uint8_t hd = head & 0xF; // 4 bits used max.
    data[offset++] = hd;
    idFieldCrc.add(hd);
    
    // SECTOR
    const uint8_t logicalSector = interleave[sec];
    data[offset] = logicalSector;
    if (sec == sectorCount-1)
    {
      // bit7=1: last physical sector on track
      data[offset] |= 0x80;
    }
    idFieldCrc.add(data[offset]);
    offset++;
    
    // CYLCOUNT_HI (bits 7, 6 set 0 for 1 logical drive), CYLCOUNT_LO
    const uint8_t cylCountHi = (hdd.getParams()->Cylinders & 0xFFF) >> 8;
    const uint8_t cylCountLo = (uint8_t)hdd.getParams()->Cylinders;
    data[offset++] = cylCountHi;
    data[offset++] = cylCountLo;
    idFieldCrc.add(cylCountHi);
    idFieldCrc.add(cylCountLo);
        
    // store ID field CRC
    const uint32_t idCrcVal = idFieldCrc.get();
    uint8_t* crc = (uint8_t*)&idCrcVal;
    data[offset++] = crc[3];
    data[offset++] = crc[2];
    data[offset++] = crc[1];
    data[offset++] = crc[0];
    
    // data field preamble: 16 bytes zeros
    memset(data+offset, 0, 16);
    offset += 16;
    
    CRC32 dataFieldCrc(CRC::Type::SM1040_DATA);  
    
    // 0xA1 with dropped clock follows
    dataFieldCrc.add(0xA1); // used with CRC computation
    clockBits.push_back(offset * 8 + 5);
    data[offset++] = 0xA1;
    
    // DATA ident 0xF8
    data[offset++] = 0xF8;
    dataFieldCrc.add(0xF8);
    
    // DATA, swap each byte pair
    const uint16_t pos = (logicalSector-startSector)*512;
    for (size_t i = 0; i < 512; i++)
    {
      const uint16_t swapper = (i % 2) ? i-1 : i+1;
      const uint8_t dataByte = dataFields ? dataFields[pos + swapper]
                                          : 0xFF; // format
      data[offset++] = dataByte;
      dataFieldCrc.add(dataByte);
    }
    
    // store data field CRC
    const uint32_t dataCrcVal = dataFieldCrc.get();
    crc = (uint8_t*)&dataCrcVal;
    data[offset++] = crc[3];
    data[offset++] = crc[2];
    data[offset++] = crc[1];
    data[offset++] = crc[0];
        
    // one zero and 32 bytes zero intersector gap
    offset += 33;
  }
  
  // encode and prepare write DMA
  std::vector<uint32_t> dmaBuffer;
  endec.encodeMFM(track.data(), track.size(), clockBits, dmaBuffer);
  
  endec.prepareWriteDMA(dmaBuffer.data(), dmaBuffer.size());
  endec.writeWholeTrack();
    
  return hdd.getLastResult() == HDD_STATUS_OK;
}