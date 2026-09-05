// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// ADT 4700, ST506 format - 32 sectors of 256 bytes

#include "config.h"

// defined in main.cpp
extern volatile uint64_t g_IndexCount;

ADT::ADT()
{
  // 1 byte sync mark + one 256 byte sector buffer + 4 bytes CRC
  m_SectorBuffer.resize(261, 0);
  
  m_AnalyzeCylNumberMismatch = false;
  m_AnalyzeHdNumberMismatch = false;
  m_AnalyzeActualCylNumber = 0;
  m_AnalyzeActualHdNumber = 0;
  
  // scanID(): is track relocated?
  m_TrackIsRelocated = false;
  m_RelocationCyl = 0;
  m_RelocationHd = 0;
}

uint8_t* ADT::getSectorBuffer()
{ 
  // single sector buffer of a data field, 256 bytes
  return &m_SectorBuffer[1]; // skip sync mark byte 00
}

bool ADT::analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave)
{
  m_AnalyzeCylNumberMismatch = false;
  m_AnalyzeHdNumberMismatch = false;
  m_AnalyzeActualCylNumber = 0;
  m_AnalyzeActualHdNumber = 0;
  
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
    
    if (!scanID(&cylinder, &head, &sector))
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
    sectors.push_back(sector);
  }
  
  // sector IDs obtained
  sectorSizeBytes = 256;
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

void ADT::getCustomAnalyzeTrackResults(bool& cylNumberMismatch, bool& hdNumberMismatch, uint16_t& actualCylNumber, uint8_t& actualHdNumber)
{
  cylNumberMismatch = m_AnalyzeCylNumberMismatch;
  hdNumberMismatch = m_AnalyzeHdNumberMismatch;
  actualCylNumber = m_AnalyzeActualCylNumber;
  actualHdNumber = m_AnalyzeActualHdNumber;
}

bool ADT::scanID(uint16_t* cylinder, uint8_t* head, uint8_t* sector, uint16_t* reserved1, uint16_t* reserved2)
{  
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }  
  
  const uint64_t startCount = g_IndexCount;
  while ((g_IndexCount - startCount) < 2)
  {    
    CRC32 crc(CRC::Type::ADT);
    
    if (!endec.lockPLL(TIMEOUT_DISK_ROTATION_US))
    {
      hdd.setLastResult(HDD_STATUS_NO_SECTOR_ID);
      return false;
    }

    // address mark: 5 bytes with missing clock bits, in this order:
    // 0x28 (dropped Clk0), 0x50 (dropped Clk1), 0xA1 (dropped Clk2), 0x42 (dropped Clk3), 0x85 (dropped Clk4)
    // MFM: 0x2448, 0x9122, 0x4489, 0x1224, 0x4891
    uint16_t partial;
    uint8_t bitShift;
    uint8_t status = endec.findSync(0x2448, partial, bitShift); // find 0x28 with dropped Clk0
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
       
    // we need to resync immediately within a short time window
    endec.setReadGate(false);
    if (!endec.lockPLL(TIMEOUT_DATA_PREAMBLE_US))
    {
      hdd.setLastResult(HDD_STATUS_NO_SECTOR_ID);
      return false;    
    }
    
    // ID field mark: 32 00, no dropped clock
    status = endec.findSync(0xA524, partial, bitShift); // sync on 0x32, no dropped clock
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
    
    uint8_t idField[9];
    size_t count = 9; // [00][CYL_LO][CYL_HI][SPD_HEAD][SECTOR] + 4 byte CRC
    bool success = endec.decodeMFM(idField, count, partial, bitShift, &crc);
    endec.setReadGate(false); // ID field has been read
    
    if (!success || (crc.get() != 0) || (idField[0] != 0))
    {
      continue;
    }
    
    // check SPD_HEAD bit D (defective track) - ID field contains relocation information
    if (idField[3] & 0x20)
    {
      m_TrackIsRelocated = true;
      m_RelocationCyl = (((uint16_t)idField[2]) << 8) | idField[1];
      m_RelocationHd = idField[3] & 0x1F;
      continue; // will return HDD_STATUS_NO_SECTOR_ID for this track
    }
    else
    {
      m_TrackIsRelocated = false;      
    }
    
    // check SPD_HEAD bit S (spare track)
    if (idField[3] & 0x80)
    {
      // ID field would contain cylinder and head information of the defective track this is replacing
      // return actual physical cylinder and head number
      
      if (cylinder)
      {
        *cylinder = hdd.getPhysicalCylinder();
      }
      if (head)
      {
        *head = hdd.getPhysicalHead();
      }
    }
    else // normal mode, use ID field information
    {
      if (cylinder)
      {
        *cylinder = (((uint16_t)idField[2]) << 8) | idField[1];
      }    
      if (head)
      {
        *head = idField[3] & 0x1F;
      }
    }

    // extract sector info        
    if (sector)
    {
      *sector = idField[4];
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

bool ADT::readSector(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  uint16_t cylinder = overrideCyl ? *overrideCyl : hdd.getPhysicalCylinder();
  uint8_t head = overrideHead ? *overrideHead : hdd.getPhysicalHead();
  
  // 0x32 consumed by findSync(), 00, 256 bytes, 4 byte CRC
  size_t dataFieldCount = 261;
      
  for (uint8_t readAttempt = 0; readAttempt < READ_SECTOR_ATTEMPTS; readAttempt++)
  {
    CRC32 crc(CRC::Type::ADT);
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
    
    // data field mark: 32 00, no dropped clock
    uint16_t partial;
    uint8_t bitShift;
    uint8_t status = endec.findSync(0xA524, partial, bitShift); // sync on 0x32, normal clock
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
    
    const bool success = endec.decodeMFM(m_SectorBuffer.data(), dataFieldCount, partial, bitShift, &crc);
    endec.setReadGate(false); // read gate can be deasserted now
    
    if (!success || (m_SectorBuffer[0] != 0))
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

bool ADT::writeSector(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  uint16_t cylinder = overrideCyl ? *overrideCyl : hdd.getPhysicalCylinder();
  uint8_t head = overrideHead ? *overrideHead : hdd.getPhysicalHead();
   
  std::vector<uint8_t> data;
  std::vector<size_t> clockBits; // no sync offsets: data field begins 0x32 with no dropped clock
  
  // reserve 10 bytes zero preamble, 32, 00 and sector data, followed by CRC (4 bytes) and 2 zero bytes
  data.reserve(274);
  
  // prepare CRC counter
  CRC32 crc(CRC::Type::ADT);
    
  // append 10 bytes preamble + 32 + 00
  data.insert(data.end(), 10, 0); 
  data.push_back(0x32);
  data.push_back(0);

  // sector data
  for (size_t i = 0; i < 256; i++)
  {
    const uint8_t byte = getSectorBuffer()[i];
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
  
  // 2 zero bytes
  data.push_back(0);
  data.push_back(0);
      
  std::vector<uint32_t> dmaBuffer;
  endec.encodeMFM(data.data(), data.size(), clockBits, dmaBuffer);
  endec.prepareWriteDMA(dmaBuffer.data(), dmaBuffer.size());  

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

bool ADT::formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields, uint16_t* overrideCyl, uint8_t* overrideHead)
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
  
  // clock bits to drop
  std::vector<size_t> clockBits;  
  
  // skip initial gap, at least 32 bytes (+ own latency) gap byte 0
  uint8_t* data = track.data();
  size_t offset = 32;  
  for (size_t sec = 0; sec < sectorCount; sec++)
  {
    // safety margin
    if (offset+600 >= maxTrackBytes)
    {
      break;
    }
    
    // GAP1 zero byte gap, 8 bytes zeros
    offset += 8;
    
    // 0x28 with dropped Ck0
    clockBits.push_back(offset * 8 + 7);
    data[offset++] = 0x28;
    // 0x50 with dropped Ck1
    clockBits.push_back(offset * 8 + 6);
    data[offset++] = 0x50;
    // 0xA1 with dropped Ck2
    clockBits.push_back(offset * 8 + 5);
    data[offset++] = 0xA1;
    // 0x42 with dropped Ck3
    clockBits.push_back(offset * 8 + 4);    
    data[offset++] = 0x42;
    // 0x85 with dropped Ck4
    clockBits.push_back(offset * 8 + 3);    
    data[offset++] = 0x85;
        
    offset += 15; // 15 bytes zeros
    data[offset++] = 0x32; // ID sync byte
    data[offset++] = 0;    // zero gap    
    
    CRC32 crc(CRC::Type::ADT);
    
    // CYL_LO
    const uint8_t cylLow = (uint8_t)cylinder;
    data[offset++] = cylLow;
    crc.add(cylLow);
    
    // CYL_HI
    const uint8_t cylHi = cylinder >> 8;
    data[offset++] = cylHi;
    crc.add(cylHi);
    
    // SPD_HEAD
    // top 3 bits: spare / protected / defective track: set to 0
    const uint8_t spdHead = head & 0x1F;
    data[offset++] = spdHead;
    crc.add(spdHead);
    
    // SECTOR
    const uint8_t logicalSector = interleave[sec];
    data[offset++] = logicalSector;
    crc.add(logicalSector);
    
    // store ID field CRC
    const uint32_t idCrcVal = crc.get();
    uint8_t* idCrcPtr = (uint8_t*)&idCrcVal;
    data[offset++] = idCrcPtr[3];
    data[offset++] = idCrcPtr[2];
    data[offset++] = idCrcPtr[1];
    data[offset++] = idCrcPtr[0];
    
    // GAP3 zero byte gap, 13 bytes zeros
    crc.setInitial();
    offset += 13;
    
    // data field sync bytes: 32, 00
    data[offset++] = 0x32;    
    data[offset++] = 0;
    
    // DATA
    const uint16_t pos = (logicalSector-startSector)*256;
    for (size_t i = 0; i < 256; i++)
    {
      const uint8_t dataByte = dataFields ? dataFields[pos + i]
                                          : 0; // format
      data[offset++] = dataByte;
      crc.add(dataByte);
    }
    
    // store data field CRC
    const uint32_t dataCrcVal = crc.get();
    uint8_t* dataCrcPtr = (uint8_t*)&dataCrcVal;
    data[offset++] = dataCrcPtr[3];
    data[offset++] = dataCrcPtr[2];
    data[offset++] = dataCrcPtr[1];
    data[offset++] = dataCrcPtr[0];
          
    // 2 bytes zeros
    offset += 2;
  }
  
  // encode and prepare write DMA
  std::vector<uint32_t> dmaBuffer;
  endec.encodeMFM(track.data(), track.size(), clockBits, dmaBuffer);
  
  endec.prepareWriteDMA(dmaBuffer.data(), dmaBuffer.size());
  endec.writeWholeTrack();
    
  return hdd.getLastResult() == HDD_STATUS_OK;
}
