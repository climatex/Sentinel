// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Standard Microsystems HDC9224 (512 byte sectors)

#include "config.h"

// defined in main.cpp
extern volatile int g_IndexCount;

HDC9224::HDC9224()
{
  // one sector buffer max. 512B + 1 byte data address mark + 4 bytes CRC
  m_SectorBuffer.resize(517, 0);
       
  // custom analyzeTrack() results
  m_AnalyzeCylNumberMismatch = false;
  m_AnalyzeHdNumberMismatch = false;
  m_AnalyzeActualCylNumber = 0;
  m_AnalyzeActualHdNumber = 0;
}

uint8_t* HDC9224::getSectorBuffer()
{ 
  // single sector buffer of a data field, max 517 bytes
  return &m_SectorBuffer[1]; // F8 data address mark ignored during data field read
}

bool HDC9224::analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave)
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
          printf(str_AnalyzeNoSectors);        
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

void HDC9224::getCustomAnalyzeTrackResults(bool& cylNumberMismatch, bool& hdNumberMismatch, uint16_t& actualCylNumber, uint8_t& actualHdNumber)
{
  cylNumberMismatch = m_AnalyzeCylNumberMismatch;
  hdNumberMismatch = m_AnalyzeHdNumberMismatch;
  actualCylNumber = m_AnalyzeActualCylNumber;
  actualHdNumber = m_AnalyzeActualHdNumber;
}

bool HDC9224::scanID(uint16_t* cylinder, uint8_t* head, uint8_t* sector, uint16_t* reserved1, uint16_t* reserved2)
{ 
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  g_IndexCount = 0;  
  while (g_IndexCount < 2)
  {
    CRC16 crc(CRC::Type::CCITT);
    
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
    
    size_t count = 6;
    uint8_t idField[6]; // [FE][CYL_LO][HEAD][SECTOR] + 2 bytes CRC
    const bool success = endec.decodeMFM(idField, count, partial, bitShift, &crc);
    endec.setReadGate(false);  // read gate can be deasserted now
   
    if (!success || (crc.get() != 0) || (idField[0] != 0xFE))
    {
      continue;
    }    
    
    // extract sector info
    if (cylinder)
    {
      *cylinder = ((uint16_t)(idField[2] & 0x70) << 4) | idField[1];
    }
    if (head)
    {
      *head = idField[2] & 0xF;
    }
    if (sector)
    {
      *sector = idField[3];
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

bool HDC9224::readSector(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  uint16_t cylinder = overrideCyl ? *overrideCyl : hdd.getPhysicalCylinder();
  uint8_t head = overrideHead ? *overrideHead : hdd.getPhysicalHead();
  
  // A1 consumed by findSync(), ident F8, sector data, 4 byte CRC
  size_t dataFieldCount = 517;
      
  for (uint8_t readAttempt = 0; readAttempt < READ_SECTOR_ATTEMPTS; readAttempt++)
  {
    CRC32 crc(CRC::Type::HDC9224);
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

bool HDC9224::writeSector(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  std::vector<uint8_t> data;
  std::vector<size_t> clockBits;
  
  // reserve 12 bytes zero preamble, A1 (dropped clock), F8 and sector data, followed by CRC (4 bytes) and a zero byte
  data.reserve(21 + 512);
  
  // prepare CRC counter
  CRC32 crc(CRC::Type::HDC9224);
    
  // append 12 bytes preamble and info where to insert MFM/RLL sync
  data.insert(data.end(), 12, 0); 
  
  // insert 0xA1, drop Ck2
  clockBits.push_back(data.size() * 8 + 5);  
  crc.add(0xA1); // used with CRC computation
  data.push_back(0xA1); 

  // ID part of CRC computation
  crc.add(0xF8);
  data.push_back(0xF8);
  
  // sector data
  for (size_t i = 0; i < 512; i++)
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
  
  // three byte DATA zero pad
  data.insert(data.end(), 3, 0);
      
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

bool HDC9224::formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields, uint16_t* overrideCyl, uint8_t* overrideHead)
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
  track.resize(maxTrackBytes, 0x4E); // fill with gap byte 4E
  
  // bit offsets in the track buffer where to drop clock bits
  std::vector<size_t> clockBits;  
   
  // skip initial gap, at least 32 bytes (+ own latency) gap byte 4E
  uint8_t* data = track.data();
  size_t offset = 32;  
  for (size_t sec = 0; sec < sectorCount; sec++)
  {
    // safety margin
    if (offset+600 >= maxTrackBytes)
    {
      break;
    }
    
    // ID field preamble: 13 bytes zeros
    memset(data+offset, 0, 13);
    offset += 13;
    
    CRC16 idFieldCrc(CRC::Type::CCITT);
    
    // 0xA1 with dropped Ck2 follows
    idFieldCrc.add(0xA1); // used with CRC computation
    clockBits.push_back(offset * 8 + 5);    
    data[offset++] = 0xA1;
    
    // IDENT 0xFE
    data[offset++] = 0xFE;
    idFieldCrc.add(0xFE);
    
    // CYL_LO
    data[offset++] = (uint8_t)cylinder;
    idFieldCrc.add((uint8_t)cylinder);
    
    // HEAD (4 bits) + upper 3 bits of cylinder, bit7 bad sector flag 0
    uint8_t hd = head & 0xF;
    hd |= (uint8_t)((cylinder & 0x700) >> 4);
    data[offset++] = hd;
    idFieldCrc.add(hd);    
    
    // SECTOR
    const uint8_t logicalSector = interleave[sec];
    data[offset++] = logicalSector;
    idFieldCrc.add(logicalSector);
        
    // store ID field CRC
    const uint16_t idCrcVal = idFieldCrc.get();
    uint8_t* crc = (uint8_t*)&idCrcVal;
    data[offset++] = crc[1];
    data[offset++] = crc[0];
    
    // data field preamble: 15 bytes zeros
    memset(data+offset, 0, 15);
    offset += 15;
    
    CRC32 dataFieldCrc(CRC::Type::HDC9224);  
    
    // 0xA1 with dropped clock follows
    dataFieldCrc.add(0xA1); // used with CRC computation    
    clockBits.push_back(offset * 8 + 5);    
    data[offset++] = 0xA1;
    
    // DATA ident 0xF8
    data[offset++] = 0xF8;
    dataFieldCrc.add(0xF8);
    
    // DATA
    const uint16_t pos = (logicalSector-startSector)*512;
    for (size_t i = 0; i < 512; i++)
    {
      const uint8_t dataByte = dataFields ? dataFields[pos + i]
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
        
    // three byte DATA zero pad
    data[offset++] = 0;
    data[offset++] = 0;
    data[offset++] = 0;
    
    // intersector gap 0x4E
    offset += 38;
  }
  
  // encode and prepare write DMA
  std::vector<uint32_t> dmaBuffer;
  endec.encodeMFM(track.data(), track.size(), clockBits, dmaBuffer);
  
  endec.prepareWriteDMA(dmaBuffer.data(), dmaBuffer.size());
  endec.writeWholeTrack();
    
  return hdd.getLastResult() == HDD_STATUS_OK;
}