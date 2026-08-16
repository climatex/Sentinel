// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Xebec ("IBM Fixed Disk Adapter"), 512-byte format used in the XT
// Also works with the Adaptec (ACB-2010A) XT controller
// ID and data field 32bit CRC; headselect 4 bits

#include "config.h"

// defined in main.cpp
extern volatile int g_IndexCount;

XebecAdaptec::XebecAdaptec()
{
  // one 512 byte sector buffer + 2 bytes data address mark + 4 bytes CRC
  m_SectorBuffer.resize(518, 0);
  
  m_AnalyzeCylNumberMismatch = false;
  m_AnalyzeHdNumberMismatch = false;
  m_AnalyzeActualCylNumber = 0;
  m_AnalyzeActualHdNumber = 0;
  m_AnalyzeSectorsPerTrack = 0;
  m_AnalyzeStartSector = 0;
  m_AnalyzeSkew = (uint16_t)-1; // unknown yet
  
  // false: data fields start 01 00 C9, true: 01 00 00
  // Xebec XT controller reads Adaptec ACB-2010 disks okay, but not the other way around
  m_WriteModeAdaptec = false;
}

uint8_t* XebecAdaptec::getSectorBuffer()
{ 
  // single sector buffer of a data field, 512 bytes
  return &m_SectorBuffer[2]; // skip address mark bytes 00, 00/C9
}

bool XebecAdaptec::analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave)
{
  m_AnalyzeCylNumberMismatch = false;
  m_AnalyzeHdNumberMismatch = false;
  m_AnalyzeActualCylNumber = 0;
  m_AnalyzeActualHdNumber = 0;
  m_AnalyzeSectorsPerTrack = 0;
  m_AnalyzeStartSector = 0;
  m_AnalyzeSkew = (uint16_t)-1;
  
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
  
  // find skew factor
  if (sectorsPerTrack > 1)
  {
    m_AnalyzeSkew = sectors[1] - sectors[0];
  }
  else
  {
    m_AnalyzeSkew = 0;
  }
  m_AnalyzeSectorsPerTrack = sectorsPerTrack;
  m_AnalyzeStartSector = startSector;
  
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

void XebecAdaptec::getCustomAnalyzeTrackResults(bool& cylNumberMismatch, bool& hdNumberMismatch, uint16_t& actualCylNumber, uint8_t& actualHdNumber)
{
  cylNumberMismatch = m_AnalyzeCylNumberMismatch;
  hdNumberMismatch = m_AnalyzeHdNumberMismatch;
  actualCylNumber = m_AnalyzeActualCylNumber;
  actualHdNumber = m_AnalyzeActualHdNumber;
}

bool XebecAdaptec::scanID(uint16_t* cylinder, uint8_t* head, uint8_t* sector, uint16_t* reserved1, uint16_t* reserved2)
{  
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }  
  
  g_IndexCount = 0;  
  while (g_IndexCount < 2)
  {    
    CRC32 crc(CRC::Type::XebecAdaptec);
    
    if (!endec.lockPLL(TIMEOUT_DISK_ROTATION_US))
    {
      hdd.setLastResult(HDD_STATUS_NO_SECTOR_ID);
      return false;
    }

    // find 0xA1 with dropped clock bit, these are followed by 0x42, 0x85, 0x0A - ignore
    uint16_t partial;
    uint8_t bitShift;
    uint8_t status = endec.findSync(DEFAULT_MFM_SYNC_PATTERN, partial, bitShift);
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
    
    // sync on 0x01, no dropped clock
    status = endec.findSync(0xAAA9, partial, bitShift);
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
    
    // ID field bytes:
    // 01 (consumed by findSync), 00, 00, C2, CYL_HI, CYL_LO, HEAD, SECTOR, FLAG, 00, 4 bytes CRC
    uint8_t idField[13];
    size_t count = 13;
    bool success = endec.decodeMFM(idField, count, partial, bitShift, &crc);
    endec.setReadGate(false); // ID field has been read
    
    if (!success || (crc.get() != 0) || (idField[2] != 0xC2))
    {
      continue;
    }

    // extract sector info
    if (cylinder)
    {
      *cylinder = ((uint16_t)idField[3] << 8) | idField[4];
    }    
    if (head)
    {
      *head = idField[5] & 0xF; // 4-bit
    }    
    if (sector)
    {
      *sector = idField[6];
    }
    // ignore FLAG: bit7 always 1, bit4: last sector
    // other bits: relocation information for bad tracks

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

bool XebecAdaptec::readSector(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  // sector has skew, was it computed?
  if (m_AnalyzeSkew == (uint16_t)-1)
  {
    uint8_t spt;
    uint8_t start;
    uint16_t ssize;
    uint8_t interleave;
    if (!analyzeTrack(MAX_SPT_LIMIT, false, spt, start, ssize, interleave))
    {
      return false;
    }
  }
  if (sector < abs((int)m_AnalyzeStartSector-(int)m_AnalyzeSectorsPerTrack))
  {
    sector = (sector + m_AnalyzeSkew) % m_AnalyzeSectorsPerTrack;  
  }
  
  uint16_t cylinder = overrideCyl ? *overrideCyl : hdd.getPhysicalCylinder();
  uint8_t head = overrideHead ? *overrideHead : hdd.getPhysicalHead();
  
  // 01 consumed by findSync(), 00, 00/C9, 512 bytes, 4 byte CRC
  size_t dataFieldCount = 518;  
      
  for (uint8_t readAttempt = 0; readAttempt < READ_SECTOR_ATTEMPTS; readAttempt++)
  {
    CRC32 crc(CRC::Type::XebecAdaptec);
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
    uint8_t status = endec.findSync(0xAAA9, partial, bitShift); // sync on 0x01, normal clock
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
    
    // data address mark must be 00 (Adaptec) or C9 (Xebec)
    if (!success || ((m_SectorBuffer[1] != 0) && (m_SectorBuffer[1] != 0xC9)))
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
    
    m_WriteModeAdaptec = m_SectorBuffer[1] == 0;    
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

bool XebecAdaptec::writeSector(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  // sector has skew, was it computed?
  if (m_AnalyzeSkew == (uint16_t)-1)
  {
    uint8_t spt;
    uint8_t start;
    uint16_t ssize;
    uint8_t interleave;
    if (!analyzeTrack(MAX_SPT_LIMIT, false, spt, start, ssize, interleave))
    {
      return false;
    }
  }
  if (sector < abs((int)m_AnalyzeStartSector-(int)m_AnalyzeSectorsPerTrack))
  {
    sector = (sector + m_AnalyzeSkew) % m_AnalyzeSectorsPerTrack;  
  }
  
  std::vector<uint8_t> data;
  std::vector<size_t> clockBits; // no sync offsets: data field begins 01 with no dropped clock
  
  // reserve 13 bytes zero preamble, 01, 00, 00/C9 and sector data, followed by CRC (4 bytes) and 1 zero byte
  data.reserve(533);
  
  // prepare CRC counter
  CRC32 crc(CRC::Type::XebecAdaptec);
    
  // append 13 bytes preamble + 01 + 00 + 00 (Adaptec) / C9 (Xebec)
  data.insert(data.end(), 13, 0); 
  data.push_back(1);
  data.push_back(0);
  data.push_back(m_WriteModeAdaptec ? 0 : 0xC9);      
  crc.add(m_WriteModeAdaptec ? 0 : 0xC9); // used with CRC computation

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

bool XebecAdaptec::formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields, uint16_t* overrideCyl, uint8_t* overrideHead)
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
  const uint8_t skewFactor = (sectorCount > 1) ? interleave[1] - interleave[0] : 0;
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
    
    // GAP1 zero byte gap, 9 bytes zeros
    offset += 9;
    
    // 0xA1 with dropped Ck2
    clockBits.push_back(offset * 8 + 5);
    data[offset++] = 0xA1;
    // 0x42 with dropped Ck3
    clockBits.push_back(offset * 8 + 4);
    data[offset++] = 0x42;
    // 0x85 with dropped Ck4
    clockBits.push_back(offset * 8 + 3);
    data[offset++] = 0x85;
    // 0x0A with dropped Ck5
    clockBits.push_back(offset * 8 + 2);    
    data[offset++] = 0x0A;
        
    offset += 9; // 9 bytes zeros
    data[offset++] = 0x01; // ID sync byte
    data[offset++] = 0;    // zero gap, 2 bytes
    data[offset++] = 0;
    
    CRC32 crc(CRC::Type::XebecAdaptec);
    
    // ID compare byte
    const uint8_t idCompare = 0xC2;
    data[offset++] = idCompare;
    crc.add(idCompare);
        
    // CYL_HI
    const uint8_t cylHi = cylinder >> 8;
    data[offset++] = cylHi;
    crc.add(cylHi);
    
    // CYL_LO
    const uint8_t cylLow = (uint8_t)cylinder;
    data[offset++] = cylLow;
    crc.add(cylLow);
    
    // HEAD
    data[offset++] = head;
    crc.add(head);
    
    // SECTOR
    const uint8_t logicalSector = interleave[sec];
    data[offset++] = logicalSector;
    crc.add(logicalSector);
    
    // FLAG
    uint8_t flag = 0x80; // bit7 always on
    if (sec == sectorCount-1)
    {
      flag |= 0x10; // last (physical) sector on track
    }
    data[offset++] = flag;
    crc.add(flag);
    
    // 0
    data[offset++] = 0;
    crc.add(0);
    
    // store ID field CRC
    const uint32_t idCrcVal = crc.get();
    uint8_t* idCrcPtr = (uint8_t*)&idCrcVal;
    data[offset++] = idCrcPtr[3];
    data[offset++] = idCrcPtr[2];
    data[offset++] = idCrcPtr[1];
    data[offset++] = idCrcPtr[0];
    
    // GAP3 zero byte gap, 16 bytes zeros
    crc.setInitial();
    offset += 16;
    
    // data field sync bytes: 01, 00, 00/C9
    data[offset++] = 1;    
    data[offset++] = 0;
    data[offset++] = m_WriteModeAdaptec ? 0 : 0xC9;
    crc.add(m_WriteModeAdaptec ? 0 : 0xC9);
    
    // DATA
    const uint16_t pos = (((int)logicalSector - skewFactor + sectorCount) % sectorCount) * 512;
    for (size_t i = 0; i < 512; i++)
    {
      const uint8_t dataByte = dataFields ? dataFields[pos + i]
                                          : 0xFF; // format
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
          
    // GAP5 intersector gap; 34 bytes zeros
    offset += 34;
  }
  
  // encode and prepare write DMA
  std::vector<uint32_t> dmaBuffer;
  endec.encodeMFM(track.data(), track.size(), clockBits, dmaBuffer);
  
  endec.prepareWriteDMA(dmaBuffer.data(), dmaBuffer.size());
  endec.writeWholeTrack();
    
  return hdd.getLastResult() == HDD_STATUS_OK;
}