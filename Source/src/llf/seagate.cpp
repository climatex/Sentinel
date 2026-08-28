// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Seagate ST21/ST22 MFM format, PC/AT (TODO: RLL)

#include "config.h"

// defined in main.cpp
extern volatile uint64_t g_IndexCount;

Seagate::Seagate()
{
  // one sector buffer 512B + 1 byte data address mark + 4 bytes ECC
  m_SectorBuffer.resize(517, 0);
  
  // custom analyzeTrack() results
  m_AnalyzeCylNumberMismatch = false;
  m_AnalyzeHdNumberMismatch = false;
  m_AnalyzeUseSpareSector = false;
  m_AnalyzeActualCylNumber = 0;
  m_AnalyzeActualHdNumber = 0;
  
  // scanID(): is track relocated?
  m_TrackIsRelocated = false;
  m_RelocationCyl = 0;
  m_RelocationHd = 0;
}

uint8_t* Seagate::getSectorBuffer()
{ 
  // single sector buffer of a data field, 512 bytes
  return &m_SectorBuffer[1]; // F8 data address mark ignored during data field read
}

bool Seagate::analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave)
{
  m_AnalyzeCylNumberMismatch = false;
  m_AnalyzeHdNumberMismatch = false;
  m_AnalyzeUseSpareSector = false;
  m_AnalyzeActualCylNumber = 0;
  m_AnalyzeActualHdNumber = 0;
  m_AnalyzeSectorsTable.clear();
  
  startSector = 0;
  sectorSizeBytes = 0;
  interleave = 0;
  m_AnalyzeSectorsTable.reserve(idSamples);
  
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
    m_AnalyzeUseSpareSector |= (sector == 0xFF); // one sector marked bad on track, without track relocation
    
    m_AnalyzeActualCylNumber = cylinder;
    m_AnalyzeActualHdNumber = head;
    if (sector != 0xFE) // ignore spare sector on track, every track has one
    {
      m_AnalyzeSectorsTable.push_back(sector);  
    }    
  }
  
  // sector IDs obtained
  sectorSizeBytes = 512;
  calculateInterleave(m_AnalyzeSectorsTable, sectorsPerTrack, startSector, interleave);
  
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
    if (m_AnalyzeUseSpareSector)
    {
      printf("#");
    }
    
    printf(str_AnalyzeSectorOrder);
    for (const uint8_t& sector : m_AnalyzeSectorsTable)
    {
      printf("%u ", sector);
    }    
  }
  
  return true;
}

void Seagate::getCustomAnalyzeTrackResults(bool& cylNumberMismatch, bool& hdNumberMismatch, bool& useSpareSector, uint16_t& actualCylNumber, uint8_t& actualHdNumber)
{
  cylNumberMismatch = m_AnalyzeCylNumberMismatch;
  hdNumberMismatch = m_AnalyzeHdNumberMismatch;
  useSpareSector = m_AnalyzeUseSpareSector;
  actualCylNumber = m_AnalyzeActualCylNumber;
  actualHdNumber = m_AnalyzeActualHdNumber;  
}

bool Seagate::scanID(uint16_t* cylinder, uint8_t* head, uint8_t* sector, uint16_t* reserved1, uint16_t* reserved2)
{ 
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  const uint64_t startCount = g_IndexCount;
  while ((g_IndexCount - startCount) < 2)
  {
    CRC32 crc(CRC::Type::Seagate);
    
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
    
    size_t count = 9;
    uint8_t idField[9]; // [FE][HEAD_CYLHI][CYL_LO][SECTOR][FLAG] + 4 bytes CRC
    const bool success = endec.decodeMFM(idField, count, partial, bitShift, &crc);
    endec.setReadGate(false);  // read gate can be deasserted now
   
    if (!success || (crc.get() != 0) || (idField[0] != 0xFE))
    {
      continue;
    }
    
    // check FLAG if track has been relocated by the Seagate formatter
    if (idField[4] == 4)
    {
      m_TrackIsRelocated = true;
      // HEAD_CYLHI, CYL_LO see below
      m_RelocationCyl = ((((uint16_t)idField[1] & 0xC0) << 2) | idField[2]) + 1;
      m_RelocationHd = idField[1] & 0xF;
      continue; // will return HDD_STATUS_NO_SECTOR_ID for this track
    }
    else
    {
      m_TrackIsRelocated = false;
    }
    
    // check if the Seagate formatter reported exactly 1 bad sector for this track
    // if yes, the track is not relocated, but the spare eighteenth sector should be used (sector byte value 0xFE)
    if ((idField[1] == 0xFF) && (idField[2] == 0xFF) && (idField[3] == 0xFF)) // if so, HEAD_CYLHI, CYL_LO, SECTOR are all 0xFF
    {
      if (cylinder)
      {
        *cylinder = hdd.getPhysicalCylinder();
      }
      if (head)
      {
        *head = hdd.getPhysicalHead();
      }
      if (sector)
      {
        *sector = idField[3]; // 0xFF: bad sector, 0xFE: spare sector
      }
    }
    
    // normal mode
    else
    {
      // HEAD_CYLHI:
      // bits 0-3: head
      // bits 6-7: upper 2 bits of cylinder number
      // 0xFF: controller-reserved cylinder
      // CYL_LO:
      // bits 0-7: lower 8 bits of cylinder number
      if (cylinder)
      {
        *cylinder = ((((uint16_t)idField[1] & 0xC0) << 2) | idField[2]) + 1; // +1: disk data starts from cyl 1, counted from 0 in ID field
      }
      if (head)
      {
        *head = idField[1] & 0xF;
      }
      
      // SECTOR:
      // bits 0-7: sector number (0xFF: bad sector, 0xFE: spare sector)
      if (sector)
      {
        *sector = idField[3];
      }      
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

bool Seagate::readSector(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
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
    CRC32 crc(CRC::Type::Seagate);
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

bool Seagate::writeSector(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  std::vector<uint8_t> data;
  std::vector<size_t> clockBits;
  
  // reserve 13 bytes zero preamble, A1 (dropped clock), F8 and sector data, followed by CRC (4 bytes) and two zero bytes
  data.reserve(533);
  
  // prepare CRC counter
  CRC32 crc(CRC::Type::Seagate);
    
  // append 13 bytes preamble and info where to insert MFM/RLL sync
  data.insert(data.end(), 13, 0); 
  
  // insert 0xA1, drop Ck2
  clockBits.push_back(data.size() * 8 + 5);  
  data.push_back(0xA1); 
  crc.add(0xA1); // used with CRC computation  

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
  
  // two zero bytes
  data.push_back(0);
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

bool Seagate::formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  // dataFields: pointer to buffer containing data for the data fields in sequential sector order
  // dataFields null: format the track with 6C's
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
  uint16_t cylinder = overrideCyl ? *overrideCyl : hdd.getPhysicalCylinder()-1;
  uint8_t head = overrideHead ? *overrideHead : hdd.getPhysicalHead();  
   
  // leave some slack (next /INDEX stops write)
  std::vector<uint8_t> track;
  const uint16_t maxTrackBytes = endec.getMaximumTrackBytes();
  track.resize(maxTrackBytes, hdd.isSeparatorRLL() ? 0x33 : 0x4E); // fill with gap byte
  
  // bit/byte offsets where special sync marks will appear
  std::vector<size_t> syncOffsets;  
   
  // skip initial gap, at least 32 bytes (+ own latency) gap byte  
  uint8_t* data = track.data();
  size_t offset = 32;  
  for (size_t sec = 0; sec <= sectorCount; sec++)
  {
    // safety margin
    if (offset+600 >= maxTrackBytes)
    {
      break;
    }
    
    // ID field preamble: 10 bytes zeros
    memset(data+offset, 0, 10);
    offset += 10;
    
    CRC32 crc(CRC::Type::Seagate);
    
    // 0xA1 with dropped clock or RLL 0x8090 follows
    crc.add(0xA1); // used with CRC computation for RLL, but 0xA1 is not physically written then
    if (!hdd.isSeparatorRLL())
    {
      syncOffsets.push_back(offset * 8 + 5); // bit where to drop Ck2
      data[offset++] = 0xA1;
    }
    else
    {
      syncOffsets.push_back(offset); // byte offset for RLL 0x8090
    }
    
    // IDENT 0xFE
    const uint8_t ident = 0xFE;
    data[offset++] = ident;
    crc.add(ident);
    
    // HEAD_CYLHI
    const uint8_t headCylHi = (uint8_t)head | ((cylinder & 0x300) >> 2);    
    data[offset++] = headCylHi;
    crc.add(headCylHi);
    
    // CYL_LO
    const uint8_t cylLow = (uint8_t)cylinder;
    data[offset++] = cylLow;
    crc.add(cylLow);
    
    // SECTOR
    uint8_t logicalSector = 0;
    if (sec == sectorCount)
    {
      logicalSector = 0xFE; // spare, unused
    }
    else
    {
      logicalSector = interleave[sec];
    }
    data[offset++] = logicalSector;
    crc.add(logicalSector);
    
    // FLAG - zero, no track remapping here
    const uint8_t flag = 0;
    data[offset++] = flag;
    crc.add(flag);
    
    // store ID field CRC
    const uint32_t idCrcVal = crc.get();
    uint8_t* crcPtr = (uint8_t*)&idCrcVal;
    data[offset++] = crcPtr[3];
    data[offset++] = crcPtr[2];
    data[offset++] = crcPtr[1];
    data[offset++] = crcPtr[0];
    
    // data field preamble: 15 bytes zeros
    memset(data+offset, 0, 15);
    offset += 15;
    
    crc.setInitial();
    
    // 0xA1 with dropped clock or RLL 0x8090 follows
    crc.add(0xA1); // used with CRC computation for RLL, but 0xA1 is not physically written then
    if (!hdd.isSeparatorRLL())
    {
      syncOffsets.push_back(offset * 8 + 5); // bit where to drop Ck2
      data[offset++] = 0xA1;
    }
    else
    {
      syncOffsets.push_back(offset); // byte offset for RLL 0x8090
    }
    
    // DATA ident 0xF8
    data[offset++] = 0xF8;
    crc.add(0xF8);
    
    // DATA
    const uint16_t pos = (logicalSector-startSector)*512;
    for (size_t i = 0; i < 512; i++)
    {
      uint8_t dataByte = 0x6C; // default: format
      if (dataFields && (sec < sectorCount))
      {
        dataByte = dataFields[pos + i];
      }
      data[offset++] = dataByte;
      crc.add(dataByte);
    }
    
    // store data field CRC
    const uint32_t dataCrcVal = crc.get();
    crcPtr = (uint8_t*)&dataCrcVal;
    data[offset++] = crcPtr[3];
    data[offset++] = crcPtr[2];
    data[offset++] = crcPtr[1];
    data[offset++] = crcPtr[0];
      
    // 2 byte DATA zero pad
    data[offset++] = 0;
    data[offset++] = 0;   
    
    // 20 byte intersector gap
    offset += 20;
  }
  
  // encode and prepare write DMA
  std::vector<uint32_t> dmaBuffer;
  if (hdd.isSeparatorRLL())
  {
    endec.encodeRLL(track.data(), track.size(), syncOffsets, dmaBuffer);
  }
  else
  {
    endec.encodeMFM(track.data(), track.size(), syncOffsets, dmaBuffer);
  }
  
  endec.prepareWriteDMA(dmaBuffer.data(), dmaBuffer.size());
  endec.writeWholeTrack();
    
  return hdd.getLastResult() == HDD_STATUS_OK;
}

bool Seagate::formatWriteReservedCylinder(uint8_t head, uint8_t interleave)
{
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  } 
  
  // reserved cylinder 0
  uint16_t cylinder = 0;
  const uint8_t sectorCount = hdd.isSeparatorRLL() ? 26 : 17;
  
  // leave some slack (next /INDEX stops write)
  std::vector<uint8_t> track;
  const uint16_t maxTrackBytes = endec.getMaximumTrackBytes();
  track.resize(maxTrackBytes, hdd.isSeparatorRLL() ? 0x33 : 0x4E); // fill with gap byte
  
  // bit/byte offsets where special sync marks will appear
  std::vector<size_t> syncOffsets;  
   
  // skip initial gap, at least 32 bytes (+ own latency) gap byte  
  uint8_t* data = track.data();
  size_t offset = 32;  
  for (size_t sec = 0; sec <= sectorCount; sec++)
  {
    // safety margin
    if (offset+600 >= maxTrackBytes)
    {
      break;
    }
    
    // ID field preamble: 10 bytes zeros
    memset(data+offset, 0, 10);
    offset += 10;
    
    CRC32 crc(CRC::Type::Seagate);
    
    // 0xA1 with dropped clock or RLL 0x8090 follows
    crc.add(0xA1); // used with CRC computation for RLL, but 0xA1 is not physically written then
    if (!hdd.isSeparatorRLL())
    {
      syncOffsets.push_back(offset * 8 + 5); // bit where to drop Ck2
      data[offset++] = 0xA1;
    }
    else
    {
      syncOffsets.push_back(offset); // byte offset for RLL 0x8090
    }
    
    // IDENT 0xFE
    const uint8_t ident = 0xFE;
    data[offset++] = ident;
    crc.add(ident);
    
    // HEAD_CYLHI - 0xFF for the reserved cylinder
    const uint8_t headCylHi = 0xFF;    
    data[offset++] = headCylHi;
    crc.add(headCylHi);
    
    // CYL_LO
    const uint8_t cylLow = (uint8_t)cylinder;
    data[offset++] = cylLow;
    crc.add(cylLow);
    
    // SECTOR
    uint8_t logicalSector = 0;
    if (sec == sectorCount)
    {
      logicalSector = 0xFE; // spare, unused
    }
    else
    {
      logicalSector = sec;
    }
    data[offset++] = logicalSector;
    crc.add(logicalSector);
    
    // FLAG - zero, no track remapping here
    const uint8_t flag = 0;
    data[offset++] = flag;
    crc.add(flag);
    
    // store ID field CRC
    const uint32_t idCrcVal = crc.get();
    uint8_t* crcPtr = (uint8_t*)&idCrcVal;
    data[offset++] = crcPtr[3];
    data[offset++] = crcPtr[2];
    data[offset++] = crcPtr[1];
    data[offset++] = crcPtr[0];
    
    // data field preamble: 15 bytes zeros
    memset(data+offset, 0, 15);
    offset += 15;
    
    crc.setInitial();
    
    // 0xA1 with dropped clock or RLL 0x8090 follows
    crc.add(0xA1); // used with CRC computation for RLL, but 0xA1 is not physically written then
    if (!hdd.isSeparatorRLL())
    {
      syncOffsets.push_back(offset * 8 + 5); // bit where to drop Ck2
      data[offset++] = 0xA1;
    }
    else
    {
      syncOffsets.push_back(offset); // byte offset for RLL 0x8090
    }
    
    // DATA ident 0xF8
    data[offset++] = 0xF8;
    crc.add(0xF8);
    
    // DATA
    uint8_t dataField[512] = {0};
    uint8_t customTrack[] = { 0xDA, 0xBE, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x06, 0x00, 0x03,
                              0x00, 0x00, 0x53, 0x45, 0x41, 0x47, 0x41, 0x54, 0x45, 0x53, 0x45, 0x4E,
                              0x54, 0x49, 0x4E, 0x45, 0x4C, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
                              0x20, 0x20, 0x20, 0x20 };
    
    // store number of cylinders, heads, format interleave, write precompensation start cylinder
    customTrack[2] = hdd.getParams()->Cylinders >> 8;
    customTrack[3] = hdd.getParams()->Cylinders;
    customTrack[4] = hdd.getParams()->Heads;
    customTrack[8] = interleave;
    customTrack[12] = hdd.getParams()->WritePrecompStartCyl >> 8;
    customTrack[13] = hdd.getParams()->WritePrecompStartCyl;
    memcpy(dataField, customTrack, sizeof(customTrack));
    
    for (size_t i = 0; i < 512; i++)
    {
      uint8_t dataByte = 0;
      
      if ((head < 2) && (sec < 2))
      {
        dataByte = dataField[i]; // written for head 0 sectors 0 and 1, and head 1 sectors 0 and 1 only
      }
      
      data[offset++] = dataByte;
      crc.add(dataByte);
    }
    
    // store data field CRC
    const uint32_t dataCrcVal = crc.get();
    crcPtr = (uint8_t*)&dataCrcVal;
    data[offset++] = crcPtr[3];
    data[offset++] = crcPtr[2];
    data[offset++] = crcPtr[1];
    data[offset++] = crcPtr[0];
      
    // 2 byte DATA zero pad
    data[offset++] = 0;
    data[offset++] = 0;   
    
    // 20 byte intersector gap
    offset += 20;
  }
  
  // encode and prepare write DMA
  std::vector<uint32_t> dmaBuffer;
  if (hdd.isSeparatorRLL())
  {
    endec.encodeRLL(track.data(), track.size(), syncOffsets, dmaBuffer);
  }
  else
  {
    endec.encodeMFM(track.data(), track.size(), syncOffsets, dmaBuffer);
  }
  
  endec.prepareWriteDMA(dmaBuffer.data(), dmaBuffer.size());
  endec.writeWholeTrack();
    
  return hdd.getLastResult() == HDD_STATUS_OK;
}