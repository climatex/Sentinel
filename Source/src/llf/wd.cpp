// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Generic Western Digital MFM/RLL format
// Sector sizes supported: 128B 256B 512B 1024B
// Data field CRC: 16-bit, 32-bit, 56-bit; head select in ID fields 3 or 4bit

#include "config.h"

// defined in main.cpp
extern volatile int g_IndexCount;

WD::WD()
{
  // one sector buffer max. 1024B + 1 byte data address mark + 7 bytes ECC
  m_SectorBuffer.resize(1032, 0);
  
  m_Sdh4Bit = false;
  m_DataCrcBits = hdd.isSeparatorRLL() ? 56 : 32; // 56 (RLL) or 32 (MFM) default, can be also 16 (MFM)
  m_WorkingSectorSizeBytes = 512; // default 512 bytes
  
  // custom analyzeTrack() results
  m_AnalyzeCylNumberMismatch = false;
  m_AnalyzeHdNumberMismatch = false;
  m_AnalyzeVariableSectorSize = false;
  m_AnalyzeActualCylNumber = 0;
  m_AnalyzeActualHdNumber = 0;
}

uint8_t WD::getIdent(uint16_t cylinder)
{
  // get IDENT for address mark field, depending on cylinder number
  // IDENT of data is always F8
  
  if (cylinder < 256)
  {
    return 0xFE;
  }
  else if (cylinder < 512)
  {
    return 0xFF;
  }
  else if (cylinder < 768)
  {
    return 0xFC;
  }
  else if (cylinder < 1024)
  {
    return 0xFD;
  }
  else if (cylinder < 1280)
  {
    return 0xF6;
  }
  else if (cylinder < 1536)
  {
    return 0xF7;
  }
  else if (cylinder < 1792)
  {
    return 0xF4;
  }
  else
  {
    return 0xF5;
  }
}

uint16_t WD::getSectorSizeFromSDH(uint8_t sdh)
{ 
  sdh &= 0x60;
  switch(sdh)
  {
    case 0x60:
      return 128;
    case 0x40:
      return 1024;
    case 0x20:
      return 512;
    default:
      return 256;
  }
}

uint8_t WD::getSDHFromSectorSize(uint16_t sectorSizeBytes)
{
  uint8_t sdh = 0;
  
  switch(sectorSizeBytes)
  {
  case 128:
    sdh = 0x60;
    break;
  case 1024:
    sdh = 0x40;
    break;
  case 512:
    sdh = 0x20;
  default:
    break;
  }
  
  return sdh;
}

void WD::setWorkingSectorSizeBytes(uint16_t sectorSizeBytes)
{
  // quick sanity check
  if ((sectorSizeBytes != 128) && 
      (sectorSizeBytes != 256) && 
      (sectorSizeBytes != 512) && 
      (sectorSizeBytes != 1024))
  {
    return;
  }
  
  m_WorkingSectorSizeBytes = sectorSizeBytes;
}

void WD::setDataCrcBits(uint8_t dataCrcBits)
{
  if ((dataCrcBits != 16) &&
      (dataCrcBits != 32) &&
      (dataCrcBits != 56))
  {
    return;
  }
  
  m_DataCrcBits = dataCrcBits;
}

uint8_t* WD::getSectorBuffer()
{ 
  // single sector buffer of a data field, valid up to m_WorkingSectorSizeBytes
  return &m_SectorBuffer[1]; // F8 data address mark ignored during data field read
}

bool WD::analyzeTrack(uint8_t idSamples, bool printOut, uint8_t& sectorsPerTrack, uint8_t& startSector, uint16_t& sectorSizeBytes, uint8_t& interleave)
{
  m_AnalyzeCylNumberMismatch = false;
  m_AnalyzeHdNumberMismatch = false;
  m_AnalyzeVariableSectorSize = false;
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
    uint8_t sdh;
    uint8_t sector;
    
    if (!scanID(&cylinder, &sdh, &sector))
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
    
    uint8_t headMask = m_Sdh4Bit ? 0xF : 7;
    m_AnalyzeCylNumberMismatch |= (cylinder != hdd.getPhysicalCylinder());
    m_AnalyzeHdNumberMismatch |= ((sdh & headMask) != (hdd.getPhysicalHead() & headMask));
    
    if (sample == 0)
    {
      sectorSizeBytes = getSectorSizeFromSDH(sdh);
    }
    else
    {
      m_AnalyzeVariableSectorSize |= getSectorSizeFromSDH(sdh) != sectorSizeBytes;
    }

    m_AnalyzeActualCylNumber = cylinder;
    m_AnalyzeActualHdNumber = sdh & 0xF; // actual head number always 4 bits
    sectors.push_back(sector);
  }
  
  if (m_AnalyzeVariableSectorSize)
  {
    sectorSizeBytes = 0; // flag as bullshit
  }
  
  // sector IDs obtained
  calculateInterleave(sectors, sectorsPerTrack, startSector, interleave);
  
  // results per track
  if (printOut)
  {
    printf(str_AnalyzeSpt, sectorsPerTrack);
    
    if (sectorSizeBytes)
    {
      printf(str_AnalyzeSectorSize, sectorSizeBytes);
    }
    else
    {
      printf(str_AnalyzeVarSectorSize);
    }
    
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

void WD::getCustomAnalyzeTrackResults(bool& cylNumberMismatch, bool& hdNumberMismatch, bool& variableSectorSize, uint16_t& actualCylNumber, uint8_t& actualHdNumber)
{
  cylNumberMismatch = m_AnalyzeCylNumberMismatch;
  hdNumberMismatch = m_AnalyzeHdNumberMismatch;
  variableSectorSize = m_AnalyzeVariableSectorSize;
  actualCylNumber = m_AnalyzeActualCylNumber;
  actualHdNumber = m_AnalyzeActualHdNumber;
}

bool WD::scanID(uint16_t* cylinder, uint8_t* sdh, uint8_t* sector, uint16_t* reserved1, uint16_t* reserved2)
{
  // note: second argument is SDH, not just head number  
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
    const uint8_t status = endec.findSync(hdd.isSeparatorRLL() ? DEFAULT_RLL_SYNC_PATTERN : 
                                                                 DEFAULT_MFM_SYNC_PATTERN, partial, bitShift);

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
    uint8_t idField[6]; // [IDENT][CYL_LO][HEAD][SECTOR][CRC_H][CRC_L]
    const bool success = hdd.isSeparatorRLL() ? endec.decodeRLL(idField, count, partial, bitShift, &crc) :
                                                endec.decodeMFM(idField, count, partial, bitShift, &crc);
    // read gate can be deasserted now
    endec.setReadGate(false);   
   
    if (!success || (crc.get() != 0))
    {
      continue;
    }
    
    // form CYL_HI from IDENT
    uint16_t cyl;
    uint8_t* cylPtr = (uint8_t*)&cyl;
    switch(idField[0])
    {
    case 0xFE:
      cylPtr[1] = 0;
      break;
    case 0xFF:
      cylPtr[1] = 1;
      break;
    case 0xFC:
      cylPtr[1] = 2;
      break;
    case 0xFD:
      cylPtr[1] = 3;
      break;
    case 0xF6:
      cylPtr[1] = 4;
      break;
    case 0xF7:
      cylPtr[1] = 5;
      break;
    case 0xF4:
      cylPtr[1] = 6;
      break;
    case 0xF5:
      cylPtr[1] = 7;
      break;
    default:
      continue; // invalid IDENT
    }
    cylPtr[0] = idField[1]; // store CYL_LO
    
    // extract sector info
    if (cylinder)
    {
      *cylinder = cyl; 
    }    
    if (sdh)
    {
      *sdh = idField[2];
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

bool WD::readSector(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  uint16_t cylinder = overrideCyl ? *overrideCyl : hdd.getPhysicalCylinder();
  uint8_t head = overrideHead ? *overrideHead : hdd.getPhysicalHead();
  
  size_t dataFieldCount = m_WorkingSectorSizeBytes + 1; // A1 consumed by findSync, skip over F8
  
  CRC* crc = NULL;
  if (m_DataCrcBits == 16)
  {
    crc = new CRC16(CRC::Type::CCITT);
    dataFieldCount += 2; // incl. 2 bytes CRC
  }
  else if (m_DataCrcBits == 32)
  {
    crc = new CRC32(CRC::Type::WD);
    dataFieldCount += 4;
  }
  else
  {
    crc = new CRC56(CRC::Type::WD);
    dataFieldCount += 7;
  }
  
  uint8_t sdhToMatch = getSDHFromSectorSize(m_WorkingSectorSizeBytes);
  sdhToMatch |= head & (m_Sdh4Bit ? 0xF : 7);
      
  for (uint8_t readAttempt = 0; readAttempt < READ_SECTOR_ATTEMPTS; readAttempt++)
  {
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
      uint8_t scanSdh;
      uint8_t scanSector;
      
      if (!scanID(&scanCyl, &scanSdh, &scanSector))
      {
        delete crc;
        return false; // no sector IDs whatsoever
      }
      
      scanSdh &= m_Sdh4Bit ? 0xEF : 0xE7;      
      if ((scanCyl == cylinder) && (scanSdh == sdhToMatch) && (scanSector == sector))
      {
        found = true;
        break;
      }
    }
    
    if (!found)
    {
      delete crc;           
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
    uint8_t status = endec.findSync(hdd.isSeparatorRLL() ? DEFAULT_RLL_SYNC_PATTERN : 
                                                           DEFAULT_MFM_SYNC_PATTERN, partial, bitShift);
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
    
    crc->setInitial();    
    crc->add(0xA1); // consumed by findSync(), part of the computation
    
    const bool success = hdd.isSeparatorRLL() ? endec.decodeRLL(m_SectorBuffer.data(), dataFieldCount, partial, bitShift, crc) :
                                                endec.decodeMFM(m_SectorBuffer.data(), dataFieldCount, partial, bitShift, crc);
    
    // read gate can be deasserted now
    endec.setReadGate(false);
    
    // data address mark must be F8
    if (!success || (m_SectorBuffer[0] != 0xF8))
    {
      continue;
    }
    
    if (crc->get() != 0)
    {
      // last, try computing correction
      if (readAttempt == READ_SECTOR_ATTEMPTS-1)
      {
        if (!crc->tryComputeCorrection(m_SectorBuffer.data(), dataFieldCount))
        {
          delete crc;
          hdd.setLastResult(HDD_STATUS_DATA_ERROR);
          return false;
        }
        
        delete crc;
        hdd.setLastResult(HDD_STATUS_DATA_CORRECTED);
        return true;
      }
      
      continue;      
    }
        
    delete crc;
    hdd.setLastResult(HDD_STATUS_OK);
    return true;
  }
  
  endec.setReadGate(false);
  delete crc;
  if (hdd.checkReadyWriteFault())
  {
    hdd.setLastResult(HDD_STATUS_NO_DATA_ID);
  }
  return false;
}

bool WD::writeSector(uint8_t sector, uint16_t* overrideCyl, uint8_t* overrideHead)
{
  if (!hdd.checkReadyWriteFault())
  {
    return false;
  }
  
  std::vector<uint8_t> data;
  std::vector<size_t> syncOffsets; // one sync offset
  
  // reserve 12 bytes zero preamble, A1 (MFM), F8 and sector data, followed by CRC (up to 7 bytes) and 3 zero bytes
  data.reserve(24 + m_WorkingSectorSizeBytes);
  
  // prepare CRC counter
  CRC* dataFieldCrc = NULL;
  if (m_DataCrcBits == 16)
  {
    dataFieldCrc = new CRC16(CRC::Type::CCITT);
  }
  else if (m_DataCrcBits == 32)
  {
    dataFieldCrc = new CRC32(CRC::Type::WD);      
  }
  else
  {
    dataFieldCrc = new CRC56(CRC::Type::WD);      
  }
    
  // append 12 bytes preamble and info where to insert MFM/RLL sync
  data.insert(data.end(), 12, 0); 
        
  dataFieldCrc->add(0xA1); // used with CRC computation for RLL: there, 0xA1 is not physically written
  if (!hdd.isSeparatorRLL())
  {
    syncOffsets.push_back(data.size() * 8 + 5); // bit offset where to drop Ck2
    data.push_back(0xA1);
  }
  else
  {
    syncOffsets.push_back(data.size()); // byte offset where to insert RLL syncmark
  }
  
   // ID part of CRC computation
  dataFieldCrc->add(0xF8);
  data.push_back(0xF8);

  // sector data
  for (size_t i = 0; i < m_WorkingSectorSizeBytes; i++)
  {
    const uint8_t byte = getSectorBuffer()[i];
    data.push_back(byte);
    dataFieldCrc->add(byte);
  }
  
  // store data field CRC
  const uint64_t dataCrcVal = dataFieldCrc->get();
  uint8_t* crcPtr = (uint8_t*)&dataCrcVal;
  if (m_DataCrcBits == 56)
  {
    data.push_back(crcPtr[6]);
    data.push_back(crcPtr[5]);
    data.push_back(crcPtr[4]);
  }
  if ((m_DataCrcBits == 56) || (m_DataCrcBits == 32))
  {
    data.push_back(crcPtr[3]);
    data.push_back(crcPtr[2]);
  }
  data.push_back(crcPtr[1]);
  data.push_back(crcPtr[0]);  
  delete dataFieldCrc;
  
  // three byte DATA zero pad
  data.insert(data.end(), 3, 0); 
  
  std::vector<uint32_t> dmaBuffer;
  if (hdd.isSeparatorRLL())
  {
    endec.encodeRLL(data.data(), data.size(), syncOffsets, dmaBuffer);
  }
  else
  {
    endec.encodeMFM(data.data(), data.size(), syncOffsets, dmaBuffer);
  }  
  endec.prepareWriteDMA(dmaBuffer.data(), dmaBuffer.size());  
  
  uint16_t cylinder = overrideCyl ? *overrideCyl : hdd.getPhysicalCylinder();
  uint8_t head = overrideHead ? *overrideHead : hdd.getPhysicalHead();  
  uint8_t sdhToMatch = getSDHFromSectorSize(m_WorkingSectorSizeBytes);
  sdhToMatch |= head & (m_Sdh4Bit ? 0xF : 7);
  
  bool found = false;  
  for (uint8_t locateAttempt = 0; locateAttempt < MAX_SPT_LIMIT; locateAttempt++)
  {
    uint16_t scanCyl;
    uint8_t scanSdh;
    uint8_t scanSector;
    
    if (!scanID(&scanCyl, &scanSdh, &scanSector))
    {
      return false; // no sector IDs whatsoever
    }
    
    scanSdh &= m_Sdh4Bit ? 0xEF : 0xE7;
    if ((scanCyl == cylinder) && (scanSdh == sdhToMatch) && (scanSector == sector))
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

bool WD::formatWriteTrack(const std::vector<uint8_t>& interleave, const uint8_t* dataFields, uint16_t* overrideCyl, uint8_t* overrideHead)
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
  uint8_t sdh = getSDHFromSectorSize(m_WorkingSectorSizeBytes);
  sdh |= head & (m_Sdh4Bit ? 0xF : 7);
   
  // leave some slack (next /INDEX stops write)
  std::vector<uint8_t> track;
  const uint16_t maxTrackBytes = endec.getMaximumTrackBytes();
  track.resize(maxTrackBytes, hdd.isSeparatorRLL() ? 0x33 : 0x4E); // fill with gap byte
  
  // bit/byte offsets where special sync marks will appear
  std::vector<size_t> syncOffsets;  
   
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
    
    // ID field preamble: 13 bytes zeros
    memset(data+offset, 0, 13);
    offset += 13;
    
    CRC16 idFieldCrc(CRC::Type::CCITT);
    
    // 0xA1 with dropped clock or RLL 0x8090 follows
    idFieldCrc.add(0xA1); // used with CRC computation for RLL, but 0xA1 is not physically written then
    if (!hdd.isSeparatorRLL())
    {
      syncOffsets.push_back(offset * 8 + 5); // bit where to drop Ck2
      data[offset++] = 0xA1;
    }
    else
    {
      syncOffsets.push_back(offset); // byte offset for RLL 0x8090
    }
    
    // IDENT
    const uint8_t ident = getIdent(cylinder);
    data[offset++] = ident;
    idFieldCrc.add(ident);
    
    // CYL_LOW
    const uint8_t cylLow = (uint8_t)cylinder;
    data[offset++] = cylLow;
    idFieldCrc.add(cylLow);
    
    // SDH
    data[offset++] = sdh;
    idFieldCrc.add(sdh);
    
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
    
    CRC* dataFieldCrc = NULL;
    if (m_DataCrcBits == 16)
    {
      dataFieldCrc = new CRC16(CRC::Type::CCITT);
    }
    else if (m_DataCrcBits == 32)
    {
      dataFieldCrc = new CRC32(CRC::Type::WD);      
    }
    else
    {
      dataFieldCrc = new CRC56(CRC::Type::WD);      
    }    
    
    // 0xA1 with dropped clock or RLL 0x8090 follows
    dataFieldCrc->add(0xA1); // used with CRC computation for RLL, but 0xA1 is not physically written then
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
    dataFieldCrc->add(0xF8);
    
    // DATA
    const uint16_t pos = (logicalSector-startSector)*m_WorkingSectorSizeBytes;
    for (size_t i = 0; i < m_WorkingSectorSizeBytes; i++)
    {
      const uint8_t dataByte = dataFields ? dataFields[pos + i]
                                          : 0xFF; // format
      data[offset++] = dataByte;
      dataFieldCrc->add(dataByte);
    }
    
    // store data field CRC
    const uint64_t dataCrcVal = dataFieldCrc->get();
    crc = (uint8_t*)&dataCrcVal;
    if (m_DataCrcBits == 56)
    {
      data[offset++] = crc[6];
      data[offset++] = crc[5];
      data[offset++] = crc[4];
    }
    if ((m_DataCrcBits == 56) || (m_DataCrcBits == 32))
    {
      data[offset++] = crc[3];
      data[offset++] = crc[2];
    }
    data[offset++] = crc[1];
    data[offset++] = crc[0];
    
    delete dataFieldCrc;
    
    // three byte DATA zero pad
    data[offset++] = 0;
    data[offset++] = 0;
    data[offset++] = 0;    
    
    // minimum intersector gap formula from the WD2010 datasheet:
    // gap = 2*M*K + 18, where M is the motor speed variation (0.01 for 1%); K sector size
    offset += round(0.02*m_WorkingSectorSizeBytes) + 18;
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