// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// CRC computation

#include "config.h"

CRC16::CRC16(CRC::Type type)
{
  switch(type)
  {
  case CCITT:  
    m_initial = 0xFFFF;
    m_polynomial = 0x1021;
    break;
  }
  
  m_crc = m_initial;
}

void CRC16::add(uint8_t byte)
{ 
  m_crc ^= (uint16_t)byte << 8;
  for (uint8_t i = 0; i < 8; i++)
  {
    if (m_crc & 0x8000)
    {
      m_crc = (m_crc << 1) ^ m_polynomial;
    }
    else
    {
      m_crc <<= 1;
    }
  }
}

CRC32::CRC32(CRC::Type type)
{
  switch(type)
  {
  case WD:
  default:
    m_initial = 0xFFFFFFFFUL;
    m_polynomial = 0x140A0445UL;
    break;
  case OMTI_ID:
    m_initial = 0x2605FB9CUL;
    m_polynomial = 0x0104C981UL;
    break;
  case OMTI_DATA:
    m_initial = 0xD4D7CA20UL;
    m_polynomial = 0x0104C981UL;
    break;   
  case XebecAdaptec:
    m_initial = 0;
    m_polynomial = 0x00A00805UL;
    break;
  case HDC9224:
    m_initial = 0xFFFFFFFFUL;
    m_polynomial = 0x00A00805UL;
    break;
  case SM1040_ID:
    m_initial = 0x6E958E56UL;
    m_polynomial = 0x140A0445UL;
    break;
  case SM1040_DATA:
    m_initial = 0xCF2105E0UL;
    m_polynomial = 0x140A0445UL;
    break;
  }
  
  m_crc = m_initial;
}

void CRC32::add(uint8_t byte)
{ 
  m_crc ^= (uint32_t)byte << 24;
  for (uint8_t i = 0; i < 8; i++)
  {
    if (m_crc & 0x80000000UL)
    {
      m_crc = (m_crc << 1) ^ m_polynomial;
    }
    else
    {
      m_crc <<= 1;
    }
  }
}

bool CRC32::tryComputeCorrection(uint8_t* buffer, size_t count)
{
  if (!hdd.getParams()->CorrectCRCErrors)
  {
    return false;
  }
  
  const uint8_t maxBurstLen = 11;
  uint32_t syndrome = m_crc;
  uint32_t mask = ~((1UL << maxBurstLen) - 1);
  uint32_t totalBits = count*8;

  // reverse walk
  for (uint32_t shift = 0; shift < totalBits; shift++)
  {
    // error pattern found?
    if ((syndrome & mask) == 0)
    {
      uint32_t errorIndex = totalBits - 1 - shift;
      
      // apply correction
      for (uint32_t i = 0; i < maxBurstLen; i++)
      {
        if (syndrome & (1UL << i))
        {
          uint32_t targetBit = errorIndex + i;
          if (targetBit >= totalBits)
          {
            break;
          }
          
          buffer[targetBit / 8] ^= (1 << (7 - (targetBit % 8))); // MSB first
        }
      }
      
      return true; // corrected
    }
    
    // step syndrome backward
    if (syndrome & 1)
    {
      syndrome = ((syndrome ^ m_polynomial) >> 1) | 0x80000000UL;
    }
    else
    {
      syndrome = syndrome >> 1;
    }
  }
  
  // multiple errors or of larger burst
  return false;
}

CRC56::CRC56(CRC::Type type)
{
  switch(type)
  {
  case WD:
  default:
    m_initial = 0xFFFFFFFFFFFFFFULL;
    m_polynomial = 0x140A0445000101ULL;
    break;
  }
  
  m_crc = m_initial;
}

void CRC56::add(uint8_t byte)
{ 
  m_crc ^= (uint64_t)byte << 48;
  for (uint8_t i = 0; i < 8; i++)
  {
    if (m_crc & 0x80000000000000ULL)
    {
      m_crc = (m_crc << 1) ^ m_polynomial;
    }
    else
    {
      m_crc <<= 1;
    }
    
    // mask off top 8 bits for 56 bits CRC
    m_crc &= 0xFFFFFFFFFFFFFFULL;
  }
}

bool CRC56::tryComputeCorrection(uint8_t* buffer, size_t count)
{
  // as above  
  if (!hdd.getParams()->CorrectCRCErrors)
  {
    return false;
  }
  
  const uint64_t mask56 = 0xFFFFFFFFFFFFFFULL;
  
  const uint8_t maxBurstLen = 22;
  uint64_t syndrome = m_crc;
  uint64_t mask = ~((1ULL << maxBurstLen) - 1) & mask56; // bits 63-56 masked
  uint32_t totalBits = count*8;

  for (uint32_t shift = 0; shift < totalBits; shift++)
  {
    if ((syndrome & mask) == 0)
    {
      uint32_t errorIndex = totalBits - 1 - shift;
      
      for (uint32_t i = 0; i < maxBurstLen; i++)
      {
        if (syndrome & (1ULL << i))
        {
          uint32_t targetBit = errorIndex + i;
          if (targetBit >= totalBits)
          {
            break;
          }
          
          buffer[targetBit / 8] ^= (1 << (7 - (targetBit % 8)));
        }
      }
      
      return true;
    }
    
    if (syndrome & 1)
    {
      syndrome = (((syndrome ^ m_polynomial) >> 1) | 0x80000000000000ULL) & mask56;
    }
    else
    {
      syndrome = syndrome >> 1;
    }
  }
  
  return false;
}