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
  case Seagate:
    m_initial = 0;
    m_polynomial = 0x41044185UL;
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
  case ADT:
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
  if (!hdd.getParams()->CorrectCRCErrors ||
      !count ||
      !m_crc ||
      !(m_polynomial & 1UL)) // does not work with even polynomials
  {
    return false;
  }

  const uint8_t crcBits = 32;
  const uint8_t maxBurstLen = 11;
  const uint32_t totalBits = count*8;

  auto reverseStep = [this](uint32_t crc)
  {
    if (crc & 1UL)
    {
      crc ^= m_polynomial;
      crc >>= 1;
      crc |= 0x80000000UL;
    }
    else
    {
      crc >>= 1;
    }

    return crc;
  };

  // reverse walk
  uint32_t syndrome = m_crc;
  for (uint32_t endBit = totalBits; endBit-- > 0;)
  {
    // remove the x^32 factor
    uint32_t errorPattern = syndrome;
    for (uint8_t i = 0; i < crcBits; i++)
    {
      errorPattern = reverseStep(errorPattern);
    }

    // check for a nonzero burst of <= 11 bits
    if (errorPattern && (errorPattern < (1UL << maxBurstLen)))
    {
      const uint8_t burstLen = 32 - __builtin_clz(errorPattern);

      // canonical burst: first and last bits are 1
      if (errorPattern & 1UL)
      {
        const uint32_t startBit = endBit + 1 - burstLen;

        // is the complete burst inside data?
        if ((startBit + burstLen) <= totalBits)
        {
          for (uint8_t i = 0; i < burstLen; i++)
          {
            // apply correction
            if (errorPattern & (1UL << i))
            {
              const uint32_t bit = endBit - i;
              buffer[bit / 8] ^= (uint8_t)(1U << (7 - (bit % 8)));
            }
          }

          return true; // corrected
        }
      }
    }

    // step syndrome backward
    syndrome = reverseStep(syndrome);
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
  // as above - for 56 bits and mask off top 8 bits of uint64
  if (!hdd.getParams()->CorrectCRCErrors ||
      !count ||
      !m_crc ||
      !(m_polynomial & 1ULL))
  {
    return false;
  }

  const uint64_t mask56 = 0xFFFFFFFFFFFFFFULL;
  const uint8_t crcBits = 56;
  const uint8_t maxBurstLen = 22;
  const uint32_t totalBits = count*8;

  auto reverseStep = [this](uint64_t crc)
  {
    if (crc & 1ULL)
    {
      crc ^= m_polynomial;
      crc >>= 1;
      crc |= 0x80000000000000ULL;
    }
    else
    {
      crc >>= 1;
    }

    return crc & mask56;
  };

  uint64_t syndrome = m_crc;
  for (uint32_t endBit = totalBits; endBit-- > 0;)
  {
    uint64_t errorPattern = syndrome;
    for (uint8_t i = 0; i < crcBits; i++)
    {
      errorPattern = reverseStep(errorPattern);
    }

    if (errorPattern && (errorPattern < (1ULL << maxBurstLen)))
    {
      const uint8_t burstLen = 64 - __builtin_clzll(errorPattern);

      if (errorPattern & 1ULL)
      {
        const uint32_t startBit = endBit + 1 - burstLen;
        
        if ((startBit + burstLen) <= totalBits)
        {
          for (uint8_t i = 0; i < burstLen; i++)
          {
            if (errorPattern & (1ULL << i))
            {
              const uint32_t bit = endBit - i;
              buffer[bit / 8] ^= (uint8_t)(1U << (7 - (bit % 8)));
            }
          }

          return true;
        }
      }
    }

    syndrome = reverseStep(syndrome);
  }

  return false;
}