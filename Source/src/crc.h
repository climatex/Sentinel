// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// CRC computation

#pragma once
#include "config.h"

class CRC
{
public:

  enum Type
  {
    CCITT = 0,
    WD,
    OMTI_ID,
    OMTI_DATA,
    XebecAdaptec,
    HDC9224,
    SM1040_ID,
    SM1040_DATA    
  };
  
  virtual ~CRC() {};

  virtual void setInitial() = 0;  
  virtual void add(uint8_t byte) = 0;
  virtual uint64_t get() = 0;
  virtual bool tryComputeCorrection(uint8_t* buffer, size_t count) = 0;
};

class CRC16 : public CRC
{
public:
  CRC16(CRC::Type type);
  
  void setInitial() { m_crc = m_initial; }
  void add(uint8_t byte);
  uint64_t get() { return m_crc; }
  bool tryComputeCorrection(uint8_t* buffer, size_t count) { return false; }
  
private:
  uint16_t m_crc;
  uint16_t m_initial;
  uint16_t m_polynomial;
};

class CRC32 : public CRC
{
public:
  CRC32(CRC::Type type);
  
  void setInitial() { m_crc = m_initial; }
  void add(uint8_t byte);
  uint64_t get() { return m_crc; }  
  bool tryComputeCorrection(uint8_t* buffer, size_t count);
  
private:
  uint32_t m_crc;
  uint32_t m_initial;
  uint32_t m_polynomial;
};

class CRC56 : public CRC
{
public:
  CRC56(CRC::Type type);
  
  void setInitial() { m_crc = m_initial; }
  void add(uint8_t byte);
  uint64_t get() { return m_crc; }
  bool tryComputeCorrection(uint8_t* buffer, size_t count);
  
private:
  uint64_t m_crc;
  uint64_t m_initial;
  uint64_t m_polynomial;
};
