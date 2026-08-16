// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Basic disk drive calls

#pragma once
#include "config.h"

// HDD_STATUS (error) codes; order is important
#define HDD_STATUS_OK             0 // no error
#define HDD_STATUS_INVALID_ARGS   1 // sanity check failed, invalid arguments
#define HDD_STATUS_TIMEOUT        2 // operation timed out
#define HDD_STATUS_NOT_READY      3 // disk drive /READY high
#define HDD_STATUS_WRITE_FAULT    4 // disk drive /WFAULT low
#define HDD_STATUS_NO_SECTOR_ID   5 // no ID address mark (sector not found)
#define HDD_STATUS_NO_DATA_ID     6 // no data address mark
#define HDD_STATUS_DATA_ERROR     7 // CRC mismatch in data field
#define HDD_STATUS_DATA_CORRECTED 8 // successful data field correction

class HDD
{
public:

  // needs to be POD
  struct DiskDriveParams
  {
    uint16_t Cylinders;
    uint8_t Heads;
    bool UseWritePrecomp;
    uint16_t WritePrecompStartCyl;
    bool UseReduceWriteCurrent;
    uint16_t RWCStartCyl;
    bool UseLandingZone;
    uint16_t LandingZone;
    bool SlowSeek;
    bool ReseekOnSectorErrors;
    bool CorrectCRCErrors;
  };
  
  HDD();
    
  DiskDriveParams* getParams() { return &m_Params; }
   
  void setSeparatorRLL(bool rll);
  bool isSeparatorRLL();
  void selectDrive(bool ds0 = true);
  bool isDriveReady();
  bool checkReadyWriteFault();
  bool isAtCylinder0();
  bool recalibrate();
  bool seekDrive(uint16_t, uint8_t);
  
  uint8_t getMicrostepping() { return m_MicroSteps; }
  void setMicrostepping(uint8_t steps) { m_MicroSteps = steps; }
  void testMicrostepping();
  void microStep(bool perform);
    
  uint16_t getPhysicalCylinder() { return m_PhysicalCylinder; }
  uint8_t getPhysicalHead() { return m_PhysicalHead; }
  
  void setLastResult(uint8_t result);
  uint8_t getLastResult() { return m_Result; }
  const char* getLastResultMessage() { return m_ResultMessage; }
  
  void diskConfigurationProvide();
  
private:
  void microStepInternal(bool perform);
  void updateShiftRegister();
  bool diskConfigurationIsPresent();
  bool diskConfigurationLoad();
  bool diskConfigurationSave(bool eraseOnly = false);
  
  DiskDriveParams m_Params = {};
  
  uint8_t m_ShiftRegister;
  uint8_t m_Result;
  const char* m_ResultMessage; 
  bool m_SeekForward;
  uint16_t m_PhysicalCylinder;
  uint8_t m_PhysicalHead;
  uint8_t m_MicroSteps;
};