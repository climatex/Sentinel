// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Basic disk drive calls

#include "config.h"

// defined in main.cpp
extern volatile bool g_SeekComplete;

HDD::HDD()
{
  m_SeekForward = false;
  m_ShiftRegister = 0;
  m_PhysicalCylinder = 0;
  m_PhysicalHead = 0;
  m_MicroSteps = 0;
  
  m_Result = HDD_STATUS_OK;
  m_ResultMessage = str_Empty;
}

void HDD::setLastResult(uint8_t res)
{
  // and assign message
  switch(res)
  {
  case HDD_STATUS_OK:
  case HDD_STATUS_INVALID_ARGS: // should not happen
  default:
    m_ResultMessage = str_Empty;
    break;
  case HDD_STATUS_TIMEOUT:      // better check why it timed out (not ready, write fault...)
    m_ResultMessage = str_StatusTimeout;
    break;
  case HDD_STATUS_NOT_READY:
    m_ResultMessage = str_StatusNotReady;
    break;
  case HDD_STATUS_WRITE_FAULT:
    m_ResultMessage = str_StatusWriteFault;
    break;
  case HDD_STATUS_NO_SECTOR_ID:
    m_ResultMessage = str_StatusNoSectorID;
    break;
  case HDD_STATUS_NO_DATA_ID:
    m_ResultMessage = str_StatusNoDataID;
    break;
  case HDD_STATUS_DATA_ERROR:
    m_ResultMessage = str_StatusDataError;
    break;
  case HDD_STATUS_DATA_CORRECTED:
    m_ResultMessage = str_StatusDataCorrected;
    break;
  }
  
  m_Result = res;
}

void HDD::updateShiftRegister()
{
  // m_ShiftRegister bits 7-0:
  // /MFM, DS, DIR, STEP, HDSEL3, HDSEL2, HDSEL1, HDSEL0
  
  gpio_put(22, false);  // RCLK low
  uint8_t bit = 8;
  while (bit--)
  {
    // set SER
    gpio_put(27, m_ShiftRegister & (1 << bit));    
    gpio_put(21, true); // toggle SRCLK
    busy_wait_at_least_cycles(80);
    gpio_put(21, false);
    busy_wait_at_least_cycles(80);
  }  
  
  gpio_put(22, true);   // RCLK high
}

void HDD::selectDrive(bool ds0 /* = true */)
{
  // false: unselects
  // drive must be selected before any operation on it
  if (ds0)
  {
    m_ShiftRegister |= 0x40;
  }
  else
  {
    m_ShiftRegister &= 0xBF;
  }
  updateShiftRegister();
}

bool HDD::isDriveReady()
{  
  // needs to be consistently low for quite a while (about 100ms)
  // as it can fire momentarily during disk powerup 
  const absolute_time_t deadline = make_timeout_time_ms(TIMEOUT_STARTUP_READY_MS);
  while (!time_reached(deadline))
  {
    if (gpio_get(15))
    {
      return false; // not ready, quit instantly
    }
  }
  
  return gpio_get(20); // to be considered ready, /WFAULT must be also high
}

bool HDD::isAtCylinder0()
{
  return !gpio_get(14); // /TRK0
}

bool HDD::checkReadyWriteFault()
{
  // check /READY and /WFAULT signals before and after operation, set error messages if so
  if (gpio_get(15))
  {
    m_Result = HDD_STATUS_NOT_READY;
    m_ResultMessage = str_StatusNotReady;
    return false;
  }
  
  else if (!gpio_get(20))
  {
    m_Result = HDD_STATUS_WRITE_FAULT;
    m_ResultMessage = str_StatusWriteFault;
    return false;
  }
  
  return true;
}

// find cylinder 0 during board initialization
// 1 slow seek step + wait for the head settle each singlestep
bool HDD::recalibrate()
{ 
  // select drive and head 0
  m_ShiftRegister &= 0xF0;
  m_PhysicalHead = 0;
  selectDrive();
  
  if (!isAtCylinder0())
  {
    // a maximum of 2048 cylinders can be configured
    uint16_t attempts = 2048;
    while (!isAtCylinder0()) // inspect /TRK0 each head settle
    {
      if (!attempts)
      {
        fatalError(str_DriveSeekFailure);
      }
      
      // direction change
      if (m_SeekForward)
      {
        m_ShiftRegister &= 0xDF; // DIRECTION low to seek towards 0 (inverted on drive, /DIRECTION)
        updateShiftRegister();
        sleep_us(1); // give it time after direction change
        m_SeekForward = false;
      }    
      
      attempts--;
      m_ShiftRegister ^= 0x10;   // toggle STEP
      updateShiftRegister();
      sleep_us(SEEK_PULSE_US);
      m_ShiftRegister ^= 0x10;   // toggle STEP
      updateShiftRegister();
      
      // wait for the head to settle each single step (ST506 18ms typical)
      // do not wait for seek complete here - the disk might perform its own recalibration, ignoring the flag
      sleep_ms(20);
    }
  }
   
  // we're at cyl 0 confirmed
  m_PhysicalCylinder = 0;  
  
  // some drives might be stuck on NOT SEEK COMPLETE if they were selected while the board was reset:
  // especially if the disk was already on cyl 0, with no slow seek to 0 done here
  // something to do with the output shift register?
  // anyway, twiddle the head back and forth to clear this flag  
  if (!g_SeekComplete)
  {
    seekDrive(1, 0); // if seekComplete is not unstuck, throw seek error here
    seekDrive(0, 0); // back to 0,0 as we were
  }
  
  return true;
}

// seek to given cylinder and head, set reduced write current or write precompensation line
bool HDD::seekDrive(uint16_t toCylinder, uint8_t toHead)
{ 
  // make sure the drive is selected and not in recovery mode
  selectDrive();
  microStepInternal(false);
  
  // set head
  if (m_PhysicalHead != toHead)
  {
    m_ShiftRegister = (m_ShiftRegister & 0xF0) | toHead;
    updateShiftRegister();
    m_PhysicalHead = toHead;    
  }
  
  // set cylinder
  if (m_PhysicalCylinder != toCylinder)
  {
    const bool forwards = toCylinder > m_PhysicalCylinder;
    uint16_t count = forwards ? (toCylinder - m_PhysicalCylinder) : (m_PhysicalCylinder - toCylinder);
    
    // direction change
    if (m_SeekForward != forwards)
    {
      if (forwards)
      {
        m_ShiftRegister |= 0x20;
      }
      else
      {
        m_ShiftRegister &= 0xDF;
      }
      
      updateShiftRegister();
      sleep_us(1);
      m_SeekForward = forwards;
    }
    
    // ST506 or buffered seek
    const bool slowSeek = m_Params.SlowSeek;
    g_SeekComplete = false;
    while (count)
    {
      m_ShiftRegister ^= 0x10;
      updateShiftRegister();
      sleep_us(SEEK_PULSE_US);
      m_ShiftRegister ^= 0x10;
      updateShiftRegister();
      
      if (slowSeek)
      {
        sleep_ms(SLOWSEEK_SRT_MS);  
      }
      else
      {
        sleep_us(FASTSEEK_SRT_US);
      }
      
      count--;
    }
    
    // wait until heads are settled after all seek pulses are done
    const absolute_time_t deadline = make_timeout_time_ms(TIMEOUT_SEEK_COMPLETE_MS);
    while (!g_SeekComplete)
    {
      if (time_reached(deadline))
      {
        fatalError(str_DriveSeekFailure);
      }    
    }
    
    m_PhysicalCylinder = toCylinder;
  }
  
  // the MSB HDSEL is on some very old drives used as reduced write current signal
  if (m_Params.UseReduceWriteCurrent && (m_Params.Heads <= 8))
  {
    if (m_PhysicalCylinder >= m_Params.RWCStartCyl)
    {
      m_ShiftRegister |= 8;    // HDSEL3 high (low /REDWC on drive)
    }
    else
    {
      m_ShiftRegister &= 0xF7; // /REDWC on drive high
    }
    updateShiftRegister();
  }

  // handle write precompensation enable line to data separator; /EARLY and /LATE done by ENDEC
  if (m_Params.UseWritePrecomp)
  {
    if (m_PhysicalCylinder >= m_Params.WritePrecompStartCyl)
    {
      gpio_put(12, true);      // WPCEN
    }
    else
    {
      gpio_put(12, false);
    }
  }
  
  return true;
}

// reusing the HDSEL3 line wired up to the RECOVERY MODE signal of the hard drive
void HDD::microStep(bool perform)
{
  if (!m_MicroSteps || (m_Params.Heads > 8) || m_Params.UseReduceWriteCurrent)
  {
    return;
  }  
  selectDrive();
 
  if (!perform) // cancel
  {
    microStepInternal(false);
    return;
  }
  
  for (uint8_t step = 0; step < m_MicroSteps; step++)
  {
    microStepInternal(true);
  }
}

void HDD::testMicrostepping()
{
  // make sure we're at track 0, then set direction forward and do 1 microstep
  seekDrive(0, 0);
  m_ShiftRegister |= 0x20;
  updateShiftRegister();
  sleep_us(1);
  microStepInternal(true);
  
  // if we're past cyl 0, it's not good
  if (!isAtCylinder0())
  {
    fatalError(str_DriveMicrostepFailure);
  }
    
  // restore
  microStepInternal(false);
  m_ShiftRegister &= 0xDF;
  updateShiftRegister();
  sleep_us(1);
}

void HDD::microStepInternal(bool perform)
{
  if (!m_MicroSteps || (m_Params.Heads > 8) || m_Params.UseReduceWriteCurrent)
  {
    return;
  }  
  
  if (!perform)
  {
    if (m_ShiftRegister & 8)
    {
      // leave recovery mode
      g_SeekComplete = false;
      m_ShiftRegister &= 0xF7;
      updateShiftRegister();
      sleep_ms(SLOWSEEK_SRT_MS);
      
      // disk must assert seek complete
      const absolute_time_t deadline = make_timeout_time_ms(TIMEOUT_SEEK_COMPLETE_MS);
      while (!g_SeekComplete)
      {
        if (time_reached(deadline))
        {      
          fatalError(str_DriveMicrostepFailure);
        }
      }      
    }
    
    return;
  }
   
  // enter recovery mode if not already in it
  if (!(m_ShiftRegister & 8))
  {
    m_ShiftRegister |= 8;
    updateShiftRegister();
    sleep_us(1); 
  }
  
  // microstep
  g_SeekComplete = false;
  m_ShiftRegister ^= 0x10;
  updateShiftRegister();
  sleep_us(SEEK_PULSE_US);
  m_ShiftRegister ^= 0x10;
  updateShiftRegister();
  sleep_ms(SLOWSEEK_SRT_MS);
  
  const absolute_time_t deadline = make_timeout_time_ms(TIMEOUT_SEEK_COMPLETE_MS);
  while (!g_SeekComplete)
  {
    if (time_reached(deadline))
    {
      fatalError(str_DriveMicrostepFailure);
    }
  }
}

void HDD::setSeparatorRLL(bool rll)
{
  // set data separator board mode, false: MFM (default), true: RLL  
  if (rll)
  {
    m_ShiftRegister |= 0x80;
  }
  else
  {
    m_ShiftRegister &= 0x7F;
  }
  
  updateShiftRegister();
}

bool HDD::isSeparatorRLL()
{
  return m_ShiftRegister & 0x80;
}

bool HDD::diskConfigurationIsPresent()
{
  // in flash
  CRC16 crc(CRC::Type::CCITT); // regular 16bit CCITT CRC precedes the parameters table  
  const uint8_t* flash = (const uint8_t*)(XIP_BASE + FLASH_CONFIG_TARGET);
  const uint16_t crcSaved = *((uint16_t*)&flash[0]);
  
  for (size_t idx = 0; idx < sizeof(DiskDriveParams); idx++)
  {
    crc.add(flash[idx+2]);
  }
  
  return crc.get() == crcSaved;
}

bool HDD::diskConfigurationLoad()
{
  if (!diskConfigurationIsPresent())
  {
    return false;
  }
  
  const uint8_t* flash = (const uint8_t*)(XIP_BASE + FLASH_CONFIG_TARGET);
  memcpy(&m_Params, &flash[2], sizeof(DiskDriveParams));
  return true;
}

bool HDD::diskConfigurationSave(bool eraseOnly)
{
  // eraseOnly: only gets rid of the old table; disk parameters are always asked during startup
  // returns true on success, false on failure  
  
  CRC16 crc(CRC::Type::CCITT);
  const uint8_t* current = (const uint8_t*)&m_Params;
  const uint8_t* flash = (const uint8_t*)(XIP_BASE + FLASH_CONFIG_TARGET);
  const uint16_t crcSaved = *((uint16_t*)&flash[0]);
  
  for (size_t idx = 0; idx < sizeof(DiskDriveParams); idx++)
  {
    crc.add(current[idx]);
  }
  const uint16_t crcNew = crc.get();
  
  // nothing changed
  if (!eraseOnly && (crcNew == crcSaved))
  {
    return true;
  }
  
  uint32_t backup = save_and_disable_interrupts();
  
  flash_range_erase(FLASH_CONFIG_TARGET, FLASH_SECTOR_SIZE);
  if (eraseOnly)
  {
    restore_interrupts(backup);
    return true;
  }
  
  uint8_t flashpage[FLASH_PAGE_SIZE];
  memset(&flashpage[0], 0xFF, FLASH_PAGE_SIZE);
  memcpy(&flashpage[0], &crcNew, sizeof(uint16_t));
  memcpy(&flashpage[2], current, sizeof(DiskDriveParams));
  flash_range_program(FLASH_CONFIG_TARGET, &flashpage[0], FLASH_PAGE_SIZE);
  
  restore_interrupts(backup);
  
  return diskConfigurationIsPresent();
}

void HDD::diskConfigurationProvide()
{
  bool eraseWished = false; // remove saved settings answered Yes
  char key;
  
  if (diskConfigurationLoad())
  {
    // show what was loaded and wait for confirmation
    printf(str_DiskCfgCylinders, m_Params.Cylinders);    
    printf(str_DiskCfgHeads, m_Params.Heads);
    
    printf(str_DiskCfgRWC);
    if (m_Params.UseReduceWriteCurrent)
    {
      printf(str_Enabled);
      printf(str_DiskCfgFromCyl, m_Params.RWCStartCyl);
    }
    else
    {
      printf(str_Disabled);
    }
    
    printf(str_DiskCfgPrecomp);
    if (m_Params.UseWritePrecomp)
    {
      printf(str_Enabled);
      printf(str_DiskCfgFromCyl, m_Params.WritePrecompStartCyl);
    }
    else
    {
      printf(str_Disabled);
    }
    
    printf(str_DiskCfgLZStatus);
    if (m_Params.UseLandingZone)
    {
      printf(str_No);
      printf(str_DiskCfgLZ, m_Params.LandingZone); 
    }
    else
    {
      printf(str_Yes);
    }
    
    printf(str_DiskCfgSeekMode);
    printf(m_Params.SlowSeek ? str_DiskCfgSeekSlow : str_DiskCfgSeekNormal);
    
    printf(str_DiskCfgReseek);
    printf(m_Params.ReseekOnSectorErrors ? str_Enabled : str_Disabled);
    
    printf(str_DiskCfgCorrectCRC);
    printf(m_Params.CorrectCRCErrors ? str_Enabled : str_Disabled);
    
    // give a timeout from keyboard to respecify these
    bool autoLoad = true;
    printf(str_DiskCfgSavedLoad);
    printf(str_Abort);
    
    absolute_time_t deadline = make_timeout_time_ms(2500);
    while (!time_reached(deadline))
    {
      key = readKey("\e", false); // check if Esc pressed and return immediately
      if (key == '\e')
      {
        autoLoad = false;
        break;
      }
    }
    printf(str_DeleteLine);
    
    if (autoLoad)
    {
      // we're done
      return;
    }
    
    // aborted; before respecifying, give option to remove them
    printf(str_DiskCfgAskRemove);
    key = toupper(readKey("YN"));
    printf(str_EchoKey, key);
    if (key == 'Y')
    {
      eraseWished = true;
    }    
    
    memset(&m_Params, 0, sizeof(DiskDriveParams)); // clear all just to be safe
  }
  
  // (re-)specify
  printf(str_DiskCfgAskParams);
  
  // cylinders 1-2048
  while(true)
  {
    printf(str_DiskCfgAskCylinders);
    uint16_t cylinders = (uint16_t)atoi(prompt(4, str_DecimalInput));
    if ((cylinders > 0) && (cylinders <= 2048))
    {
      m_Params.Cylinders = cylinders;      
      printf("\n");
      break;
    }
    
    printf(str_DeleteLine);
  }
  
  // heads 1-16
  while(true)
  {
    printf(str_DiskCfgAskHeads);
    uint16_t heads = (uint16_t)atoi(prompt(2, str_DecimalInput));
    if ((heads > 0) && (heads <= 16))
    {
      m_Params.Heads = heads;      
      printf("\n");
      break;
    }
    
    printf(str_DeleteLine);
  }
  
  // ask for reduced write current, if heads <= 8 (HDSEL3 used for RWC)
  if (m_Params.Heads <= 8)
  {
    printf(str_DiskCfgAskRWC);
    key = toupper(readKey("YN"));
    m_Params.UseReduceWriteCurrent = (key == 'Y');
    printf(str_EchoKey, key);
    
    if (key == 'Y')
    {
      // RWC starting cylinder 0-2047
      while(true)
      {
        printf(str_DiskCfgAskCylRWC);
        uint16_t startCyl = (uint16_t)atoi(prompt(4, str_DecimalInput));
        if (startCyl < 2048)
        {
          m_Params.RWCStartCyl = startCyl;
          if (!startCyl && !strlen(getPromptBuffer())) printf("0");
          printf("\n");
          break;
        }
        
        printf(str_DeleteLine);
      }
    }
  }
  
  // ask for reduced write precompensation
  printf(str_DiskCfgAskPrecomp);
  key = toupper(readKey("YN"));
  m_Params.UseWritePrecomp = (key == 'Y');
  printf(str_EchoKey, key);
  
  if (key == 'Y')
  {
    // precomp starting cylinder 0-2047
    while(true)
    {
      printf(str_DiskCfgAskCylPrecomp);
      uint16_t startCyl = (uint16_t)atoi(prompt(4, str_DecimalInput));
      if (startCyl < 2048)
      {
        m_Params.WritePrecompStartCyl = startCyl;  
        if (!startCyl && !strlen(getPromptBuffer())) printf("0");        
        printf("\n");
        break;
      }
      
      printf(str_DeleteLine);
    }
  }
  
  // ask if drive has landing zone
  printf(str_DiskCfgAskLZ);
  key = toupper(readKey("YN"));
  m_Params.UseLandingZone = (key == 'Y');
  printf(str_EchoKey, key);
  
  if (key == 'Y')
  {
    // LZ starting cylinder 0-2047
    while(true)
    {
      printf(str_DiskCfgAskCylLZ);
      uint16_t startCyl = (uint16_t)atoi(prompt(4, str_DecimalInput));
      if (startCyl < 2048)
      {
        m_Params.LandingZone = startCyl;
        if (!startCyl && !strlen(getPromptBuffer())) printf("0");
        printf("\n");
        break;
      }
      
      printf(str_DeleteLine);
    }
  }
 
  // seek slow or buffered 
  printf(str_DiskCfgAskSeek);
  key = toupper(readKey("NL"));
  m_Params.SlowSeek = (key == 'L');
  printf(str_EchoKey, key);
  
  // re-seek on sector errors
  printf(str_DiskCfgAskReseek);
  key = toupper(readKey("YN"));
  m_Params.ReseekOnSectorErrors = (key == 'Y');
  printf(str_EchoKey, key);
  
  // correct CRC data errors
  printf(str_DiskCfgAskCorrectCRC);
  key = toupper(readKey("YN"));
  m_Params.CorrectCRCErrors = (key == 'Y');
  printf(str_EchoKey, key);
    
  // save settings?
  printf(str_DiskCfgAskSave);
  key = toupper(readKey("YN"));
  printf(str_EchoKey, key);
  if (key == 'Y')
  {
    printf(str_OperationPending);
        
    bool saveResult = diskConfigurationSave();
    printf(" "); printf(saveResult ? str_OK : str_Error); printf("\n");    
  }
  
  // get rid of saved only
  else if (eraseWished)
  {
    diskConfigurationSave(true);
  }
}    