// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Main menu commands

#include "config.h"

// from main.cpp
extern float g_WclockRate;

// internal forward decls of format menu commands
void commandAnalyze();
void commandHexdump();
void commandRead(bool verifyOnly = false);
void commandWrite(bool formatOnly = false);
void commandMicrostep();
void commandRawdiskOperation(bool readDiskIntoFile, const char* ext, uint16_t channelBytes);

LLF* llf;
WD* wd;
OMTI* omti;
XebecAdaptec* xebecAdaptec;
HDC9224* hdc9224;
SM1040* sm1040;

// low level format menu
void formatMenu(LLF* format)
{
  llf = format;

  // -fno-rtti, no dynamic_cast
  wd = NULL;
  omti = NULL;
  xebecAdaptec = NULL;
  hdc9224 = NULL;  
  sm1040 = NULL;
  switch(llf->getType())
  {
  case LLF::FormatType::WD:
    wd = (WD*)llf;
    break;
  case LLF::FormatType::OMTI:
    omti = (OMTI*)llf;
    break;
  case LLF::FormatType::XebecAdaptec:
    xebecAdaptec = (XebecAdaptec*)llf;
    break;
  case LLF::FormatType::HDC9224:
    hdc9224 = (HDC9224*)llf;
    break;
  case LLF::FormatType::SM1040:
    sm1040 = (SM1040*)llf;
    break;
  }
  
  for (;;)
  {
    char key;
    char menuOptions[10] = {0};
    
    hdd.seekDrive(0, 0);
    hdd.selectDrive(false);
    printf("\n");
    
    char microstepIndicator[10] = {0};
    if (!hdd.getMicrostepping())
    {
      strcpy(microstepIndicator, str_MicrostepOff);
    }
    else
    {
      snprintf(microstepIndicator, sizeof(microstepIndicator), str_MicrostepSteps, hdd.getMicrostepping());
      if (hdd.getMicrostepping() == 1)
      {
        microstepIndicator[strlen(microstepIndicator)-1] = 0; // steps -> step :)
      }
    }
    
    strcat(menuOptions, "AHVRMFWB");
    printf(str_LlfMenu, microstepIndicator);
    
    if (!sm1040) // add mount DOS option for all except this one
    {
      printf(str_LlfMountDOS);
      strcat(menuOptions, "I");
    }
    printf(str_LlfBack);
    printf("\n"); printf(str_ChooseOption);
    key = toupper(readKey(menuOptions));
    printf(str_EchoKey, key);
    
    if (key == 'A')
    {
      commandAnalyze();
    }
    else if (key == 'H')
    {
      commandHexdump();
    }
    else if (key == 'V')
    {
      commandRead(true);
    }
    else if (key == 'R')
    {
      commandRead();
    }
    else if (key == 'M')
    {
      commandMicrostep();
    }
    else if (key == 'F')
    {
      commandWrite(true);
    }
    else if (key == 'W')
    {
      commandWrite();
    }
    else if (key == 'I')
    {
      commandDos(llf); // in dos.cpp
    }
    else if (key == 'B')
    {
      return;
    }
  }
}

// launched from the main menu
void commandAutodetect()
{
  printf("\n");
  hdd.seekDrive(0, 0); // select drive and inspect track 0
  
  if (!hdd.checkReadyWriteFault())
  {
    printf(hdd.getLastResultMessage());
    printf("\n");
    return;    
  }
  
  uint8_t sectorsPerTrack;
  uint8_t startSector;
  uint16_t sectorSizeBytes;
  uint8_t interleave;
  uint16_t actualCyl;
  uint8_t actualHd;
  printf(str_OperationPending);
  
  // try the SMC HDC9224 before WD, as these have similar ID fields
  hdc9224 = new HDC9224;
  if (hdc9224->analyzeTrack(MAX_SPT_LIMIT, false, sectorsPerTrack, startSector, sectorSizeBytes, interleave))
  {
    bool dummy;
    hdc9224->getCustomAnalyzeTrackResults(dummy, dummy, actualCyl, actualHd);
    
    // do a test read of one data field on track; if the CRC is okay, it is the SMC
    if (hdc9224->readSector(startSector, &actualCyl, &actualHd))
    {
      printf(str_DetectFormat);
      printf("HDC9224");
          
      printf("\n");
      delete hdc9224;
      hdc9224 = NULL;
      return;
    }
  }
  delete hdc9224;
  hdc9224 = NULL;
  
  // WD - MFM first, then RLL
  for (uint8_t attempt = 0; attempt < 2; attempt++)
  {
    hdd.setSeparatorRLL(attempt == 1);
    wd = new WD;
    wd->setSdh4Bit(true); // detect if extended to 4    
    
    // assume both unknown
    uint8_t sdh4Bit = (uint8_t)-1;
    uint8_t dataCrcBits = 0;
    
    if (wd->analyzeTrack(MAX_SPT_LIMIT, false, sectorsPerTrack, startSector, sectorSizeBytes, interleave))
    {
      // sector ID fields present
      bool dummy;
      wd->getCustomAnalyzeTrackResults(dummy, dummy, dummy, actualCyl, actualHd);
      if (actualHd > 7) // on track 0 for some reason?
      {
        sdh4Bit = 1;
      }
      
      // if on MFM, determine data field CRC type; RLL is always 56 bits
      if (hdd.isSeparatorRLL())
      {
        dataCrcBits = 56;
      }
      else
      {
        // read first sector on track to determine CRC used
        // get extended analysis results for actual cylinder and head number, if it differs from 0 for some reason...
        wd->setWorkingSectorSizeBytes(sectorSizeBytes);
        
        // attempt 0: 32 bit CRC - default, then try 16 and 56
        dataCrcBits = 32;
        for (uint8_t crcAttempt = 0; crcAttempt < 3; crcAttempt++)
        {
          if (crcAttempt > 0)
          {
            dataCrcBits = (crcAttempt == 1) ? 16 : 56;
            wd->setDataCrcBits(dataCrcBits);
          }
          
          if (wd->readSector(startSector, &actualCyl, &actualHd))
          {
            break;
          }
          dataCrcBits = 0; // couldn't determine this loop
        }
      }
      
      // try to seek to the maximum configured head on cyl 0
      if ((hdd.getParams()->Heads > 1) && (sdh4Bit != 1))
      {
        hdd.seekDrive(0, hdd.getParams()->Heads-1);        
        if (wd->analyzeTrack(MAX_SPT_LIMIT, false, sectorsPerTrack, startSector, sectorSizeBytes, interleave))
        {
          wd->getCustomAnalyzeTrackResults(dummy, dummy, dummy, actualCyl, actualHd);
          sdh4Bit = (actualHd > 7) ? 1 : 0;
        }
      }
      
      printf(str_DetectFormat);
      if (!dataCrcBits || (sdh4Bit == (uint8_t)-1))
      {
        printf(str_DetectLikely);
      }
      printf("WD (%s)", hdd.isSeparatorRLL() ? "RLL" : "MFM");
      printf(str_DetectCRC);
      if (!dataCrcBits)
      {
        printf(str_DetectUnknown);
      }
      else
      {
        printf(str_DetectBits, dataCrcBits);
      }
      printf(str_DetectHeadSelect);
      if (sdh4Bit == (uint8_t)-1)
      {
        printf(str_DetectUnknown);
      }
      else
      {
        printf(str_DetectBits, sdh4Bit ? 4 : 3);
      }
      
      printf("\n");
      delete wd;
      wd = NULL;
      return;
    }    
    
    delete wd;
    wd = NULL;
  }  
  hdd.setSeparatorRLL(false);
  
  // Xebec/Adaptec
  xebecAdaptec = new XebecAdaptec;
  if (xebecAdaptec->analyzeTrack(MAX_SPT_LIMIT, false, sectorsPerTrack, startSector, sectorSizeBytes, interleave))
  {
    bool dummy;
    xebecAdaptec->getCustomAnalyzeTrackResults(dummy, dummy, actualCyl, actualHd);
    
    uint8_t dataFieldsAdaptec = (uint8_t)-1;
    if (xebecAdaptec->readSector(startSector, &actualCyl, &actualHd))
    {
      dataFieldsAdaptec = xebecAdaptec->getWriteModeAdaptec() ? 1 : 0;
    }
    
    printf(str_DetectFormat);
    if (dataFieldsAdaptec == (uint8_t)-1)
    {
      printf(str_DetectLikely);
    }
    printf("Xebec/Adaptec");
    printf(str_DetectDataFieldXebec);
    if (dataFieldsAdaptec == (uint8_t)-1)
    {
      printf(str_DetectUnknown);
    }
    else
    {
      printf(dataFieldsAdaptec ? "00 (Adaptec)" : "C9 (Xebec)");
    }
    
    printf("\n");
    delete xebecAdaptec;
    xebecAdaptec = NULL;
    return;
  }
  delete xebecAdaptec;
  xebecAdaptec = NULL;
  
  // OMTI, no data field reads
  omti = new OMTI;
  if (omti->analyzeTrack(MAX_SPT_LIMIT, false, sectorsPerTrack, startSector, sectorSizeBytes, interleave))
  {
    printf(str_DetectFormat);
    printf("OMTI");
        
    printf("\n");
    delete omti;
    omti = NULL;
    return;
  }
  delete omti;
  omti = NULL;
  
  // SM1040, no data field reads
  sm1040 = new SM1040;
  if (sm1040->analyzeTrack(MAX_SPT_LIMIT, false, sectorsPerTrack, startSector, sectorSizeBytes, interleave))
  {
    printf(str_DetectFormat);
    printf("SM1040");
        
    printf("\n");
    delete sm1040;
    sm1040 = NULL;
    return;
  }
  delete sm1040;
  sm1040 = NULL;
  
  // we tried :)
  printf(str_DetectNoFormat);  
}

// launched from the main menu
void commandRawdisk()
{
  // memory card required
  if (!sdDetect())
  {
    return;
  }
  
  printf("\n");
  
  // compute average time from /INDEX to /INDEX
  uint64_t averager = 0;
  const uint16_t samples = 450;
  
  hdd.selectDrive();
  for (uint16_t sample = 0; sample < samples; sample++)
  {
    bool checkReady = hdd.checkReadyWriteFault();
    bool startOfTrack = true;
    if (!checkReady || !endec.waitForTrackStart())
    {
      printf(str_Error); printf("\n"); printf(hdd.getLastResultMessage()); printf("\n");
      return;
    }
    
    absolute_time_t t1 = get_absolute_time();
    
    // abort on not ready, write fault or timeout
    while (!gpio_get(15) &&
           gpio_get(20) && 
           (startOfTrack || gpio_get(6)))
    {
      if (gpio_get(6)) // /INDEX is high, reset start of track flag
      {
        startOfTrack = false;
      }
    }    
    
    absolute_time_t t2 = get_absolute_time();
    averager += absolute_time_diff_us(t1,t2);
    
    printf(str_RawdiskAveraging, ((sample*1.0f) / (samples*1.0f))*100.0f);
  }
  hdd.selectDrive(false);
  
  uint32_t averageBytes = ((g_WclockRate / 8.0) * ((averager * 1.0) / (samples * 1.0)) * 2.0); // x2, sampling both RCLOCK/WCLOCK edges
  printf(str_RawdiskAveraged, averageBytes, str_Bytes);
  
  // ask to customize maximum number of bytes read
  printf(str_EscGoBack);
  uint32_t channelBytes = 0;
  while(true)
  {
    printf(str_RawdiskCustomTrackLen);
    const char* promptStr = prompt(5, str_DecimalInputEsc, true);
    if (!promptStr) // ESC key returns to main menu
    {
      printf("\n");
      return;
    }
    channelBytes = (uint32_t)atoi(promptStr);
    if (channelBytes <= 65535)
    {  
      if (!channelBytes && !strlen(getPromptBuffer())) printf("0");
      printf("\n");
      break;
    }
    
    printf(str_RawdiskCustomTrackOver);
  }
  if (!channelBytes)
  {
    channelBytes = averageBytes;
  }
  
  // data separator mode to choose
  printf(str_ChooseSeparatorMode, ((int)g_WclockRate == 5) ? "MFM" : "RLL");
  char key = toupper(readKey("MR\e"));        
  if (key == '\e')
  {
    printf("\n");
    return;
  }
  printf(str_EchoKey, key);  
  hdd.setSeparatorRLL(key == 'R');
  
  // usage
  printf(str_RawdiskDescription, channelBytes);
  
  for (;;)
  {
    hdd.seekDrive(0, 0);
    hdd.selectDrive(false);
    
    char mfmRll[] = "MFM";  
    char fileExt[] = ".mfm";
    if (hdd.isSeparatorRLL())
    {
      strcpy(mfmRll, "RLL");
      strcpy(fileExt, ".rll");
    }
    printf(str_RawdiskMenu, mfmRll, mfmRll);
    printf("\n"); printf(str_ChooseOption);
    
    char key = toupper(readKey("RWB\e"));
    if (key == '\e')
    {
      printf("\n");
      return;
    }
    printf(str_EchoKey, key);
    if (key == 'B')
    {
      return;
    }
    
    commandRawdiskOperation(key == 'R', fileExt, channelBytes);
  }
}

void commandRawdiskOperation(bool readDiskIntoFile, const char* ext, uint16_t channelBytes)
{
  if (!sdFilePicker(readDiskIntoFile, ext))
  {
    return;
  }
  
  // start and end cylinder
  uint16_t startCylinder = 0;
  if (hdd.getParams()->Cylinders > 1)
  {
    while(true)
    {
      printf(str_ChooseStartCyl, 0, hdd.getParams()->Cylinders-1);
      const char* promptStr = prompt(4, str_DecimalInputEsc, true);
      if (!promptStr)
      {
        printf("\n");
        return;
      }
      startCylinder = (uint16_t)atoi(promptStr);
      if (startCylinder < hdd.getParams()->Cylinders)
      {  
        if (!startCylinder && !strlen(getPromptBuffer())) printf("0");
        printf("\n");
        break;
      }
      
      printf(str_DeleteLine);
    }
  }  
  
  uint16_t endCylinder = hdd.getParams()->Cylinders-1;
  if (startCylinder != endCylinder)
  {
    while(true)
    {
      printf(str_ChooseEndCyl, startCylinder, hdd.getParams()->Cylinders-1);
      const char* promptStr = prompt(4, str_DecimalInputEsc, true);
      if (!promptStr)
      {
        printf("\n");
        return;
      }
      endCylinder = (uint16_t)atoi(promptStr);
      if ((endCylinder >= startCylinder) && (endCylinder < hdd.getParams()->Cylinders))
      {  
        if (!endCylinder && !strlen(getPromptBuffer())) printf("0");
        printf("\n");
        break;
      }
      
      printf(str_DeleteLine);
    }  
  }
  
  // show warning before write
  if (!readDiskIntoFile)
  {
    printf(str_RawdiskWriteWarning, channelBytes, endCylinder-startCylinder+1, hdd.getParams()->Heads);
    printf(str_ContinueAbort);
    char key = readKey("\r\e");
    printf(str_DeleteLine);
    if (key == '\e')
    {
      sdCloseFile();
      return;
    }
  }
  
  // do I/O
  std::vector<uint8_t> trackBuffer;
  trackBuffer.resize(channelBytes+1); //+1 for readFifo() which reads in byte pairs
  
  for (uint16_t cylinder = startCylinder; cylinder <= endCylinder; cylinder++)
  {    
    for (uint8_t head = 0; head < hdd.getParams()->Heads; head++)    
    {   
      memset(trackBuffer.data(), 0, trackBuffer.size());
      hdd.seekDrive(cylinder, head);
      if (readDiskIntoFile)
      {
        hdd.microStep(true);  // if reading, apply microstep if configured
      }      
      printf(str_CHInfo, cylinder, head);
      
      if (!hdd.checkReadyWriteFault())
      {
        printf(hdd.getLastResultMessage());
        printf("\n");
        sdCloseFile();
        return;
      }
      
      size_t bytesOkay = 0; // bytes read or written okay from file, should be equal to channelBytes
      size_t seekPos = 0;
      const bool rll = hdd.isSeparatorRLL();
      
      // write disk from file - precompensate encoded data and put them in DMA writer
      if (!readDiskIntoFile)
      {
        seekPos = sdGetSeekPos();
        if (!sdReadFile(trackBuffer.data(), channelBytes, &bytesOkay))
        {
          printf(str_SdErrorFS); // filesystem error
          sdCloseFile();
          return;  
        }
        
        std::vector<uint32_t> dmaBuffer;
        dmaBuffer.reserve(bytesOkay); // in uint32_ts, each bit occupies 4 bits with precompensation information
        size_t bitsTotal = bytesOkay * 8;       
        
        uint32_t fifoWord = 0;
        uint8_t nibblesInWord = 0;
        
        // apply write precompensation for an already encoded bitstream
        for (size_t i = 0; i < bitsTotal; i++)
        {         
          const bool wdata = endec.getBit(trackBuffer.data(), bytesOkay, i);
          ENDEC::Precomp precomp = ENDEC::PRECOMP_NOMINAL; // default
          
          if (wdata)
          {
            if (rll)
            {
              // precomp rules see ENDEC::encodeRLL()
              uint8_t precedingRun = 4;
              if (i >= 3 && endec.getBit(trackBuffer.data(), bytesOkay, i - 3)) precedingRun = 2;
              else if (i >= 4 && endec.getBit(trackBuffer.data(), bytesOkay, i - 4)) precedingRun = 3;
         
              uint8_t followingRun = 4;
              if (i + 3 < bitsTotal && endec.getBit(trackBuffer.data(), bytesOkay, i + 3)) followingRun = 2;
              else if (i + 4 < bitsTotal && endec.getBit(trackBuffer.data(), bytesOkay, i + 4)) followingRun = 3;
         
              if ((precedingRun == 2) && (followingRun >= 4))      precomp = ENDEC::PRECOMP_EARLY;
              else if ((precedingRun >= 4) && (followingRun == 2)) precomp = ENDEC::PRECOMP_LATE;
            }
            
            // MFM
            else
            {
              // even bitindex: clock bit, odd: data bit
              size_t i_data = (i % 2) ? i : i+1; // always index of a data bit
              
              // precompensation applied on data - odd indexes
              bool prev2 = (i_data >= 4) ? endec.getBit(trackBuffer.data(), bytesOkay, i_data - 4) : false;
              bool prev1 = (i_data >= 2) ? endec.getBit(trackBuffer.data(), bytesOkay, i_data - 2) : false;
              bool comp  = endec.getBit(trackBuffer.data(), bytesOkay, i_data);
              bool next  = (i_data + 2 < bitsTotal) ? endec.getBit(trackBuffer.data(), bytesOkay, i_data + 2) : false;

              uint8_t lookup = ((uint8_t)prev2 << 3) | ((uint8_t)prev1 << 2) | ((uint8_t)comp << 1) | (uint8_t)next;
              precomp = ENDEC::MFM_PRECOMP[lookup];
            }
          }
          
          fifoWord = (fifoWord << 4) | endec.getPrecompNibble(wdata, precomp);
          if (++nibblesInWord == 8) // pushed when full
          {
            dmaBuffer.push_back(fifoWord);
            fifoWord = 0;
            nibblesInWord = 0;
          }
        }
        
        // pad to full 32 bits for PIO TX
        while (nibblesInWord > 0 && nibblesInWord < 8)
        {
          fifoWord = (fifoWord << 4) | endec.getPrecompNibble(0, ENDEC::PRECOMP_NOMINAL);
          nibblesInWord++;
        }
        if (nibblesInWord == 8)
        {
          dmaBuffer.push_back(fifoWord);
        }

        endec.prepareWriteDMA(dmaBuffer.data(), dmaBuffer.size());
      }
      
      // wait for INDEX for both read and write
      bool startOfTrack = true;
      if (!endec.waitForTrackStart())
      {
        endec.setReadGate(false);
        printf(hdd.getLastResultMessage());
        printf("\n");
        sdCloseFile();        
        return;
      }
           
      // disk dump
      if (readDiskIntoFile)
      {
        uint16_t bytesRead = 0;
        
        // break on not ready, write fault, target reached or FIFO read error        
        endec.setReadGate(true);        
        while ((bytesRead < channelBytes) && !gpio_get(15) && gpio_get(20))
        {
          uint16_t fifoWord = 0;
          if (!endec.readFifo16(fifoWord))
          {
            endec.setReadGate(false);
            hdd.setLastResult(HDD_STATUS_TIMEOUT);
            printf(hdd.getLastResultMessage());
            printf("\n");
            sdCloseFile();
            return;
          }
          if (!fifoWord)
          {
            continue; // sampling zero words, wait for RDATA to wake up
          }
                    
          // in pairs
          trackBuffer[bytesRead++] = (uint8_t)(fifoWord >> 8);
          trackBuffer[bytesRead++] = fifoWord;                   
        }
        endec.setReadGate(false);        
        
        if (!hdd.checkReadyWriteFault()) // yet another check
        {
          printf(hdd.getLastResultMessage());
          printf("\n");
          sdCloseFile();
          return;
        }
        
        // save track buffer to card
        seekPos = sdGetSeekPos();
        if (!sdWriteFile(trackBuffer.data(), channelBytes, &bytesOkay))
        {
          printf(str_SdErrorFS); // filesystem error
          sdCloseFile();
          return;  
        }
        
        printf(str_RawdiskProgress, "at", seekPos);
        if (bytesOkay != channelBytes)
        {
          printf("\n");
          printf(str_SdErrorFull); // likely full
          printf("\n");
          sdCloseFile();
          return;
        }
      }

      // disk write
      else
      {
        endec.setWriteGate(true); // until not ready, write fault, end of track or transfer done
        while (!gpio_get(15) &&
               gpio_get(20) &&
               (startOfTrack || gpio_get(6)) &&
               !pio_sm_is_tx_fifo_empty(pio0, 1))
        {
          if (gpio_get(6)) // /INDEX went high, reset start of track flag
          {
            startOfTrack = false;
          }
        }
        endec.setWriteGate(false);
        
        if (!hdd.checkReadyWriteFault())
        {
          printf(hdd.getLastResultMessage());
          printf("\n");
          sdCloseFile();
          return;
        }        
        
        printf(str_RawdiskProgress, "from", seekPos);
        if ((bytesOkay != channelBytes) || sdIsEndOfFile())
        {
          printf("\n");
          printf(str_SdErrorEndOfFile); // reached end-of-file?
          printf("\n");
          sdCloseFile();
          return;
        }
      }      
    }
  }

  endec.setReadGate(false);
  sdCloseFile();
  printf("\n");  
}

// launched from the main menu
void commandErase()
{
  printf(str_ErasePrompt);  
  char* promptStr = (char*)prompt(3, "yes\r\b\e", true);
  if (!promptStr || (strcmp(strupr(promptStr), "YES") != 0))
  {
    printf("\n");
    return;
  }  
  printf("\n\n");
  
  // nuke whole tracks
  const bool isRLL = hdd.isSeparatorRLL();
  hdd.setSeparatorRLL(false);
  std::vector<uint8_t> track;  
  track.resize(36*1024, 0);
  
  std::vector<uint32_t> dmaBuffer;
  std::vector<size_t> clockBits; // dummy
  endec.encodeMFM(track.data(), track.size(), clockBits, dmaBuffer);
  
  for (uint16_t cylinder = 0; cylinder < hdd.getParams()->Cylinders; cylinder++)
  { 
    printf(str_ProcessingCyl, cylinder);
    
    // for all heads    
    for (uint8_t head = 0; head < hdd.getParams()->Heads; head++)
    {  
      hdd.seekDrive(cylinder, head);      
      endec.prepareWriteDMA(dmaBuffer.data(), dmaBuffer.size());
      endec.writeWholeTrack();
      
      // can only fail with timeout, drive not ready or write fault
      if (hdd.getLastResult() != HDD_STATUS_OK)
      {
        printf("\n"); printf(hdd.getLastResultMessage()); printf("\n");
        hdd.setSeparatorRLL(isRLL);
        return;
      }
    }    
  }
   
  printf(str_EraseComplete);
  hdd.setSeparatorRLL(isRLL);
}

// launched from the main menu
void commandSeekTest()
{
  printf(str_EscGoBack);
  if (hdd.getParams()->SlowSeek)
  {
    printf(str_SeektestLegacy);
  }
  
  // start and end cylinder
  uint16_t startCylinder = 0;  
  uint16_t endCylinder = hdd.getParams()->Cylinders-1;
    
  // test repetitions for back and forth moves, butterfly tests and random seeks
  const char input[] = ": ";
  const char ellipsis[] = "...";
  printf(str_SeektestRepeats);
  
  uint16_t backForthTests = 0;
  printf(str_SeektestBackForth);
  printf(input);
  const char* promptStr = prompt(4, str_DecimalInputEsc, true);
  if (!promptStr)
  {
    printf("\n");
    return;
  }
  backForthTests = (uint16_t)atoi(promptStr);
  if (!backForthTests && !strlen(getPromptBuffer())) printf("0");
  printf("\n");
  
  // full butterfly tests offered on buffered seek only
  uint16_t butterflyTests = 0;
  if (!hdd.getParams()->SlowSeek)
  {
    printf(str_SeektestButterfly);
    printf(input);
    promptStr = prompt(4, str_DecimalInputEsc, true);
    if (!promptStr)
    {
      printf("\n");
      return;
    }
    butterflyTests = (uint16_t)atoi(promptStr);
    if (!butterflyTests && !strlen(getPromptBuffer())) printf("0");
    printf("\n");
  }
  
  uint16_t randomTests = 0;
  printf(str_SeektestRandom);
  printf(input);
  promptStr = prompt(4, str_DecimalInputEsc, true);
  if (!promptStr)
  {
    printf("\n");
    return;
  }
  randomTests = (uint16_t)atoi(promptStr);
  if (!randomTests && !strlen(getPromptBuffer())) printf("0");
  printf("\n");
  
  // nothing selected
  if (!backForthTests && !butterflyTests && !randomTests)
  {
    return;
  }
  
  uint16_t count;
  printf(str_SeektestProgress, startCylinder, endCylinder);
  
  // back and forth tests (seekDrive on error halts execution)
  if (backForthTests)
  {
    printf(str_SeektestBackForth);
    printf(ellipsis);
    
    for (count = 0; count < backForthTests; count++)
    {
      hdd.seekDrive(endCylinder, 0);
      hdd.seekDrive(startCylinder, 0);
    }
    
    printf("\n");
  }
  
  // butterfly tests
  if (butterflyTests)
  {
    printf(str_SeektestButterfly);
    printf(ellipsis);
    
    for (count = 0; count < butterflyTests; count++)
    {
      uint16_t start = startCylinder;
      uint16_t end = endCylinder;
      uint16_t count2 = end-start + 1;
      
      while (count2--)
      {
        hdd.seekDrive(start++, 0);
        hdd.seekDrive(end--, 0);        
      }
    }
    
    printf("\n");
  }
  
  // random
  if (randomTests)
  {
    printf(str_SeektestRandom);
    printf(ellipsis);
    
    for (count = 0; count < randomTests; count++)
    {
      hdd.seekDrive(rand() % (endCylinder - startCylinder + 1) + startCylinder, 0);
    }
    
    printf("\n");
  }
}

// launched from the main menu
void commandPark()
{
  printf("\n");
  printf(str_OperationPending);
  
  // switch to slow seek (some drives ignore seeking past number of usable cylinders in buffered seek mode)
  const bool slowSeek = hdd.getParams()->SlowSeek;
  hdd.getParams()->SlowSeek = true;
  hdd.seekDrive(hdd.getParams()->LandingZone, 0);
  hdd.getParams()->SlowSeek = slowSeek;
  hdd.selectDrive(false);
  
  printf(str_ParkSuccess, hdd.getParams()->LandingZone);
  printf(str_ParkPowerdownSafe);
  printf(str_ParkContinue);
  readKey(NULL);

  // use the slow recalibrate command to seek back to 0, as the landing zone might be sometimes outside the cylinders count
  printf(str_ParkRecalibrating);
  hdd.recalibrate();
  printf(str_DeleteLine);
}
     
// launched from the format menu
void commandAnalyze()
{
  printf(str_EscGoBack);
  
  // start and end cylinder
  uint16_t startCylinder = 0;
  if (hdd.getParams()->Cylinders > 1)
  {
    while(true)
    {
      printf(str_ChooseStartCyl, 0, hdd.getParams()->Cylinders-1);
      const char* promptStr = prompt(4, str_DecimalInputEsc, true);
      if (!promptStr)
      {
        printf("\n");
        return;
      }
      startCylinder = (uint16_t)atoi(promptStr);
      if (startCylinder < hdd.getParams()->Cylinders)
      {  
        if (!startCylinder && !strlen(getPromptBuffer())) printf("0");
        printf("\n");
        break;
      }
      
      printf(str_DeleteLine);
    }
  }  
  
  // allow to customize end cylinder, do not ask for it if start is the last one
  uint16_t endCylinder = hdd.getParams()->Cylinders-1;
  if (startCylinder != endCylinder)
  {
    while(true)
    {
      printf(str_ChooseEndCyl, startCylinder, hdd.getParams()->Cylinders-1);
      const char* promptStr = prompt(4, str_DecimalInputEsc, true);
      if (!promptStr)
      {
        printf("\n");
        return;
      }
      endCylinder = (uint16_t)atoi(promptStr);
      if ((endCylinder >= startCylinder) && (endCylinder < hdd.getParams()->Cylinders))
      {  
        if (!endCylinder && !strlen(getPromptBuffer())) printf("0");
        printf("\n");
        break;
      }
      
      printf(str_DeleteLine);
    }  
  }
   
  bool diskNotEmpty = false;
  
  // warnings
  bool headMismatch = false;
  bool cylinderMismatch = false;  
  bool variableSectorSize = false;  
  
  for (uint16_t cylinder = startCylinder; cylinder <= endCylinder; cylinder++)
  {
    for (uint8_t head = 0; head < hdd.getParams()->Heads; head++)    
    {   
      hdd.seekDrive(cylinder, head);  
      hdd.microStep(true);
      
      bool thisVariableSectorSize = false; // flag to show "variable bytes" in the following status message
      bool thisHeadMismatch = false;       // shown with "@" at the end of line
      bool thisCylinderMismatch = false;   // "*"
      
       // actual details are unused here, only for display
      uint8_t sectorsPerTrack;
      uint8_t startSector;
      uint16_t sectorSizeBytes;
      uint8_t interleave;
      if (!llf->analyzeTrack(MAX_SPT_LIMIT, true, sectorsPerTrack, startSector, sectorSizeBytes, interleave))
      {
        if (hdd.getLastResult() == HDD_STATUS_NO_SECTOR_ID)
        {
          continue;
        }
        
        // timeout, not ready, write fault
        printf("\n");
        return;
      }
      
      uint16_t dummy;
      uint8_t dummy2;
      if (wd)
      {
        wd->getCustomAnalyzeTrackResults(thisCylinderMismatch, thisHeadMismatch, thisVariableSectorSize, dummy, dummy2);
      }
      else if (xebecAdaptec)
      {
        xebecAdaptec->getCustomAnalyzeTrackResults(thisCylinderMismatch, thisHeadMismatch, dummy, dummy2);
      }
      else if (sm1040)
      {
        sm1040->getCustomAnalyzeTrackResults(thisCylinderMismatch, thisHeadMismatch, dummy, dummy2, dummy); 
      }
      else if (omti)
      {
        omti->getCustomAnalyzeTrackResults(thisCylinderMismatch, thisHeadMismatch, dummy, dummy2); 
      }
      else if (hdc9224)
      {
        hdc9224->getCustomAnalyzeTrackResults(thisCylinderMismatch, thisHeadMismatch, dummy, dummy2); 
      }

      diskNotEmpty |= true;
      
      // at least on one track ?
      variableSectorSize |= thisVariableSectorSize; 
      headMismatch |= thisHeadMismatch;
      cylinderMismatch |= thisCylinderMismatch;
    }    
  }

  printf(diskNotEmpty ? "\n\n" : "\n");
  
  // print out analysis warnings
  if (cylinderMismatch || headMismatch || variableSectorSize)
  {
    printf(str_AnalyzeWarning);
    
    if (cylinderMismatch)
    {
      printf(str_AnalyzeCylMismatch);
    }
    if (headMismatch)
    {
      printf(str_AnalyzeHdMismatch);
    }
    if (variableSectorSize)
    {
      printf(str_AnalyzeVarSsize);
    }
    
    printf("\n");
  }
  
  // normal status
  if (diskNotEmpty)
  {
    if (!cylinderMismatch && !headMismatch)
    {
      printf(str_AnalyzeCylHdNormal);
    }
    
    if (wd)
    {
      if (!variableSectorSize)
      {
        printf(str_AnalyzeConstSsize);
      }  
    }    
  }
}

// launched from the format menu
void commandHexdump()
{
  printf(str_HexdumpNote);
  printf(str_EscGoBack);
  printf("\n");
  
  // CHS
  uint16_t cylinder = 0;
  uint8_t head = 0;
  uint8_t sector = 0;
  
  if (hdd.getParams()->Cylinders > 1)
  {
    while(true)
    {
      printf(str_ChooseCylinder, 0, hdd.getParams()->Cylinders-1);
      const char* promptStr = prompt(4, str_DecimalInputEsc, true);
      if (!promptStr)
      {
        printf("\n");
        return;
      }
      cylinder = (uint16_t)atoi(promptStr);
      if (cylinder < hdd.getParams()->Cylinders)
      {  
        if (!cylinder && !strlen(getPromptBuffer())) printf("0");
        printf("\n");
        break;
      }
      
      printf(str_DeleteLine);
    }  
  }  
  
  if (hdd.getParams()->Heads > 1)
  {
    while(true)
    {
      printf(str_ChooseHead, 0, hdd.getParams()->Heads-1);
      const char* promptStr = prompt(2, str_DecimalInputEsc, true);
      if (!promptStr)
      {
        printf("\n");
        return;
      }
      head = (uint16_t)atoi(promptStr);
      if (head < hdd.getParams()->Heads)
      {  
        if (!head && !strlen(getPromptBuffer())) printf("0");
        printf("\n");
        break;
      }
      
      printf(str_DeleteLine);
    }
  }  
  
  while(true)
  {
    printf(str_ChooseSector, 0, 255);
    const char* promptStr = prompt(3, str_DecimalInputEsc, true);
    if (!promptStr)
    {
      printf("\n");
      return;
    }
    uint16_t chooseSector = (uint16_t)atoi(promptStr);
    if (chooseSector <= 255)
    {  
      sector = (uint8_t)chooseSector;
      if (!sector && !strlen(getPromptBuffer())) printf("0");
      printf("\n");      
      break;
    }
    
    printf(str_DeleteLine);
  }
  
  // sector size
  uint16_t sectorSizeBytes = 512;
  
  if (wd)
  {
    printf(str_ChooseSecSize);
    uint8_t key = toupper(readKey("125K\e"));
    switch(key)
    {
      case '1':
        sectorSizeBytes = 128;
        break;
      case '2':
        sectorSizeBytes = 256;
        break;
      case '5':
        sectorSizeBytes = 512;
        break;
      case 'K':
        sectorSizeBytes = 1024;
        break;
      case '\e':
        printf("\n");
        return;
    }
    
    wd->setWorkingSectorSizeBytes(sectorSizeBytes);
    printf(str_EchoKey, key);
  } 
  
  hdd.seekDrive(cylinder, head);
  hdd.microStep(true);
  llf->readSector(sector);
  
  if (hdd.getLastResult() && (hdd.getLastResult() < HDD_STATUS_DATA_ERROR))
  {
    // timeout, drive not ready, writefault abort the command
    if (hdd.getLastResult() < HDD_STATUS_NO_SECTOR_ID)
    {  
      printf("\n");
      printf(hdd.getLastResultMessage());
      printf("\n");
      return;        
    }
    
    // prepend CHS information
    printf(str_CHSInfo, cylinder, head, sector);
    printf(hdd.getLastResultMessage());
    printf("\n");
  }
  
  // print the sector dump if no error, but also if there was a CRC/ECC error
  else
  {
    printf("\n");
    printf(str_HexdumpDump);
    
    for (uint16_t index = 0; index < sectorSizeBytes; index++)
    {
      printf("%02X ", llf->getSectorBuffer()[index]);
    }
    printf("\n");

    printf(str_CHSInfo, cylinder, head, sector);
    if (hdd.getLastResult() == HDD_STATUS_OK)
    {
      printf(str_HexdumpOk);  
    }
    else
    {
      printf(hdd.getLastResultMessage());
    }
    printf("\n");
  }
}

// launched from the format menu: verify / read into image
void commandRead(bool verifyOnly)
{
  if (!verifyOnly && !sdDetect())
  {
    return;
  }
  
  printf(str_EscGoBack);
  
  // start and end cylinder (SM1040: always whole disk)
  uint16_t startCylinder = 0;
  uint16_t endCylinder = hdd.getParams()->Cylinders-1;
  char key;
  
  if (!sm1040)
  {
    // whole disk?
    if (hdd.getParams()->Cylinders > 1)
    {
      printf(str_ReadWholeDisk);
      key = toupper(readKey("YN\e"));        
      if (key == '\e') // ESC key returns to main menu
      {
        printf("\n");
        return;
      }
      
      printf(str_EchoKey, key);
      if (key == 'N')
      {
        while(true)
        {
          printf(str_ChooseStartCyl, 0, hdd.getParams()->Cylinders-1);
          const char* promptStr = prompt(4, str_DecimalInputEsc, true);
          if (!promptStr)
          {
            printf("\n");
            return;
          }
          startCylinder = (uint16_t)atoi(promptStr);
          if (startCylinder < hdd.getParams()->Cylinders)
          {  
            if (!startCylinder && !strlen(getPromptBuffer())) printf("0");
            printf("\n");
            break;
          }
          
          printf(str_DeleteLine);
        }
        
        // allow to customize end cylinder, do not ask for it if start is the last one
        if (startCylinder != endCylinder)
        {
          while(true)
          {
            printf(str_ChooseEndCyl, startCylinder, hdd.getParams()->Cylinders-1);
            const char* promptStr = prompt(4, str_DecimalInputEsc, true);
            if (!promptStr)
            {
              printf("\n");
              return;
            }
            endCylinder = (uint16_t)atoi(promptStr);
            if ((endCylinder >= startCylinder) && (endCylinder < hdd.getParams()->Cylinders))
            {  
              if (!endCylinder && !strlen(getPromptBuffer())) printf("0");
              printf("\n");
              break;
            }
            
            printf(str_DeleteLine);
          }  
        }
      }
    }
  }
  
  // analyze first track of chosen bounds
  printf(str_ReadAnalyze);
  printf((startCylinder == 0) ? str_ReadAnalyzeTrack0 : str_ReadAnalyzeFirstTrack);
  hdd.seekDrive(startCylinder, 0);
  hdd.microStep(true);
  
  uint8_t sectorsPerTrack;
  uint8_t startSector;
  uint16_t sectorSizeBytes;
  uint8_t interleave;
  if (!llf->analyzeTrack(MAX_SPT_LIMIT, true, sectorsPerTrack, startSector, sectorSizeBytes, interleave))
  {
    // timeout, not ready, write fault, no valid sectors read
    printf("\n");
    return;
  }
  
  hdd.selectDrive(false);
  
  // ask for expected sectors per track
  uint8_t expectedSectorsPerTrack = 0;
  uint8_t expectedSptDefault = 0;
  printf(str_ReadExpectedSpt1);
  if (!verifyOnly)
  {
    printf(str_ReadExpectedSpt2);
  }
  printf(str_ReadExpectedSpt3);
  if (hdd.isSeparatorRLL() && (sectorSizeBytes == 512))
  {
    expectedSptDefault = 26;
  }
  else if (!hdd.isSeparatorRLL())
  {
    if (sectorSizeBytes == 512)
    {
      expectedSptDefault = 17;
    }
    else if (sectorSizeBytes == 256)
    {
      expectedSptDefault = 32;
    }
    else if (sectorSizeBytes == 1024)
    {
      expectedSptDefault = 8;
    }
  }
  while(true)
  {
    if (expectedSptDefault)
    {
      printf(str_ChooseExpectedSptDef, 0, MAX_SPT_LIMIT, expectedSptDefault);
    }
    else
    {
      printf(str_ChooseExpectedSpt, 0, MAX_SPT_LIMIT);
    }
    const char* promptStr = prompt(2, str_DecimalInputEsc, true);
    if (!promptStr)
    {
      printf("\n");
      return;
    }
    expectedSectorsPerTrack = (uint8_t)atoi(promptStr);
    if (expectedSectorsPerTrack <= MAX_SPT_LIMIT)
    {  
      if (!expectedSectorsPerTrack && !strlen(getPromptBuffer())) printf("0");
      printf("\n");
      break;
    }    
    printf(str_DeleteLine);
  }
  
  // file picker
  if (!verifyOnly)
  {
    if (!sdFilePicker(true))
    {
      return;
    }
    
    printf(str_ReadSavingNoInterleave);
  }
  printf("\n");
  
  // prepare buffer
  std::vector<uint8_t> trackBuffer;
  if (!verifyOnly)
  {
    trackBuffer.resize(1024 * (expectedSectorsPerTrack ? expectedSectorsPerTrack : MAX_SPT_LIMIT)); // worst case
  }  
  
  // SM1040 specific
  uint8_t sm1040LogDriveCounter = 1;
  uint16_t sm1040LogDriveCylCounter = 0;
  uint16_t sm1040LogDriveCylCount = 0;
  
  // read
  uint32_t unreadableTracks = 0;
  uint32_t sectorErrors = 0;
  uint16_t lastValidSectorSize = sectorSizeBytes;
  bool trackError = false;
    
  for (uint16_t cylinder = startCylinder; cylinder <= endCylinder; cylinder++)
  { 
    printf(str_ProcessingCyl, cylinder);
    trackError = false;
       
    for (uint8_t head = 0; head < hdd.getParams()->Heads; head++)
    {
      if (!verifyOnly)
      {
        memset(trackBuffer.data(), 0, trackBuffer.size()); // zero-pad
      }
      
      hdd.seekDrive(cylinder, head);
_afterSeek:
      hdd.microStep(true);
      
      if (!llf->analyzeTrack(MAX_SPT_LIMIT, false, sectorsPerTrack, startSector, sectorSizeBytes, interleave))
      {
        if (hdd.getLastResult() == HDD_STATUS_NO_SECTOR_ID)
        {
          // SM1040: check for relocation, and track is within drive bounds
          if (sm1040 && sm1040->isTrackRelocated())
          {
            uint16_t relocationCyl;
            uint8_t relocationHd;
            sm1040->getRelocation(relocationCyl, relocationHd);
            
            if ((relocationCyl < hdd.getParams()->Cylinders) && (relocationHd < hdd.getParams()->Heads))
            {
              hdd.seekDrive(relocationCyl, relocationHd);
              goto _afterSeek;
            }
          }
          
          trackError = true;
          unreadableTracks++;
          printf(str_CHInfo, hdd.getPhysicalCylinder(), hdd.getPhysicalHead());
          printf(str_AnalyzeNoSectors);

          // flush empty track buffer to card, if the "expected sectors per track" was given, otherwise skip
          if (!verifyOnly && expectedSectorsPerTrack)
          {
            const size_t bytesToWrite = lastValidSectorSize * expectedSectorsPerTrack;
            size_t bytesOkay = 0;
            if (!sdWriteFile(trackBuffer.data(), bytesToWrite, &bytesOkay))
            {
              printf("\n");
              printf(str_SdErrorFS);
              sdCloseFile();
              return;  
            }
            
            if (bytesOkay != bytesToWrite)
            {
              printf("\n");
              printf(str_SdErrorFull);
              printf("\n");
              sdCloseFile();
              return;
            }  
          }
          
          continue;
        }
        
        // timeout, not ready, write fault
        printf("\n");
        return;
      }
      if (sectorSizeBytes)
      {
        lastValidSectorSize = sectorSizeBytes;  
        
        // set it
        if (wd)
        {
          wd->setWorkingSectorSizeBytes(sectorSizeBytes);  
        } 
      }
      
      // check if ID field cylinder or head number differs from the physical cyl/head
      bool dummy;
      uint16_t logicalCylinder = hdd.getPhysicalCylinder();
      uint8_t logicalHead = hdd.getPhysicalHead();
      if (wd)
      {
        wd->getCustomAnalyzeTrackResults(dummy, dummy, dummy, logicalCylinder, logicalHead);
      }
      else if (xebecAdaptec)
      {
        xebecAdaptec->getCustomAnalyzeTrackResults(dummy, dummy, logicalCylinder, logicalHead);
      }
      else if (omti)
      {
        omti->getCustomAnalyzeTrackResults(dummy, dummy, logicalCylinder, logicalHead);
      }
      else if (sm1040)
      {
        sm1040->getCustomAnalyzeTrackResults(dummy, dummy, logicalCylinder, logicalHead, sm1040LogDriveCylCount);
      }
      else if (hdc9224)
      {
        hdc9224->getCustomAnalyzeTrackResults(dummy, dummy, logicalCylinder, logicalHead);
      }

      // prepare a sectors table of the original format interleave for the fastest track read
      // if returned interleave == 0: unknown, or missing sectors on track - fallback to 1:1
      std::vector<uint8_t> sectors;
      if ((interleave == 0) || xebecAdaptec /* performs its own internal skew */)
      {
        interleave = 1;
      }
      if (expectedSectorsPerTrack) // override detected
      {
        sectorsPerTrack = expectedSectorsPerTrack;
      }       
      LLF::getInterleaveTable(sectorsPerTrack, startSector, interleave, sectors);      
      
      // read track with original interleave
      for (const uint8_t& sector : sectors)
      {
        llf->readSector(sector, &logicalCylinder, &logicalHead);
        const uint8_t result = hdd.getLastResult();

        if (result != HDD_STATUS_OK)
        {
          trackError = true;
          
          if (result < HDD_STATUS_NO_SECTOR_ID)
          {
            // timeout, not ready, write fault
            printf(str_CHInfo, hdd.getPhysicalCylinder(), hdd.getPhysicalHead());
            printf(hdd.getLastResultMessage());
            printf("\n");
            if (!verifyOnly)
            {
              sdCloseFile();
            }
            return;
          }
          
          sectorErrors++;
          printf(str_CHSInfo, hdd.getPhysicalCylinder(), hdd.getPhysicalHead(), sector);          
          printf(hdd.getLastResultMessage());
        }
        
        if (!verifyOnly)
        {
          uint8_t* target = &trackBuffer[(sector-startSector)*sectorSizeBytes];
          
          // copy sector buffer to track buffer on success, CRC error or corrected data, otherwise keep zeros in track buffer
          if ((result != HDD_STATUS_NO_SECTOR_ID) && (result != HDD_STATUS_NO_DATA_ID))
          {
            memcpy(target, llf->getSectorBuffer(), sectorSizeBytes);
          }
        }
      }
      
      // write trackBuffer to card 
      if (!verifyOnly)
      {
        const size_t bytesToWrite = sectorSizeBytes * sectorsPerTrack;
        size_t bytesOkay = 0;
        if (!sdWriteFile(trackBuffer.data(), bytesToWrite, &bytesOkay))
        {
          printf("\n");
          printf(str_SdErrorFS);
          sdCloseFile();
          return;  
        }
        
        if (bytesOkay != bytesToWrite)
        {
          printf("\n");
          printf(str_SdErrorFull);
          printf("\n");
          sdCloseFile();
          return;
        }
      }
    }
    
    if (trackError)
    {
      printf("\n");
    }
    
    // SM1040: inform about logical drives
    if (sm1040)
    {
      char rkType[] = "RK07";
      if (sm1040->isDriveTypeRK06())
      {
        rkType[3] = '6';
      }
      
      sm1040LogDriveCylCounter++;
      if (sm1040LogDriveCylCounter == sm1040LogDriveCylCount)
      {
        if (!trackError)
        {
          printf("\n");
        }        
        sm1040LogDriveCylCounter = 0;
        if (!verifyOnly)
        {
          printf(str_ReadSM1040LogicalDrive1, rkType, sm1040LogDriveCounter++, sdGetSeekPos());  
        }
        else
        {
          printf(str_ReadSM1040LogicalDrive2, rkType, sm1040LogDriveCounter++, hdd.getPhysicalCylinder(), hdd.getPhysicalHead());
        }        
        trackError = true; // just a newline flag here between tracks        
      }
    }
  }
  
  printf(trackError ? "\n" : "\n\n");
  printf(str_UnreadableTracks, unreadableTracks);
  printf(str_SectorErrors, sectorErrors);

  if (!verifyOnly)
  {
    sdCloseFile();
  }
}

// launched from the format menu
void commandWrite(bool formatOnly)
{  
  if (!formatOnly && !sdDetect())
  {
    return;
  }
  
  printf(str_EscGoBack);
  
  // start and end cylinder (SM1040: always whole disk)
  uint16_t startCylinder = 0;
  uint16_t endCylinder = hdd.getParams()->Cylinders-1;
  char key;  
  if (!sm1040)
  {
    // whole disk?
    if (hdd.getParams()->Cylinders > 1)
    {
      printf(str_WriteWholeDisk, formatOnly ? "Format" : "Write");
      key = toupper(readKey("YN\e"));        
      if (key == '\e')
      {
        printf("\n");
        return;
      }
      
      printf(str_EchoKey, key);
      if (key == 'N')
      {
        while(true)
        {
          printf(str_ChooseStartCyl, 0, hdd.getParams()->Cylinders-1);
          const char* promptStr = prompt(4, str_DecimalInputEsc, true);
          if (!promptStr)
          {
            printf("\n");
            return;
          }
          startCylinder = (uint16_t)atoi(promptStr);
          if (startCylinder < hdd.getParams()->Cylinders)
          {  
            if (!startCylinder && !strlen(getPromptBuffer())) printf("0");
            printf("\n");
            break;
          }
          
          printf(str_DeleteLine);
        }
        
        // allow to customize end cylinder, do not ask for it if start is the last one
        if (startCylinder != endCylinder)
        {
          while(true)
          {
            printf(str_ChooseEndCyl, startCylinder, hdd.getParams()->Cylinders-1);
            const char* promptStr = prompt(4, str_DecimalInputEsc, true);
            if (!promptStr)
            {
              printf("\n");
              return;
            }
            endCylinder = (uint16_t)atoi(promptStr);
            if ((endCylinder >= startCylinder) && (endCylinder < hdd.getParams()->Cylinders))
            {  
              if (!endCylinder && !strlen(getPromptBuffer())) printf("0");
              printf("\n");
              break;
            }
            
            printf(str_DeleteLine);
          }  
        }
      }
    }
  }  
  
  
  // enter format parameters
  printf(str_WriteParameters);
    
  uint16_t sectorSizeBytes = 512;
  if (!wd) // allow customizing sector size for WD
  {
    printf(str_SectorSizeBytes, sectorSizeBytes);
  }
  else
  {
    printf(str_ChooseSecSize);
    key = toupper(readKey("125K\e"));
    if (key == '\e')
    {
      printf("\n");
      return;
    }
    printf(str_EchoKey, key);
    
    switch(key)
    {
    case '1':
      sectorSizeBytes = 128;
      break;
    case '2':
      sectorSizeBytes = 256;
      break;
    case '5':
      sectorSizeBytes = 512;
      break;
    case 'K':
      sectorSizeBytes = 1024;
      break;
    }
    
    wd->setWorkingSectorSizeBytes(sectorSizeBytes);
  }
  
  // ask for sectors per track
  uint8_t sectorsPerTrack = 0;
  uint8_t sectorsPerTrackDefault = 0;
  uint8_t sectorsPerTrackMax = endec.getMaximumSectorCountFor(llf, sectorSizeBytes);
  if (hdd.isSeparatorRLL() && (sectorSizeBytes == 512))
  {
    sectorsPerTrackDefault = 26;
  }
  else if (!hdd.isSeparatorRLL())
  {
    if (sectorSizeBytes == 512)
    {
      sectorsPerTrackDefault = 17;
    }
    else if (sectorSizeBytes == 256)
    {
      sectorsPerTrackDefault = 32;
    }
    else if (sectorSizeBytes == 1024)
    {
      sectorsPerTrackDefault = 8;
    }
  }
  while(true)
  {
    if (sectorsPerTrackDefault)
    {
      printf(str_ChooseSptDef, 1, sectorsPerTrackMax, sectorsPerTrackDefault);
    }
    else
    {
      printf(str_ChooseSpt, 1, sectorsPerTrackMax);
    }
    const char* promptStr = prompt(2, str_DecimalInputEsc, true);
    if (!promptStr)
    {
      printf("\n");
      return;
    }
    sectorsPerTrack = (uint8_t)atoi(promptStr);
    if (sectorsPerTrack && (sectorsPerTrack <= sectorsPerTrackMax))
    {  
      printf("\n");
      break;
    }    
    printf(str_DeleteLine);
  }
  
  // starting sector
  uint8_t startSector = 0;
  uint8_t startSectorMax = sm1040 ? 64-sectorsPerTrack : 256-sectorsPerTrack; // 6 bits or 8 bits used for sector number
  while(true)
  {
    if (wd) // show default for XT and AT
    {
      printf(str_ChooseStartSectorXTAT, startSector, startSectorMax);
    }
    else // default: 0
    {
      printf(str_ChooseStartSectorDef, startSector, startSectorMax, 0);
    }
    const char* promptStr = prompt(3, str_DecimalInputEsc, true);
    if (!promptStr)
    {
      printf("\n");
      return;
    }
    const uint16_t start = (uint16_t)atoi(promptStr);
    if (start <= startSectorMax)
    {  
      startSector = start;
      if (!startSector && !strlen(getPromptBuffer())) printf("0");
      printf("\n");
      break;
    }    
    printf(str_DeleteLine);
  }
 
  
  // format interleave
  uint8_t interleave = 1;
  if (sectorsPerTrack > 2)
  {
    while(true)
    {
      printf(str_ChooseFormatInterleave, sectorsPerTrack-1);

      const char* promptStr = prompt(2, str_DecimalInputEsc, true);
      if (!promptStr)
      {
        printf("\n");
        return;
      }
      interleave = (uint8_t)atoi(promptStr);
      if (interleave && (interleave < sectorsPerTrack))
      {  
        printf("\n");
        break;
      }    
      printf(str_DeleteLine);
    }  
  }  
  
  // with verify?  
  printf(str_WriteVerify, formatOnly ? "format" : "write");
  key = toupper(readKey("YN\e"));
  if (key == '\e')
  {
    printf("\n");
    return;
  }  
  printf(str_EchoKey, key);
  bool withVerify = key == 'Y';
  
  // SM1040: ask for drive type RK07/RK06
  if (sm1040)
  {
    printf(str_WriteSMDriveType);
    key = toupper(readKey("67\e"));
    if (key == '\e')
    {
      printf("\n");
      return;
    }  
    printf(str_EchoKey, key);
    
    sm1040->setDriveTypeRK06(key == '6');
  }
    
  // file picker
  if (!formatOnly && !sdFilePicker(false))
  {
    return;
  }
  
  // inform about format/write
  if (formatOnly)
  {
    printf(str_WriteDetailsFormat, endCylinder-startCylinder+1);
  }
  else
  {
    size_t bytesFile = sdGetFileSize();
    size_t bytesDisk = (size_t)(endCylinder-startCylinder+1) * hdd.getParams()->Heads * sectorsPerTrack * sectorSizeBytes;
    printf(str_WriteDetails, endCylinder-startCylinder+1, (bytesFile > bytesDisk) ? bytesDisk : bytesFile);
  }  
  printf(str_ContinueAbort);
  key = readKey("\r\e");
  printf(str_DeleteLine);
  if (key == '\e')
  {
    if (!formatOnly)
    {
      sdCloseFile();
    }
    return;
  }  
  printf("\n");
  
  // prepare interleave table and track buffer
  std::vector<uint8_t> interleaveTable;
  LLF::getInterleaveTable(sectorsPerTrack, startSector, interleave, interleaveTable);
  
  std::vector<uint8_t> trackBuffer;
  if (!formatOnly)
  {
    trackBuffer.resize(sectorSizeBytes * sectorsPerTrack);  
  }
  
  // format/write + with verify
  uint32_t unreadableTracks = 0;
  uint32_t sectorErrors = 0;
  bool trackError = false;
  bool fileOpened = !formatOnly;
  bool endOfFile = false;
  
  for (uint16_t cylinder = startCylinder; cylinder <= endCylinder; cylinder++)
  { 
    printf(str_ProcessingCyl, cylinder);
    trackError = false;
    endOfFile = false;
       
    for (uint8_t head = 0; head < hdd.getParams()->Heads; head++)
    {  
      const size_t bytesToRead = sectorSizeBytes * sectorsPerTrack;
      size_t bytesOkay = 0;
      
      if (fileOpened)
      {
        memset(trackBuffer.data(), 0, trackBuffer.size()); // zero-pad
        if (!sdReadFile(trackBuffer.data(), bytesToRead, &bytesOkay))
        {
          printf("\n");
          printf(str_SdErrorFS);
          sdCloseFile();
          return;
        }
        
        // reached end-of-file?
        if ((bytesOkay != bytesToRead) || sdIsEndOfFile())
        {
          printf(" ");
          printf(str_SdErrorEndOfFile);
        }
      }
      
      hdd.seekDrive(cylinder, head); // write - no microstepping here
      if (!llf->formatWriteTrack(interleaveTable, fileOpened ? trackBuffer.data() : NULL))
      {
        // can only fail on timeout, not ready or write fault
        printf(str_CHInfo, cylinder, head);
        printf(hdd.getLastResultMessage());
        printf("\n");
        if (fileOpened)
        {
          sdCloseFile();  
        }        
        return;
      }
            
      // flag end of file and close it; will format following tracks only
      if (fileOpened && ((bytesOkay != bytesToRead) || sdIsEndOfFile()))
      {
        endOfFile = true;
        fileOpened = false;
        sdCloseFile();
      }
      
      if (withVerify)
      {
        // read track with original interleave
        for (const uint8_t& sector : interleaveTable)
        {
          llf->readSector(sector);
          const uint8_t result = hdd.getLastResult();

          if (result != HDD_STATUS_OK)
          {
            trackError = true;
            
            if (result < HDD_STATUS_NO_SECTOR_ID)
            {
              // timeout, not ready, write fault
              printf(str_CHInfo, cylinder, head);
              printf(hdd.getLastResultMessage());
              printf("\n");
              if (fileOpened)
              {
                sdCloseFile();  
              } 
              return;
            }
            
            sectorErrors++;
            printf(str_CHSInfo, cylinder, head, sector);
            printf(hdd.getLastResultMessage());
          }
        }
      }
    }
    
    if (trackError || endOfFile)
    {
      printf("\n");
    }
  }
  
  printf(trackError || endOfFile || !withVerify ? "\n" : "\n\n");
  if (withVerify)
  {
    printf(str_UnreadableTracks, unreadableTracks);
    printf(str_SectorErrors, sectorErrors);
  }
  
  if (fileOpened)
  {
    sdCloseFile();
  }
}

void commandMicrostep()
{
  printf(str_MicrostepDescription);
  
  // too bad, we're out of pins here that can sink 48 mA :)
  if (hdd.getParams()->UseReduceWriteCurrent)
  {
    printf(str_MicrostepRWCInUse);
    return;
  }
  if (hdd.getParams()->Heads > 8)
  {
    printf(str_MicrostepTooManyHeads, hdd.getParams()->Heads);
    return;
  }
  
  printf(str_EscGoBack);
  uint8_t microsteps = 0;
  const uint8_t microstepsMax = 8; // an ST225 has eight algorithms maximum
  while(true)
  {
    printf(str_MicrostepCount, microstepsMax);
    const char* promptStr = prompt(1, str_DecimalInputEsc, true);
    if (!promptStr)
    {
      printf("\n");
      return;
    }
    microsteps = (uint8_t)atoi(promptStr);
    if (microsteps <= microstepsMax)
    {  
      if (!microsteps && !strlen(getPromptBuffer())) printf("0");
      printf("\n");
      break;
    }
    
    printf(str_RawdiskCustomTrackOver);
  }
  
  if (microsteps)
  {
    hdd.setMicrostepping(microsteps);
    
    // test recovery mode if it works; stops on error if it doesn't
    printf(str_MicrostepTesting);
    hdd.testMicrostepping();
       
    printf(" ");
    printf(str_OK);
    printf("\n");
    
    // turn off reseeking if enabled
    if (hdd.getParams()->ReseekOnSectorErrors)
    {
      hdd.getParams()->ReseekOnSectorErrors = false;
      printf(str_MicrostepReseekOff);
    }
  }
  else
  {
    // turn off
    if (hdd.getMicrostepping())
    {
      hdd.microStep(false);
    }
    
    hdd.setMicrostepping(0);
  }
}