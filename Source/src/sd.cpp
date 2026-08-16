// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// SD wrapper

#include "config.h"

// library: no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
#include "hw_config.h"
#include "f_util.h"
#include "ff.h"  

// library FATFS from no-OS-FatFS-SD-SDIO-SPI-RPi-Pico: reused for both SD card and hard drive access
// defined in glue.c
extern bool diskio_use_sd;
FATFS sdFat = {0};
FIL sdFile = {0};
char sdPath[MAX_PATH+1] = {0};

// SD/SPI config
spi_t spi
{
  .hw_inst = spi0,
  .miso_gpio = 16,
  .mosi_gpio = 19,
  .sck_gpio = 18,  
  .baud_rate = 10416666
};

sd_spi_if_t spiInterface
{
  .spi = &spi,
  .ss_gpio = 17 // SCS
};

sd_card_t sdCard
{
  .type = SD_IF_SPI,
  .spi_if_p = &spiInterface
};

size_t sd_get_num()
{
  return 1;
}

sd_card_t* sd_get_by_num(size_t num)
{
  return (num == 0) ? &sdCard : NULL;
}

bool sdDetect()
{
  // unmount if already mounted; reinit and attempt mount
  diskio_use_sd = true;
  f_close(&sdFile);
  f_unmount("");
  sd_init_driver();
  
  FRESULT result = f_mount(&sdFat, "", 1);
  if (result != FR_OK)
  {
    if ((result == FR_DISK_ERR) || (result == FR_NOT_READY))
    {
      printf("\n");
      printf(str_SdNotPresent);
    }
    else
    {
      printf("\n");
      printf(str_SdErrorFS);
    }
    
    return false;
  }
  
  f_unmount("");
  return true;
}

bool sdFilePicker(bool writeOperation, const char* extension /* = NULL */) // dot and 3 chars
{ 
  if (!sdDetect())
  {
    return false;
  }
  
  // try to mount root directory
  DIR dir;
  f_mount(&sdFat, "", 1);
  strcpy(sdPath, "/");
  if (f_opendir(&dir, sdPath) != FR_OK)
  {
    printf(str_SdErrorFS); printf("\n");
    return false;
  }
  
  bool noFiles = true;
  printf(str_SdDirListing, (extension && (strlen(extension) > 0)) ? extension : ".*");  
  
  while(true)
  {
    FILINFO info = {0};
    FRESULT result = f_readdir(&dir, &info);
    if ((result != FR_OK) || (info.fname[0] == 0))
    {
      break;
    }
    if (info.fattrib & AM_HID)
    {
      continue;
    }
    
    // filter based on extension
    const bool isDirectory = info.fattrib & AM_DIR;  
    if (!isDirectory && extension && (strlen(extension) > 0))
    {
      const char* ext = strcasestr(info.fname, extension);
      if (!ext)
      {
        continue;
      }
      
      // also ending with it?
      const char extPos = ext-info.fname;
      if (extPos != strlen(info.fname)-4)
      {
        continue;
      }
    }
    
    // do directory listing
    noFiles = false;
    printf(isDirectory ? "<DIR> " : "      ");
    printf("%s\n", info.fname);
  }
  f_closedir(&dir);
  
  if (noFiles)
  {
    printf(str_DosDirectoryEmpty);
    printf("\n");
    
    if (!writeOperation)
    {
      printf(str_Continue);
      readKey("\r");  
      printf("\n");
      return false;  
    }    
  }
  
  while(true)
  {
    strcpy(sdPath, "/");
    printf(writeOperation ? str_SdDirPickWrite : str_SdDirPickRead, 
           (extension && (strlen(extension) > 0)) ? extension : "\b");
    const char* promptBuffer = prompt(MAX_PATH-1, NULL, true); // -1 to account for initial '/' in sdPath buffer
    if (!promptBuffer) // cancelled
    {
      printf("\n");
      return false;
    }
    else if (strlen(promptBuffer) == 0)
    {
      continue;
    }
    
    uint16_t idx = 0;
    bool emptyString = true;
    while (idx < strlen(promptBuffer))
    {
      if (!isspace(promptBuffer[idx++]))
      {
        emptyString = false;
        break;
      }
    }
    if (emptyString)
    {
      continue;
    }    
    printf("\n");
    
    strcat(sdPath, promptBuffer);    
    if (extension && (strlen(extension) > 0))
    {
      // append extension if not specified
      bool appendExt = false;      
      const char* ext = strcasestr(sdPath, extension);
      if (!ext)
      {
        appendExt = true;
      }
      else
      {
        const char extPos = ext-&sdPath[0];
        if (extPos != strlen(sdPath)-4)
        {
          appendExt = true;
        }
      }  
      if (appendExt)
      {
        if (strlen(sdPath) > MAX_PATH-4)
        {
          sdPath[MAX_PATH-4] = 0; // make space :)
        }
        strcat(sdPath, extension);
      }
    }    
    
    // check if file exists; if writing, ask to overwrite
    if (!sdDetect())
    {
      return false;
    }
    f_mount(&sdFat, "", 1); // remount    
    
    FRESULT result = f_open(&sdFile, sdPath, FA_READ);
    bool fileExists = false;
    if (result == FR_OK)
    {
      fileExists = true;
      f_close(&sdFile);
    }
    if (!writeOperation && !fileExists)
    {
      printf(str_SdDirInvalid);
      continue;
    }
    else if (writeOperation && fileExists)
    {
      printf(str_SdDirOverwrite);
      char key = toupper(readKey("YN\e"));
      if (key == '\e')
      {
        printf("\n");
        f_unmount("");
        return false;
      }
      
      printf(str_EchoKey, key);
      if (key == 'N')
      {
        continue;
      }
    }
    
    result = f_open(&sdFile, sdPath, writeOperation ? FA_CREATE_ALWAYS | FA_WRITE : FA_READ);
    if (result != FR_OK)
    {
      printf("\n"); printf(str_SdFileError); printf("\n");
      printf(str_Continue);
      readKey("\r");  
      printf("\n");
      return false;
    }
    
    return true;
  }
}

bool sdSeekFile(size_t offset)
{
  return f_lseek(&sdFile, offset) == FR_OK;
}

void sdCloseFile()
{
  f_close(&sdFile);
}

bool sdReadFile(void* buffer, size_t bytesCount, size_t* bytesSuccessful)
{
  return f_read(&sdFile, buffer, bytesCount, bytesSuccessful) == FR_OK;
}

bool sdWriteFile(const void* buffer, size_t bytesCount, size_t* bytesSuccessful)
{
  // and flush the buffer
  FRESULT result = f_write(&sdFile, buffer, bytesCount, bytesSuccessful);
  if (result == FR_OK)
  {
    f_sync(&sdFile);
  }
  
  return result == FR_OK;
}

size_t sdGetSeekPos()
{
  return (size_t)f_tell(&sdFile);
}

bool sdIsEndOfFile()
{
  return f_eof(&sdFile);
}

size_t sdGetFileSize()
{
  return (size_t)f_size(&sdFile);
}