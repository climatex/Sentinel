// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// DOS filesystem viewer

#include "config.h"

// FATFS of no-OS-FatFS-SD-SDIO-SPI-RPi-Pico (glue.c) reused to handle the connected hard drive
#include "diskio.h"

// verify FAT operation macros
#define FAT_EXECUTE(fn)       if (dosResult((fn)) != FR_OK) return;
#define FAT_EXECUTE_0(fn)     if (dosResult((fn)) != FR_OK) return 0;
#define FAT_EXECUTE_DIR(fn)   if (dosResult((fn)) != FR_OK) { f_closedir(&dir); return; }
#define FAT_EXECUTE_FILE(fn)  if (dosResult((fn)) != FR_OK) { f_close(&file); return; }

FATFS fat = {0};
FIL file = {0};
DIR dir = {0};
LLF* fmt = NULL;

// path buffers; current and with filename
char path[MAX_PATH+1] = {0};
char addPath[MAX_PATH+1] = {0};

uint8_t startSector = 0;      // 0: XT, 1: AT - but not always, this is computed
uint8_t sectorsPerTrack = 0;  // uniform for all
uint16_t sectorSizeBytes = 0; // ditto + shall be max. 512 bytes
char* fsErrorMessage = 0;     // Progmem index

extern "C"
{
  
// defined in glue.c
extern bool diskio_use_sd;

unsigned int dosDiskRead(uint8_t pdrv, uint8_t* buff, uint64_t sec, unsigned int count)
{
  if ((count != 1) || (sec > dosGetTotalSectorCount()))
  {
    return RES_PARERR; 
  }
  
  uint16_t cyl;
  uint8_t head;
  uint8_t sector;
  dosConvertLogicalSectorToCHS(sec, cyl, head, sector);
  
  hdd.seekDrive(cyl, head);
  hdd.microStep(true); // reading, do microstep if configured
  fmt->readSector(sector);
  
  // allow ECC
  if (hdd.getLastResult() && (hdd.getLastResult() != HDD_STATUS_DATA_CORRECTED))
  {
    return RES_ERROR;
  }
  
  memcpy(buff, fmt->getSectorBuffer(), dosGetSectorSize());
  return RES_OK;
}

unsigned int dosDiskWrite(uint8_t pdrv, uint8_t* buff, uint64_t sec, unsigned int count)
{
  if ((count != 1) || (sec > dosGetTotalSectorCount()))
  {
    return RES_PARERR;
  }
  
  uint16_t cyl;
  uint8_t head;
  uint8_t sector;
  dosConvertLogicalSectorToCHS(sec, cyl, head, sector);
  
  memcpy(fmt->getSectorBuffer(), buff, dosGetSectorSize());
  
  hdd.seekDrive(cyl, head);
  fmt->writeSector(sector);
    
  // write
  if (hdd.getLastResult())
  {
    return RES_ERROR;
  }
  
  return RES_OK;
}

unsigned int dosIoctl(uint8_t pdrv, uint8_t cmd, uint8_t *buff)
{
  DRESULT res = RES_ERROR;
  
  switch (cmd)
  {
  case CTRL_SYNC: 
    res = RES_OK; 
    break;
  case GET_SECTOR_COUNT:
    *((uint32_t *) buff) = (uint32_t)dosGetTotalSectorCount();
    res = RES_OK;
    break;
  case GET_SECTOR_SIZE:
    *((uint32_t *) buff) = (uint32_t)dosGetSectorSize();
    break;
  case GET_BLOCK_SIZE:
    *((uint32_t *) buff) = 1;
    break;
  }
  
  return res;
}  

} // extern "C"

uint16_t dosGetSectorSize()
{
  return sectorSizeBytes;
}

uint32_t dosGetTotalSectorCount()
{
  return (uint32_t)hdd.getParams()->Cylinders * hdd.getParams()->Heads * sectorsPerTrack;
}

void dosConvertLogicalSectorToCHS(const uint32_t& logical, uint16_t& cylinder, uint8_t& head, uint8_t& sector)
{ 
  uint32_t log = logical;
  
  cylinder = (log / sectorsPerTrack) / hdd.getParams()->Heads;
  head = (log / sectorsPerTrack) % hdd.getParams()->Heads;
  sector = (log % sectorsPerTrack) + startSector;
}

FRESULT dosResult(FRESULT result)
{ 
  switch (result)
  {
  case FR_OK:
    fsErrorMessage = NULL;
    break;
  case FR_NO_FILE:
    fsErrorMessage = (char*)str_DosFileNotFound;
    break;
  case FR_NO_PATH:
    fsErrorMessage = (char*)str_DosPathNotFound;
    break;
  case FR_INVALID_NAME:
    fsErrorMessage = (char*)str_DosInvalidName;
    break;
  case FR_DENIED:
    fsErrorMessage = (char*)str_DosDirectoryFull;
    break;
  case FR_EXIST:
    fsErrorMessage = (char*)str_DosFileExists;
    break;
  case FR_INVALID_OBJECT:
    fsErrorMessage = (char*)str_DosFsError;
    break;
  case FR_NOT_ENABLED:
    fsErrorMessage = (char*)str_DosFsMountError;
    break;
  case FR_NO_FILESYSTEM:
    fsErrorMessage = (char*)str_DosFsMountError;
    break;  
  case FR_NOT_ENOUGH_CORE:
    fsErrorMessage = (char*)str_OutOfMemory;
    break;
  case FR_INVALID_PARAMETER:
    fsErrorMessage = (char*)str_DosFsError;
    break;
  default:
    fsErrorMessage = (char*)str_DosDiskError;
    break;
  }
   
  if (fsErrorMessage)
  {
    printf(fsErrorMessage);
    
    // disk error?
    if ((fsErrorMessage == str_DosDiskError) && hdd.getLastResultMessage())
    {
      printf(hdd.getLastResultMessage());
      printf("\n");
    }
    
    printf("\n");
  }
  
  return result;
}

bool dosInitialize()
{
  // switch FATFS diskio to use our functions
  diskio_use_sd = false;
  
  // look at track 0
  uint8_t interleave;
  hdd.seekDrive(0, 0);
  if (!fmt->analyzeTrack(MAX_SPT_LIMIT, false, sectorsPerTrack, startSector, sectorSizeBytes, interleave))
  {
    if (hdd.getLastResult() == HDD_STATUS_NO_SECTOR_ID)
    {
      printf(str_DosTrack0Bad);      
    }
    else
    {
      printf(hdd.getLastResultMessage());
    }

    printf("\n");
    return false;
  }
  
  if (fmt->getType() == LLF::FormatType::WD)
  {
    ((WD*)fmt)->setWorkingSectorSizeBytes(sectorSizeBytes);
  }
  
  // set root directory and try to mount first partition
  memset(path, 0, sizeof(path));
  FAT_EXECUTE_0(f_mount(&fat, "0:", 1));

  return true;
}

void dosFinish()
{
  // unmount
  f_closedir(&dir);
  f_unmount("0:");
}

void dosMkdir(const char* dirName)
{ 
  // overflow checks in CD command, as absolute paths in names are not allowed  
  memset(addPath, 0, sizeof(addPath));
  strcat(addPath, path);
  strcat(addPath, dirName);
  
  FAT_EXECUTE(f_mkdir(addPath));
}

// remove empty directory
void dosRmdir(const char* dirName)
{  
  memset(addPath, 0, sizeof(addPath));
  strcat(addPath, path);
  strcat(addPath, dirName);
  
  FAT_EXECUTE(f_rmdir(addPath));
  printf("\n");
}

void dosDel(const char* fileName)
{ 
  memset(addPath, 0, sizeof(addPath));
  strcat(addPath, path);
  strcat(addPath, fileName);
  
  FAT_EXECUTE(f_unlink(addPath));
  printf("\n");
}

void dosHexdump(const char* fileName)
{
  memset(addPath, 0, sizeof(addPath));
  strcat(addPath, path);
  strcat(addPath, fileName);
  
  unsigned int count = 1;
  FAT_EXECUTE(f_open(&file, addPath, FA_READ));
  
  uint8_t chunk[512];
  
  printf("\n");
  printf(str_HexdumpDump);
  
  while (count)
  {
    if (dosResult(f_read(&file, chunk, 512, &count)) != FR_OK)
    {
      f_close(&file);
      return;
    }
    
    for (uint16_t idx = 0; idx < count; idx++)
    {
      printf("%02X ", chunk[idx]);
    }
  }
  
  FAT_EXECUTE(f_close(&file));
  printf("\n\n");
}

void dosType(const char* fileName)
{  
  memset(addPath, 0, sizeof(addPath));
  strcat(addPath, path);
  strcat(addPath, fileName);
  
  unsigned int count = 1;
  FAT_EXECUTE(f_open(&file, addPath, FA_READ));
  
  uint8_t chunk[512] = {0};
  while (count)
  {
    FAT_EXECUTE_FILE(f_read(&file, chunk, 512, &count));
    for (uint16_t idx = 0; idx < count; idx++)
    {
      putchar(chunk[idx]);
    }    
  }
  
  FAT_EXECUTE(f_close(&file));
  printf("\n");
}

void dosTypeInto(const char* fileName)
{ 
  memset(addPath, 0, sizeof(addPath));
  strcat(addPath, path);
  strcat(addPath, fileName);
  
  FAT_EXECUTE(f_open(&file, addPath, FA_WRITE | FA_OPEN_APPEND));
  
  // wait for ENTER keypress
  printf(str_DosTypeInto);
  printf(str_Continue);
  readKey("\r");
  printf("\n");
  
  bool emptyLine = false;  
  for (;;)
  {
    // all valid keys allowed
    const char* promptBuffer = prompt();
    uint16_t length = strlen(promptBuffer);
    
    // quit?
    if (!length && emptyLine)
    {
      break;
    }    
    emptyLine = length == 0;
    
    // write line   
    unsigned int dummy;
    FAT_EXECUTE_FILE(f_write(&file, promptBuffer, length, &dummy));
    FAT_EXECUTE_FILE(f_write(&file, "\r\n", 2, &dummy));

    printf("\n");
  }
  
  FAT_EXECUTE(f_close(&file));
  printf("\n");
}

bool dosChdir()
{  
  FAT_EXECUTE_0(f_opendir(&dir, path));
  FAT_EXECUTE_0(f_closedir(&dir));
  return true;
}

// list contents of current working directory
void dosDir()
{ 
  uint8_t key = 0;
  uint16_t entriesCount = 0;
  FAT_EXECUTE(f_opendir(&dir, path));
  
  while(true)
  {
    FILINFO info = {0};
    FAT_EXECUTE_DIR(f_readdir(&dir, &info));
    
    // empty entry
    char* name = info.fname;   
    if (!name || !strlen(name))
    {
      break;
    }
    
    // directory?
    if (info.fattrib & AM_DIR)
    {
      printf(str_DosDirectory);
    }
    
    // file, print size
    else
    {     
      printf("%9lu %s ", (uint32_t)info.fsize, str_Bytes);      
    }
    
    // attributes readonly, archive, hidden, system
    printf(info.fattrib & AM_RDO ? "R" : "-");
    printf(info.fattrib & AM_ARC ? "A" : "-");
    printf(info.fattrib & AM_HID ? "H" : "-");
    printf(info.fattrib & AM_SYS ? "S" : "-");
        
    // name, next entry
    printf(" %s\n", name);
    entriesCount++;
  }
  
  // end of listing
  FAT_EXECUTE(f_closedir(&dir));
  if (!entriesCount)
  {
    printf(str_DosDirectoryEmpty);
    printf("\n");
  }
  
  // show free space
  FATFS* dummy;
  uint32_t freeClusters = 0;
  FAT_EXECUTE(f_getfree("0:", &freeClusters, &dummy));
  
  printf("%9lu ", (freeClusters * fat.csize * sectorSizeBytes));
  printf("%s\n", str_DosBytesFree);
}

// uppercase string
void ToUpper(char* str)
{
  if (!str)
  {
    return;
  }
  
  for (uint16_t index = 0; index < strlen(str); index++)
  {
    str[index] = toupper(str[index]);
  }
}

// check for invalid characters
bool VerifySuppliedPath(const char* pathToCheck)
{
  return !strpbrk(pathToCheck, "*?\\/\":<>|");
}

void commandDos(LLF* format)
{
  fmt = format;
  if (!dosInitialize())
  {
    return;
  }
  
  // print partition size
  const uint16_t partitionSize = round((((fat.n_fatent-2) * fat.csize) * (uint32_t)sectorSizeBytes) / 1048576.0);
  clear();
  printf(str_DosMounted, partitionSize);
  printf(str_DosCommands);  
  
  // commands loop
  while(true)
  {
    hdd.selectDrive(false); // drive not needed at the moment
    
    // commands - max length 8
    // arguments - only 8.3 file name allowed for all, with a dot and a terminating \0
    char command[8 + 1] = {0};
    char arguments[12 + 1] = {0};
    
    printf("C:\\");
    if (strlen(path))
    {
      // don't display the trailing backslash
      char* backslash = &path[strlen(path)-1];
      *backslash = 0;
      printf(path);
      *backslash = '\\';      
    }
    printf(">");
    
    // wait for command (8+12 characters and a space)
    sscanf(prompt(21), "%8s %12s", command, arguments);
    ToUpper(command);
    ToUpper(arguments);
        
    // empty command
    if (!strlen(command))
    {
      printf("\r");
      continue;
    }
    
    printf("\n");
    
    // EXIT
    if (strcmp(command, "EXIT") == 0)
    {
      dosFinish(); // unmount
      return;
    }
    
    // CD
    else if ((strcmp(command, "CD") == 0) ||
             (strcmp(command, "CD.") == 0) ||
             (strcmp(command, "CD..") == 0) ||
             (strcmp(command, "CD\\") == 0))
    {     
      // CD\ and CD \ <- go to the root directory
      if ((strcmp(command, "CD\\") == 0) ||
         ((strcmp(command, "CD") == 0) && (strcmp(arguments, "\\") == 0)))
      {
        path[0] = 0;
        
        printf("\n");
        continue;
      }
      
      // CD.. and CD .. <- go one level up
      else if ((strcmp(command, "CD..") == 0) ||
              ((strcmp(command, "CD") == 0) && (strcmp(arguments, "..") == 0)))
      {
        if (strlen(path) > 1)
        {
          // cancel out ending backslash and find the second to last          
          path[strlen(path)-1] = 0;         
          char* prevBackslash = strrchr(path, '\\');
          
          // go back to root directory
          if (!prevBackslash)
          {
            path[0] = 0;
          }
          else
          {
            *(prevBackslash+1) = 0;
          }
        }
                
        printf("\n");
        continue;        
      }
      
      // CD (empty argument), CD. and CD . <- display current directory on new line
      else if (((strcmp(command, "CD") == 0) && (!strlen(arguments))) ||
                (strcmp(command, "CD.") == 0) ||
               ((strcmp(command, "CD") == 0) && (strcmp(arguments, ".") == 0)))
      {
        printf("C:\\");
        if (strlen(path))
        {
          char* backslash = &path[strlen(path)-1];
          *backslash = 0;
          printf(path);
          *backslash = '\\';
        }
        printf("\n\n");
        continue;
      }
           
      // verify path
      if (!VerifySuppliedPath(arguments))
      {
        printf(str_DosInvalidDirName);
        printf("\n\n");
        continue;
      }
      
      // CD directory, verify we're not too nested deeply
      // (current path + new directory + backslash after + 8.3 filename with dot)
      const uint8_t newDirLen = strlen(arguments);      
      if ((strlen(path) + newDirLen + 12) > MAX_PATH)
      {
        printf(str_DosMaxPath);
        printf("\n\n");
        continue;
      }
      
      // try if current path works
      if (!dosChdir())
      {
        continue;
      }
                 
      //append path and backslash, try changing directory
      strncat(path, arguments, MAX_PATH);
      strncat(path, "\\", MAX_PATH);
            
      if (!dosChdir())
      {
        // failed, shorten the path
        path[strlen(path)-1] = 0;         
        char* prevBackslash = strrchr(path, '\\');
        
        // go back to root directory
        if (!prevBackslash)
        {
          path[0] = 0;
        }
        else
        {
          *(prevBackslash+1) = 0;
        }
      }
      else
      {
        printf("\n");
      }

      continue;
    }
    
    // DIR
    else if (strcmp(command, "DIR") == 0)
    {
      
      // provided 1 directory from our working path
      if (strlen(arguments))
      {
        if (!VerifySuppliedPath(arguments))
        {
          printf(str_DosInvalidDirName);
          printf("\n\n");
          continue;
        }
        
        // verify we're not too nested deeply (1 char extra for backslash)
        const uint8_t newDirLen = strlen(arguments);      
        if ((strlen(path) + newDirLen + 1) > MAX_PATH)
        {
          printf(str_DosMaxPath);
          printf("\n\n");
          continue;
        }
        
        // try if current path works        
        if (!dosChdir())
        {
          continue;
        }
        
        // change working directory for a while (similar to CD above)
        strncat(path, arguments, MAX_PATH);
        strncat(path, "\\", MAX_PATH);
        
        if (dosChdir())
        {
          printf("\n");
          dosDir();
        }
        
        // get the path back again to original
        path[strlen(path)-1] = 0;         
        char* prevBackslash = strrchr(path, '\\');
        
        if (!prevBackslash)
        {
          path[0] = 0;
        }
        else
        {
          *(prevBackslash+1) = 0;
        }
        
        continue;
      }
      
      // show current directory contents
      printf("\n");      
      dosDir();
      continue;
    }
    
    // MKDIR, RMDIR
    else if ((strcmp(command, "MKDIR") == 0) ||
             (strcmp(command, "RMDIR") == 0))
    {
      
      // 1 directory name
      if (strlen(arguments))
      {
        if (!VerifySuppliedPath(arguments))
        {
          printf(str_DosInvalidDirName);
          printf("\n\n");
          continue;
        }
              
        // remove directory
        if (strcmp(command, "RMDIR") == 0)
        {
          dosRmdir(arguments);  
        }
        
        // make directory
        else
        {
          dosMkdir(arguments);
        }        

        continue;
      }
      
      printf(str_DosInvalidDirName);
      printf("\n\n");
      continue;
    }
    
    // DEL, TYPE, HEXDUMP
    else if ((strcmp(command, "DEL") == 0) ||
             (strcmp(command, "TYPE") == 0) ||
             (strcmp(command, "HEXDUMP") == 0))
    {
      
      // 1 file name
      if (strlen(arguments))
      {
        if (!VerifySuppliedPath(arguments))
        {
          printf(str_DosInvalidName);
          printf("\n\n");
          continue;
        }
        
        // DEL
        if (strcmp(command, "DEL") == 0)
        {
          dosDel(arguments);
        }
        
        // TYPE
        else if (strcmp(command, "TYPE") == 0)
        {                     
          dosType(arguments);            
        }
        
        // HEXDUMP
        else
        {
          dosHexdump(arguments);
        }
        
        continue;
      }
      
      printf(str_DosInvalidName);
      printf("\n\n");
      continue;
    }
    
    // TYPEINTO
    else if (strcmp(command, "TYPEINTO") == 0)
    {
            
      // 1 file name
      if (strlen(arguments))
      {
        if (!VerifySuppliedPath(arguments))
        {
          printf(str_DosInvalidName);
          printf("\n\n");
          continue;
        }
        
        dosTypeInto(arguments);
        continue;
      }
      
      printf(str_DosInvalidName);
      printf("\n\n");
      continue;
    }
    
    // prompt bubbled through - unrecognized command
    printf(str_DosInvalidCommand);
    printf("\n\n");
  }
}