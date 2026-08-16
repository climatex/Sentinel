// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Console helpers

#include "config.h"

// readkey with wait or without
#define READKEY_CHECK_WAIT if (withWait) continue; else return 0;

char m_promptBuffer[MAX_PROMPT_LEN + 1];
const char* getPromptBuffer()
{
  return (const char*)&m_promptBuffer[0];
}

void clear()
{
  printf(str_ClearTerminal);
}

char readKey(const char* allowedKeys, bool withWait)
{ 
  // check for allowed keys; null pointer means all (supported) are allowed
  bool checkAllowed = (allowedKeys != NULL);
  
  while(true)
  {
    int read = stdio_getchar_timeout_us(0);
    if (read == PICO_ERROR_TIMEOUT)
    {
      READKEY_CHECK_WAIT;
    }
    
    // filter out invalid characters
    else if (read == 0x7f)
    {
      read = 0x08; // treat DEL as backspace if it came thru serial
    }
    else if (read > 0x7e)
    {  
      READKEY_CHECK_WAIT;
    }
    // allow CR (ENTER), Escape, and backspace
    else if ((read < 0x20) && (read != 0x0D) && (read != 0x1B) && (read != 0x08))
    { 
      READKEY_CHECK_WAIT;
    }
    
    char chr = (char)read;
    
    // no check for certain keys? return here
    if (!checkAllowed)
    {
      return chr;
    }
    
    // go thru the allowed characters string, re-read keyboard if key not in there
    for (size_t index = 0; index < strlen(allowedKeys); index++)
    {
      if (toupper(chr) == toupper(allowedKeys[index]))
      {
        return chr;
      }
    }
    
    READKEY_CHECK_WAIT;
  }
}

// prompt for string with a maximum length if set; allowed keys (if not null) shall contain at least \r\b
const char* prompt(uint8_t maximumPromptLen, const char* allowedKeys, bool escReturnsNull)
{ 
  // buffer overflow check
  if (!maximumPromptLen || (maximumPromptLen > (sizeof(m_promptBuffer)-1)))
  {
    maximumPromptLen = sizeof(m_promptBuffer)-1;
  }
    
  // prompt for a command
  uint8_t index = 0;
  memset(m_promptBuffer, 0, sizeof(m_promptBuffer));
  
  while(true)
  {
    const char chr = readKey(allowedKeys);
       
    // ENTER or CR on terminal: confirm prompt
    if (chr == '\r')
    {
      break;
    }
      
    // ESC, if allowed
    else if (chr == '\e')
    {
      // return as if prompt canceled
      if (escReturnsNull)
      {
        return NULL;
      }
      
      // default: cancel out current command and make a newline (like in DOS)      
      while (index > 0)
      {
        m_promptBuffer[--index] = 0;
      }
            
      // echo backslash with a newline
      printf("\\\r\n");      
      continue;
    }
    
    // backspace - shorten the string by one
    else if (chr == '\b')
    {
      if (index > 0)
      {
        // make sure the backspace is destructive on a serial terminal
        printf("\b \b");        
        m_promptBuffer[--index] = 0;
      }
      
      continue;
    }
    
    // any other key and the buffer is full?
    else if (index == maximumPromptLen)
    {
      continue;
    }
    
    // echo and store prompt buffer
    printf("%c", chr);   
    m_promptBuffer[index++] = chr;    
  }
  
  return (const char*)&m_promptBuffer;
}

// print an error that doesn't continue
void fatalError(const char* message)
{ 
  hdd.selectDrive(false);
  
  printf(str_FatalError, message);
  for (;;) {}
}