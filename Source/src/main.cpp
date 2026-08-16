// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Entry point

#include "config.h"

// global objects
HDD hdd;
ENDEC endec;

// modified by global GPIO callback
volatile bool g_SeekComplete = false;
volatile int g_IndexCount = 0;

// PIO offsets
extern const uint g_PioSamplerOffset    = pio_add_program(pio0, &pio_sampler_program);     // SM 0
extern const uint g_PioWriterOffset     = pio_add_program(pio0, &pio_writer_program);      // SM 1
extern const uint g_PioWclockTestOffset = pio_add_program(pio0, &pio_wclock_test_program); // SM 2

// DMA channel for PIO writer
extern const int  g_PioWriterDma        = dma_claim_unused_channel(true);

// separator data rate in Mbps determined during startup
float g_WclockRate = 0.0f;

// tinyUSB callback: auto-reboot if the serial connection drops
#if defined(AUTOREBOOT_ON_DISCONNECT) && (AUTOREBOOT_ON_DISCONNECT == 1)
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  static bool connEstablished = false;
  if (dtr)
  {
    connEstablished = true;
  }
  else if (connEstablished)
  {
    watchdog_reboot(0, 0, 0);
  }
}
#endif

// GPIO: /SC, /INDEX
void __no_inline_not_in_flash_func(gpioCallback)(uint gpio, uint32_t events)
{
  if (gpio == 13) // input from disk drive: /SC
  {
    if (events & GPIO_IRQ_EDGE_RISE)
    {
      g_SeekComplete = false;
      gpio_put(28, g_SeekComplete); // also output SC to the separator board      
    }

    if (events & GPIO_IRQ_EDGE_FALL)
    {
      g_SeekComplete = true;
      gpio_put(28, g_SeekComplete);      
    }
  }
  
  else if (gpio == 6) // input from disk drive: /INDEX
  {
    if (events & GPIO_IRQ_EDGE_FALL)
    {
      g_IndexCount += 1;
    }
  }
}

// sample WCLOCK of data separator and set g_WclockRate
// halt in infinite loop if the board is not working
void testWclock()
{
  printf(str_DataSeparator);
  
  // start SM 2
  pio_sm_restart(pio0, 2);
  pio_sm_clear_fifos(pio0, 2);
  pio_sm_exec(pio0, 2, pio_encode_jmp(g_PioWclockTestOffset));
  pio_sm_set_enabled(pio0, 2, true);
    
  // set jmp to (public) results label after 100ms
  const uint measureMs = 100;
  sleep_ms(measureMs);    
  pio_sm_exec(pio0, 2, pio_encode_jmp(g_PioWclockTestOffset + pio_wclock_test_offset_results));
    
  // stop PIO, round to 1 decimal place and display
  const uint32_t pulses = 0xFFFFFFFF - pio_sm_get_blocking(pio0, 2);
  pio_sm_set_enabled(pio0, 2, false);
  
  const float MHz = (((float)pulses * 1000.0f) / (float)measureMs) / 1000000.0f;
  g_WclockRate = round(MHz * 10.0f) / 10.0f;
  if (g_WclockRate < 1)
  {
    printf(str_Error);
    fatalError(str_DataSeparatorFailure);
  }
  
  printf(((g_WclockRate*10) != (int)g_WclockRate * 10) ? "%.1f" : "%.0f", g_WclockRate);
  printf(" Mbps\n\n");
}

// in KB
void getMemoryInfo(uint16_t& ramUsed, uint16_t& flashUsed)
{
  // linker map symbols
  extern char __flash_binary_start[];
  extern char __flash_binary_end[];
  extern char ram_vector_table[];
  extern char __end__[];    
 
  const float ram = (((uintptr_t)__end__ - (uintptr_t)ram_vector_table) + mallinfo().uordblks) / 1024.0f;
  const float flash = ((uintptr_t)__flash_binary_end - (uintptr_t)__flash_binary_start) / 1024.0f;
  ramUsed = (uint16_t)(round(ram*10.0f) / 10.0f);
  flashUsed = (uint16_t)(round(flash*10.0f) / 10.0f);
}

// MHz, V, °C
void getSystemInfo(uint16_t& sysclk, float& vreg, int& degrees)
{
  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);
  degrees = 0;
  
  const uint8_t samples = 50;  
  for (uint8_t sample = 0; sample < samples; sample++)
  {
    degrees += adc_read();
  }
  degrees /= samples;  
  degrees = 27.0f - ((degrees * (3.3f / (1 << 12)) - 0.706f) / 0.001721f);
  
  sysclk = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) / 1000;
  
  vreg_voltage volts = vreg_get_voltage();
  switch(volts)
  {
    case VREG_VOLTAGE_0_80: vreg = 0.80f; break;
    case VREG_VOLTAGE_0_85: vreg = 0.85f; break;
    case VREG_VOLTAGE_0_90: vreg = 0.90f; break;
    case VREG_VOLTAGE_0_95: vreg = 0.95f; break;
    case VREG_VOLTAGE_1_00: vreg = 1.00f; break;
    case VREG_VOLTAGE_1_05: vreg = 1.05f; break;
    case VREG_VOLTAGE_1_10: vreg = 1.10f; break;
    case VREG_VOLTAGE_1_15: vreg = 1.15f; break;
    case VREG_VOLTAGE_1_20: vreg = 1.20f; break;
    case VREG_VOLTAGE_1_25: vreg = 1.25f; break;
    case VREG_VOLTAGE_1_30: vreg = 1.30f; break;
    case VREG_VOLTAGE_1_35: vreg = 1.35f; break;
    case VREG_VOLTAGE_1_40: vreg = 1.40f; break;
    case VREG_VOLTAGE_1_50: vreg = 1.50f; break;
    case VREG_VOLTAGE_1_60: vreg = 1.60f; break;
    case VREG_VOLTAGE_1_65: vreg = 1.65f; break;
    case VREG_VOLTAGE_1_70: vreg = 1.70f; break;
    case VREG_VOLTAGE_1_80: vreg = 1.80f; break;
    case VREG_VOLTAGE_1_90: vreg = 1.90f; break;
    case VREG_VOLTAGE_2_00: vreg = 2.00f; break;
  }
}

int main()
{
  // :)
  vreg_disable_voltage_limit();
  vreg_set_voltage(VREG_VOLTAGE_1_10);
  set_sys_clock_khz(250000, true);
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, clock_get_hz(clk_sys), clock_get_hz(clk_sys));
  // minimum for proper operation (especially RLL): 250MHz with gcc -O3
  // 250MHz: 1.10V, PICO_FLASH_SPI_CLKDIV=2 (flash @ 125MHz)
  // 300MHz: 1.20V, PICO_FLASH_SPI_CLKDIV=3 (flash @ 100MHz)
  // 400MHz: 1.40V, PICO_FLASH_SPI_CLKDIV=4 (flash @ 100MHz)
  // 500MHz: 1.60V, PICO_FLASH_SPI_CLKDIV=4 (flash @ 125MHz)
  // 600MHz: 1.80V, PICO_FLASH_SPI_CLKDIV=5 (flash @ 120MHz)
  // RP2350: CLKDIV doesn't have to be even 
  
  srand(get_rand_32());
   
  // GPIOs used: 2-22, 26-28
  gpio_init_mask(0x1C7FFFFC);

  // set initial states of GPIOs
  // outputs:
  // 28 (separator: SC), 27 (74HC595: SER), 26 (74HC595: /OE), 22 (74HC595: RCLK), 21 (74HC595: SRCLK), 12 (separator: WPCEN), 
  // 11 (separator: /EARLY), 10 (separator: /LATE), 8 (separator: WDATA), 7 (separator: WGATE), 3 (separator: RGATE)
  // inputs:
  // 20 (drive: /WFAULT), 19 (SD: MOSI), 18 (SD: SCK), 17 (SD: SCS), 16 (SD: MISO), 15 (drive: /READY), 14 (drive: /TRK0),
  // 13 (drive: /SC), 9 (separator: WCLOCK), 6 (drive: /INDEX), 5 (separator: RCLOCK), 4 (separator: RDATA), 2 (separator: DRUN)
  gpio_set_dir_masked(0x1C7FFFFC, 0x1C601D88);

  // drive outputs: 26 (74HC595: /OE), 11 (separator: /EARLY), 10 (separator: /LATE) high; and the rest of the outputs low
  gpio_put_masked(0x1C601D88, 0x4000C00);

  // set initial states of the output shift register 74HC595
  // (Q0: HDSEL0, Q1: HDSEL1, Q2: HDSEL2, Q3: HDSEL3, Q4: STEP, Q5: DIR, Q6: DS, Q7: RLL/MFM)
  // all outputs low
  gpio_put(22, false);  // RCLK low
  uint8_t bit = 8;
  while (bit--)
  {
    gpio_put(21, true); // toggle SRCLK
    busy_wait_at_least_cycles(80);
    gpio_put(21, false); 
    busy_wait_at_least_cycles(80);
  }
  gpio_put(22, true);  // RCLK high
  gpio_put(26, false); // set /OE low
  
  // set global GPIO callback
  gpio_set_irq_callback(gpioCallback);
  irq_set_enabled(IO_IRQ_BANK0, true);
  gpio_set_irq_enabled(13, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true); // /SC
  gpio_set_irq_enabled(6, GPIO_IRQ_EDGE_FALL, true); // /INDEX
  
  // initialize three PIO state machines: sampler, writer, WCLOCK test
  pio_sampler_program_init(pio0, 0, g_PioSamplerOffset);
  pio_writer_program_init(pio0, 1, g_PioWriterOffset);
  pio_wclock_test_program_init(pio0, 2, g_PioWclockTestOffset);
  
  // proceed after serial console has been established
  stdio_init_all();
  while (!stdio_usb_connected())
  {
    sleep_ms(100);
  }
  
  // get and print info
  uint16_t initialRamUsed;
  uint16_t flashUsed;
  uint16_t sysclk;
  float vreg;
  int degrees;
  getMemoryInfo(initialRamUsed, flashUsed);
  getSystemInfo(sysclk, vreg, degrees);
  
  clear();
  printf(str_Splash);
  printf(str_SystemInfo, sysclk, vreg, degrees);
  printf(str_MemoryUsage, initialRamUsed, flashUsed);  
  testWclock();
    
  // wait for drive
  hdd.selectDrive();
  printf(str_WaitingReady);
  while (!hdd.isDriveReady()) {}
  printf(" "); printf(str_OK); printf("\n");
    
  // detect track 0, this throws an internal error if it fails
  printf(str_SeekingToCyl0);
  hdd.recalibrate();
  // if we're already at track 0
  printf(" "); printf(str_OK); printf("\n");
  
  // we can continue; load or provide disk parameters
  hdd.selectDrive(false);
  hdd.diskConfigurationProvide();
  
  // main menu  
  LLF* format = NULL;  
  for (;;)
  {
    char key;
    char menuOptions[20] = {0};
    
    if (format)
    {
      delete format;
      format = NULL;
    }
       
    hdd.seekDrive(0, 0);
    hdd.selectDrive(false);
    hdd.setSeparatorRLL(false); // MFM by default
    printf("\n");
    
    strcat(menuOptions, "12345ABCD");
    printf(str_MainMenu);
    if (hdd.getParams()->UseLandingZone) // add park option
    {
      printf(str_MainMenuOptionPark);
      strcat(menuOptions, "E");
    }
    printf("\n\n");
    printf(str_ChooseOption);
    key = toupper(readKey(menuOptions));
    printf(str_EchoKey, key);
    
    if (key == '1')
    {     
      // WD format specifics: data separator mode, data verify mode, SDH head select bit width
      printf(str_FormatSpecificOptions, "WD");
      printf(str_EscGoBack);
      
      printf("\n");
      printf(str_ChooseSeparatorMode, ((int)g_WclockRate == 5) ? "MFM" : "RLL");
      key = toupper(readKey("MR\e"));
      if (key == '\e') { printf("\n"); continue; }
      printf(str_EchoKey, key);
      hdd.setSeparatorRLL(key == 'R');
      
      format = new WD;
      
      // always 56 bits CRC if RLL
      if (!hdd.isSeparatorRLL())
      {
        printf(str_OptWDChooseCrcWidth);
        key = toupper(readKey("135\e"));
        if (key == '\e') { printf("\n"); continue; }
        printf(str_EchoKey, key);
        if (key == '1') ((WD*)format)->setDataCrcBits(16);
        else if (key == '3') ((WD*)format)->setDataCrcBits(32);
        else ((WD*)format)->setDataCrcBits(56);  
      }
   
      if (hdd.getParams()->Heads > 8)
      {
        printf(str_OptWDChooseSdhWidth);
        key = toupper(readKey("34\e"));
        if (key == '\e') { printf("\n"); continue; }
        printf(str_EchoKey, key);
        ((WD*)format)->setSdh4Bit(key == '4');
      }
      
      formatMenu(format);
    }    
    else if (key == '2')
    {
      format = new OMTI;
      
      // inform about nonstandard tracks presence
      printf(str_SpecialTracksNote);
      printf(str_Continue);
      char key = readKey("\r");
      printf(str_DeleteLine);
      
      formatMenu(format);
    }
    else if (key == '3')
    {
      format = new XebecAdaptec;
      
      // Xebec/Adaptec format specifics: data field write type
      printf(str_FormatSpecificOptions, "Xebec/Adaptec");
      printf(str_EscGoBack);
      printf(str_OptXAChooseDataPrefix);
      key = toupper(readKey("XA\e"));
      if (key == '\e') { printf("\n"); continue; }
      printf(str_EchoKey, key);
      ((XebecAdaptec*)format)->setWriteModeAdaptec(key == 'A');
      
      formatMenu(format);
    }
    else if (key == '4')
    {
      format = new HDC9224;
      formatMenu(format);
    }
    else if (key == '5')
    {
      format = new SM1040;
      formatMenu(format);
    }
    else if (key == 'A')
    {
      commandAutodetect();
    }
    else if (key == 'B')
    {
      commandRawdisk();
    }
    else if (key == 'C')
    {
      commandErase();
    }
    else if (key == 'D')
    {
      commandSeekTest();
    }
    else if (key == 'E')
    {
      commandPark();
    }
  }
}