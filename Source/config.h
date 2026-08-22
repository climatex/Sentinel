// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Build configuration

#pragma once

// tinyUSB
#define AUTOREBOOT_ON_DISCONNECT    1       // 1: reboots the Pico when the terminal session ends; 0: leaves running in last state

// seeking 
#define SEEK_PULSE_US               3       // seek pulse length, microseconds
#define SLOWSEEK_SRT_MS             4       // head step rate time in ms to wait before next step (slow seek mode)
#define FASTSEEK_SRT_US             10      // microseconds to wait before next step (buffered seek)

// timeouts
#define TIMEOUT_STARTUP_READY_MS    100     // during powerup test, minimum time for disk to signal /READY to be considered such
#define TIMEOUT_SEEK_COMPLETE_MS    500     // seek must be complete within half a second of last pulse sent
#define TIMEOUT_DISK_ROTATION_US    17333   // nominal: 16.67ms @ 3600RPM
#define TIMEOUT_FIFO_READS_US       30      // max. time to wait for 16 bits from PIO during disk reads
#define TIMEOUT_DATA_PREAMBLE_US    15      // max. time to wait for DRUN after a sector ID field has been read and verified

// encoder/decoder
#define DEFAULT_MFM_SYNC_PATTERN    0x4489  // MFM 0xA1 with clk=0x0A instead of 0x0E (Ck2: 0), intertwinned as Ck7,D7,Ck6,D6...Ck0,D0
#define DEFAULT_RLL_SYNC_PATTERN    0x8090  // RLL 2,7 1000000010010000 which violates coding table rules
#define MAX_SPT_LIMIT               50      // maximum number of sectors per track supported, for all sector sizes

// low-level formats
#define READ_SECTOR_ATTEMPTS        10      // how many attempts to read a sector (if it has been found) before failing

// UI defines
#define MAX_PROMPT_LEN              100     // prompt() buffer size

// filesystem defines
#define MAX_PATH                    100     // max path, MAX_PATH+1 size of path buffer

// miscellaneous
#define FLASH_CONFIG_TARGET        0x3FF000 // last 4K of flash used to store disk configuration 
#define INLINE inline __attribute__((always_inline))

// standard library, STL and internal Pico includes
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <malloc.h>
#include <vector>
#include <array>
#include <algorithm>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/rand.h"
#include "hardware/adc.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/vreg.h"
#include "hardware/flash.h"
#include "tusb.h"

// our own
#include "stringtable.h"
#include "console.h"
#include "hdd.h"
#include "crc.h"
#include "llf.h"
#include "endec.h"
#include "dos.h"
#include "sd.h"
#include "commands.h"

// individual formats
#include "wd.h"
#include "omti.h"
#include "hdc9224.h"
#include "xebec_adaptec.h"
#include "sm1040.h"

// PIO
#include "sampler.pio.h"
#include "writer.pio.h"
#include "wclock_test.pio.h"

// public globals
extern HDD hdd;
extern ENDEC endec;