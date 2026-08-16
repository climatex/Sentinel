// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// String table

#pragma once
#include "config.h"

#define STRINGTABLE static const char

// shared
STRINGTABLE str_Empty[]                   = "";
STRINGTABLE str_DeleteLine[]              = "\r                                                                  \r";
STRINGTABLE str_ClearTerminal[]           = "\e[H\e[2J\r                                                         \r";
STRINGTABLE str_EchoKey[]                 = "%c\n";
STRINGTABLE str_Enabled[]                 = "enabled";
STRINGTABLE str_Disabled[]                = "disabled";
STRINGTABLE str_Yes[]                     = "yes";
STRINGTABLE str_No[]                      = "no";
STRINGTABLE str_OK[]                      = "OK";
STRINGTABLE str_Error[]                   = "error";
STRINGTABLE str_Continue[]                = "ENTER to continue...";
STRINGTABLE str_ContinueAbort[]           = "ENTER: continue, Esc: abort...";
STRINGTABLE str_Abort[]                   = "Press Esc now to abort."; 
STRINGTABLE str_DecimalInput[]            = "0123456789\r\b";
STRINGTABLE str_DecimalInputEsc[]         = "0123456789\r\b\e";
STRINGTABLE str_Bytes[]                   = "bytes";
STRINGTABLE str_OperationPending[]        = "Busy...";
STRINGTABLE str_ChooseOption[]            = "Choose: ";
STRINGTABLE str_ChooseCylinder[]          = "Cylinder (%u-%u): ";
STRINGTABLE str_ChooseHead[]              = "Head (%u-%u): ";
STRINGTABLE str_ChooseSector[]            = "Sector (%u-%u): ";
STRINGTABLE str_ChooseStartCyl[]          = "Starting cylinder (%u-%u): ";
STRINGTABLE str_ChooseEndCyl[]            = "Ending cylinder (%u-%u): ";
STRINGTABLE str_ChooseSecSize[]           = "Sector size (1)28B (2)56B (5)12B 1(K)B: ";
STRINGTABLE str_ChooseExpectedSpt[]       = "Expected sectors per track (%u-%u): ";
STRINGTABLE str_ChooseExpectedSptDef[]    = "Expected sectors per track (%u-%u, default: %u): ";
STRINGTABLE str_ChooseSpt[]               = "Sectors per track (%u-%u): ";
STRINGTABLE str_ChooseSptDef[]            = "Sectors per track (%u-%u, default: %u): ";
STRINGTABLE str_ChooseStartSector[]       = "Starting sector (%u-%u): ";
STRINGTABLE str_ChooseStartSectorDef[]    = "Starting sector (%u-%u, default: %u): ";
STRINGTABLE str_ChooseStartSectorXTAT[]   = "Starting sector (%u-%u, XT: 0, AT: 1): ";
STRINGTABLE str_ChooseFormatInterleave[]  = "Format interleave (1-%u, 1: none): ";
STRINGTABLE str_ChooseSeparatorMode[]     = "Data separator mode: (M)FM / (R)LL (likely %s): ";
STRINGTABLE str_EscGoBack[]               = "\nPress Esc to go back.\n";
STRINGTABLE str_ProcessingCyl[]           = "\rProcessing cylinder %u...";
STRINGTABLE str_UnreadableTracks[]        = "Unreadable tracks: %lu\n";
STRINGTABLE str_SectorErrors[]            = "Sector errors: %lu\n";
STRINGTABLE str_SectorSizeBytes[]         = "Sector size: %u bytes\n";

// startup
STRINGTABLE str_Splash[]                  = "Sentinel (c) 2026 J. Bogin, https://boginjr.com\n\n"
                                            "Build date:     16 Aug 2026\n";
STRINGTABLE str_SystemInfo[]              = "System clock:   %u MHz (Vreg %.1f V; %d \u00B0C)\n";
STRINGTABLE str_MemoryUsage[]             = "Memory usage:   initial %uKB of 512KB RAM\n"
                                            "                %uKB of 4MB flash\n";
STRINGTABLE str_DataSeparator[]           = "Data separator: ";
STRINGTABLE str_WaitingReady[]            = "Waiting until drive becomes /READY...";
STRINGTABLE str_SeekingToCyl0[]           = "Determining position of cylinder 0...";

// disk drive configuration
STRINGTABLE str_DiskCfgAskParams[]        = "\nSet disk drive parameters:\n\n";
STRINGTABLE str_DiskCfgAskCylinders[]     = "Cylinders (1-2048): ";
STRINGTABLE str_DiskCfgAskHeads[]         = "Heads (1-16): ";
STRINGTABLE str_DiskCfgAskRWC[]           = "Drive requires reduced write current? Y/N: ";
STRINGTABLE str_DiskCfgAskCylRWC[]        = "Reduced write current start cylinder (0: always): ";
STRINGTABLE str_DiskCfgAskPrecomp[]       = "Drive requires write precompensation? Y/N: ";
STRINGTABLE str_DiskCfgAskCylPrecomp[]    = "Write precompensation start cylinder (0: always): ";
STRINGTABLE str_DiskCfgAskLZ[]            = "Is landing zone specified (no auto parking)? Y/N: ";
STRINGTABLE str_DiskCfgAskCylLZ[]         = "Landing zone (parking cylinder): ";
STRINGTABLE str_DiskCfgAskSeek[]          = "Drive seeking: (N)ormal buffered / (L)egacy ST-506: ";
STRINGTABLE str_DiskCfgAskReseek[]        = "Re-seek on sector read errors? Y/N: ";
STRINGTABLE str_DiskCfgAskCorrectCRC[]    = "Attempt to correct CRC data errors? Y/N: ";
STRINGTABLE str_DiskCfgAskSave[]          = "\nRemember these settings? Y/N: ";
STRINGTABLE str_DiskCfgAskRemove[]        = "Remove saved settings? Y/N: ";
STRINGTABLE str_DiskCfgSaved[]            = "Stored settings:\n";
STRINGTABLE str_DiskCfgSavedLoad[]        = "\n\nLoading these settings.\n";
STRINGTABLE str_DiskCfgFromCyl[]          = ", from cylinder %u";
STRINGTABLE str_DiskCfgSeekSlow[]         = "legacy ST-506";
STRINGTABLE str_DiskCfgSeekNormal[]       = "normal buffered";
STRINGTABLE str_DiskCfgCylinders[]        = "\nCylinders:             %u";
STRINGTABLE str_DiskCfgHeads[]            = "\nHeads:                 %u";
STRINGTABLE str_DiskCfgRWC[]              = "\nReduced write current: ";
STRINGTABLE str_DiskCfgPrecomp[]          = "\nWrite precompensation: ";
STRINGTABLE str_DiskCfgLZStatus[]         = "\nAutopark on powerdown: ";
STRINGTABLE str_DiskCfgLZ[]               = "\nLanding zone cylinder: %u";
STRINGTABLE str_DiskCfgSeekMode[]         = "\nDrive seeking mode:    ";
STRINGTABLE str_DiskCfgReseek[]           = "\nRe-seek on errors:     ";
STRINGTABLE str_DiskCfgCorrectCRC[]       = "\nCorrect CRC errors:    ";

// main menu
STRINGTABLE str_MainMenu[]                = "Select low-level format:               Special options:\n\n"
                                            "1) Western Digital, MFM/RLL, generic   A) Autodetect format\n"
                                            "2) SMS OMTI, MFM, PC/AT                B) Raw disk operations\n"
                                            "3) Xebec/Adaptec, MFM, PC/XT           C) Erase disk\n"
                                            "4) SMC HDC9224, MFM, PC/XT             D) Heads seek test / exercise\n"
                                            "5) VUVT SMEP SM 1040, MFM              ";
STRINGTABLE str_MainMenuOptionPark[]      = "E) Park drive heads";

// format-specific options
STRINGTABLE str_FormatSpecificOptions[]   = "\nEnter format specifics for the %s format.\n"
                                            "You can try the \"Autodetect format\" menu option to get the details.";
// WD
STRINGTABLE str_OptWDChooseCrcWidth[]     = "\nChoose sector data CRC length to format, write and expect during reads.\n"
                                            "32 bits was the most common.\n"
                                            "Sector data fields CRC: (1)6-bit / (3)2-bit / (5)6-bit: ";
STRINGTABLE str_OptWDChooseSdhWidth[]     = "\nDisk with more than 8 heads has been configured.\n"
                                            "Sector ID field (SDH) head number width, in bits, needs to be set.\n"
                                            "WD1010/2010: always 3 bits. WD50C12/42C22 or later could be either 3 or 4.\n"
                                            "SDH head number width: (3) bits / (4) bits: ";
// Xebec/Adaptec
STRINGTABLE str_OptXAChooseDataPrefix[]   = "\nIBM/Xebec XT controllers start sector data fields with byte C9,\n"
                                            "the Adaptec (such as ACB-2010) with byte 00.\n"
                                            "Usually, Xebecs can work with both and Adaptecs require the zero.\n\n"
                                            "Data fields to write/format: (X)ebec C9 / (A)daptec-compatible 00: ";
// nonstandard tracks note
STRINGTABLE str_SpecialTracksNote[]       = "\nNOTE: with this format you may observe tracks where every sector fails CRC\n"
                                            "or where the Analyze command returns fields that do not match seek position.\n"
                                            "If written to using the Write from binary image or Format commands,\n"
                                            "these will be re-encoded as normal tracks. This may cause issues.\n";

// low level format menu
STRINGTABLE str_LlfMenu[]                 = "(A)nalyze disk for sector ID fields\n"
                                            "(H)ex dump of one sector data field\n"
                                            "(V)erify data fields\n"
                                            "(R)ead data fields into binary image\n"
                                            "(M)icrostepping during reads (recovery mode): %s\n"
                                            "(F)ormat disk\n"
                                            "(W)rite data fields from binary image, with format\n";                                            
STRINGTABLE str_LlfMountDOS[]             = "(I)nspect first DOS primary partition\n";
STRINGTABLE str_LlfBack[]                 = "(B)ack to the main menu\n";

// fatal errors
STRINGTABLE str_FatalError[]              = "\n\nFatal error:\n%s\n\nSystem halted, reset required.";
STRINGTABLE str_DataSeparatorFailure[]    = "Disk data separator board missing or malfunctioning (check WCLOCK).";
STRINGTABLE str_DriveSeekFailure[]        = "Drive is not seeking properly!"; 
STRINGTABLE str_DriveMicrostepFailure[]   = "Drive is not microstepping!"; 
STRINGTABLE str_OutOfMemory[]             = "Memory allocation error!";

// disk operation status messages
STRINGTABLE str_CHSInfo[]                 = "\nCylinder: %-4u Head: %-2u Sector: %-3u - ";
STRINGTABLE str_CHInfo[]                  = "\nCylinder: %-4u Head: %-2u - ";
STRINGTABLE str_StatusTimeout[]           = "Operation timed out";
STRINGTABLE str_StatusNotReady[]          = "Disk not ready";
STRINGTABLE str_StatusWriteFault[]        = "Disk write fault signal";  
STRINGTABLE str_StatusNoSectorID[]        = "Sector not found";
STRINGTABLE str_StatusNoDataID[]          = "No data address mark";
STRINGTABLE str_StatusDataError[]         = "CRC mismatch/data error";
STRINGTABLE str_StatusDataCorrected[]     = "Corrected data error";

// SD card
STRINGTABLE str_SdNotPresent[]            = "No memory card present\n";
STRINGTABLE str_SdErrorFS[]               = "Memory card or filesystem error\n";
STRINGTABLE str_SdErrorFull[]             = "Memory card likely full";
STRINGTABLE str_SdErrorEndOfFile[]        = "Reached end-of-file";
STRINGTABLE str_SdDirListing[]            = "\nMemory card root directory (filter: *%s):\n\n";
STRINGTABLE str_SdDirPickRead[]           = "\nName of %s file to read: ";
STRINGTABLE str_SdDirPickWrite[]          = "\nName of %s file to write: ";
STRINGTABLE str_SdDirInvalid[]            = "File or path not found\r\n";
STRINGTABLE str_SdDirOverwrite[]          = "File already exists, overwrite? Y/N: ";
STRINGTABLE str_SdFileError[]             = "Path to the given filename does not exist";

// analyze command
STRINGTABLE str_AnalyzeNoSectors[]        = "No valid sectors read";
STRINGTABLE str_AnalyzeRelocated[]        = "Relocated to cylinder %u head %u";
STRINGTABLE str_AnalyzeSpt[]              = "%u sector(s), ";
STRINGTABLE str_AnalyzeSectorSize[]       = "%u bytes each, ";
STRINGTABLE str_AnalyzeVarSectorSize[]    = "VARIABLE SIZE!, ";
STRINGTABLE str_AnalyzeInterleave[]       = "%u:1 interleave ";
STRINGTABLE str_AnalyzeBadInterleave[]    = "unknown interleave ";
STRINGTABLE str_AnalyzeSectorOrder[]      = "\nOrder: ";
STRINGTABLE str_AnalyzeCylHdNormal[]      = "ID field cylinder and head numbers match seek position.\n";
STRINGTABLE str_AnalyzeConstSsize[]       = "Sector sizes inside single tracks are consistent.\n";
STRINGTABLE str_AnalyzeWarning[]          = "Warning(s):\n";
STRINGTABLE str_AnalyzeCylMismatch[]      = "*: ID field cylinder differs from the physical cylinder\n";
STRINGTABLE str_AnalyzeHdMismatch[]       = "@: ID field head differs from the physical head\n";
STRINGTABLE str_AnalyzeVarSsize[]         = "Variable sector size detected inside tracks!\n";

// hexdump command
STRINGTABLE str_HexdumpNote[]             = "\nAssumes ID field cylinder and head numbers match seek position.";
STRINGTABLE str_HexdumpDump[]             = "Dump:\n";
STRINGTABLE str_HexdumpOk[]               = "Read OK";

// read/verify command
STRINGTABLE str_ReadWholeDisk[]           = "Read whole disk? Y/N: ";
STRINGTABLE str_ReadAnalyze[]             = "\nAnalyzing ";
STRINGTABLE str_ReadAnalyzeTrack0[]       = "track 0:";
STRINGTABLE str_ReadAnalyzeFirstTrack[]   = "first track of given range:";
STRINGTABLE str_ReadExpectedSpt1[]        = "\n\nChoose how many sectors to read each track.\n";
STRINGTABLE str_ReadExpectedSpt2[]        = "Unreadable sectors and tracks will be zero-padded in the output file.\n";
STRINGTABLE str_ReadExpectedSpt3[]        = "Enter 0 only for non-uniform disk formats, where this may vary each track.\n";
STRINGTABLE str_ReadSavingNoInterleave[]  = "Saving output file with 1:1 interleave.\n";
STRINGTABLE str_ReadSM1040LogicalDrive1[] = "SM1040: %s logical drive %u ends at file offset %08lX\n";
STRINGTABLE str_ReadSM1040LogicalDrive2[] = "SM1040: %s logical drive %u ends at cylinder %u head %u\n";

// format/write command
STRINGTABLE str_WriteWholeDisk[]          = "%s whole disk? Y/N: ";
STRINGTABLE str_WriteParameters[]         = "\nEnter format parameters:\n";
STRINGTABLE str_WriteVerify[]             = "Verify during %s? Y/N: ";
STRINGTABLE str_WriteSMDriveType[]        = "\nOne logical drive will be created.\nSelect type: RK0(6) / RK0(7): ";
STRINGTABLE str_WriteDetails[]            = "\nWill format %u cylinders and write %lu bytes from file to disk.\n";
STRINGTABLE str_WriteDetailsFormat[]      = "\nWill format %u cylinders.\n";

// microstepping command
STRINGTABLE str_MicrostepDescription[]    = "\nReuses disk control cable pin 2 (/HDSEL3) to enable /RECOVERYMODE.\n"
                                            "Use this to mitigate disk data recording that has drifted over time:\n"
                                            "wire /HDSEL3 to the recovery mode signal of the disk connector.\n"
                                            "Most likely, a jumper will need to be set for proper operation.\n";
STRINGTABLE str_MicrostepCount[]          = "Microsteps to execute before reads (0-%u, 0: disable): ";
STRINGTABLE str_MicrostepTesting[]        = "\nTesting recovery mode...";
STRINGTABLE str_MicrostepReseekOff[]      = "NOTE: re-seeking after sector errors turned off until board reset.\n";
STRINGTABLE str_MicrostepRWCInUse[]       = "\nCannot reuse /HDSEL3 if reduced write current signal is used.\n";
STRINGTABLE str_MicrostepTooManyHeads[]   = "\nCannot reuse /HDSEL3; %u disk drive heads configured.\n";
STRINGTABLE str_MicrostepOff[]            = "OFF";
STRINGTABLE str_MicrostepSteps[]          = "%u STEPS";

// mount DOS partition command
STRINGTABLE str_DosTrack0Bad[]            = "No valid sectors read on track 0";
STRINGTABLE str_DosFsMountError[]         = "No primary DOS partition, not formatted or nonstandard bootsector";
STRINGTABLE str_DosDiskError[]            = "\rAborted due to disk error\n";  
STRINGTABLE str_DosFileNotFound[]         = "Not found in current path\n";
STRINGTABLE str_DosPathNotFound[]         = "Path not found\n";
STRINGTABLE str_DosDirectoryFull[]        = "Directory is not empty, or a file is read-only\n";
STRINGTABLE str_DosFileExists[]           = "Name already exists\n";
STRINGTABLE str_DosFsError[]              = "Filesystem error\n";  
STRINGTABLE str_DosInvalidName[]          = "Provide 1 valid file name";
STRINGTABLE str_DosInvalidDirName[]       = "Provide 1 valid directory name";
STRINGTABLE str_DosMaxPath[]              = "Path too deep";
STRINGTABLE str_DosInvalidCommand[]       = "Unrecognized command";
STRINGTABLE str_DosDirectory[]            = "          [DIR] ";
STRINGTABLE str_DosDirectoryEmpty[]       = "No files";
STRINGTABLE str_DosBytesFree[]            = "bytes free on disk.\n";  
STRINGTABLE str_DosTypeInto[]             = "Type two empty newlines to quit\n";
STRINGTABLE str_DosMounted[]              = "%u MB partition mounted.\n\n";
STRINGTABLE str_DosCommands[]             = "Supported commands:\nCD, DIR, MKDIR, RMDIR, DEL, HEXDUMP, TYPE, TYPEINTO, EXIT.\n\n";

// raw disk command
STRINGTABLE str_RawdiskAveraging[]        = "\rAveraging... %.f %% ";
STRINGTABLE str_RawdiskAveraged[]         = "\rRaw MFM/RLL track bitstream length for this disk, on average: %lu %s\n";
STRINGTABLE str_RawdiskCustomTrackLen[]   = "Bytes to use each track operation (0: use above): ";
STRINGTABLE str_RawdiskCustomTrackOver[]  = "\nMust be below 64K\n\n";
STRINGTABLE str_RawdiskDescription[]      = "\nSamples RDATA / outputs WDATA every RCLOCK / WCLOCK edge (falling and rising),"
                                            "\nfrom INDEX until %lu bytes per track are reached."
                                            "\nReduced write current and write precompensation applied as configured.\n";
STRINGTABLE str_RawdiskWriteWarning[]     = "\nBe careful with this function, this is experimental.\n"
                                            "Make sure the same track length (%u bytes), cylinders (%u) and heads (%u)\n"
                                            "match the file. Otherwise, sectors or entire tracks may drift or overlap.\n\n";                                            
STRINGTABLE str_RawdiskMenu[]             = "\n(R)ead disk into raw %s file\n"
                                            "(W)rite disk from raw %s file\n"
                                            "(B)ack to the main menu\n";
STRINGTABLE str_RawdiskProgress[]         = "%s file offset %08lX";

// erase command
STRINGTABLE str_ErasePrompt[]             = "\nAll sectors and data on disk will be obliterated."
                                            "\nEnter 'yes' to proceed, Esc or any other prompt quits: ";
STRINGTABLE str_EraseComplete[]           = "\nDisk purged\n";

// seektest command
STRINGTABLE str_SeektestLegacy[]          = "Use buffered drive seeking to offer butterfly seek tests.\n";
STRINGTABLE str_SeektestRepeats[]         = "\nEnter number of repetitions for each test, 0: skip:\n";
STRINGTABLE str_SeektestProgress[]        = "\nNow testing in between cylinders %u to %u:\n";
STRINGTABLE str_SeektestBackForth[]       = "Back and forth seeks";
STRINGTABLE str_SeektestButterfly[]       = "Full butterfly tests";
STRINGTABLE str_SeektestRandom[]          = "Random seeks";

// park command
STRINGTABLE str_ParkSuccess[]             = "\rDrive heads sent to landing zone cylinder %u.\n";
STRINGTABLE str_ParkPowerdownSafe[]       = "After powerdown, it is safe to relocate the drive.\n";
STRINGTABLE str_ParkContinue[]            = "\nOr, press any key to resume working with the drive.\n";
STRINGTABLE str_ParkRecalibrating[]       = "Recalibrating, please wait...";

// autodetect command
STRINGTABLE str_DetectFormat[]            = "\rDetected format: ";
STRINGTABLE str_DetectLikely[]            = "likely ";
STRINGTABLE str_DetectUnknown[]           = "unknown";
STRINGTABLE str_DetectBits[]              = "%u bits";
STRINGTABLE str_DetectCRC[]               = "\nData field CRC width: ";
STRINGTABLE str_DetectHeadSelect[]        = "\nHead select width: ";
STRINGTABLE str_DetectDataFieldXebec[]    = "\nData fields start byte: ";
STRINGTABLE str_DetectNoFormat[]          = "\rNo supported format detected.\n"
                                            "Use the Raw disk operations menu to dump the raw bitstream to file.\n";