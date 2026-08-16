// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// SD wrapper

#pragma once
#include "config.h"

bool sdDetect();
bool sdFilePicker(bool writeOperation, const char* extension = NULL);
bool sdSeekFile(size_t offset);
size_t sdGetSeekPos();
size_t sdGetFileSize();
void sdCloseFile();
bool sdReadFile(void* buffer, size_t bytesCount, size_t* bytesSuccessful);
bool sdWriteFile(const void* buffer, size_t bytesCount, size_t* bytesSuccessful);
bool sdIsEndOfFile();

