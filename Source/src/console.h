// Sentinel (c) 2026 J. Bogin, https://boginjr.com
// Console helpers

#pragma once
#include "config.h"

// console
void clear();
char readKey(const char* allowedKeys = NULL, bool withWait = true);
const char* prompt(uint8_t maximumPromptLen = 0, const char* allowedKeys = NULL, bool escReturnsNull = false);    
const char* getPromptBuffer();
void fatalError(const char* message);

