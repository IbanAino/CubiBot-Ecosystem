#ifndef TIME_SYSTEM_H
#define TIME_SYSTEM_H

#include <Arduino.h>

// Déclaration de la fonction accessible partout
uint64_t get_system_time_ms();

void set_system_time_ms(uint64_t unix_time_ms);

#endif
