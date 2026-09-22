#include "time_system.h"

uint64_t get_system_time_ms() {
    static uint32_t last_millis = 0;
    static uint64_t overflow_counter = 0;
    
    // On bloque brièvement les interruptions pour garantir une lecture atomique de millis()
    // et éviter qu'un changement de tâche RTOS n'intervienne au milieu du calcul.
    noInterrupts(); 
    uint32_t current_millis = millis();
    
    if (current_millis < last_millis) {
        overflow_counter++;
    }
    last_millis = current_millis;
    
    uint64_t uptime = (overflow_counter << 32) + current_millis;
    interrupts(); // Réactivation des interruptions
    
    return uptime;
}
