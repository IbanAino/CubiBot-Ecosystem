#include "time_system.h"

// uint64_t get_system_time_ms() {
//     static uint32_t last_millis = 0;
//     static uint64_t overflow_counter = 0;
    
//     // On bloque brièvement les interruptions pour garantir une lecture atomique de millis()
//     // et éviter qu'un changement de tâche RTOS n'intervienne au milieu du calcul.
//     noInterrupts(); 
//     uint32_t current_millis = millis();
    
//     if (current_millis < last_millis) {
//         overflow_counter++;
//     }
//     last_millis = current_millis;
    
//     uint64_t uptime = (overflow_counter << 32) + current_millis;
//     interrupts(); // Réactivation des interruptions
    
//     return uptime;
// }


// Variable globale volatile partagée
volatile uint64_t g_system_uptime_ms = 0;
static uint32_t last_millis = 0;

uint64_t get_system_time_ms() {
    noInterrupts(); // Protection de lecture atomique
    uint32_t current_millis = millis();
    
    // Gestion du débordement classique de millis()
    if (current_millis < last_millis) {
        g_system_uptime_ms += (0xFFFFFFFF - last_millis) + current_millis + 1;
    } else {
        g_system_uptime_ms += (current_millis - last_millis);
    }
    last_millis = current_millis;
    
    uint64_t current_time = g_system_uptime_ms;
    interrupts();
    
    return current_time;
}

// NOUVELLE FONCTION : Appelé par votre tâche Control lors de la réception du temps PC
void set_system_time_ms(uint64_t unix_time_ms) {
    noInterrupts(); // Section critique : on bloque les interruptions pour éviter une corruption
    g_system_uptime_ms = unix_time_ms;
    last_millis = millis(); // On réaligne le point de repère millis
    interrupts();
}
