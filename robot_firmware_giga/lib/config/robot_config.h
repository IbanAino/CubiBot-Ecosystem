#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include <rtos.h>
#include <chrono>

#include "LidarTypes.h"

using namespace std::chrono_literals;

// --- CADENCEMENT TEMPOREL ---
const std::chrono::milliseconds PERIODE_MOTEURS = 20ms;   // 1000 Hz
const std::chrono::milliseconds PERIODE_PERCEPT = 20ms;  // 50 Hz
const std::chrono::milliseconds PERIODE_COM     = 50ms;  // 20 Hz (Fréquence d'envoi réseau)

// --- STRUCTURE DU PAQUET ENVOYÉ AU SERVEUR ---
// Cette structure fait exactement 24 octets en tout. C'est ultra-léger !
struct __attribute__((packed)) PacketTelemeter {
    float tension_batterie; // 4 octets
    float position_x;       // 4 octets
    float position_y;       // 4 octets
    float orientation_th;   // 4 octets
    float vitesse_lineaire; // 4 octets
    float vitesse_angulaire;// 4 octets
};

// --- VARIABLES PARTAGÉES (Inter-tâches) ---
extern float batterie_tension_partagee;
extern PacketTelemeter telemeter_partagee;

// --- MUTEX DE PROTECTION ---
extern rtos::Mutex mutex_batterie;
extern rtos::Mutex mutex_telemeter;
extern rtos::Mutex    mutex_lidar;
extern lidar::Data    lidar_data_partagee;

// --- CONSTANTES DU ROBOT (TEST) ---
const uint16_t TICKS_PER_REV = 2770;

// --- BROCHES MATÉRIELLES MOTEUR GAUCHE ---
const uint8_t PIN_LEFT_MOTOR_EN  = 4; // 3
const uint8_t PIN_LEFT_MOTOR_IN1 = 32; // 36
const uint8_t PIN_LEFT_MOTOR_IN2 = 34; // 30

const uint8_t PIN_LEFT_ENCODER_A = 35; // 31
const uint8_t PIN_LEFT_ENCODER_B = 37; //33

#endif
