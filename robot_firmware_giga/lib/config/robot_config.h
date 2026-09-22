#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include <rtos.h>
#include <chrono>
#include <deque>

#include "LidarTypes.h"
#include "DifferentialOdometry.h"


using namespace std::chrono_literals;

// --- CADENCEMENT TEMPOREL ---
const std::chrono::milliseconds PERIODE_MOTEURS = 20ms;   // 1000 Hz
const std::chrono::milliseconds PERIODE_PERCEPT = 20ms;  // 50 Hz
const std::chrono::milliseconds PERIODE_COM     = 50ms;  // 20 Hz (Fréquence d'envoi réseau)
const std::chrono::milliseconds PERIODE_CONTROL = 20ms;  // 50 Hz

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


static constexpr float WHEEL_RADIUS_METERS = 0.0425f; // 4.3 cm diameter
static constexpr float WHEEL_BASE_METERS   = 0.088f;

static constexpr size_t MAX_FRAMES_PER_BATCH = 20; // même taille que lidarTypes.h/MAX_FRAME_QUEUE_SIZE

struct LidarBatch {
    uint8_t     count;   // nombre de trames valides dans frames[]
    lidar::Data frames[MAX_FRAMES_PER_BATCH];
};


extern rtos::Mutex  mutex_lidar_batch;
extern LidarBatch   lidar_batch_partagee;

extern rtos::Mutex              mutex_lidar_batch;
extern std::deque<LidarBatch>   lidar_batch_queue;
static constexpr size_t         MAX_LIDAR_QUEUE = 20;


extern rtos::Mutex              mutex_lidar_frames;
extern std::deque<lidar::Data>  lidar_frames_queue_partagee;
static constexpr size_t         MAX_LIDAR_FRAMES_QUEUE = 20;




// --- CONSTANTES DU ROBOT (TEST) ---
const uint16_t TICKS_PER_REV = 2770;



static constexpr size_t MAX_FRAME_QUEUE_SIZE = 10;

// --- BROCHES MATÉRIELLES MOTEUR GAUCHE ---
const uint8_t PIN_LEFT_MOTOR_EN  = 4; // 3
const uint8_t PIN_LEFT_MOTOR_IN1 = 32; // 36
const uint8_t PIN_LEFT_MOTOR_IN2 = 34; // 30

const uint8_t PIN_LEFT_ENCODER_A = 35; // 31
const uint8_t PIN_LEFT_ENCODER_B = 37; //33

// Mother Board V1 :
// static const uint8_t R_IN4 = 34;
// static const uint8_t R2_C1 = 35;
// static const uint8_t R_EN1 = 3;
// static const uint8_t R2_C2 = 37;
// static const uint8_t R_IN1 = 36;
// static const uint8_t R1_C1 = 31;
// static const uint8_t R_IN3 = 32;
// static const uint8_t R1_C2 = 33;
// static const uint8_t R_EN2 = 4;
// static const uint8_t R_IN2 = 30;

// static const uint8_t L_IN4 = 29;
// static const uint8_t L2_C1 = 25;
// static const uint8_t L_EN1 = 5;
// static const uint8_t L2_C2 = 27;
// static const uint8_t L_IN1 = 28;
// static const uint8_t L1_C1 = 22;
// static const uint8_t L_IN3 = 26;
// static const uint8_t L1_C2 = 24;
// static const uint8_t L_EN2 = 6;
// static const uint8_t L_IN2 = 23;

// Mother Board V2 :
static const uint8_t R_IN4 = 38;
static const uint8_t R2_C1 = 23; // 39 = BUUUUG !!!
static const uint8_t R_EN1 = 3;
static const uint8_t R2_C2 = 41;
static const uint8_t R_IN1 = 40;
static const uint8_t R1_C1 = 35;
static const uint8_t R_IN3 = 36;
static const uint8_t R1_C2 = 37;
static const uint8_t R_EN2 = 4;
static const uint8_t R_IN2 = 34;

static const uint8_t L_IN4 = 33;
static const uint8_t L2_C1 = 29;
static const uint8_t L_EN1 = 5;
static const uint8_t L2_C2 = 31;
static const uint8_t L_IN1 = 32;
static const uint8_t L1_C1 = 26;
static const uint8_t L_IN3 = 30;
static const uint8_t L1_C2 = 28;
static const uint8_t L_EN2 = 6;
static const uint8_t L_IN2 = 27;

static const uint8_t LIDAR_EN = 53;

// ------------------------------------------------------------------
// Pose partagée (produite par task_control, consommée par task_com_ros2)
// ------------------------------------------------------------------
 
struct __attribute__((packed)) OdomData {
	uint64_t timestamp;
    float x;             // mètres
    float y;             // mètres
    float theta;         // radians, normalisé [-π, π]
    float linearVel;     // m/s
    float angularVel;    // rad/s
};
 
extern rtos::Mutex mutex_odom;
extern OdomData    odom_partagee;
 
// ------------------------------------------------------------------
// Consigne de vitesse partagée
// (produite par task_com_ros2, consommée par task_control)
// ------------------------------------------------------------------
 
struct CmdVel {
    float linearVel;   // m/s
    float angularVel;  // rad/s
};
 
extern rtos::Mutex mutex_cmd;
extern CmdVel      cmd_partagee;

extern rtos::Mutex mutex_cmd_vel;
extern CmdVel cmd_vel_partagee;


#endif
