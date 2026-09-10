#include "robot_config.h"


float batterie_tension_partagee = 12.0f;
PacketTelemeter telemeter_partagee = {12.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

rtos::Mutex mutex_batterie;
rtos::Mutex mutex_telemeter;
rtos::Mutex    mutex_lidar;
lidar::Data    lidar_data_partagee;
