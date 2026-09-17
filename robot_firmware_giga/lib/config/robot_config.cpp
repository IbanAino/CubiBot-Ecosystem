#include "robot_config.h"


float batterie_tension_partagee = 12.0f;
PacketTelemeter telemeter_partagee = {12.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

rtos::Mutex mutex_batterie;
rtos::Mutex mutex_telemeter;
rtos::Mutex mutex_lidar;
lidar::Data lidar_data_partagee;

// Lidar : tableau ou liste de frames, (MAX_FRAME_QUEUE_SIZE)
rtos::Mutex  mutex_lidar_batch;
LidarBatch   lidar_batch_partagee;
std::deque<LidarBatch>   lidar_batch_queue;

rtos::Mutex mutex_lidar_frames;
std::deque<lidar::Data>   lidar_frames_queue_partagee;



rtos::Mutex mutex_cmd_vel;
CmdVel cmd_vel_partagee;


rtos::Mutex mutex_cmd;
CmdVel      cmd_partagee;

rtos::Mutex mutex_odom;
OdomData    odom_partagee;