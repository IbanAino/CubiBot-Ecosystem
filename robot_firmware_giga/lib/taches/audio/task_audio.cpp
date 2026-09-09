#include "task_audio.h"
#include "robot_config.h"

void task_audio() {
  Serial.println("[Audio] Initialisation du matériel audio...");
  while (true) {
    rtos::ThisThread::sleep_for(10ms);
  }
}
