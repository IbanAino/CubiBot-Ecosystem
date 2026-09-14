#include "task_perception.h"
#include "robot_config.h"

lidar::Driver lidarDriver(Serial4);

void task_perception() {
  auto prochain_reveil = rtos::Kernel::Clock::now();
  Serial.println("[Perception] Initialisation du bus I2C et du LIDAR...");

  lidarDriver.begin();

  while (true) {
    // 1. Lecture du buffer du LIDAR 2D
        lidarDriver.process();

        lidar::Data data;
        if (lidarDriver.GetLastFrame(data)) {
            // Sécurisation de l'accès aux données lidar partagées

			// for (uint8_t i = 0; i < lidar::POINT_PER_PACK; ++i) {
			// 	Serial.print(data.points[i].angle);
			// 	Serial.print(",");
			// 	Serial.println(data.points[i].distance);
			// }



			// Serial.print("LIDAR DATA: ");
			// Serial.print("speed=");
			// Serial.print(data.speed);

			// Serial.print(" angles=");

			// for (uint8_t i = 0; i < lidar::POINT_PER_PACK; ++i) {
			// 	Serial.print(data.points[i].angle / 100.0f);

			// 	if (i < lidar::POINT_PER_PACK - 1) {
			// 		Serial.print(" -> ");
			// 	}
			// }

			// Serial.println();





            mutex_lidar.lock();
            lidar_data_partagee = data;
            mutex_lidar.unlock();
        }
    // 2. Interrogation séquentielle des lasers I2C
    
    // 3. Lecture analogique de la batterie
    float tension_lue = 11.8f; // Exemple de valeur simulée

    // On sécurise l'accès à la variable partagée avant d'écrire
    mutex_batterie.lock();
    batterie_tension_partagee = tension_lue;
    mutex_batterie.unlock();
    
    prochain_reveil += PERIODE_PERCEPT;
    rtos::ThisThread::sleep_until(prochain_reveil);
  }
}
