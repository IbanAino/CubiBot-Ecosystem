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

		//---------------------
		//--- Get one frame ---
		//---------------------
		/*
        lidar::Data data;
        if (lidarDriver.GetLastFrame(data)) {
            // Sécurisation de l'accès aux données lidar partagées

			// for (uint8_t i = 0; i < lidar::POINT_PER_PACK; ++i) {
			// 	Serial.print(data.points[i].angle);
			// 	Serial.print(",");
			// 	Serial.println(data.points[i].distance);
			// }



			// Serial.print("LIDAR DATA: ");
			// Serial.print("start angle =");
			// Serial.println(data.startAngle);

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
		*/

		//-----------------------------
		//--- Get all of the frames ---
		//-----------------------------
		
		// lidar::Data batch[MAX_FRAMES_PER_BATCH];

		// if (lidarDriver.GetFrames(batch, MAX_FRAMES_PER_BATCH)) {
        //     mutex_lidar_batch.lock();
        //     lidar_batch_partagee = batch;
        //     mutex_lidar_batch.unlock();
		// }

		// 1. Créer une structure LidarBatch locale sur la pile
		LidarBatch batchLocal;

		// 2. Remplir directement le tableau interne de la structure locale
		// batchLocal.frames se dégrade en pointeur (lidar::Data*) automatiquement
		batchLocal.count = lidarDriver.GetFrames(batchLocal.frames, MAX_FRAMES_PER_BATCH);

		// 3. Si on a récupéré des trames, on met à jour la variable partagée
		if (batchLocal.count > 0) {
			//std::lock_guard<std::mutex> lock(mutex_lidar_batch);
			mutex_lidar_batch.lock();
			lidar_batch_partagee = batchLocal;
			mutex_lidar_batch.unlock();
			// ICI LE SIGNE = FONCTIONNE PARFAITEMENT !
			// Le compilateur sait copier une structure complète d'un seul coup
			// --- LOGS ARDUINO ---
			// Serial.print(F("[LiDAR] Succès : "));
			// Serial.print((int)batchLocal.count);
			// Serial.println(F(" trame(s) récupérée(s) et partagée(s)."));

			// Affichage des détails de la première trame du batch à titre d'exemple
			// (Pensez à remplacer .id ou .distance par les vrais membres de lidar::Data)
			// Serial.print(F("        -> Première trame : timestamp = "));
			// Serial.print((int)batchLocal.frames[0].timestamp); 
			// Serial.print(F(" | first distance = "));
			// Serial.print(batchLocal.frames[0].points[0].distance);
			// Serial.println(F(" mm"));

			// Boucle sur chaque trame valide du batch
			// for (uint8_t i = 0; i < batchLocal.count; i++) {
			// 	Serial.print(F("  Trame #"));
			// 	Serial.print((int)i);
			// 	Serial.print(F(" -> "));

			// 	// /!\ Remplacer .distance et .angle par vos vrais membres de lidar::Data
			// 	Serial.print(F("Dist: "));
			// 	Serial.print(batchLocal.frames[i].points[0].distance);
			// 	Serial.print(F(" mm | Start Angle: "));
			// 	Serial.print(batchLocal.frames[i].startAngle);
			// 	Serial.print(F(" ° | Start Angle: "));
			// 	Serial.print(batchLocal.frames[i].endAngle);
			// 	Serial.println(F("°"));
			// }
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
