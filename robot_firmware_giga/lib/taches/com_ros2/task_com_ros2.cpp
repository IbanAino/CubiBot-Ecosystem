#include "task_com_ros2.h"
#include "robot_config.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

int debugCounter2 = 0;
int lastAngle = 0;

// Configuration réseau
const char* ssid       = WIFI_SSID;
const char* password   = WIFI_PASS;
const char* server_ip  = "192.168.1.21";

// Ports UDP séparés par type de données
const uint16_t PORT_TELEMETRE = 5005;
const uint16_t PORT_LIDAR     = 5006;
const uint16_t PORT_ODOMETRIE = 5007;
const uint16_t PORT_COMMANDES = 5010;


WiFiUDP udp_telemetre;
WiFiUDP udp_lidar;
WiFiUDP udp_odometrie;
WiFiUDP udp_commandes;


PacketTelemeter local_packet;
lidar::Data     local_lidar;
LidarBatch batchLocal;
OdomData local_odom;

// ---------------------------------------------------------------------------
// Structure binaire envoyée sur le réseau pour un point lidar
// Compact : angle (float 4B) + distance (uint16 2B) + intensity (uint8 1B)
// = 7 octets par point × 12 points = 84 octets par trame
// ---------------------------------------------------------------------------
struct __attribute__((packed)) LidarPoint_UDP {
    float    angle;
    uint16_t distance;
    uint8_t  intensity;
};

struct __attribute__((packed)) PacketLidar {
    uint16_t      timestamp;
    uint16_t      speed;
    LidarPoint_UDP points[lidar::POINT_PER_PACK];
};

//Structure du paquet batch envoyé en un seul UDP
struct __attribute__((packed)) PacketLidarBatch {
    uint8_t      frameCount;                        // nombre de trames dans ce paquet
    PacketLidar  frames[MAX_FRAMES_PER_BATCH]; // trames concaténées
};

struct __attribute__((packed)) PacketOdometrie {
    float x;
    float y;
    float theta;
    float linearVel;
    float angularVel;
};

struct __attribute__((packed)) PacketCommande {
    float linearVel;
    float angularVel;
};

// struct __attribute__((packed)) CmdVel {
//     float linearVel;
//     float angularVel;
// };

// ---------------------------------------------------------------------------
// Reconnexion WiFi
// ---------------------------------------------------------------------------
static void connectWiFi()
{
    Serial.println("[Réseau] Try to connect WiFi"); // Do not delete, force the serial buffer allocation before wifi calling
	WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        rtos::ThisThread::sleep_for(500ms);
    }
    Serial.println("[Réseau] Wi-Fi connecté.");
}

// ---------------------------------------------------------------------------
// Tâche
// ---------------------------------------------------------------------------
void task_com_ros2()
{
    auto prochain_reveil = rtos::Kernel::Clock::now();

    connectWiFi();

    udp_telemetre.begin(PORT_TELEMETRE);
    udp_lidar.begin(PORT_LIDAR);
	udp_odometrie.begin(PORT_ODOMETRIE);
	udp_commandes.begin(PORT_COMMANDES);

    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            connectWiFi();
        }

		// ---------------------------
		// --- Récpetion commandes ---
		// ---------------------------
		
		int packetSize = udp_commandes.parsePacket();

		if (packetSize == sizeof(PacketCommande)) {

			PacketCommande packet_commande;

			udp_commandes.read(
				(uint8_t*)&packet_commande,
				sizeof(PacketCommande)
			);

			mutex_cmd_vel.lock();

			cmd_vel_partagee.linearVel  = packet_commande.linearVel;
			cmd_vel_partagee.angularVel = packet_commande.angularVel;

			mutex_cmd_vel.unlock();

			Serial.print("[ROS2] Commande : ");
			Serial.print("linearVel=");
			Serial.print(packet_commande.linearVel, 3);
			Serial.print(" m/s | angularVel=");
			Serial.print(packet_commande.angularVel, 3);
			Serial.println(" rad/s");

		}
		else if (packetSize > 0) {

			// Paquet reçu mais de taille incorrecte
			Serial.print("[ROS2] Paquet commande incorrect : ");
			Serial.print(packetSize);
			Serial.print(" octets reçus, ");
			Serial.print(sizeof(PacketCommande));
			Serial.println(" attendus");

			// Vider le paquet incorrect
			while (udp_commandes.available() > 0) {
				udp_commandes.read();
			}
		}




		// -------------------------
        // --- Paquet télémétrie ---
		// -------------------------
		
        mutex_telemeter.lock();
        local_packet = telemeter_partagee;
        mutex_telemeter.unlock();

        mutex_batterie.lock();
        local_packet.tension_batterie = batterie_tension_partagee;
        mutex_batterie.unlock();

		// --- Paquet telemetre ---
        udp_telemetre.beginPacket(server_ip, PORT_TELEMETRE);
        udp_telemetre.write((uint8_t*)&local_packet, sizeof(PacketTelemeter));
        udp_telemetre.endPacket();


        // --- Paquet lidar ---
        // mutex_lidar.lock();
        // local_lidar = lidar_data_partagee;
        // mutex_lidar.unlock();

		// mutex_lidar_batch.lock();
		// batchLocal = lidar_batch_partagee;
		// mutex_lidar_batch.unlock();

        // PacketLidar packet_lidar;

        // packet_lidar.timestamp = batchLocal.frames[0].timestamp;
        // packet_lidar.speed     = batchLocal.frames[0].speed;

        // for (uint8_t i = 0; i < lidar::POINT_PER_PACK; i++) {
        //     //packet_lidar.points[i].angle     = local_lidar.points[i].angle;
		// 	packet_lidar.points[i].angle = static_cast<float>(batchLocal.frames[0].points[i].angle) / 100.0f;
        //     packet_lidar.points[i].distance  = batchLocal.frames[0].points[i].distance;
        //     packet_lidar.points[i].intensity = batchLocal.frames[0].points[i].intensity;
        // }

        // udp_lidar.beginPacket(server_ip, PORT_LIDAR);
        // udp_lidar.write(
		// 	(uint8_t*)&packet_lidar,
		// 	sizeof(PacketLidar)
		// );
        // udp_lidar.endPacket();

		// Serial.print("[ROS2] point 0 : ");
		// Serial.print(local_lidar.points[0].angle);
		// Serial.print(",");
		// Serial.println(local_lidar.points[0].distance);

		// Serial.print("[ROS2] point 11 : ");
		// Serial.print(local_lidar.points[11].angle);
		// Serial.print(",");
		// Serial.println(local_lidar.points[11].distance);


		/*
		mutex_lidar_batch.lock();
		batchLocal = lidar_batch_partagee;
		mutex_lidar_batch.unlock();

		for (uint8_t frameIndex = 0; frameIndex < batchLocal.count; frameIndex++) {

			PacketLidar packet_lidar;

			packet_lidar.timestamp = batchLocal.frames[frameIndex].timestamp;
			packet_lidar.speed     = batchLocal.frames[frameIndex].speed;

			for (uint8_t i = 0; i < lidar::POINT_PER_PACK; i++) {

				packet_lidar.points[i].angle =
					static_cast<float>(
						batchLocal.frames[frameIndex].points[i].angle
					) / 100.0f;

				packet_lidar.points[i].distance =
					batchLocal.frames[frameIndex].points[i].distance;

				packet_lidar.points[i].intensity =
					batchLocal.frames[frameIndex].points[i].intensity;
			}

			udp_lidar.beginPacket(server_ip, PORT_LIDAR);

			udp_lidar.write(
				(uint8_t*)&packet_lidar,
				sizeof(PacketLidar)
			);

			udp_lidar.endPacket();
		}
		*/






		
		// mutex_lidar_batch.lock();
		// batchLocal = lidar_batch_partagee;
		// mutex_lidar_batch.unlock();
		/*
		mutex_lidar_batch.lock();
		if (!lidar_batch_queue.empty()) {
			batchLocal = lidar_batch_queue.back();  // consomme le plus ancien
			lidar_batch_queue.pop_back();
		}
		mutex_lidar_batch.unlock();




		debugCounter2 += batchLocal.count;
		Serial.print("[task_com_ros2] Trames sortantes du batch : ");
		Serial.println(debugCounter2);

		if (batchLocal.count == 0) {
			// Rien à envoyer
		} else {
			PacketLidarBatch packetBatch;
			packetBatch.frameCount = batchLocal.count;


			for (uint8_t frameIndex = 0; frameIndex < batchLocal.count; frameIndex++) {

				packetBatch.frames[frameIndex].timestamp =
					batchLocal.frames[frameIndex].timestamp;
				packetBatch.frames[frameIndex].speed =
					batchLocal.frames[frameIndex].speed;

				for (uint8_t i = 0; i < lidar::POINT_PER_PACK; i++) {
					packetBatch.frames[frameIndex].points[i].angle =
						static_cast<float>(
							batchLocal.frames[frameIndex].points[i].angle
						) / 100.0f;
					packetBatch.frames[frameIndex].points[i].distance =
						batchLocal.frames[frameIndex].points[i].distance;
					packetBatch.frames[frameIndex].points[i].intensity =
						batchLocal.frames[frameIndex].points[i].intensity;
				}
			}











			// Un seul envoi UDP pour toutes les trames du batch
			const size_t payloadSize =
				sizeof(uint8_t) +                          // frameCount
				batchLocal.count * sizeof(PacketLidar);    // trames effectives seulement

			udp_lidar.beginPacket(server_ip, PORT_LIDAR);
			udp_lidar.write((uint8_t*)&packetBatch, payloadSize);
			udp_lidar.endPacket();



		}
		*/





		static lidar::Data tramesLocales[MAX_FRAMES_PER_BATCH];
		mutex_lidar_frames.lock();
		uint8_t nbTrames = lidar_frames_queue_partagee.size();

		if (nbTrames != 0){
			for (uint8_t i = 0; i < nbTrames; i++) {
				tramesLocales[i] = lidar_frames_queue_partagee.front();
				lidar_frames_queue_partagee.pop_front();
			}

			lidar_frames_queue_partagee.clear();
		}
		
		mutex_lidar_frames.unlock();


		// for (uint8_t i = 0; i < nbTrames; i++) {
		// 	// Serial.print(tramesLocales[i].startAngle / 100.0);
		// 	// Serial.print("  ->  ");
		// 	// Serial.println(tramesLocales[i].endAngle / 100.0);
		// 	Serial.println((tramesLocales[i].startAngle - lastAngle) / 100.0);
		// 	lastAngle = tramesLocales[i].endAngle;
		// }

		if (nbTrames != 0){
			// LidarBatch batchLocalForUDP;
			// batchLocalForUDP.count = nbTrames;

			// for (uint8_t i = 0; i < nbTrames; i++) {
			// 	batchLocalForUDP.frames[i] = tramesLocales[i];
			// }

			// const size_t payloadSize =
			// 	sizeof(uint8_t) +                          // frameCount
			// 	batchLocalForUDP.count * sizeof(PacketLidar);    // trames effectives seulement

			// Serial.println("---");
			// Serial.println(batchLocalForUDP.count);
			// for(uint8_t i = 0; i < nbTrames; i++){
			// 	Serial.println(batchLocalForUDP.frames[i].startAngle);
			// 	Serial.println(batchLocalForUDP.frames[i].points[0].distance);
			// }


			// 2. Affichage de la taille dans le moniteur série
			// Serial.print("Taille du paquet UDP envoyé : ");
			// Serial.print(payloadSize);
			// Serial.println(" octets");

			// udp_lidar.beginPacket(server_ip, PORT_LIDAR);
			// udp_lidar.write((uint8_t*)&batchLocalForUDP, payloadSize);
			// udp_lidar.endPacket();

			// udp_lidar.beginPacket(server_ip, PORT_LIDAR);
			// // 1. On envoie le premier octet (le count)
			// udp_lidar.write(&(batchLocalForUDP.count), sizeof(uint8_t));
			// // 2. On envoie uniquement les trames utiles
			// udp_lidar.write((uint8_t*)batchLocalForUDP.frames, nbTrames * sizeof(PacketLidar));
			// udp_lidar.endPacket();

			for (uint8_t i = 0; i < nbTrames; i++) {
				Serial.print(tramesLocales[i].timestamp);
				Serial.print(" - ");
				Serial.print(tramesLocales[i].speed);
				Serial.print(" - ");
				Serial.print(tramesLocales[i].startAngle);
				Serial.print(" - ");
				Serial.print(tramesLocales[i].endAngle);
				Serial.print(" --> ");
				for (uint8_t y = 0; y < nbTrames; y++) {
					Serial.print(tramesLocales[i].points[y].distance);
					Serial.print("|");
				}
				Serial.println("");



				udp_lidar.beginPacket(server_ip, PORT_LIDAR);
				udp_lidar.write((uint8_t*)&tramesLocales[i], 44); // 44 = nombre d'octets envoyés
				udp_lidar.endPacket();
			}
		}


			// if (nbTrames > MAX_FRAMES_PER_BATCH) {
			// 	nbTrames = MAX_FRAMES_PER_BATCH;
			// }

			// for (uint8_t i = 0; i < nbTrames; i++) {
			// 	tramesLocales[i] = lidar_frames_queue_partagee.front();
			// 	lidar_frames_queue_partagee.pop_front();
			// }

			// lidar_frames_queue_partagee.clear();
			// mutex_lidar_frames.unlock();

			// LidarBatch batchLocalForUDP;
			// batchLocalForUDP.count = nbTrames;

			// for (uint8_t i = 0; i < nbTrames; i++) {
			// 	batchLocalForUDP.frames[i] = tramesLocales[i];
			// }

			// const size_t payloadSize =
			// 	sizeof(uint8_t) +                          // frameCount
			// 	batchLocalForUDP.count * sizeof(PacketLidar);    // trames effectives seulement

			// udp_lidar.beginPacket(server_ip, PORT_LIDAR);
			// udp_lidar.write((uint8_t*)&batchLocalForUDP, payloadSize);
			// udp_lidar.endPacket();




		//}
		//mutex_lidar_frames.unlock();


		//Serial.println(packetBatch.frameCount);
			
		











		// ------------------------
		// --- Paquet odométrie ---
		//-------------------------

		mutex_odom.lock();
		local_odom = odom_partagee;
		mutex_odom.unlock();

		PacketOdometrie packet_odom;

		packet_odom.x          = local_odom.x;
		packet_odom.y          = local_odom.y;
		packet_odom.theta      = local_odom.theta;
		packet_odom.linearVel  = local_odom.linearVel;
		packet_odom.angularVel = local_odom.angularVel;

		udp_odometrie.beginPacket(server_ip, PORT_ODOMETRIE);
		udp_odometrie.write(
			(uint8_t*)&packet_odom,
			sizeof(PacketOdometrie)
		);
		udp_odometrie.endPacket();

		// --- Debug odométrie ---
		// Serial.print("[ROS2] Odom : ");
		// Serial.print("x=");
		// Serial.print(local_odom.x, 3);
		// Serial.print(" m | y=");
		// Serial.print(local_odom.y, 3);
		// Serial.print(" m | theta=");
		// Serial.print(local_odom.theta, 3);
		// Serial.print(" rad | linearVel=");
		// Serial.print(local_odom.linearVel, 3);
		// Serial.print(" m/s | angularVel=");
		// Serial.print(local_odom.angularVel, 3);
		// Serial.println(" rad/s");

		prochain_reveil += PERIODE_COM;
        rtos::ThisThread::sleep_until(prochain_reveil);
    }
}