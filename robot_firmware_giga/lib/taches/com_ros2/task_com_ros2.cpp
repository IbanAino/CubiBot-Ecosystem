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

        udp_telemetre.beginPacket(server_ip, PORT_TELEMETRE);
        udp_telemetre.write((uint8_t*)&local_packet, sizeof(PacketTelemeter));
        udp_telemetre.endPacket();




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





		// --------------------
        // --- Paquet lidar ---
		// --------------------

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
				udp_lidar.write((uint8_t*)&tramesLocales[i], 46); // 46 = nombre d'octets envoyés
				udp_lidar.endPacket();
			}
		}



    }
}