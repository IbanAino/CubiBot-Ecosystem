#include "task_com_ros2.h"
#include "robot_config.h"
#include <WiFi.h>
#include <WiFiUdp.h>

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

		// --- Récpetion commandes ---
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





        // --- Paquet télémétrie ---
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
        mutex_lidar.lock();
        local_lidar = lidar_data_partagee;
        mutex_lidar.unlock();


		// Serial.print("[ROS2] point 0 : ");
		// Serial.print(local_lidar.points[0].angle);
		// Serial.print(",");
		// Serial.println(local_lidar.points[0].distance);

		// Serial.print("[ROS2] point 11 : ");
		// Serial.print(local_lidar.points[11].angle);
		// Serial.print(",");
		// Serial.println(local_lidar.points[11].distance);


        PacketLidar packet_lidar;
        packet_lidar.timestamp = local_lidar.timestamp;
        packet_lidar.speed     = local_lidar.speed;

        for (uint8_t i = 0; i < lidar::POINT_PER_PACK; i++) {
            //packet_lidar.points[i].angle     = local_lidar.points[i].angle;
			packet_lidar.points[i].angle = static_cast<float>(local_lidar.points[i].angle) / 100.0f;
            packet_lidar.points[i].distance  = local_lidar.points[i].distance;
            packet_lidar.points[i].intensity = local_lidar.points[i].intensity;
        }

        udp_lidar.beginPacket(server_ip, PORT_LIDAR);
        udp_lidar.write(
			(uint8_t*)&packet_lidar,
			sizeof(PacketLidar)
		);
        udp_lidar.endPacket();



		// --- Paquet odométrie ---
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