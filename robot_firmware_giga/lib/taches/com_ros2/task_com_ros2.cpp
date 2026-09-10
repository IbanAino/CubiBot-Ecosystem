#include "task_com_ros2.h"
#include "robot_config.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// Configuration réseau
const char* ssid       = WIFI_SSID;
const char* password   = WIFI_PASS;
const char* server_ip  = "192.168.1.14";

// Ports UDP séparés par type de données
const uint16_t PORT_TELEMETRE = 5005;
const uint16_t PORT_LIDAR     = 5006;
const uint16_t PORT_LOCAL     = 5007;

WiFiUDP udp_telemetre;
WiFiUDP udp_lidar;

PacketTelemeter local_packet;
lidar::Data     local_lidar;

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

// ---------------------------------------------------------------------------
// Reconnexion WiFi
// ---------------------------------------------------------------------------
static void connectWiFi()
{
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

    udp_telemetre.begin(PORT_LOCAL);
    // udp_lidar n'a pas besoin de port local d'écoute — émission uniquement

    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            connectWiFi();
        }

        // --- Paquet télémétrie ---
        mutex_telemeter.lock();
        local_packet = telemeter_partagee;
        mutex_telemeter.unlock();

        mutex_batterie.lock();
        local_packet.tension_batterie = batterie_tension_partagee;
        mutex_batterie.unlock();

        udp_telemetre.beginPacket(server_ip, PORT_TELEMETRE);
        udp_telemetre.write((uint8_t*)&local_packet, sizeof(PacketTelemeter));
        udp_telemetre.endPacket();

        // --- Paquet lidar ---
        mutex_lidar.lock();
        local_lidar = lidar_data_partagee;
        mutex_lidar.unlock();

        PacketLidar packet_lidar;
        packet_lidar.timestamp = local_lidar.timestamp;
        packet_lidar.speed     = local_lidar.speed;

        for (uint8_t i = 0; i < lidar::POINT_PER_PACK; i++) {
            packet_lidar.points[i].angle     = local_lidar.points[i].angle;
            packet_lidar.points[i].distance  = local_lidar.points[i].distance;
            packet_lidar.points[i].intensity = local_lidar.points[i].intensity;
        }

        udp_lidar.beginPacket(server_ip, PORT_LIDAR);
        udp_lidar.write((uint8_t*)&packet_lidar, sizeof(PacketLidar));
        udp_lidar.endPacket();

        prochain_reveil += PERIODE_COM;
        rtos::ThisThread::sleep_until(prochain_reveil);
    }
}