#include "task_com_ros2.h"
#include "robot_config.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// Configuration réseau du serveur
const char* ssid = "VOTRE_SSID";
const char* password = "VOTRE_MOT_DE_PASSE";
const char* server_ip = "192.168.1.50"; // L'IP de votre PC ROS2
const uint16_t server_port = 5005;       // Port d'écoute UDP choisi

WiFiUDP udp_client;
PacketTelemeter local_packet; // Copie locale pour éviter de bloquer le mutex pendant l'envoi réseau

void task_com_ros2() {
  auto prochain_reveil = rtos::Kernel::Clock::now();
  
  // 1. Connexion au Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    rtos::ThisThread::sleep_for(500ms);
  }
  Serial.println("[Réseau] Wi-Fi Connecté avec succès !");

  // 2. Initialisation du protocole UDP local
  udp_client.begin(5006); // Port local d'écoute de l'Arduino

  while (true) {
    // Si la connexion Wi-Fi est toujours active
    if (WiFi.status() == WL_CONNECTED) {
      
      // 3. Récupération sécurisée des dernières données calculées par les autres threads
      mutex_telemeter.lock();
      local_packet = telemeter_partagee;
      mutex_telemeter.unlock();

      // Mettre à jour la batterie séparément si besoin
      mutex_batterie.lock();
      local_packet.tension_batterie = batterie_tension_partagee;
      mutex_batterie.unlock();

      // 4. Envoi du paquet binaire brut sur le réseau
      udp_client.beginPacket(server_ip, server_port);
      
      // On envoie directement la zone mémoire de la structure
      udp_client.write((uint8_t*)&local_packet, sizeof(PacketTelemeter));
      
      udp_client.endPacket();
    } else {
      // Tentative automatique de reconnexion en tâche de fond si coupure
      WiFi.begin(ssid, password);
    }

    prochain_reveil += PERIODE_COM;
    rtos::ThisThread::sleep_until(prochain_reveil);
  }
}
