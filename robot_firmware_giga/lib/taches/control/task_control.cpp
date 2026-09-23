#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <rtos.h>

#include "task_com_ros2.h"
#include "robot_config.h"
#include "time_system.h"

const uint16_t PORT_COMMANDES_RX = 5010;
WiFiUDP udp_rx_commandes;

static uint8_t rx_buffer[64];

struct __attribute__((packed)) PacketCommande {
    float linearVel;
    float angularVel;
};

void task_control()
{
    while (WiFi.status() != WL_CONNECTED) {
        rtos::ThisThread::sleep_for(100ms);
    }

    udp_rx_commandes.begin(PORT_COMMANDES_RX);
    Serial.println("[Task Control] Écoute UDP activée sur le port 5010.");

    // CORRECTION : Déclaration d'un vrai tableau de stockage
    //uint8_t rx_buffer[64]; 

    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            rtos::ThisThread::sleep_for(500ms);
            continue;
        }

        int packetSize = udp_rx_commandes.parsePacket();

        if (packetSize > 0) {

			Serial.println("[Task Control] Packet reçu !!! ");

            // Sécurité : on borne la lecture à la taille de notre tableau
            int bytesToRead = (packetSize > 64) ? 64 : packetSize;
            
            // CORRECTION : On passe le tableau, ce qui fournit le bon pointeur à la fonction
            udp_rx_commandes.read(rx_buffer, bytesToRead);
            
            // Extraction de l'ID du premier octet
            uint8_t command_id = rx_buffer[0];

            // CAS 1 : Synchronisation Temporelle ROS2 (0xEE)
            if (command_id == 0xEE && bytesToRead >= 9) {
                uint64_t pc_unix_time_ms = 0;
                
                // CORRECTION : On lit à partir de l'index 1 (juste après le 0xEE)
                memcpy(&pc_unix_time_ms, &rx_buffer[1], sizeof(pc_unix_time_ms));
                
                set_system_time_ms(pc_unix_time_ms);
                
                Serial.print("[Task Control] Packet reçu !!! Horloge calée sur : ");
                Serial.println((unsigned long)(pc_unix_time_ms / 1000));
            }
            
            // CAS 2 : Commande de vitesse (8 octets bruts envoyés par Python '<ff')
            // Note : Comme votre Python actuel n'envoie PAS de header ID pour la vitesse, 
            // le paquet fait exactement 8 octets, et le premier float commence à l'index 0.
            else if (packetSize == sizeof(PacketCommande)) {
                PacketCommande packet_commande;
                
                // Copie directe de tout le buffer
                memcpy(&packet_commande, rx_buffer, sizeof(PacketCommande));

                mutex_cmd_vel.lock();
                cmd_vel_partagee.linearVel  = packet_commande.linearVel;
                cmd_vel_partagee.angularVel = packet_commande.angularVel;
                mutex_cmd_vel.unlock();

                Serial.print("[Task Control] Packet reçu !!! Vitesse Lin: ");
                Serial.print(packet_commande.linearVel, 3);
                Serial.print(" | Ang: ");
                Serial.println(packet_commande.angularVel, 3);
            }
        }

        // Temps de repos pour que Mbed OS rafraîchisse la pile WiFi
        rtos::ThisThread::sleep_for(10ms);
    }
}
