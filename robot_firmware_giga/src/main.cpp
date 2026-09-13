#include <Arduino.h>
#include <rtos.h>
#include <WiFi.h>
#include "robot_config.h"
#include "task_moteurs.h"
#include "task_audio.h"
#include "task_perception.h"
#include "task_com_ros2.h"
#include "task_control.h"

// --- DÉFINITION DES THREADS ---
//rtos::Thread thread_moteurs(osPriorityHigh,        4096, nullptr, "moteurs");
rtos::Thread thread_audio(  osPriorityAboveNormal, 4096, nullptr, "audio");
rtos::Thread thread_percept(osPriorityNormal,      4096, nullptr, "perception");
rtos::Thread thread_ros2(   osPriorityBelowNormal, 8192, nullptr, "ros2");
rtos::Thread thread_control(osPriorityHigh,		   8192, nullptr, "control");

void setup() {
  Serial.begin(9600);
  delay(1000); 

  // --- CONFIGURATION DE LA LED MATÉRIELLE ---
  // On configure la broche de la LED Bleue intégrée en sortie
  pinMode(LEDB, OUTPUT);
  // Par défaut, sur la GIGA, la LED RGB intégrée est inversée : 
  // écrire HIGH l'éteint, écrire LOW l'allume. On commence éteint.
  digitalWrite(LEDB, HIGH); 

  Serial.println("=========================================");
  Serial.println("[Robot] Démarrage du système modulaire...");
  Serial.println("=========================================");

  // Lancement des fonctions de tâches déportées
   //thread_moteurs.start(mbed::callback(task_moteurs));
   thread_audio.start(mbed::callback(task_audio));
   thread_percept.start(mbed::callback(task_perception));
   thread_ros2.start(mbed::callback(task_com_ros2));
   //thread_control.start(mbed::callback(task_control));

  Serial.println("[Système] Initialisation des modules complète.");
}

void loop() {
  // --- CODE DE CLIGNOTEMENT (HEARTBEAT) ---
  // On inverse l'état actuel de la LED Bleue à chaque seconde
  digitalWrite(LEDB, !digitalRead(LEDB)); 
  
  // La boucle principale dort et laisse travailler les autres threads
  rtos::ThisThread::sleep_for(1s); 
}
