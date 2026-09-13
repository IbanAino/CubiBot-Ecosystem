#pragma once
 
/**
 * @brief Tâche de contrôle moteur et odométrie — 50 Hz.
 *
 * Regroupe dans la même boucle temps réel :
 *   - Mise à jour des contrôleurs de vitesse (PID) pour les 4 roues
 *   - Mise à jour de l'odométrie différentielle
 *
 * Ces deux fonctions partagent la même période et les mêmes lectures
 * d'encodeurs — les grouper garantit leur cohérence temporelle, comme
 * le fait ros2_control en production.
 *
 * Données produites (variables partagées dans robot_config.h) :
 *   - odom_partagee   : pose et vitesses du robot (mutex_odom)
 *
 * Données consommées :
 *   - cmd_partagee    : consignes de vitesse (linear, angular)
 *                       envoyées par task_com_ros2 (mutex_cmd)
 */
void task_control();