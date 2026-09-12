#include "task_moteurs.h"
#include "robot_config.h"

// Inclusions de vos briques logicielles stockées dans lib/drivers/
#include "DCMotor.h"
#include "RotaryIncrementalEncoder.h"
#include "PIDController.h"
#include "StallWatchdog.h"
#include "VelocityMotorController.h"

// --- 1. Instanciation des composants pour le moteur gauche ---
RotaryIncrementalEncoder leftEncoder(PIN_LEFT_ENCODER_A, PIN_LEFT_ENCODER_B, TICKS_PER_REV);
DCMotor leftMotor(PIN_LEFT_MOTOR_EN, PIN_LEFT_MOTOR_IN1, PIN_LEFT_MOTOR_IN2);

// Votre PID (Kp, Ki, Kd, OutMax, Ramp)
PIDController leftPID(200.0f, 100.0f, 0.0f, 255.0f, 0.2f); 

// Chien de garde (Seuil vitesse min, temps max en ms)
StallWatchdog leftStallWatchdog(0.2f, 1000); 

// Le contrôleur global qui unifie le tout
VelocityMotorController leftWheel(leftEncoder, leftMotor, leftPID, leftStallWatchdog);

// --- 2. Wrappers pour les Interruptions (ISR) ---
// Mbed OS gère les interruptions très rapidement en tâche de fond
void isrLeftA() { leftEncoder.handleChannelA(); }
void isrLeftB() { leftEncoder.handleChannelB(); }

// --- 3. Corps de la Tâche de contrôle ---
void task_moteurs() {
  // Liaison des interruptions matérielles de l'encodeur
  leftEncoder.attachInterrupts(isrLeftA, isrLeftB);

  // Configuration initiale du PID
  leftPID.setOutputLimits(-255.0f, 255.0f);
  
  // Consigne de test : 0.5 tour par seconde
  leftWheel.setTargetVelocity(1.0f); 

  auto prochain_reveil = rtos::Kernel::Clock::now();

	int compteur_log = 0;

	while (true) {
	/*
	// Calcul de l'asservissement
	leftWheel.update();

	compteur_log++;
	if (compteur_log >= 25) { // 25 * 20ms = 500ms
		compteur_log = 0;

		// Récupération des valeurs internes de vos briques logicielles
		float vit_mesuree = leftEncoder.getSpeed(); // ou la fonction équivalente de votre classe
		int ticks = leftEncoder.getTicks();            // pour vérifier si l'encodeur bouge
		uint8_t pwm_envoi = leftMotor.getSpeed();

		Serial.print("[MOTEUR GAUCHE] Ticks: ");
		Serial.print(ticks);
		Serial.print(" | Vit. Mesurée: ");
		Serial.print(vit_mesuree);
		Serial.print(" | PWM envoyé: ");
		Serial.println(pwm_envoi);
	}
	*/
	prochain_reveil += PERIODE_MOTEURS;
	
	rtos::ThisThread::sleep_until(prochain_reveil);
	
	}
}
