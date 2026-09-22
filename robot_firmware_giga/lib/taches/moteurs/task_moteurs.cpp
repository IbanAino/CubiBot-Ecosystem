#include "task_moteurs.h"
#include "robot_config.h"

// Inclusions de vos briques logicielles stockées dans lib/drivers/
#include "DCMotor.h"
#include "RotaryIncrementalEncoder.h"
#include "PIDController.h"
#include "StallWatchdog.h"
#include "VelocityMotorController.h"
#include "DifferentialOdometryController.h"

// --- 1. Instanciation des composants pour le moteur gauche ---
RotaryIncrementalEncoder leftEncoder1(L1_C1, L1_C2, TICKS_PER_REV);
RotaryIncrementalEncoder leftEncoder2(L2_C1, L2_C2, TICKS_PER_REV);
RotaryIncrementalEncoder rightEncoder1(R1_C2, R1_C1, TICKS_PER_REV);
RotaryIncrementalEncoder rightEncoder2(R2_C2, R2_C1, TICKS_PER_REV);

DCMotor leftMotor1(L_EN1, L_IN2, L_IN1);
DCMotor leftMotor2(L_EN2, L_IN3, L_IN4);
DCMotor rightMotor1(R_EN1, R_IN1, R_IN2);
DCMotor rightMotor2(R_EN2, R_IN4, R_IN3);

PIDController leftPID1(200.0f, 100.0f, 0.0f, 100.0f, 0.2f);
PIDController leftPID2(200.0f, 100.0f, 0.0f, 100.0f, 0.2f);
PIDController rightPID1(200.0f, 100.0f, 0.0f, 100.0f, 0.2f);
PIDController rightPID2(200.0f, 100.0f, 0.0f, 100.0f, 0.2f);

StallWatchdog leftStallWatchdog1(0.2f, 1000);
StallWatchdog leftStallWatchdog2(0.2f, 1000);
StallWatchdog rightStallWatchdog1(0.2f, 1000);
StallWatchdog rightStallWatchdog2(0.2f, 1000);

VelocityMotorController leftWheel1(leftEncoder1, leftMotor1, leftPID1, leftStallWatchdog1);
VelocityMotorController leftWheel2(leftEncoder2, leftMotor2, leftPID2, leftStallWatchdog2);
VelocityMotorController rightWheel1(rightEncoder1, rightMotor1, rightPID1, rightStallWatchdog1);
VelocityMotorController rightWheel2(rightEncoder2, rightMotor2, rightPID2, rightStallWatchdog2);

DifferentialOdometry odometry(WHEEL_RADIUS_METERS, WHEEL_BASE_METERS);
DifferentialOdometryController odomController(leftEncoder1, rightEncoder1, odometry);

void isrLeft1A() { leftEncoder1.handleChannelA(); }
void isrLeft1B() { leftEncoder1.handleChannelB(); }
void isrLeft2A() { leftEncoder2.handleChannelA(); }
void isrLeft2B() { leftEncoder2.handleChannelB(); }
void isrRight1A() { rightEncoder1.handleChannelA(); }
void isrRight1B() { rightEncoder1.handleChannelB(); }
void isrRight2A() { rightEncoder2.handleChannelA(); }
void isrRight2B() { rightEncoder2.handleChannelB(); }

float TARGET_VELOCITY = 1.0f; // rev/s


// --- 3. Corps de la Tâche de contrôle ---
void task_moteurs() {

  // Liaison des interruptions matérielles de l'encodeur
  leftEncoder1.attachInterrupts(isrLeft1A, isrLeft1B);
  leftEncoder2.attachInterrupts(isrLeft2A, isrLeft2B);
  rightEncoder1.attachInterrupts(isrRight1A, isrRight1B);
  rightEncoder2.attachInterrupts(isrRight2A, isrRight2B);

  leftPID1.setOutputLimits(-255.0f, 255.0f);
  leftPID2.setOutputLimits(-255.0f, 255.0f);
  rightPID1.setOutputLimits(-255.0f, 255.0f);
  rightPID2.setOutputLimits(-255.0f, 255.0f);

  leftWheel1.setTargetVelocity(TARGET_VELOCITY);
  leftWheel2.setTargetVelocity(TARGET_VELOCITY);
  rightWheel1.setTargetVelocity(TARGET_VELOCITY);
  rightWheel2.setTargetVelocity(TARGET_VELOCITY);

  // Configuration initiale du PID
  //leftPID.setOutputLimits(-255.0f, 255.0f);
  
  // Consigne de test : 0.5 tour par seconde
  //leftWheel.setTargetVelocity(1.0f); 

  auto prochain_reveil = rtos::Kernel::Clock::now();


	while (true) {
		leftWheel1.update();
		leftWheel2.update();
		rightWheel1.update();
		rightWheel2.update();

		odomController.update();

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
