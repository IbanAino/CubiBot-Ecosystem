#include "task_moteurs.h"
#include "robot_config.h"


// Inclusions de vos briques logicielles stockées dans lib/drivers/
#include "DCMotor.h"
#include "RotaryIncrementalEncoder.h"
#include "PIDController.h"
#include "StallWatchdog.h"
#include "VelocityMotorController.h"
#include "DifferentialOdometryController.h"
#include "time_system.h"

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

DifferentialOdometry odometry(WHEEL_RADIUS, WHEEL_BASE);
DifferentialOdometryController odomController(leftEncoder1, rightEncoder1, odometry);

void isrLeft1A() { leftEncoder1.handleChannelA(); }
void isrLeft1B() { leftEncoder1.handleChannelB(); }
void isrLeft2A() { leftEncoder2.handleChannelA(); }
void isrLeft2B() { leftEncoder2.handleChannelB(); }
void isrRight1A() { rightEncoder1.handleChannelA(); }
void isrRight1B() { rightEncoder1.handleChannelB(); }
void isrRight2A() { rightEncoder2.handleChannelA(); }
void isrRight2B() { rightEncoder2.handleChannelB(); }


MotorCommand motor_cmd_local;
// float linearVel;   // m/s
// float angularVel;  // rad/s
// float TARGET_VELOCITY = 0.0f; // rev/s




namespace {
    OdomData local_odom;
}


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

	// leftWheel1.setTargetVelocity(TARGET_VELOCITY);
	// leftWheel2.setTargetVelocity(TARGET_VELOCITY);
	// rightWheel1.setTargetVelocity(TARGET_VELOCITY);
	// rightWheel2.setTargetVelocity(TARGET_VELOCITY);

	leftWheel1.stop();
	leftWheel2.stop();
	rightWheel1.stop();
	rightWheel2.stop();

	// Configuration initiale du PID
	//leftPID.setOutputLimits(-255.0f, 255.0f);

	// Consigne de test : 0.5 tour par seconde
	//leftWheel.setTargetVelocity(1.0f); 

	auto prochain_reveil = rtos::Kernel::Clock::now();


	while (true) {
		// --- 1. Réception des commandes
		mutex_motor_cmd.lock();
		motor_cmd_local = motor_cmd_partagee;
		mutex_motor_cmd.unlock();

		if (motor_cmd_local.stop) {

			leftWheel1.stop();
			leftWheel2.stop();
			rightWheel1.stop();
			rightWheel2.stop();

		} else {

			leftWheel1.setTargetVelocity(
				motor_cmd_local.leftVelocity);

			leftWheel2.setTargetVelocity(
				motor_cmd_local.leftVelocity);

			rightWheel1.setTargetVelocity(
				motor_cmd_local.rightVelocity);

			rightWheel2.setTargetVelocity(
				motor_cmd_local.rightVelocity);
		}




		// --- 2. Mise à jour des moteurs
		leftWheel1.update();
		leftWheel2.update();
		rightWheel1.update();
		rightWheel2.update();

        // --- 3. Mise à jour de l'odométrie ---
        // Même appel de boucle que le PID → cohérence temporelle garantie
        odomController.update();

        //--- 4. Calcul des vitesses instantanées ---
        //Moyenne des deux roues de chaque côté pour un robot à 4 roues
        const float leftVel  = (leftWheel1.getMeasuredVelocity()
                              + leftWheel2.getMeasuredVelocity()) / 2.0f;
        const float rightVel = (rightWheel1.getMeasuredVelocity()
                              + rightWheel2.getMeasuredVelocity()) / 2.0f;

        // // Vitesses robot en m/s et rad/s (cinématique directe)
        const float circumference = 2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS;
        const float linearVel     = (leftVel + rightVel) / 2.0f * circumference;
        const float angularVel    = (rightVel - leftVel)  * circumference / WHEEL_BASE;
		
        // // --- 5. Publication de la pose dans la variable partagée ---
		local_odom.timestamp = get_system_time_ms();
		local_odom.x = odomController.getX();
		local_odom.y = odomController.getY();
		local_odom.theta = odomController.getTheta();
		local_odom.linearVel = linearVel;
		local_odom.angularVel = angularVel;

		mutex_odom_partagee.lock();
		odom_partagee = local_odom;
		mutex_odom_partagee.unlock();

		// Serial.print("[task_moteurs] Odom : ");
		// Serial.print("timeStamp=");
		// Serial.print(local_odom.timestamp);
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

		prochain_reveil += PERIODE_MOTEURS;
		rtos::ThisThread::sleep_until(prochain_reveil);
	}
}
