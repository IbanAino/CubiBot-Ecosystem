#include "task_control.h"
#include "robot_config.h"

// Contrôleurs moteur
#include "VelocityMotorController.h"
#include "DCMotor.h"
#include "RotaryIncrementalEncoder.h"
#include "PIDController.h"
#include "StallWatchdog.h"

// Odométrie
#include "DifferentialOdometry.h"
#include "DifferentialOdometryController.h"

#include <math.h>

// -----------------------------------------------------------------------
// Hardware — encodeurs et moteurs
// -----------------------------------------------------------------------

static RotaryIncrementalEncoder leftEncoder1(L1_C1, L1_C2, TICKS_PER_REV);
static RotaryIncrementalEncoder leftEncoder2(L2_C1, L2_C2, TICKS_PER_REV);
static RotaryIncrementalEncoder rightEncoder1(R1_C1, R1_C2, TICKS_PER_REV);
static RotaryIncrementalEncoder rightEncoder2(R2_C1, R2_C2, TICKS_PER_REV); // R2_C1 BUUUUUG !!!

static DCMotor leftMotor1(L_EN1, L_IN1, L_IN2);
static DCMotor leftMotor2(L_EN2, L_IN3, L_IN4);
static DCMotor rightMotor1(R_EN1, R_IN1, R_IN2);
static DCMotor rightMotor2(R_EN2, R_IN3, R_IN4);

// -----------------------------------------------------------------------
// PID — un par roue, gains à régler empiriquement
// -----------------------------------------------------------------------

static PIDController leftPID1(200.0f,  100.0f, 0.0f, 100.0f, 0.2f);
static PIDController leftPID2(200.0f,  100.0f, 0.0f, 100.0f, 0.2f);
static PIDController rightPID1(200.0f, 100.0f, 0.0f, 100.0f, 0.2f);
static PIDController rightPID2(200.0f, 100.0f, 0.0f, 100.0f, 0.2f);

// -----------------------------------------------------------------------
// Watchdogs — un par roue
// -----------------------------------------------------------------------

static StallWatchdog leftStall1(0.2f,  1000);
static StallWatchdog leftStall2(0.2f,  1000);
static StallWatchdog rightStall1(0.2f, 1000);
static StallWatchdog rightStall2(0.2f, 1000);

// -----------------------------------------------------------------------
// Contrôleurs de vitesse
// -----------------------------------------------------------------------

static VelocityMotorController leftWheel1(leftEncoder1,  leftMotor1,  leftPID1,  leftStall1);
static VelocityMotorController leftWheel2(leftEncoder2,  leftMotor2,  leftPID2,  leftStall2);
static VelocityMotorController rightWheel1(rightEncoder1, rightMotor1, rightPID1, rightStall1);
static VelocityMotorController rightWheel2(rightEncoder2, rightMotor2, rightPID2, rightStall2);

// -----------------------------------------------------------------------
// Odométrie
// Robot différentiel à deux roues motrices moyennées :
//   vitesse gauche = moyenne(leftWheel1, leftWheel2)
//   vitesse droite = moyenne(rightWheel1, rightWheel2)
// -----------------------------------------------------------------------

static DifferentialOdometry odometry(WHEEL_RADIUS_METERS, WHEEL_BASE_METERS);
static DifferentialOdometryController odomController(leftEncoder1, rightEncoder1, odometry);

// -----------------------------------------------------------------------
// ISR wrappers — un par canal d'encodeur
// -----------------------------------------------------------------------

static void isrLeft1A()  { leftEncoder1.handleChannelA(); }
static void isrLeft1B()  { leftEncoder1.handleChannelB(); }
static void isrLeft2A()  { leftEncoder2.handleChannelA(); }
static void isrLeft2B()  { leftEncoder2.handleChannelB(); }
static void isrRight1A() { rightEncoder1.handleChannelA(); }
static void isrRight1B() { rightEncoder1.handleChannelB(); }
static void isrRight2A() { rightEncoder2.handleChannelA(); }
static void isrRight2B() { rightEncoder2.handleChannelB(); }

// -----------------------------------------------------------------------
// Cinématique inverse : cmd_vel → consignes par roue
//
// v_left  = linear - angular * wheelBase / 2
// v_right = linear + angular * wheelBase / 2
// Converti de m/s en rev/s via la circonférence de la roue.
// -----------------------------------------------------------------------

static void applyVelocityCommand(float linearVelocity, float angularVelocity)
{
    const float halfBase       = WHEEL_BASE_METERS / 2.0f;
    const float circumference  = 2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_METERS;

    const float leftRevPerSec  = (linearVelocity - angularVelocity * halfBase) / circumference;
    const float rightRevPerSec = (linearVelocity + angularVelocity * halfBase) / circumference;

    leftWheel1.setTargetVelocity(leftRevPerSec);
    leftWheel2.setTargetVelocity(leftRevPerSec);
    rightWheel1.setTargetVelocity(rightRevPerSec);
    rightWheel2.setTargetVelocity(rightRevPerSec);
}

// -----------------------------------------------------------------------
// Tâche principale
// -----------------------------------------------------------------------

void task_control()
{
    auto prochain_reveil = rtos::Kernel::Clock::now();

    // --- Initialisation ---
	
    leftEncoder1.attachInterrupts(isrLeft1A,  isrLeft1B);
	leftEncoder2.attachInterrupts(isrLeft2A,  isrLeft2B);
    rightEncoder1.attachInterrupts(isrRight1A, isrRight1B);
    rightEncoder2.attachInterrupts(isrRight2A, isrRight2B);
	
    leftPID1.setOutputLimits(-255.0f, 255.0f);
    leftPID2.setOutputLimits(-255.0f, 255.0f);
    rightPID1.setOutputLimits(-255.0f, 255.0f);
    rightPID2.setOutputLimits(-255.0f, 255.0f);

    Serial.println("[Control] Tâche contrôle moteur + odométrie démarrée.");
	
	
	
	// Serial.print("applyVelocityCommand");
	// applyVelocityCommand(1.0f, 0.0f);




    while (true)
    {
        
		// --- 1. Lecture de la consigne de vitesse ---
        CmdVel cmd;
        mutex_cmd.lock();
        cmd = cmd_partagee;
        mutex_cmd.unlock();

        applyVelocityCommand(cmd.linearVel, cmd.angularVel);

        // --- 2. Mise à jour des contrôleurs moteur (PID) ---
        leftWheel1.update();
        leftWheel2.update();
        rightWheel1.update();
        rightWheel2.update();

        // --- 3. Mise à jour de l'odométrie ---
        // Même appel de boucle que le PID → cohérence temporelle garantie
        odomController.update();

        // --- 4. Calcul des vitesses instantanées ---
        // Moyenne des deux roues de chaque côté pour un robot à 4 roues
        const float leftVel  = (leftWheel1.getMeasuredVelocity()
                              + leftWheel2.getMeasuredVelocity()) / 2.0f;
        const float rightVel = (rightWheel1.getMeasuredVelocity()
                              + rightWheel2.getMeasuredVelocity()) / 2.0f;

        // Vitesses robot en m/s et rad/s (cinématique directe)
        const float circumference = 2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS_METERS;
        const float linearVel     = (leftVel + rightVel) / 2.0f * circumference;
        const float angularVel    = (rightVel - leftVel)  * circumference / WHEEL_BASE_METERS;

        // --- 5. Publication de la pose dans la variable partagée ---
        mutex_odom.lock();
        odom_partagee.x          = odomController.getX();
        odom_partagee.y          = odomController.getY();
        odom_partagee.theta      = odomController.getTheta();
        odom_partagee.linearVel  = linearVel;
        odom_partagee.angularVel = angularVel;
        mutex_odom.unlock();

        // --- 6. Gestion des stalls ---
        if (leftWheel1.isStalled() || leftWheel2.isStalled()) {
            Serial.println("[Control] Stall côté gauche détecté.");
        }
        if (rightWheel1.isStalled() || rightWheel2.isStalled()) {
            Serial.println("[Control] Stall côté droit détecté.");
        }
		

        prochain_reveil += PERIODE_CONTROL;
        rtos::ThisThread::sleep_until(prochain_reveil);
    }
}