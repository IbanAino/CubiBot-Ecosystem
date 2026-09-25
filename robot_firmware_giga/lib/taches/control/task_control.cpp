#include <Arduino.h>
#include <rtos.h>

#include "task_com_ros2.h"
#include "robot_config.h"
#include "time_system.h"


static constexpr uint32_t COMMAND_TIMEOUT_MS = 500;




// ============================================================================
// Tâche de contrôle
// ============================================================================

void task_control()
{
    CmdVel cmd_vel_local;

	bool resetRequested = false;

    while (true) {

        // --------------------------------------------------------------------
        // 1. Lecture de la dernière commande reçue
        // --------------------------------------------------------------------

        mutex_cmd_vel.lock();
        cmd_vel_local = cmd_vel_partagee;
        mutex_cmd_vel.unlock();


        // --------------------------------------------------------------------
        // 2. Vérification du watchdog
        // --------------------------------------------------------------------

        const uint32_t now = millis();

        if ((now - cmd_vel_local.lastUpdate) > COMMAND_TIMEOUT_MS) {

			//Serial.println("[task_control] WATCHDOG - no input command");

            // Perte de communication :
            // on envoie des consignes nulles aux moteurs.

            mutex_motor_cmd.lock();

            motor_cmd_partagee.leftVelocity = 0.0f;
            motor_cmd_partagee.rightVelocity = 0.0f;
            motor_cmd_partagee.stop = true;
			motor_cmd_partagee.reset = false;

            mutex_motor_cmd.unlock();
        }


        // --------------------------------------------------------------------
        // 3. Cinématique différentielle
        // --------------------------------------------------------------------

        const float linearVel = cmd_vel_local.linearVel;
        const float angularVel = cmd_vel_local.angularVel;

        const float leftVelocity =
            linearVel - angularVel * WHEEL_BASE / 2.0f;

        const float rightVelocity =
            linearVel + angularVel * WHEEL_BASE / 2.0f;


        // --------------------------------------------------------------------
        // 4. Conversion m/s -> tours/seconde
        // --------------------------------------------------------------------

        const float wheelCircumference =
            2.0f * PI * WHEEL_RADIUS;

        const float leftRevPerSec =
            leftVelocity / wheelCircumference;

        const float rightRevPerSec =
            rightVelocity / wheelCircumference;


        // --------------------------------------------------------------------
        // 5. Publication des consignes pour task_moteurs
        // --------------------------------------------------------------------

        mutex_motor_cmd.lock();

        motor_cmd_partagee.leftVelocity = leftRevPerSec;
        motor_cmd_partagee.rightVelocity = rightRevPerSec;
        motor_cmd_partagee.stop = cmd_vel_local.stopRequested;


		if (cmd_vel_local.resetRequested && !resetRequested){
			motor_cmd_partagee.reset = true;
			resetRequested = true;
		}
		else if (!cmd_vel_local.resetRequested && resetRequested){
			resetRequested = false;
			motor_cmd_partagee.reset = false;
		}

		//motor_cmd_partagee.reset = cmd_vel_local.resetRequested;


        mutex_motor_cmd.unlock();

        // --------------------------------------------------------------------
        // 6. Période de contrôle
        // --------------------------------------------------------------------

        rtos::ThisThread::sleep_for(10ms);
    }
}