#include <driver/uart.h>
#include <math.h>
#include <esp_timer.h>

#include "servo.h"

#include "dynamixel.h"
#include "durin.h"
#include "hardware.h"

#define SERVO_ID_START 32

#define SERVO1 32
#define SERVO2 33
#define SERVO3 34
#define SERVO4 35

#define PI 3.141592
#define DIAMETER 60
#define MMS_TO_RPM (60/(DIAMETER*PI))

uint8_t servos[] = {SERVO1, SERVO2, SERVO3, SERVO4};

dynamixel_t dx;

void init_servo() {
    durin.control.control_mode = DURIN_MOTOR_VELOCITY;
    durin.control.motor_velocity.motor_1 = 0;
    durin.control.motor_velocity.motor_2 = 0;
    durin.control.motor_velocity.motor_3 = 0;
    durin.control.motor_velocity.motor_4 = 0;

    dx_init(&dx, UART_SERVO);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    for (uint8_t i = 0; i < 4; i++) {
        dx_set_status_return_level(&dx, servos[i], DX_PING);
        vTaskDelay(1);
        dx_set_operating_mode(&dx, servos[i], DX_VELOCITY_CONTROL);
        vTaskDelay(1);
        dx_enable_torque(&dx, servos[i], true);
        vTaskDelay(1);
        dx_set_goal_velocity(&dx, servos[i], 0, false);
        vTaskDelay(1);
    }
}

void update_servo(struct pt *pt) {
    PT_BEGIN(pt);
    static float speed1;
    static float speed2;
    static float speed3;
    static float speed4;
    while(1) {
        if (esp_timer_get_time() - durin.info.last_message_received > 5*1000*1000) {
            durin.control.control_mode = DURIN_MOTOR_VELOCITY;
            durin.control.motor_velocity.motor_1 = 0;
            durin.control.motor_velocity.motor_2 = 0;
            durin.control.motor_velocity.motor_3 = 0;
            durin.control.motor_velocity.motor_4 = 0;   
        }

        if (durin.control.control_mode == DURIN_ROBOT_VELOCITY) {
            float x = durin.control.robot_velocity.velocity_x;
            float y = durin.control.robot_velocity.velocity_y;
            float angle = atan2f(y, x);
            float magnitude = sqrtf(x*x + y*y);
            float speed14 = sinf(angle + PI / 4) * magnitude * MMS_TO_RPM * 2; // multiply by two because the vector is cut in half???
            float speed23 = sinf(angle - PI / 4) * magnitude * MMS_TO_RPM * 2;
            
            speed1 = speed14;
            speed2 = -speed23;
            speed3 = speed23;
            speed4 = -speed14;

            dx_set_goal_velocity(&dx, SERVO1, speed1, 1);
            PT_YIELD(pt);
            dx_set_goal_velocity(&dx, SERVO2, speed2, 1);
            PT_YIELD(pt);
            dx_set_goal_velocity(&dx, SERVO3, speed3, 1);
            PT_YIELD(pt);
            dx_set_goal_velocity(&dx, SERVO4, speed4, 1);
            PT_YIELD(pt);
            dx_action(&dx, DX_ID_BROADCAST);

        } else {
            speed1 = durin.control.motor_velocity.motor_1 * MMS_TO_RPM;
            speed2 = durin.control.motor_velocity.motor_2 * MMS_TO_RPM;
            speed3 = durin.control.motor_velocity.motor_3 * MMS_TO_RPM;
            speed4 = durin.control.motor_velocity.motor_4 * MMS_TO_RPM;

            dx_set_goal_velocity(&dx, SERVO1, speed1, 1);
            PT_YIELD(pt);
            dx_set_goal_velocity(&dx, SERVO2, speed2, 1);
            PT_YIELD(pt);
            dx_set_goal_velocity(&dx, SERVO3, speed3, 1);
            PT_YIELD(pt);
            dx_set_goal_velocity(&dx, SERVO4, speed4, 1);
            PT_YIELD(pt);
            dx_action(&dx, DX_ID_BROADCAST);
        }
    }
    PT_END(pt);
}