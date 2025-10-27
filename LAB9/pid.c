#include <Wire.h>
#include <AFMotor.h>
#include "Adafruit.TCS_34725.h"



#define DIR_A 12
#define DIR_B 13
#define PWM_A 3
#define PWM_B 11

float Kp = 24.0;
float Ki = 1.0;
float Kd = 1.0;
int base_speed = 60;

int middle_value = 3;
float error = 0, previous_error = 0, sum_error = 0;

void setup() {
    Serial.begin(9600);

    if (!tcs.begin()) {
        Serial.println("error");
        while (1);
    }
    Serial.println("TCS detected")

    pinMode(DIR_A, OUTPUT);
    pinMode(DIR_B, OUTPUT);
    pinMode(PWM_A, OUTPUT);
    pinMode(PWM_B, OUTPUT);

    calibrateSensor();
}

void loop() {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    error = middle_value - (float)c;
    sum_error += error;
    sum_error = constrain(sum_error, -4, 4);

    if (abs(error) < 2.0) {
        sum_error = 0;
    }

    float differential = error - previous_error;
    float correction = Kp * error + Ki * sum_error + Kd * differential;
    previous_error = error;

    int speed_left = base_speed + (int)correction;
    int speed_right = base_speed - (int)correction;

    speed_left = constrain(speed_left, 20, 255);
    speed_right = constrain(speed_right, 20, 255);

    control_motors(speed_left, speed_right);
}

void calibrate_sensor() {
    uint16_t r, g, b, c;
    Serial.println("calibrating measuring black");
    control_motors(-85, 85);
    delay(350);
    control_motors(0, 0);
    delay(50);
    tcs.getRawData(&r, &g, &b, &c);
    int black_value = c;
    Serial.print("black value: ");
    Serial.println(black_value);

    Serial.println("calibrating measuring white");
    control_motors(85, -85);
    delay(500);
    control_motors(0, 0);
    delay(50);
    tcs.getRawData(&r, &g, &b, &c);
    int white_value = c;
    Serial.print("white value: ");
    Serial.println(white_value);

    middle_value = (black_value + white_value) / 2;
    Serial.print("middle value = ");
    Serial.println(middle_value);

    control_motors(0, 0);
}

void control_motors(int speed_left, int speed_right) {
    if (speed_left >= 0) {
        digitalWrite(DIR_A, HIGH);
    } else {
        digitalWrite(DIR_A, LOW);
        speed_left = abs(speed_left);
    }

    if (speed_right >= 0) {
        digitalWrite(DIR_B, HIGH);
    } else {
        digitalWrite(DIR_B, LOW);
        speed_right = abs(speed_right);
    }

    analog_write(PWM_A, speed_left);
    analog_write(PWM_B, speed_right)
}
