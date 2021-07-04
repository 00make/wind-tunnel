#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

float desired = 200;
int initalspeed = 1300;
double kp = 3.55;  //3.55
double ki = 0.005; //0.003
double kd = 2.55;  //2.05

float elapsedTime, time, timePrev, PID, error;
float previous_error = 0;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
int distance;
int low = initalspeed - 1000;
int high = 2000 - initalspeed - 200;
#define MOTOR_PIN 9
Servo motor;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;

void setup()
{
    Serial.begin(250000);
    // Serial.begin(9600);
    motor.attach(9, 1000, 2000);
    time = millis();
    motor.writeMicroseconds(1000);
    if (!lox.begin())
        while (1)
            Serial.println(F("Failed to boot VL53L0X sensor"));
    delay(3000);
    motor.writeMicroseconds(initalspeed);
}
void loop()
{
    // n = digitalRead(2);
    // if (n == HIGH)
    //     while (1)
    //     {
    //         motor.writeMicroseconds(1000);
    //         Serial.println(F("PAUSE"));
    //         delay(500);
    //         n = digitalRead(2);
    //         if (n == HIGH)
    //             break;
    //         delay(500);
    //     }
    timePrev = time;
    time = millis();
    elapsedTime = (time - timePrev) / 1000;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4)
        distance = measure.RangeMilliMeter;
    else
        while (1)
            Serial.println(F("Failed to measure"));
    Serial.print(measure.RangeMilliMeter);
    error = distance - desired;
    pid_p = kp * error;
    if (-8 < error && error < 8)
        pid_i = pid_i + (ki * error);
    pid_d = kd * ((error - previous_error) / elapsedTime);
    previous_error = error;
    PID = pid_p + pid_i + pid_d;
    Serial.print(" ");
    Serial.println(PID);
    if (PID < -low)
        PID = -low;
    if (PID > high)
        PID = high;
    PID = PID + initalspeed;
    motor.writeMicroseconds(PID);
}