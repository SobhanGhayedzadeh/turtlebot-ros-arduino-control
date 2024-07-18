#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// Motor A (Left)
int ENA = 10; // Enable pin for motor A
int IN1 = 9;  // Control pin 1 for motor A
int IN2 = 11; // Control pin 2 for motor A

// Motor B (Right)
int ENB = 5; // Enable pin for motor B
int IN3 = 6; // Control pin 1 for motor B
int IN4 = 3; // Control pin 2 for motor B

// Robot dimensions
float wheelDiameter = 0.1; // in meters
float robotWidth = 0.435; // in meters, distance between wheels

float wheelCircumference = 3.14159 * wheelDiameter;

void driveMotor(int enablePin, int controlPin1, int controlPin2, int speedPWM) {
    speedPWM = constrain(speedPWM, -255, 255);

    // Map the PWM to the desired range
    if (speedPWM > 0) {
        speedPWM = map(speedPWM, 0, 255, 35, 255);
    } else if (speedPWM < 0) {
        speedPWM = map(speedPWM, -255, 0, -255, -35);
    }

    analogWrite(enablePin, abs(speedPWM));
    digitalWrite(controlPin1, speedPWM > 0 ? HIGH : LOW);
    digitalWrite(controlPin2, speedPWM < 0 ? HIGH : LOW);
}

int velocityToPWM(float linearVelocity, float angularVelocity, float offset) {
    // Convert m/s to wheel rotations per second
    float wheelLinearVelocity = linearVelocity + angularVelocity * offset;
    // Convert wheel rotations per second to PWM
    return (int)(255 * (wheelLinearVelocity / wheelCircumference));
}

void commandVelocity(const geometry_msgs::Twist& msg) {
    int leftPWM = velocityToPWM(msg.linear.x, msg.angular.z, -robotWidth / 2);
    int rightPWM = velocityToPWM(msg.linear.x, msg.angular.z, robotWidth / 2);

    // Drive motors
    driveMotor(ENA, IN1, IN2, leftPWM);
    driveMotor(ENB, IN3, IN4, rightPWM);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel_autonomous", &commandVelocity);

void setup() {
    nh.initNode();
    nh.subscribe(sub);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
    nh.spinOnce();
    delay(10);
}
