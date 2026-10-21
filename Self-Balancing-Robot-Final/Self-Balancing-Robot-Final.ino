// Author - Jan Morada
// My script for my Self-Balancing Robot build. These constants in this script for the PD control and wheel velocity are what worked the best for me.
// Last edited 21/10/26 (annotation)
#include <Wire.h>

// Variables to calculate dt
uint32_t timeLast = 0;
uint32_t timeCurrent;

// Encoder pins
// Left - 1, Right - 2.
const int encoderA1 = 32;
const int encoderB1 = 33;
const int encoderA2 = 19;
const int encoderB2 = 23;

// Motor driver pins
// Left - 1, Right - 2.
const int RPWM1 = 14; //anticlockwise
const int LPWM1 = 13;
const int RPWM2 = 18;
const int LPWM2 = 4;

// PD and velocity feedback constants
const float Kp = 26.0;
const float Kd = 0.3;
float Kv = 0.0075;

// The tilt calculated with complementary filer
float tiltFiltered;

// The tilt calulated by using the accelerometer measurements
float tiltAccel;

// The PWM to be outputted to the motors
float pwm;

// Stores accelerometer readings
float accelX;
float accelY;
float accelZ;

// Stores gyroscopes readings
float gyroX;
float gyroY;
float gyroZ;

// The tilt to be achieved for balance (found through experimentation)
float targetAngle = -1.3;

// Counts for the encoders
volatile int32_t c1 = 0, c2 = 0;


// Interrupt functions to count the encoder pulses
void IRAM_ATTR pulse1() {
    c1 += digitalRead(encoderB1) ? 1 : -1; 
}
void IRAM_ATTR pulse2() {
    c2 += digitalRead(encoderB2) ? -1 : 1; 
} 

// Variables to find the current speed (encoder counts/s) of motors via encoder singals.
int32_t lastC = 0;
float speed = 0;


void accelI2C (float &x, float &y, float &z) {
    // Function that fetches the accelerometer readings from the IMU with I2C

    // Usual I2C
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);

    // Accelerations stored as 16 bit with 2 bytes, a higher and lower, so fetch both for each axis.
    byte accelXHIGH = Wire.read();
    byte accelXLOW = Wire.read();
    byte accelYHIGH = Wire.read();
    byte accelYLOW = Wire.read();
    byte accelZHIGH = Wire.read();
    byte accelZLOW = Wire.read();

    int16_t accelXRaw = (accelXHIGH << 8) | accelXLOW; // Combine bytes to get acceleration in sensor units
    x = accelXRaw*9.81/16384.0; // In ms^-2

    int16_t accelYRaw = (accelYHIGH << 8) | accelYLOW; 
    y = accelYRaw*9.81/16384.0;

    int16_t accelZRaw = (accelZHIGH << 8) | accelZLOW; 
    z = accelZRaw*9.81/16384.0;

    // Measurements had offsets. Constants found through experimentation.
    x -= 0.44;
    y += 0.03;
    z += 1.19;
}


void gyroI2C(float &x, float &y, float &z) {
    // Function that fetches the gyroscope readings from the IMU with I2C - near identical to accelI2C

    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);

    byte gyroXHIGH = Wire.read();
    byte gyroXLOW = Wire.read();
    byte gyroYHIGH = Wire.read();
    byte gyroYLOW = Wire.read();
    byte gyroZHIGH = Wire.read();
    byte gyroZLOW = Wire.read();

    int16_t gyroXRaw = (gyroXHIGH << 8) | gyroXLOW; // Combine bytes to get angular velocity in sensor units
    x = gyroXRaw/131.0; // In degrees/s

    int16_t gyroYRaw = (gyroYHIGH << 8) | gyroYLOW; 
    y = gyroYRaw/131.0;

    int16_t gyroZRaw = (gyroZHIGH << 8) | gyroZLOW; 
    z = gyroZRaw/131.0;
    
    // Measurements had offsets. Constants found through experimentation.
    x += 2.0;
    z += -3.0;
    y += -0.41166;
}
void setup() {
    Serial.begin(115200);
    
    // Define I2C connections
    Wire.begin(21, 22);

    // Set clock speed of IMU to get faster readings
    Wire.setClock(400000);

    // Turning off sleep mode of IMU (I2C)
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    // Define the motor driver PWM pins
    pinMode(RPWM1, OUTPUT);
    pinMode(RPWM2, OUTPUT);
    pinMode(LPWM1, OUTPUT);
    pinMode(LPWM2, OUTPUT);

    // Set motor PWM frequency to 20kHz
    analogWriteFrequency(RPWM1, 20000);
    analogWriteFrequency(LPWM1, 20000);
    analogWriteFrequency(RPWM2, 20000);
    analogWriteFrequency(LPWM2, 20000);

    // Ensure motor's didn't run during set up - this fixed an observed bug.
    analogWrite(RPWM1, 0);
    analogWrite(LPWM1, 0);
    analogWrite(RPWM2, 0);
    analogWrite(LPWM2, 0);
    delay(1000);

    // Set intial orientation.
    accelI2C(accelX, accelY, accelZ);
    tiltAccel = atan2(accelX,accelZ);
    tiltAccel *= 180.0/PI;
    tiltFiltered = tiltAccel; 

    // Get initial angular velocity.
    gyroI2C(gyroX, gyroY, gyroZ);

    // TiltFiltered set so take the time.
    timeLast = micros();

    // Assign pinmode to encoder pins
    pinMode(encoderA1, INPUT);
    pinMode(encoderB1, INPUT);
    pinMode(encoderA2, INPUT);
    pinMode(encoderB2, INPUT);
    attachInterrupt(encoderA1, pulse1, RISING);
    attachInterrupt(encoderA2, pulse2, RISING);
}

void loop() {
    // Get accelerometer and gyroscope readings
    accelI2C(accelX, accelY, accelZ);
    gyroI2C(gyroX, gyroY, gyroZ);

    // Get the tilt from the accelerometer readings
    tiltAccel = atan2(accelX,accelZ);

    // Conver it to degrees
    tiltAccel *= 180/PI;

    // Get current time and find dt in seconds
    timeCurrent = micros();
    uint32_t dt = timeCurrent - timeLast;
    float dtS = dt / 1e6;

    // Set timeLast for the next loop
    timeLast = timeCurrent;

    // Keeps time constant
    const float tau = 0.4; 
    float alpha = tau / (tau + dtS);

    // Checks the reliability of the accelerometer. Should read about 9.81 ms^-2 (g) but accelerations from wheels may affect it.    
    float mag = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    if (fabs(mag - 9.81) > 2.0){
        // Makes the accelerometer have no impact on tilt on iterations the accelerometer is deemed unreliable
        alpha = 1.0;
    }

    // Complementary filter
    tiltFiltered = tiltAccel * (1-alpha) + alpha * (tiltFiltered - gyroY * dtS);
        
    // Ensures no interrupts happen when calculating avg current counts by encoder
    noInterrupts();
    long c = (c1 + c2) / 2;
    interrupts();

    // Puts wheel speed through a low pass filter
    speed = 0.9*speed + 0.1*((c - lastC) / dtS);
    lastC = c;

    // Change the targetAngle depending on the current speed of the robot, with +-4 as maximum range away from original
    float dynTarget = constrain(targetAngle + Kv*speed, targetAngle - 4, targetAngle + 4);

    // Find tilt error
    float error = tiltFiltered - dynTarget;

    // Calculate PWM
    pwm = Kp * error - Kd * gyroY; // if tilt is positive, gyroY is a negative value

    // Get the absolute value of PWM
    int absPwm = constrain((int)abs(pwm), 0, 255);

    // For motor deadband
    if (absPwm > 0 && absPwm < 14) {
        absPwm= 14;
    }


    if (pwm > 0) {
        // For positive PWM
        analogWrite(RPWM1, absPwm); 
        analogWrite(LPWM1, 0);
        analogWrite(RPWM2, absPwm);
        analogWrite(LPWM2, 0);

    } 
    else {
        // For negative PWM
        analogWrite(RPWM1, 0); 
        analogWrite(LPWM1, absPwm); 
        analogWrite(RPWM2, 0);
        analogWrite(LPWM2, absPwm);
    }

}