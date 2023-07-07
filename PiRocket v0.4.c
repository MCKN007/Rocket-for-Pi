#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

// ����MPU6050��I2C��ַ�ͼĴ�����ַ
#define MPU6050_I2C_ADDR 0x68
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

// ��������������
#define SERVO_PIN_1 0  // ����ʹ�������ź�0������Ը���ʵ����������޸�
#define SERVO_PIN_2 1  // ����ʹ�������ź�1������Ը���ʵ����������޸�
#define SERVO_PIN_3 2  // ����ʹ�������ź�2������Ը���ʵ����������޸�
#define SERVO_PIN_4 3  // ����ʹ�������ź�3������Ը���ʵ����������޸�

// �������˲����Ĳ���
#define Q_angle 0.001
#define Q_bias 0.003
#define R_measure 0.03

// ȫ�ֱ���
float angle = 0;       // ����Ƕ�
float bias = 0;        // ���ٶ�ƫ��
float P[2][2] = {{1, 0}, {0, 1}};  // ���Э�������

// ��ʼ��MPU6050
int initMPU6050(int fd) {
    wiringPiI2CWriteReg8(fd, 0x6B, 0x00);  // ���͸�λ����
    delay(100);
    wiringPiI2CWriteReg8(fd, 0x6B, 0x80);  // �ر�����ģʽ
    delay(100);

    int whoAmI = wiringPiI2CReadReg8(fd, 0x75);  // ��ȡWHO_AM_I�Ĵ���
    if (whoAmI != 0x68) {
        printf("MPU6050 initialization failed. WHO_AM_I = 0x%02X\n", whoAmI);
        return -1;
    }

    printf("MPU6050 initialized successfully.\n");
    return 0;
}

// ��ȡMPU6050�ļ��ٶ�ֵ
int readMPU6050Accel(int fd, int *accelX) {
    *accelX = wiringPiI2CReadReg16(fd, MPU6050_REG_ACCEL_XOUT_H);
    *accelX = ((*accelX << 8) & 0xFF00) | ((*accelX >> 8) & 0x00FF);
    return 0;
}

// ���¿������˲���
void updateKalmanFilter(float newAngle, float newRate, float dt) {
    float y, S;
    float K[2];

    angle += dt * (newRate - bias);
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    y = newAngle - angle;
    S = P[0][0] + R_measure;
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    angle += K[0] * y;
    bias += K[1] * y;
    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];
}

// ���ƶ���Ƕ�
void setServoAngle(int servoPin, float angle) {
    int pulseWidth = (int)(500 + angle * 10);  // ����������
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(servoPin, LOW);
    delay(20);  // �����ź�����Ϊ20ms��ÿ������Ŀ����źż��20ms
}

int main() {
    wiringPiSetup();  // ��ʼ��wiringPi��

    // ��ʼ���������
    pinMode(SERVO_PIN_1, OUTPUT);
    pinMode(SERVO_PIN_2, OUTPUT);
    pinMode(SERVO_PIN_3, OUTPUT);
    pinMode(SERVO_PIN_4, OUTPUT);

    // ��ʼ��MPU6050
    int mpu6050_fd = wiringPiI2CSetup(MPU6050_I2C_ADDR);
    if (mpu6050_fd == -1) {
        printf("Failed to initialize MPU6050.\n");
        return -1;
    }
    if (initMPU6050(mpu6050_fd) == -1) {
        return -1;
    }

    // ��ѭ��
    while (1) {
        // ��ȡMPU6050�ļ��ٶ�ֵ
        int accelX;
        readMPU6050Accel(mpu6050_fd, &accelX);

        // ���������ƽǶ�
        float accelAngle = (float)accelX / 16384;  // ����ʵ�������������
        updateKalmanFilter(accelAngle, 0, 0.01);  // ���¿������˲���

        // ���ƶ��
        setServoAngle(SERVO_PIN_1, angle);
        setServoAngle(SERVO_PIN_2, angle);
        setServoAngle(SERVO_PIN_3, angle);
        setServoAngle(SERVO_PIN_4, angle);

        delay(10);  // ��������Ϊ10ms
    }

    return 0;
}
/*
��ݮ��pi400     MPU6050
---------------------------------
3.3V������1��        ->     VCC
GND������6��         ->     GND
SDA������3��         ->     SDA
SCL������5��         ->     SCL

��ݮ��                 ���
---------------------------------
GPIO0������11��    ->     ���1�ź���
GPIO1������12��    ->     ���2�ź���
GPIO2������13��    ->     ���3�ź���
GPIO3������15��    ->     ���4�ź���
*/
