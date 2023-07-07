#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

// 定义MPU6050的I2C地址和寄存器地址
#define MPU6050_I2C_ADDR 0x68
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

// 定义舵机控制引脚
#define SERVO_PIN_1 0  // 这里使用了引脚号0，你可以根据实际情况进行修改
#define SERVO_PIN_2 1  // 这里使用了引脚号1，你可以根据实际情况进行修改
#define SERVO_PIN_3 2  // 这里使用了引脚号2，你可以根据实际情况进行修改
#define SERVO_PIN_4 3  // 这里使用了引脚号3，你可以根据实际情况进行修改

// 卡尔曼滤波器的参数
#define Q_angle 0.001
#define Q_bias 0.003
#define R_measure 0.03

// 全局变量
float angle = 0;       // 舵机角度
float bias = 0;        // 角速度偏差
float P[2][2] = {{1, 0}, {0, 1}};  // 误差协方差矩阵

// 初始化MPU6050
int initMPU6050(int fd) {
    wiringPiI2CWriteReg8(fd, 0x6B, 0x00);  // 发送复位命令
    delay(100);
    wiringPiI2CWriteReg8(fd, 0x6B, 0x80);  // 关闭休眠模式
    delay(100);

    int whoAmI = wiringPiI2CReadReg8(fd, 0x75);  // 读取WHO_AM_I寄存器
    if (whoAmI != 0x68) {
        printf("MPU6050 initialization failed. WHO_AM_I = 0x%02X\n", whoAmI);
        return -1;
    }

    printf("MPU6050 initialized successfully.\n");
    return 0;
}

// 读取MPU6050的加速度值
int readMPU6050Accel(int fd, int *accelX) {
    *accelX = wiringPiI2CReadReg16(fd, MPU6050_REG_ACCEL_XOUT_H);
    *accelX = ((*accelX << 8) & 0xFF00) | ((*accelX >> 8) & 0x00FF);
    return 0;
}

// 更新卡尔曼滤波器
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

// 控制舵机角度
void setServoAngle(int servoPin, float angle) {
    int pulseWidth = (int)(500 + angle * 10);  // 计算脉冲宽度
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(servoPin, LOW);
    delay(20);  // 控制信号周期为20ms，每个舵机的控制信号间隔20ms
}

int main() {
    wiringPiSetup();  // 初始化wiringPi库

    // 初始化舵机引脚
    pinMode(SERVO_PIN_1, OUTPUT);
    pinMode(SERVO_PIN_2, OUTPUT);
    pinMode(SERVO_PIN_3, OUTPUT);
    pinMode(SERVO_PIN_4, OUTPUT);

    // 初始化MPU6050
    int mpu6050_fd = wiringPiI2CSetup(MPU6050_I2C_ADDR);
    if (mpu6050_fd == -1) {
        printf("Failed to initialize MPU6050.\n");
        return -1;
    }
    if (initMPU6050(mpu6050_fd) == -1) {
        return -1;
    }

    // 主循环
    while (1) {
        // 读取MPU6050的加速度值
        int accelX;
        readMPU6050Accel(mpu6050_fd, &accelX);

        // 计算舵机控制角度
        float accelAngle = (float)accelX / 16384;  // 根据实际情况进行缩放
        updateKalmanFilter(accelAngle, 0, 0.01);  // 更新卡尔曼滤波器

        // 控制舵机
        setServoAngle(SERVO_PIN_1, angle);
        setServoAngle(SERVO_PIN_2, angle);
        setServoAngle(SERVO_PIN_3, angle);
        setServoAngle(SERVO_PIN_4, angle);

        delay(10);  // 控制周期为10ms
    }

    return 0;
}
/*
树莓派pi400     MPU6050
---------------------------------
3.3V（引脚1）        ->     VCC
GND（引脚6）         ->     GND
SDA（引脚3）         ->     SDA
SCL（引脚5）         ->     SCL

树莓派                 舵机
---------------------------------
GPIO0（引脚11）    ->     舵机1信号线
GPIO1（引脚12）    ->     舵机2信号线
GPIO2（引脚13）    ->     舵机3信号线
GPIO3（引脚15）    ->     舵机4信号线
*/
