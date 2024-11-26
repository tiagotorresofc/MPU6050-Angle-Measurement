#ifndef MPU6050_H
#define MPU6050_H

using namespace std;
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <cstdio>
#include <cstdint>
#include <cstdlib>

// I2C defines
#define MPU6050_ADDR                   0x68

// MPU6050 register addresses
#define REG_PWR_MGMT_1                 0x6B
#define REG_GYRO_CONFIG                0x1B
#define REG_ACCEL_CONFIG               0x1C
#define REG_SMPLRT_DIV                 0x19
#define WHO_AM_I_REG                   0x75
//IMPORTANT! Some clone mpu have the address of i2c different of the default value 0x68 and 0x69
//so, identify the correct value and alter this macro
#define WHO_AM_I_VALUE                  0x72

#define REG_ACCEL_XOUT_H                0x3B
#define	REG_ACCEL_YOUT_H	               0x3D
#define	REG_ACCEL_ZOUT_H	               0x3F

#define	REG_GYRO_XOUT_H		0x43
#define	REG_GYRO_YOUT_H		0x45
#define	REG_GYRO_ZOUT_H		0x47

// Sensitivity scale factors for different ranges
#define ACCEL_SCALE_FACTOR_2G 16384.0 // for ±2g
#define ACCEL_SCALE_FACTOR_4G (float)8192.0  // for ±4g
#define ACCEL_SCALE_FACTOR_8G 4096.0  // for ±8g
#define ACCEL_SCALE_FACTOR_16G 2048.0 // for ±16g

#define GYRO_SCALE_FACTOR_250DPS (float)131.0 // for ±250 degrees per second
#define GYRO_SCALE_FACTOR_500DPS 65.5  // for ±500 degrees per second
#define GYRO_SCALE_FACTOR_1000DPS 32.8 // for ±1000 degrees per second
#define GYRO_SCALE_FACTOR_2000DPS 16.4 // for ±2000 degrees per second

// Select the desired scale factor
#define ACCEL_SCALE_FACTOR ACCEL_SCALE_FACTOR_4G   // Change this to the desired accelerometer range
#define GYRO_SCALE_FACTOR GYRO_SCALE_FACTOR_250DPS // Change this to the desired gyroscope range

// Corresponding configuration values
#define ACCEL_CONFIG_VALUE 0x08 // for ±4g
#define GYRO_CONFIG_VALUE 0x00  // for ±250 degrees per second
#define SAMPLE_RATE_DIV 1       // Sample rate = 1kHz / (1 + 1) = 500Hz

typedef struct
{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} gyro;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
}accel;

class mpu6050
{
private:
    /* data */
    i2c_inst *i2c_port;
    uint16_t sda;
    uint16_t scl;

    mutable float previousAngle = 0.0f;  // Para armazenar o ângulo anterior no filtro EMA
    float alpha = 0.5f;                   // Fator de suavização do filtro EMA

    void mpu6050_reset() const;

    void mpu6050_port_configure() const;

    void mpu6050_sensors_configure() const;

    /*
     * [NAME]:        Init
     * [FUNCTION]:    mpu6050_init()
     * [PARAMETERS]:  void
     * [DESCRIPTION]: Initialize every necessary pin and i2c parameters for the mpu6050 sensors usage
     */
    void mpu6050_init() const;
public:

    /*
     * [NAME]:        mpu6050
     * [FUNCTION]:    mpu6050()
     * [PARAMETERS]:  *i2c_port - type of i2c, sda - number of sda pin , scl - number of scl pin
     * [DESCRIPTION]: Constructor of class
     */
    mpu6050(i2c_inst *i2c_port,uint16_t sda,uint16_t scl);

    /*
     * [NAME]:        mpu6050
     * [FUNCTION]:    ~mpu6050()
     * [PARAMETERS]:  void
     * [DESCRIPTION]: Destructor of class
     */
    ~mpu6050();

    /*
     *
     * [NAME]:        getAccel
     * [FUNCTION]:    getAccel(accel *accel)
     * [PARAMETERS]:  accel *accel - pointer to accel struct to obtain data
     * [DESCRIPTION]: Read the raw values of accelerometer and store to a pointer accel struct
     */
    void getAccel(accel *accel) const;

    /*
     *
     * [NAME]:        getGyro
     * [FUNCTION]:    getGyro(gyro *gyro)
     * [PARAMETERS]:  gyro *gyro - pointer to gyro struct to obtain data
     * [DESCRIPTION]: Read the raw values of gyroscope and store to a pointer gyro struct
     */
    void getGyro(gyro *gyro) const;

    /*
     * [NAME]:        Print Raw
     * [FUNCTION]:    print_raw_data(accel accelData, gyro gyroData)
     * [PARAMETERS]:  accel accelData - reference to accelData structure, gyro gyroData - reference to gyroData structure
     * [DESCRIPTION]: Print the converted and formated values of accelerometer and gyroscope sensors of mpu6050 of type:
     *                aX = %.2f g | aY = %.2f g | aZ = %.2f g \n
     *                gX = %.2f dps | gY = %.2f dps | gZ = %.2f dps | \n
     */
    void print_raw_data(accel accelData, gyro gyroData) const;

    /*
    * [NAME]:        Format String
    * [FUNCTION]:    toString(char *buffer, size_t buffer_size, accel accelData, gyro gyroData)
    * [PARAMETERS]:  char* buffer - pointer to buffer that store the formated data
    *                size_t buffer_size size of the buffer
    *                accel accelData - reference to accelData structure,
    *                gyro gyroData - reference to gyroData structure,
    * [DESCRIPTION]: Return a formated string that contains the values of accelerometer and
    *                gyroscope sensors of mpu6050 by parameter pointer, of type:
    *                aX = %.2f g | aY = %.2f g | aZ = %.2f g \n
    *                gX = %.2f dps | gY = %.2f dps | gZ = %.2f dps | \n
    */
    void toString(char *buffer, size_t buffer_size, accel accelData, gyro gyroData) const;

    /**
     * [NAME]:        getAngle
     * [FUNCTION]:    getAngle(float &angle)
     * [PARAMETERS]:  float &angle - referência para armazenar o ângulo calculado
     * [DESCRIPTION]: Calcula o ângulo de inclinação da gangorra com base nos dados do acelerômetro,
     *                utilizando o método original sem filtragem.
     */
    void getAngle(float &angle) const;
    
    /**
     * [NAME]:        getAngleFiltered
     * [FUNCTION]:    getAngleFiltered(float &angleFiltered)
     * [PARAMETERS]:  float &angleFiltered - referência para armazenar o ângulo filtrado
     * [DESCRIPTION]: Calcula o ângulo de inclinação da gangorra com base nos dados do acelerômetro,
     *                utilizando o filtro de Média Móvel Exponencial (EMA).
     */
    void getAngleFiltered(float &angleFiltered) const;



};

#endif
