#include "stm32f4xx_hal.h"

typedef enum {
  SENS_OK       = 0x00,
  SENS_ERROR    = 0x01
} SENS_StatusTypeDef;
typedef enum {
  SENS_DATA_READY = 0x00,
  SENS_DATA_NOT_READY = 0x01
} SENS_DataReadyTypeDef;

// HTS221
#define HTS221_ADDR_W 0xBE
#define HTS221_ADDR_R 0xBF
#define HTS221_WHOIAM_ADDR 0x0F
#define HTS221_WHOIAM_VALUE 0xBC
#define HTS221_AV_CONF_ADDR 0x10
#define HTS221_AV_CONF_VALUE 0x1B
#define HTS221_CTRL_REG1_ADDR 0x20
#define HTS221_CTRL_REG1_VALUE 0x83
#define HTS221_CTRL_REG2_ADDR 0x21
#define HTS221_CTRL_REG2_VALUE 0x00
#define HTS221_CTRL_REG3_ADDR 0x22
#define HTS221_CTRL_REG3_VALUE 0x00
#define HTS221_START_READING 0x27

// LIS3MDL
#define LIS3MDL_ADDR_W 0x3C
#define LIS3MDL_ADDR_R 0x3D
#define LIS3MDL_WHOIAM_ADDR 0x0F
#define LIS3MDL_WHOIAM_VALUE 0x3D
#define LIS3MDL_CTRL_REG1_ADDR 0x20
#define LIS3MDL_CTRL_REG1_VALUE 0x10
#define LIS3MDL_CTRL_REG2_ADDR 0x21
#define LIS3MDL_CTRL_REG2_VALUE 0x00
#define LIS3MDL_CTRL_REG3_ADDR 0x22
#define LIS3MDL_CTRL_REG3_VALUE 0x00
#define LIS3MDL_CTRL_REG4_ADDR 0x23
#define LIS3MDL_CTRL_REG4_VALUE 0x00
#define LIS3MDL_CTRL_REG5_ADDR 0x24
#define LIS3MDL_CTRL_REG5_VALUE 0x00
#define LIS3MDL_START_READING 0x27
    
// LSM6DS0
#define LSM6DS0_ADDR_W 0xD6
#define LSM6DS0_ADDR_R 0xD7
#define LSM6DS0_WHOIAM_ADDR 0x0F
#define LSM6DS0_WHOIAM_VALUE 0x68
#define LSM6DS0_ACTTHS_ADDR 0x04
#define LSM6DS0_ACTTHS_VALUE 0x00
#define LSM6DS0_ACTDUR_ADDR 0x05
#define LSM6DS0_ACTDUR_VALUE 0x00
#define LSM6DS0_INT_GEN_CFG_XL_ADDR 0x06
#define LSM6DS0_INT_GEN_CFG_XL_VALUE 0x00
#define LSM6DS0_INT_GEN_THS_X_XL_ADDR 0x07
#define LSM6DS0_INT_GEN_THS_X_XL_VALUE 0x00
#define LSM6DS0_INT_GEN_THS_Y_XL_ADDR 0x08
#define LSM6DS0_INT_GEN_THS_Y_XL_VALUE 0x00
#define LSM6DS0_INT_GEN_THS_Z_XL_ADDR 0x09
#define LSM6DS0_INT_GEN_THS_Z_XL_VALUE 0x00
#define LSM6DS0_INT_GEN_DUR_XL_ADDR 0x0A
#define LSM6DS0_INT_GEN_DUR_XL_VALUE 0x00
#define LSM6DS0_REFG_ADDR 0x0B
#define LSM6DS0_REFG_VALUE 0x00
#define LSM6DS0_INT_CTRL_ADDR 0x0C
#define LSM6DS0_INT_CTRL_VALUE 0x00
#define LSM6DS0_CTRL_REG1_G_ADDR 0x10
#define LSM6DS0_CTRL_REG1_G_VALUE 0xC0
#define LSM6DS0_CTRL_REG2_G_ADDR 0x11
#define LSM6DS0_CTRL_REG2_G_VALUE 0x00
#define LSM6DS0_CTRL_REG3_G_ADDR 0x12
#define LSM6DS0_CTRL_REG3_G_VALUE 0x00
#define LSM6DS0_ORIENT_CFG_G_ADDR 0x13
#define LSM6DS0_ORIENT_CFG_G_VALUE 0x00
#define LSM6DS0_CTRL_REG4_ADDR 0x1E
#define LSM6DS0_CTRL_REG4_VALUE 0x38
#define LSM6DS0_CTRL_REG5_XL_ADDR 0x1F
#define LSM6DS0_CTRL_REG5_XL_VALUE 0x38
#define LSM6DS0_CTRL_REG6_XL_ADDR 0x20
#define LSM6DS0_CTRL_REG6_XL_VALUE 0x00
#define LSM6DS0_CTRL_REG7_XL_ADDR 0x21
#define LSM6DS0_CTRL_REG7_XL_VALUE 0x00
#define LSM6DS0_CTRL_REG8_ADDR 0x22
#define LSM6DS0_CTRL_REG8_VALUE 0x04
#define LSM6DS0_CTRL_REG9_ADDR 0x23
#define LSM6DS0_CTRL_REG9_VALUE 0x00
#define LSM6DS0_CTRL_REG10_ADDR 0x24
#define LSM6DS0_CTRL_REG10_VALUE 0x00
#define LSM6DS0_FIFO_CTRL_ADDR 0x2E
#define LSM6DS0_FIFO_CTRL_VALUE 0x00
#define LSM6DS0_INT_GEN_CFG_G_ADDR 0x30
#define LSM6DS0_INT_GEN_CFG_G_VALUE 0x00
#define LSM6DS0_INT_GEN_THS_XH_G_ADDR 0x31
#define LSM6DS0_INT_GEN_THS_XH_G_VALUE 0x00
#define LSM6DS0_INT_GEN_THS_XL_G_ADDR 0x32
#define LSM6DS0_INT_GEN_THS_XL_G_VALUE 0x00
#define LSM6DS0_INT_GEN_THS_YH_G_ADDR 0x33
#define LSM6DS0_INT_GEN_THS_YH_G_VALUE 0x00
#define LSM6DS0_INT_GEN_THS_YL_G_ADDR 0x34
#define LSM6DS0_INT_GEN_THS_YL_G_VALUE 0x00
#define LSM6DS0_INT_GEN_THS_ZH_G_ADDR 0x35
#define LSM6DS0_INT_GEN_THS_ZH_G_VALUE 0x00
#define LSM6DS0_INT_GEN_THS_ZL_G_ADDR 0x36
#define LSM6DS0_INT_GEN_THS_ZL_G_VALUE 0x00
#define LSM6DS0_INT_GEN_DUR_G_ADDR 0x37
#define LSM6DS0_INT_GEN_DUR_G_VALUE 0x00
#define LSM6DS0_START_READING_GYRO 0x18
#define LSM6DS0_START_READING_ACCE 0x28


// Verify sensors
SENS_StatusTypeDef userSensorHello(unsigned char address, unsigned char reg, unsigned char val);

// Configure sensors
void userConfig(void);
void userConfigComm(void);
void userConfigSens(void);
SENS_StatusTypeDef userConfigLSM6DS0(void);
SENS_StatusTypeDef userConfigLIS3MDL(void);
SENS_StatusTypeDef userConfigLPS25HB(void);
SENS_StatusTypeDef userConfigHTS221(void);

// Read sensors
SENS_StatusTypeDef userReadHTS221(uint8_t *read);
SENS_StatusTypeDef userReadLIS3MDL(uint8_t *read);
SENS_StatusTypeDef userReadLSM6DS0(uint8_t *acce, uint8_t *gyro, uint8_t *status);

// Interprete sensors
SENS_StatusTypeDef userInterpHTS221(uint8_t *read, uint16_t *humidity, int16_t *temperature);
void _userInterpHTS221_humidity(uint8_t *read, uint16_t *humidity);
void _userInterpHTS221_temperature(uint8_t *read, int16_t *temperature);
SENS_StatusTypeDef userInterpLIS3MDL(uint8_t *read, int16_t *magnetometer);
SENS_StatusTypeDef userInterpLSM6DS0(uint8_t *read_accelerometer, uint8_t *read_gyroscope, int16_t *accelerometer, int16_t *gyroscope);

// PWM
void userStartPWM(TIM_HandleTypeDef *htim, uint32_t Channel);
void userSetPWM(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t period, float percentage);
