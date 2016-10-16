#include "user.h"

extern I2C_HandleTypeDef hi2c1;
extern uint8_t HTS221_calib[16];

// Verify sensors
SENS_StatusTypeDef userSensorHello(unsigned char address, unsigned char reg, unsigned char val) {
  uint8_t txBuffer[1], rxBuffer[1];
  
  txBuffer[0] = reg;
  HAL_I2C_Master_Transmit(&hi2c1, address, (uint8_t*) txBuffer, (uint16_t) sizeof(txBuffer), (uint32_t)1000);
  if(HAL_I2C_Master_Receive(&hi2c1, address, (uint8_t*) rxBuffer, 1, 1000) != HAL_OK) {
    return SENS_ERROR;
  } else {
    return (rxBuffer[0] == val) ? SENS_OK : SENS_ERROR;
  }
}

// Configure sensors
void userConfig() {
  userConfigComm();
  userConfigSens();
}
void userConfigComm() {
}
void userConfigSens() {
  userConfigLIS3MDL();
  userConfigLPS25HB();
  userConfigHTS221();
}
SENS_StatusTypeDef userConfigLSM6DS0() {
  uint8_t txBuffer[2];
  
  if(userSensorHello(LSM6DS0_ADDR_R, LSM6DS0_WHOIAM_ADDR, LSM6DS0_WHOIAM_VALUE) == SENS_OK) {
    txBuffer[0] = LSM6DS0_ACTTHS_ADDR;
    txBuffer[1] = LSM6DS0_ACTTHS_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_ACTDUR_ADDR;
    txBuffer[1] = LSM6DS0_ACTDUR_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_CFG_XL_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_CFG_XL_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_THS_X_XL_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_THS_X_XL_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_THS_Y_XL_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_THS_Y_XL_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_THS_Z_XL_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_THS_Z_XL_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_DUR_XL_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_DUR_XL_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_REFG_ADDR;
    txBuffer[1] = LSM6DS0_REFG_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_CTRL_REG1_G_ADDR;
    txBuffer[1] = LSM6DS0_CTRL_REG1_G_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_CTRL_REG2_G_ADDR;
    txBuffer[1] = LSM6DS0_CTRL_REG2_G_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_CTRL_REG3_G_ADDR;
    txBuffer[1] = LSM6DS0_CTRL_REG3_G_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_ORIENT_CFG_G_ADDR;
    txBuffer[1] = LSM6DS0_ORIENT_CFG_G_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_CTRL_REG4_ADDR;
    txBuffer[1] = LSM6DS0_CTRL_REG4_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_CTRL_REG5_XL_ADDR;
    txBuffer[1] = LSM6DS0_CTRL_REG5_XL_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_CTRL_REG6_XL_ADDR;
    txBuffer[1] = LSM6DS0_CTRL_REG6_XL_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_CTRL_REG7_XL_ADDR;
    txBuffer[1] = LSM6DS0_CTRL_REG7_XL_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_CTRL_REG8_ADDR;
    txBuffer[1] = LSM6DS0_CTRL_REG8_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_CTRL_REG9_ADDR;
    txBuffer[1] = LSM6DS0_CTRL_REG9_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_CTRL_REG10_ADDR;
    txBuffer[1] = LSM6DS0_CTRL_REG10_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_FIFO_CTRL_ADDR;
    txBuffer[1] = LSM6DS0_FIFO_CTRL_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_CFG_G_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_CFG_G_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_THS_XH_G_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_THS_XH_G_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_THS_XL_G_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_THS_XL_G_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_THS_YH_G_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_THS_YH_G_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_THS_YL_G_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_THS_YL_G_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_THS_ZH_G_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_THS_ZH_G_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_THS_ZL_G_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_THS_ZL_G_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LSM6DS0_INT_GEN_DUR_G_ADDR;
    txBuffer[1] = LSM6DS0_INT_GEN_DUR_G_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
  }
  
  return SENS_OK;
}
SENS_StatusTypeDef userConfigLIS3MDL() {
  uint8_t txBuffer[2];
  
  if(userSensorHello(LIS3MDL_ADDR_R, LIS3MDL_WHOIAM_ADDR, LIS3MDL_WHOIAM_VALUE) == SENS_OK) {
    txBuffer[0] = LIS3MDL_CTRL_REG1_ADDR;
    txBuffer[1] = LIS3MDL_CTRL_REG1_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LIS3MDL_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LIS3MDL_CTRL_REG2_ADDR;
    txBuffer[1] = LIS3MDL_CTRL_REG2_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LIS3MDL_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LIS3MDL_CTRL_REG3_ADDR;
    txBuffer[1] = LIS3MDL_CTRL_REG3_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LIS3MDL_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LIS3MDL_CTRL_REG4_ADDR;
    txBuffer[1] = LIS3MDL_CTRL_REG4_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LIS3MDL_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = LIS3MDL_CTRL_REG5_ADDR;
    txBuffer[1] = LIS3MDL_CTRL_REG5_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, LIS3MDL_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    return SENS_OK;
  } else {
    return SENS_ERROR;
  }
}
SENS_StatusTypeDef userConfigLPS25HB() {
  
  return SENS_OK;
}
SENS_StatusTypeDef userConfigHTS221() {
  uint8_t txBuffer[2];
  
  if(userSensorHello(HTS221_ADDR_R, HTS221_WHOIAM_ADDR, HTS221_WHOIAM_VALUE) == SENS_OK) {
    txBuffer[0] = HTS221_AV_CONF_ADDR;
    txBuffer[1] = HTS221_AV_CONF_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, HTS221_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = HTS221_CTRL_REG1_ADDR;
    txBuffer[1] = HTS221_CTRL_REG1_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, HTS221_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = HTS221_CTRL_REG2_ADDR;
    txBuffer[1] = HTS221_CTRL_REG2_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, HTS221_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = HTS221_CTRL_REG3_ADDR;
    txBuffer[1] = HTS221_CTRL_REG3_VALUE;
    HAL_I2C_Master_Transmit(&hi2c1, HTS221_ADDR_W, (uint8_t*) txBuffer, (uint16_t) 2, (uint32_t)1000);
    
    txBuffer[0] = 0x30 + 0x80;
    HAL_I2C_Master_Transmit(&hi2c1, HTS221_ADDR_R, (uint8_t*) txBuffer, (uint16_t) 1, (uint32_t)1000);
    HAL_I2C_Master_Receive(&hi2c1, HTS221_ADDR_R, &HTS221_calib[0], 16, 1000);
    
    return SENS_OK;
  } else {
    return SENS_ERROR;
  }
}

// Read sensors
SENS_StatusTypeDef userReadLSM6DS0(uint8_t *acce, uint8_t *gyro, uint8_t *status) {
  uint8_t txBuffer[1];
  
  txBuffer[0] = LSM6DS0_START_READING_ACCE;
  HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_R, (uint8_t*) txBuffer, (uint16_t) sizeof(txBuffer), (uint32_t)1000);
  HAL_I2C_Master_Receive(&hi2c1, LSM6DS0_ADDR_R, acce, 6, 1000);
  
  txBuffer[0] = LSM6DS0_START_READING_GYRO;
  HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_R, (uint8_t*) txBuffer, (uint16_t) sizeof(txBuffer), (uint32_t)1000);
  HAL_I2C_Master_Receive(&hi2c1, LSM6DS0_ADDR_R, gyro, 6, 1000);
  
  //txBuffer[0] = 0x17;
  //HAL_I2C_Master_Transmit(&hi2c1, LSM6DS0_ADDR_R, (uint8_t*) txBuffer, (uint16_t) sizeof(txBuffer), (uint32_t)1000);
  //HAL_I2C_Master_Receive(&hi2c1, LSM6DS0_ADDR_R, status, 1, 1000);
  
  return SENS_OK;
}
SENS_StatusTypeDef userReadHTS221(uint8_t *read) {
  uint8_t txBuffer[1];
  
  txBuffer[0] = HTS221_START_READING + 0x80;
  HAL_I2C_Master_Transmit(&hi2c1, HTS221_ADDR_R, (uint8_t*) txBuffer, (uint16_t) sizeof(txBuffer), (uint32_t)1000);
  HAL_I2C_Master_Receive(&hi2c1, HTS221_ADDR_R, read, 5, 1000);
  
  return SENS_OK;
}
SENS_StatusTypeDef userReadLIS3MDL(uint8_t *read) {
  uint8_t txBuffer[1];
  
  txBuffer[0] = LIS3MDL_START_READING + 0x80;
  HAL_I2C_Master_Transmit(&hi2c1, LIS3MDL_ADDR_R, (uint8_t*) txBuffer, (uint16_t) sizeof(txBuffer), (uint32_t)1000);
  HAL_I2C_Master_Receive(&hi2c1, LIS3MDL_ADDR_R, read, 9, 1000);
  
  return SENS_OK;
}

// Interprete sensors readings
SENS_StatusTypeDef userInterpHTS221(uint8_t *read, uint16_t *humidity, int16_t *temperature) {
  _userInterpHTS221_humidity(read, humidity);
  _userInterpHTS221_temperature(read, temperature);
  return SENS_OK;
}
void _userInterpHTS221_humidity(uint8_t *read, uint16_t *humidity) {
  int16_t H0_T0_OUT, H1_T0_OUT, H_T_OUT;
  int16_t H0_rh, H1_rh;
  uint8_t buffer[2];
  uint32_t tmp;
  
  buffer[0] = HTS221_calib[0];
  buffer[1] = HTS221_calib[1];
  H0_rh = buffer[0]>>1;
  H1_rh = buffer[1]>>1;
  
  buffer[0] = HTS221_calib[6];
  buffer[1] = HTS221_calib[7];
  H0_T0_OUT = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
  
  buffer[0] = HTS221_calib[10];
  buffer[1] = HTS221_calib[11];
  H1_T0_OUT = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
  
  buffer[0] = (uint8_t) *(read+1);
  buffer[1] = (uint8_t) *(read+2);
  H_T_OUT = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
  
  tmp = ((uint32_t)(H_T_OUT - H0_T0_OUT)) * ((uint32_t)(H1_rh - H0_rh)*10);
  *humidity = tmp/(H1_T0_OUT - H0_T0_OUT) + H0_rh*10;
  
  if(*humidity>1000) *humidity = 1000;
}
void _userInterpHTS221_temperature(uint8_t *read, int16_t *temperature) {
  int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
  int16_t T0_degC, T1_degC;
  uint8_t buffer[4], tmp;
  uint32_t tmp32;
  
  buffer[0] = HTS221_calib[2];
  buffer[1] = HTS221_calib[3];
  
  tmp = HTS221_calib[5];
  
  T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)buffer[0]);
  T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)buffer[1]);
  T0_degC = T0_degC_x8_u16>>3;
  T1_degC = T1_degC_x8_u16>>3;
  
  buffer[0] = HTS221_calib[12];
  buffer[1] = HTS221_calib[13];
  buffer[2] = HTS221_calib[14];
  buffer[3] = HTS221_calib[15];
  
  T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
  T1_out = (((uint16_t)buffer[3])<<8) | (uint16_t)buffer[2];
  
  buffer[0] = (uint8_t) *(read+3);
  buffer[1] = (uint8_t) *(read+4);
  T_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
  
  tmp32 = ((uint32_t)(T_out - T0_out)) * ((uint32_t)(T1_degC - T0_degC)*10);
  *temperature = tmp32 /(T1_out - T0_out) + T0_degC*10;
}
SENS_StatusTypeDef userInterpLIS3MDL(uint8_t *read, int16_t *magnetometer) {
  magnetometer[0] = ((int16_t)read[2]<<8) + ((int16_t)read[1]);
  magnetometer[1] = ((int16_t)read[4]<<8) + ((int16_t)read[3]);
  magnetometer[2] = ((int16_t)read[6]<<8) + ((int16_t)read[5]);
  
  return SENS_OK;
}
SENS_StatusTypeDef userInterpLSM6DS0(uint8_t *read_accelerometer, uint8_t *read_gyroscope, int16_t *accelerometer, int16_t *gyroscope) {
  accelerometer[0] = ((int16_t)read_accelerometer[1]<<8) + ((int16_t)read_accelerometer[0]);
  accelerometer[1] = ((int16_t)read_accelerometer[3]<<8) + ((int16_t)read_accelerometer[2]);
  accelerometer[2] = ((int16_t)read_accelerometer[5]<<8) + ((int16_t)read_accelerometer[4]);
  
  gyroscope[0] = ((int16_t)read_gyroscope[1]<<8) + ((int16_t)read_gyroscope[0]);
  gyroscope[1] = ((int16_t)read_gyroscope[3]<<8) + ((int16_t)read_gyroscope[2]);
  gyroscope[2] = ((int16_t)read_gyroscope[5]<<8) + ((int16_t)read_gyroscope[4]);
  
  return SENS_OK;
}

