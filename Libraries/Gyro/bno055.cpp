#include "bno055.h"

extern UART_HandleTypeDef huart2;

BNO055::BNO055(I2C_HandleTypeDef *hi2c)
{
  this->hi2c = hi2c;

  accelScale = 100;
  tempScale = 1;
  angularRateScale = 16;
  eulerScale = 16;
  magScale = 16;
  quaScale = (1<<14);    // 2^14
};

void BNO055::writeData(uint8_t reg, uint8_t data)
{
  uint8_t txdata[2] = {reg, data};
  uint8_t status;
  vTaskSuspendAll();
  status = HAL_I2C_Master_Transmit(this->hi2c, BNO055_I2C_ADDR << 1, txdata, sizeof(txdata), 10);
  xTaskResumeAll();
  if (status == HAL_OK)
	{
    return;
  }
  if (status == HAL_ERROR)
	{
    printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
  }
	else if (status == HAL_TIMEOUT)
	{
    printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
  }
	else if (status == HAL_BUSY)
	{
    printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
  }
	else
	{
    printf("Unknown status data %d", status);
  }

  uint32_t error = HAL_I2C_GetError(this->hi2c);
  if (error == HAL_I2C_ERROR_NONE)
	{
    return;
  }
	else if (error == HAL_I2C_ERROR_BERR)
	{
    printf("HAL_I2C_ERROR_BERR\r\n");
  }
	else if (error == HAL_I2C_ERROR_ARLO)
	{
    printf("HAL_I2C_ERROR_ARLO\r\n");
  }
  else if (error == HAL_I2C_ERROR_AF)
	{
    printf("HAL_I2C_ERROR_AF\r\n");
  }
	else if (error == HAL_I2C_ERROR_OVR)
	{
    printf("HAL_I2C_ERROR_OVR\r\n");
  }
	else if (error == HAL_I2C_ERROR_DMA)
	{
    printf("HAL_I2C_ERROR_DMA\r\n");
  }
	else if (error == HAL_I2C_ERROR_TIMEOUT)
	{
    printf("HAL_I2C_ERROR_TIMEOUT\r\n");
  }

  HAL_I2C_StateTypeDef state = HAL_I2C_GetState(this->hi2c);
  if (state == HAL_I2C_STATE_RESET)
	{
    printf("HAL_I2C_STATE_RESET\r\n");
  }
	else if (state == HAL_I2C_STATE_READY)
  {
    printf("HAL_I2C_STATE_RESET\r\n");
  }
	else if (state == HAL_I2C_STATE_BUSY)
	{
    printf("HAL_I2C_STATE_BUSY\r\n");
  }
	else if (state == HAL_I2C_STATE_BUSY_TX)
	{
    printf("HAL_I2C_STATE_BUSY_TX\r\n");
  }
	else if (state == HAL_I2C_STATE_BUSY_RX)
	{
    printf("HAL_I2C_STATE_BUSY_RX\r\n");
  }
	else if (state == HAL_I2C_STATE_LISTEN)
	{
    printf("HAL_I2C_STATE_LISTEN\r\n");
  }
	else if (state == HAL_I2C_STATE_BUSY_TX_LISTEN)
	{
    printf("HAL_I2C_STATE_BUSY_TX_LISTEN\r\n");
  }
	else if (state == HAL_I2C_STATE_BUSY_RX_LISTEN)
	{
    printf("HAL_I2C_STATE_BUSY_RX_LISTEN\r\n");
  }
	else if (state == HAL_I2C_STATE_ABORT)
	{
    printf("HAL_I2C_STATE_ABORT\r\n");
  }
	else if (state == HAL_I2C_STATE_TIMEOUT)
	{
    printf("HAL_I2C_STATE_TIMEOUT\r\n");
  }
	else if (state == HAL_I2C_STATE_ERROR)
	{
    printf("HAL_I2C_STATE_ERROR\r\n");
  }
}


void BNO055::readData(uint8_t reg, uint8_t *data, uint8_t len)
{
	vTaskSuspendAll();
  HAL_I2C_Master_Transmit(this->hi2c, BNO055_I2C_ADDR << 1, &reg, 1, 100);
  HAL_I2C_Master_Receive(this->hi2c, BNO055_I2C_ADDR << 1, data, len, 100);
  xTaskResumeAll();
}

void BNO055::setPage(uint8_t page)
{
  writeData(BNO055_PAGE_ID, page);
}

bno055_opmode_t BNO055::getOperationMode(void)
{
	uint8_t mode;
  readData(BNO055_OPR_MODE, &mode, 1);
  return (bno055_opmode_t)mode;
}

void BNO055::setOperationMode(bno055_opmode_t mode)
{
  writeData(BNO055_OPR_MODE, mode);
  if (mode == BNO055_OPERATION_MODE_CONFIG)
	{
    HAL_Delay(19);
  }
	else
	{
    HAL_Delay(7);
  }
}

void BNO055::setOperationModeConfig(void)
{
  setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void BNO055::setOperationModeNDOF(void)
{
  setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

void BNO055::setExternalCrystalUse(bool state)
{
  setPage(0);
  uint8_t tmp = 0;
  readData(BNO055_SYS_TRIGGER, &tmp, 1);
  tmp |= (state == true) ? 0x80 : 0x0;
  writeData(BNO055_SYS_TRIGGER, tmp);
  HAL_Delay(700);
}

void BNO055::enableExternalCrystal(void)
{
  setExternalCrystalUse(true);
}

void BNO055::disableExternalCrystal(void)
{
  setExternalCrystalUse(false);
}

void BNO055::reset(void)
{
  writeData(BNO055_SYS_TRIGGER, 0x20);
  HAL_Delay(700);
}

int8_t BNO055::bno055_getTemp(void)
{
  setPage(0);
  uint8_t t;
  readData(BNO055_TEMP, &t, 1);
  return t;
}

void BNO055::setup(void) {
  reset();

  uint8_t id = 0;
  char str[80];
  readData(BNO055_CHIP_ID, &id, 1);
  if (id != BNO055_ID) {
	  HAL_UART_Transmit(&huart2, (uint8_t*)str, sprintf(str, "Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n"), 1000);
	  //printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
  }
  setPage(0);
  writeData(BNO055_SYS_TRIGGER, 0x0);

  // Select BNO055 config mode
  setOperationModeConfig();
  HAL_Delay(10);
}

int16_t BNO055::getSWRevision(void)
{
  setPage(0);
  uint8_t buffer[2];
  readData(BNO055_SW_REV_ID_LSB, buffer, 2);
  return (int16_t)((buffer[1] << 8) | buffer[0]);
}

uint8_t BNO055::getBootloaderRevision(void)
{
  setPage(0);
  uint8_t tmp;
  readData(BNO055_BL_REV_ID, &tmp, 1);
  return tmp;
}

uint8_t BNO055::getSystemStatus(void)
{
  setPage(0);
  uint8_t tmp;
  readData(BNO055_SYS_STATUS, &tmp, 1);
  return tmp;
}

bno055_self_test_result_t BNO055::getSelfTestResult(void)
{
  setPage(0);
  uint8_t tmp;
  bno055_self_test_result_t res;
	res.mcuState = 0;
	res.gyrState = 0;
	res.magState = 0;
	res.accState = 0;

  readData(BNO055_ST_RESULT, &tmp, 1);
  res.mcuState = (tmp >> 3) & 0x01;
  res.gyrState = (tmp >> 2) & 0x01;
  res.magState = (tmp >> 1) & 0x01;
  res.accState = (tmp >> 0) & 0x01;
  return res;
}

uint8_t BNO055::getSystemError(void)
{
  setPage(0);
  uint8_t tmp;
  readData(BNO055_SYS_ERR, &tmp, 1);
  return tmp;
}

BNO055::bno055_calibration_state_t BNO055::getCalibrationState(void)
{
  setPage(0);
  bno055_calibration_state_t cal;
	cal.sys = 0;
	cal.gyro = 0;
	cal.mag = 0;
	cal.accel = 0;

  uint8_t calState = 0;

  readData(BNO055_CALIB_STAT, &calState, 1);
  cal.sys = (calState >> 6) & 0x03;
  cal.gyro = (calState >> 4) & 0x03;
  cal.accel = (calState >> 2) & 0x03;
  cal.mag = calState & 0x03;
  return cal;
}

BNO055::bno055_calibration_data_t BNO055::getCalibrationData(void)
{
  bno055_calibration_data_t calData;
  uint8_t buffer[22];
  bno055_opmode_t operationMode = getOperationMode();
  setOperationModeConfig();
  setPage(0);

  readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

  // Assumes little endian processor
  memcpy(&calData.offset.accel, buffer, 6);
  memcpy(&calData.offset.mag, buffer + 6, 6);
  memcpy(&calData.offset.gyro, buffer + 12, 6);
  memcpy(&calData.radius.accel, buffer + 18, 2);
  memcpy(&calData.radius.mag, buffer + 20, 2);

  setOperationMode(operationMode);

  return calData;
}

void BNO055::setCalibrationData(bno055_calibration_data_t calData)
{
  uint8_t buffer[22];
  bno055_opmode_t operationMode = getOperationMode();
  setOperationModeConfig();
  setPage(0);

  // Assumes litle endian processor
  memcpy(buffer, &calData.offset.accel, 6);
  memcpy(buffer + 6, &calData.offset.mag, 6);
  memcpy(buffer + 12, &calData.offset.gyro, 6);
  memcpy(buffer + 18, &calData.radius.accel, 2);
  memcpy(buffer + 20, &calData.radius.mag, 2);

  for (uint8_t i=0; i < 22; i++)
	{
    writeData(BNO055_ACC_OFFSET_X_LSB+i, buffer[i]);
  }

  setOperationMode(operationMode);
}

bno055_vector_t BNO055::bno055_getVector(uint8_t vec)
{
  setPage(0);
  uint8_t buffer[8];    // Quaternion need 8 bytes

  if (vec == BNO055_VECTOR_QUATERNION)
    readData(vec, buffer, 8);
  else
    readData(vec, buffer, 6);

  double scale = 1;

  if (vec == BNO055_VECTOR_MAGNETOMETER)
	{
    scale = magScale;
  }
	else if (vec == BNO055_VECTOR_ACCELEROMETER || vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY)
	{
    scale = accelScale;
  }
	else if (vec == BNO055_VECTOR_GYROSCOPE)
	{
    scale = angularRateScale;
  }
	else if (vec == BNO055_VECTOR_EULER)
	{
    scale = eulerScale;
  }
	else if (vec == BNO055_VECTOR_QUATERNION)
	{
    scale = quaScale;
  }

  bno055_vector_t xyz;
	xyz.w = 0;
	xyz.x = 0;
	xyz.y = 0;
	xyz.z = 0;

  if (vec == BNO055_VECTOR_QUATERNION)
	{
    xyz.w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
    xyz.z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
  }
	else
	{
    xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
  }
  return xyz;
}

bno055_vector_t BNO055::getVectorAccelerometer(void)
{
  return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}

bno055_vector_t BNO055::getVectorMagnetometer(void)
{
  return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}

bno055_vector_t BNO055::getVectorGyroscope(void)
{
  return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}

bno055_vector_t BNO055::getVectorEuler(void)
{
  return bno055_getVector(BNO055_VECTOR_EULER);
}

bno055_vector_t BNO055::getVectorLinearAccel(void)
{
  return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}

bno055_vector_t BNO055::getVectorGravity(void)
{
  return bno055_getVector(BNO055_VECTOR_GRAVITY);
}

bno055_vector_t BNO055::getVectorQuaternion(void)
{
  return bno055_getVector(BNO055_VECTOR_QUATERNION);
}

void BNO055::setAxisMap(bno055_axis_map_t axis)
{
  uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
  uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
  writeData(BNO055_AXIS_MAP_CONFIG, axisRemap);
  writeData(BNO055_AXIS_MAP_SIGN, axisMapSign);
}

bno055_vector_t BNO055::getVectorEulerRemap(void)
{
	bno055_vector_t buf;
	bno055_vector_t output;
	buf = bno055_getVector(BNO055_VECTOR_EULER);
    output.z = -buf.y;
    output.y = buf.x;
    output.x = buf.z;

    return output;
}

bno055_vector_t BNO055::getVectorGyroscopeRemap(void)
{
	bno055_vector_t buf;
	bno055_vector_t output;
	buf = bno055_getVector(BNO055_VECTOR_GYROSCOPE);
    output.z = buf.y;
    output.y = buf.z;
    output.x = -buf.x;

    return output;
}

bno055_vector_t BNO055::getVectorAccelerometerRemap(void)
{
	bno055_vector_t buf;
	bno055_vector_t output;
	buf = bno055_getVector(BNO055_VECTOR_LINEARACCEL);
	output.z = buf.y;
	output.y = buf.z;
	output.x = -buf.x;

	return output;
}
