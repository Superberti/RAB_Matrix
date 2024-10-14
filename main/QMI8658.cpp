/*
 * QMI8658 Sensor
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "QMI8658.h"
#include <string.h>
#include "driver/gpio.h"

#define DEV_TIMEOUT 100
#define TAG "QMI8658"

QMI8658::QMI8658()
{
  mDevHandle = NULL;
  mBusHandle = NULL;
  mFifoBuffer = new uint8_t[1536]; // Maximalgröße FIFO
}

esp_err_t QMI8658::Init(i2c_master_bus_handle_t aBusHandle, uint8_t aI2CAddr, uint32_t aI2CSpeed_Hz, 
gpio_num_t aIrqPin1, gpio_num_t aIrqPin2,bool ReInit)
{
  mIrqPin1 = aIrqPin1;
  mIrqPin2 = aIrqPin2;
  esp_err_t ret = ESP_OK;
  mBusHandle = aBusHandle;
  if (mDevHandle == NULL || ReInit)
  {
    i2c_device_config_t conf;
    conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    conf.device_address = aI2CAddr;
    conf.scl_speed_hz = aI2CSpeed_Hz;
    conf.flags.disable_ack_check = false;
    ret = i2c_master_bus_add_device(mBusHandle, &conf, &mDevHandle);
    if (ret != ESP_OK)
      return ret;

    uint8_t buffer[6] = {0};

    // Wird schon im Hauptprogramm aktiviert
    // IRQ 0/1-Input aktivieren
    // if (__irq != -1) {
    //    this->setGpioMode(__irq, INPUT);
    //}

    ret = Reset();
    if (ret != ESP_OK)
      return ret;

    uint8_t id = WhoAmI();
    if (id != QMI8658_REG_WHOAMI_DEFAULT)
    {
      ESP_LOGE(TAG, "ERROR! ID NOT MATCH QMI8658 , Response id is 0x%x", id);
      return ESP_ERR_INVALID_RESPONSE;
    }
    // Enable address auto increment, Big-Endian format
    // writeRegister(QMI8658_REG_CTRL1, 0x60);

    // Little-Endian / address auto increment
    // writeRegister(QMI8658_REG_CTRL1, 0x40);

    // no need . reset function has set
    // EN.ADDR_AI
    // setRegisterBit(QMI8658_REG_CTRL1, 6);

    // Use STATUS_INT.bit7 as CTRL9 handshake
    ret = WriteRegister8(QMI8658_REG_CTRL8, 0x80);
    if (ret != ESP_OK)
      return ret;

    // Get firmware version and usid
    ret = WriteCommand(CTRL_CMD_COPY_USID);
    if (ret != ESP_OK)
      return ret;

    ret = ReadRegister(QMI8658_REG_DQW_L, buffer, 3);
    if (ret != ESP_OK)
      return ret;
    mRevisionID = buffer[0] | (uint32_t)(buffer[1] << 8) | (uint32_t)(buffer[2] << 16);
    ESP_LOGI(TAG, "FW Version :0x%02X%02X%02X", buffer[0], buffer[1], buffer[2]);

    ret = ReadRegister(QMI8658_REG_DVX_L, usid, 6);
    if (ret != ESP_OK)
      return ret;
    ESP_LOGI(TAG, "USID :%02X%02X%02X%02X%02X%02X",
             usid[0], usid[1], usid[2],
             usid[3], usid[4], usid[5]);
  }
  return ESP_OK;
}

QMI8658::~QMI8658(void)
{
  Close();
  delete[] mFifoBuffer;
}

void QMI8658::Close()
{
  if (mDevHandle != NULL)
  {
    i2c_master_bus_rm_device(mDevHandle);
    mDevHandle = NULL;
  }
}

esp_err_t QMI8658::WriteCommand(CommandTable cmd, uint32_t wait_ms)
{
  int val;
  int64_t StartTime_us = 0;
  esp_err_t ret;
  ret = WriteRegister8(QMI8658_REG_CTRL9, (uint8_t)cmd);
  if (ret != ESP_OK)
    return ret;

  StartTime_us = GetTime_us();
  do
  {
    val = ReadRegister8(QMI8658_REG_STATUS_INT, &ret);
    vTaskDelay(pdMS_TO_TICKS(1));
    if (ret != ESP_OK)
      return ret;
    if (GetTime_us() - StartTime_us > wait_ms * 1000)
    {
      ESP_LOGE(TAG, "wait for ctrl9 command done time out : %d val:%d", cmd, val);
      return ESP_ERR_TIMEOUT;
    }
  } while (!(val & 0x80));

  ret = WriteRegister8(QMI8658_REG_CTRL9, CTRL_CMD_ACK);
  if (ret != ESP_OK)
    return ret;

  StartTime_us = GetTime_us();
  do
  {
    val = ReadRegister8(QMI8658_REG_STATUS_INT, &ret);
    vTaskDelay(pdMS_TO_TICKS(1));
    if (ret != ESP_OK)
      return ret;

    if (GetTime_us() - StartTime_us > wait_ms * 1000)
    {
      ESP_LOGE(TAG, "Clear ctrl9 command done flag timeout : %d val:%d", cmd, val);
      return ESP_ERR_TIMEOUT;
    }
  } while ((val & 0x80));

  return ret;
}

esp_err_t QMI8658::ReadRegister(uint8_t aStartReg, uint8_t *aReadBuf, uint8_t aSize)
{
  esp_err_t ret;
  ret = i2c_master_transmit_receive(mDevHandle, &aStartReg, 1, aReadBuf, 2, DEV_TIMEOUT);
  if (ret != ESP_OK)
    ESP_LOGE("ADS1015::ReadRegister", "I2C error no.: %d", ret);
  return ret;
}

esp_err_t QMI8658::WriteRegister(uint8_t aStartReg, uint8_t *aWriteBuf, uint8_t aSize)
{
  esp_err_t ret;
  uint8_t *wb = new uint8_t[aSize + 1];
  wb[0] = aStartReg;
  memcpy(wb + 1, aWriteBuf, aSize);
  ret = i2c_master_transmit(mDevHandle, wb, aSize + 1, DEV_TIMEOUT);
  delete[] wb;
  if (ret != ESP_OK)
    ESP_LOGE("ADS1015::WriteRegister", "I2C error no.: %d", ret);
  return ret;
}

esp_err_t QMI8658::WriteRegister(uint8_t reg, uint8_t norVal, uint8_t orVal)
{
  esp_err_t ret;
  uint8_t val = ReadRegister8(reg, &ret);
  if (ret != ESP_OK)
    return ret;

  val &= norVal;
  val |= orVal;
  return WriteRegister8(reg, val);
}

uint8_t QMI8658::ReadRegister8(uint8_t aAddr, esp_err_t *aErr)
{
  uint8_t buf = 0;
  esp_err_t ret = ReadRegister(aAddr, &buf, 1);
  if (aErr)
    *aErr = ret;
  return buf;
}

esp_err_t QMI8658::WriteRegister8(uint8_t aAddr, uint8_t aWriteByte)
{
  return WriteRegister(aAddr, &aWriteByte, 1);
}

esp_err_t QMI8658::Reset(bool waitResult)
{
  esp_err_t ret;
  int val = 0; // initialize with some value to avoid compilation errors
  ret = WriteRegister8(QMI8658_REG_RESET, QMI8658_REG_RESET_DEFAULT);
  if (ret != ESP_OK)
    return ret;
  // Maximum 15ms for the Reset process to be finished
  if (waitResult)
  {
    int64_t start = GetTime_us();
    esp_err_t ret;
    while (GetTime_us() - start < 15000)
    {
      val = ReadRegister8(QMI8658_REG_RST_RESULT, &ret);
      if (ret == ESP_OK && val == QMI8658_REG_RST_RESULT_VAL)
      {
        // EN.ADDR_AI
        SetRegisterBit(QMI8658_REG_CTRL1, 6);
        return true;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(TAG, "Reset chip failed, Response val = %d - 0x%X", val, val);
    return false;
  }

  // EN.ADDR_AI
  SetRegisterBit(QMI8658_REG_CTRL1, 6);

  return true;
}

int64_t QMI8658::GetTime_us()
{
  return xTaskGetTickCount() * 1000; // esp_timer_get_time();
}

esp_err_t QMI8658::SetRegisterBit(uint8_t aAddr, uint8_t bit)
{
  esp_err_t ret;
  uint8_t val = ReadRegister8(aAddr, &ret);
  if (ret != ESP_OK)
    return ret;
  return WriteRegister8(aAddr, (val | (_BV(bit))));
}

bool QMI8658::GetRegisterBit(uint8_t aAddr, uint8_t bit, esp_err_t *aErr)
{
  esp_err_t ret;
  esp_err_t *err = aErr != NULL ? aErr : &ret;
  uint8_t val = ReadRegister8(aAddr, err);
  if (*err != ESP_OK)
    return false;
  return val & _BV(bit);
}

esp_err_t QMI8658::ClrRegisterBit(uint8_t aAddr, uint8_t bit)
{
  esp_err_t ret;
  uint8_t val = ReadRegister8(aAddr, &ret);
  if (ret != ESP_OK)
    return ret;

  return WriteRegister8(aAddr, (val & (~_BV(bit))));
}

uint8_t QMI8658::GetChipID(esp_err_t *aErr)
{
  return ReadRegister8(QMI8658_REG_REVISION, aErr);
}

uint8_t QMI8658::WhoAmI(esp_err_t *aErr)
{
  return ReadRegister8(QMI8658_REG_WHOAMI, aErr);
}

uint32_t QMI8658::GetTimestamp(esp_err_t *aErr)
{
  uint8_t buffer[3];
  uint32_t timestamp;
  esp_err_t ret = ReadRegister(QMI8658_REG_TIMESTAMP_L, buffer, 3);
  if (aErr != NULL)
    *aErr = ret;
  if (ret != ESP_OK)
    return 0;
  timestamp = (uint32_t)(((uint32_t)buffer[2] << 16) |
                         ((uint32_t)buffer[1] << 8) | buffer[0]);
  if (timestamp > mLastTimestamp)
  {
    mLastTimestamp = timestamp;
  }
  else
  {
    mLastTimestamp = (timestamp + 0x1000000 - mLastTimestamp);
  }
  return mLastTimestamp;
}

float QMI8658::GetTemperature_C(esp_err_t *aErr)
{
  uint8_t buffer[2];
  esp_err_t ret = ReadRegister(QMI8658_REG_TEMPERATURE_L, buffer, 2);
  if (aErr != NULL)
    *aErr = ret;
  if (ret != ESP_OK)
    return 0;
  return (float)buffer[1] + ((float)buffer[0] / 256.0);
}

esp_err_t QMI8658::EnableINT(IntPin pin, bool enable)
{
  esp_err_t ret = ESP_OK;
  uint8_t BitPos = pin == INTERRUPT_PIN_1 ? 3 : 4;
  uint8_t EnableMask = pin == INTERRUPT_PIN_1 ? 0x01 : 0x02;
  uint8_t DisableMask = pin == INTERRUPT_PIN_1 ? 0xFE : 0xFD;
  if (enable)
  {
    mIrqEnableMask = mIrqEnableMask | EnableMask;
    ret = SetRegisterBit(QMI8658_REG_CTRL1, BitPos);
  }
  else
  {
    mIrqEnableMask = mIrqEnableMask & DisableMask;
    ret = ClrRegisterBit(QMI8658_REG_CTRL1, BitPos);
  }
  return ret;
}

uint8_t QMI8658::GetIrqStatus(esp_err_t *aErr)
{
  return ReadRegister8(QMI8658_REG_STATUS_INT, aErr);
}

esp_err_t QMI8658::EnableDataReadyINT(bool enable)
{
  esp_err_t ret = ESP_OK;
  if (enable)
    ret = ClrRegisterBit(QMI8658_REG_CTRL7, 5);
  else
    ret = SetRegisterBit(QMI8658_REG_CTRL7, 5);
  return ret;
}

esp_err_t QMI8658::ConfigAccelerometer(AccelRange range, AccelODR odr, LpfMode lpfOdr)
{
  bool en = IsEnableAccelerometer();

  if (en)
  {
    DisableAccelerometer();
  }

  // setAccelRange
  esp_err_t ret = WriteRegister(QMI8658_REG_CTRL2, 0x8F, (range << 4));
  if (ret != ESP_OK)
    return ret;

  switch (range)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
  // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that
  // 2-bit value:
  case ACC_RANGE_2G:
    mAccelScales = 2.0 / 32768.0;
    break;
  case ACC_RANGE_4G:
    mAccelScales = 4.0 / 32768.0;
    break;
  case ACC_RANGE_8G:
    mAccelScales = 8.0 / 32768.0;
    break;
  case ACC_RANGE_16G:
    mAccelScales = 16.0 / 32768.0;
    break;
  }

  // setAccelOutputDataRate
  ret = WriteRegister(QMI8658_REG_CTRL2, 0xF0, odr);
  if (ret != ESP_OK)
    return ret;

  if (lpfOdr != LPF_OFF)
  {
    // setAccelLowPassFitterOdr
    ret = WriteRegister(QMI8658_REG_CTRL5, QMI8658_ACCEL_LPF_MASK, (lpfOdr << 1));
    if (ret != ESP_OK)
      return ret;
    // Enable Low-Pass Fitter
    ret = SetRegisterBit(QMI8658_REG_CTRL5, 0);
    if (ret != ESP_OK)
      return ret;
  }
  else
  {
    // Disable Low-Pass Fitter
    ret = ClrRegisterBit(QMI8658_REG_CTRL5, 0);
    if (ret != ESP_OK)
      return ret;
  }

  // setAccelSelfTest
  // selfTest ? setRegisterBit(QMI8658_REG_CTRL2, 7) : clrRegisterBit(QMI8658_REG_CTRL2, 7);

  if (en)
  {
    return EnableAccelerometer();
  }

  return ESP_OK;
}

esp_err_t QMI8658::ConfigGyroscope(GyroRange range, GyroODR odr, LpfMode lpfOdr)
{
  esp_err_t ret;
  bool en = IsEnableGyroscope();

  if (en)
  {
    ret = DisableGyroscope();
    if (ret != ESP_OK)
      return ret;
  }

  // setGyroRange
  ret = WriteRegister(QMI8658_REG_CTRL3, 0x8F, (range << 4));
  if (ret != ESP_OK)
    return ret;

  switch (range)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
  // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that
  // 2-bit value:
  case GYR_RANGE_16DPS:
    mGyroScales = 16.0 / 32768.0;
    break;
  case GYR_RANGE_32DPS:
    mGyroScales = 32.0 / 32768.0;
    break;
  case GYR_RANGE_64DPS:
    mGyroScales = 64.0 / 32768.0;
    break;
  case GYR_RANGE_128DPS:
    mGyroScales = 128.0 / 32768.0;
    break;
  case GYR_RANGE_256DPS:
    mGyroScales = 256.0 / 32768.0;
    break;
  case GYR_RANGE_512DPS:
    mGyroScales = 512.0 / 32768.0;
    break;
  case GYR_RANGE_1024DPS:
    mGyroScales = 1024.0 / 32768.0;
    break;
  }

  // setGyroOutputDataRate
  ret = WriteRegister(QMI8658_REG_CTRL3, 0xF0, odr);
  if (ret != ESP_OK)
    return ret;

  // setGyroLowPassFitterOdr
  if (lpfOdr != LPF_OFF)
  {
    ret = WriteRegister(QMI8658_REG_CTRL5, QMI8658_GYRO_LPF_MASK, (lpfOdr << 5));
    if (ret != ESP_OK)
      return ret;

    // Enable Low-Pass Fitter
    ret = SetRegisterBit(QMI8658_REG_CTRL5, 4);
    if (ret != ESP_OK)
      return ret;
  }
  else
  {
    // Disable Low-Pass Fitter
    ret = ClrRegisterBit(QMI8658_REG_CTRL5, 4);
    if (ret != ESP_OK)
      return ret;
  }

  // setGyroSelfTest
  // selfTest ? setRegisterBit(QMI8658_REG_CTRL3, 7) : clrRegisterBit(QMI8658_REG_CTRL3, 7);

  if (en)
  {
    return EnableGyroscope();
  }

  return ESP_OK;
}

esp_err_t QMI8658::ConfigFIFO(FIFO_Mode mode, FIFO_Samples samples, IntPin pin, uint8_t trigger_samples)
{
  bool enGyro = IsEnableGyroscope();
  bool enAccel = IsEnableAccelerometer();
  esp_err_t ret;
  if (enGyro)
  {
    ret = DisableGyroscope();
    if (ret != ESP_OK)
      return ret;
  }

  if (enAccel)
  {
    ret = DisableAccelerometer();
    if (ret != ESP_OK)
      return ret;
  }

  // Reset FIFO configure
  ret = WriteCommand(CTRL_CMD_RST_FIFO);
  if (ret != ESP_OK)
  {
    return ret;
    ESP_LOGE(TAG, "Reset fifo failed!");
  }

  mFifoInterrupt = true;

  switch (pin)
  {
  case INTERRUPT_PIN_1:
    ret = SetRegisterBit(QMI8658_REG_CTRL1, 2);
    if (ret != ESP_OK)
      return ret;
    break;
  case INTERRUPT_PIN_2:
    ret = ClrRegisterBit(QMI8658_REG_CTRL1, 2);
    if (ret != ESP_OK)
      return ret;
    break;
  case INTERRUPT_PIN_DISABLE:
    // Saves whether the fifo interrupt pin is enabled
    mFifoInterrupt = false;
    break;
  default:
    break;
  }

  // Set fifo mode and samples len
  mFifoMode = (samples << 2) | mode;
  ret = WriteRegister8(QMI8658_REG_FIFO_CTRL, mFifoMode);
  if (ret != ESP_OK)
    return ret;

  /*
   * The FIFO_WTM register(0x13) indicates the expected level of FIFO data that host wants to get the FIFO Watermark interrupt.
   * The unit is sample, which means 6 bytes if one of accelerometer and gyroscope is enabled, and 12 bytes if both are enabled.
   * */
  ret = WriteRegister8(QMI8658_REG_FIFO_WTM_TH, trigger_samples);
  if (ret != ESP_OK)
    return ret;

  if (enGyro)
  {
    ret = EnableGyroscope();
    if (ret != ESP_OK)
      return ret;
  }

  if (enAccel)
  {
    ret = EnableAccelerometer();
    if (ret != ESP_OK)
      return ret;
  }

  uint8_t res = ReadRegister8(QMI8658_REG_FIFO_CTRL, &ret);
  if (ret != ESP_OK)
    return ret;
  ESP_LOGI(TAG, "QMI8658_REG_FIFO_CTRL : 0x%X", res);
  if ((res & 0x02) == 0x02)
  {
    ESP_LOGI(TAG, "Enabled Stream mode.");
  }
  else if ((res & 0x01) == 0x01)
  {
    ESP_LOGI(TAG, "Enabled FIFO mode.");
  }
  else if ((res & 0x03) == 0x00)
  {
    ESP_LOGI(TAG, "Disabled FIFO.");
  }
  res >>= 2;
  if ((res & 0x03) == 0x03)
  {
    ESP_LOGI(TAG, "128 samples.");
  }
  else if ((res & 0x02) == 0x02)
  {
    ESP_LOGI(TAG, "64 samples.");
  }
  else if ((res & 0x01) == 0x01)
  {
    ESP_LOGI(TAG, "32 samples.");
  }
  else if ((res & 0x03) == 0x00)
  {
    ESP_LOGI(TAG, "16 samples.");
  }

  return ESP_OK;
}

uint16_t QMI8658::ReadFromFifo(IMUdata *acc, uint16_t accLength, IMUdata *gyro, uint16_t gyrLength)
{
  if (mFifoMode == FIFO_MODE_BYPASS)
  {
    ESP_LOGE(TAG, "FIFO is not configured.");
    return 0;
  }

  if (!mGyroEnabled && !mAccelEnabled)
  {
    ESP_LOGE(TAG, "Sensor not enabled.");
    return 0;
  }

  uint16_t data_bytes = ReadFromFifo();
  if (data_bytes == 0)
  {
    return 0;
  }

  if (!mFifoBuffer)
  {
    ESP_LOGE(TAG, "FIFO buffer is NULL");
    return 0;
  }

  uint8_t enabled_sensor_count = (mAccelEnabled && mGyroEnabled) ? 2 : 1;
  uint16_t samples_per_sensor = data_bytes / (6 * enabled_sensor_count);
  uint16_t total_samples = samples_per_sensor * enabled_sensor_count;

  ESP_LOGI(TAG, "Total samples: %u", total_samples);

  uint16_t accel_index = 0;
  uint16_t gyro_index = 0;

  for (uint16_t i = 0; i < total_samples; ++i)
  {
    auto data = reinterpret_cast<int16_t *>(&mFifoBuffer[i * 6]);
    int16_t x = data[0];
    int16_t y = data[1];
    int16_t z = data[2];

    if (mAccelEnabled && mGyroEnabled)
    {
      if (i % 2 == 0)
      {
        // Accel
        if (accel_index < accLength)
        {
          acc[accel_index].x = x * mAccelScales;
          acc[accel_index].y = y * mAccelScales;
          acc[accel_index].z = z * mAccelScales;
          accel_index++;
        }
      }
      else
      {
        // Gyro
        if (gyro_index < gyrLength)
        {
          gyro[gyro_index].x = x * mGyroScales;
          gyro[gyro_index].y = y * mGyroScales;
          gyro[gyro_index].z = z * mGyroScales;
          gyro_index++;
        }
      }
    }
    else if (mAccelEnabled)
    {
      if (accel_index < accLength)
      {
        acc[accel_index].x = x * mAccelScales;
        acc[accel_index].y = y * mAccelScales;
        acc[accel_index].z = z * mAccelScales;
        accel_index++;
      }
    }
    else if (mGyroEnabled)
    {
      if (gyro_index < gyrLength)
      {
        gyro[gyro_index].x = x * mGyroScales;
        gyro[gyro_index].y = y * mGyroScales;
        gyro[gyro_index].z = z * mGyroScales;
        gyro_index++;
      }
    }
  }
  return samples_per_sensor;
}

esp_err_t QMI8658::EnableAccelerometer()
{
  esp_err_t ret = SetRegisterBit(QMI8658_REG_CTRL7, 0);
  if (ret != ESP_OK)
    return ret;
  mAccelEnabled = true;
  return ret;
}

esp_err_t QMI8658::DisableAccelerometer()
{
  esp_err_t ret = ClrRegisterBit(QMI8658_REG_CTRL7, 0);
  if (ret != ESP_OK)
    return ret;
  mAccelEnabled = false;
  return ret;
}

bool QMI8658::IsEnableAccelerometer()
{
  return mAccelEnabled;
}

bool QMI8658::IsEnableGyroscope()
{
  return mGyroEnabled;
}

esp_err_t QMI8658::EnableGyroscope()
{
  esp_err_t ret = SetRegisterBit(QMI8658_REG_CTRL7, 1);
  if (ret != ESP_OK)
    return ret;
  mGyroEnabled = true;
  return ret;
}

esp_err_t QMI8658::DisableGyroscope()
{
  esp_err_t ret = ClrRegisterBit(QMI8658_REG_CTRL7, 1);
  if (ret != ESP_OK)
    return ret;
  mGyroEnabled = false;
  return ret;
}

esp_err_t QMI8658::GetAccelRaw(int16_t *rawBuffer)
{
  if (!mAccelEnabled)
  {
    return false;
  }
  uint8_t buffer[6] = {0};
  esp_err_t ret = ReadRegister(QMI8658_REG_AX_L, buffer, 6);
  if (ret != ESP_OK)
    return ret;
  rawBuffer[0] = (int16_t)(buffer[1] << 8) | (buffer[0]);
  rawBuffer[1] = (int16_t)(buffer[3] << 8) | (buffer[2]);
  rawBuffer[2] = (int16_t)(buffer[5] << 8) | (buffer[4]);

  return ret;
}

esp_err_t QMI8658::GetAccelerometer(float &x, float &y, float &z)
{
  if (!mAccelEnabled)
  {
    return false;
  }
  int16_t raw[3];
  esp_err_t ret = GetAccelRaw(raw);
  if (ret != ESP_OK)
    return ret;
  x = raw[0] * mAccelScales;
  y = raw[1] * mAccelScales;
  z = raw[2] * mAccelScales;
  return ret;
}

float QMI8658::GetAccelerometerScales()
{
  return mAccelScales;
}

float QMI8658::GetGyroscopeScales()
{
  return mGyroScales;
}

esp_err_t QMI8658::GetGyroRaw(int16_t *rawBuffer)
{
  if (!mGyroEnabled)
  {
    return false;
  }
  uint8_t buffer[6] = {0};
  esp_err_t ret = ReadRegister(QMI8658_REG_GX_L, buffer, 6);
  if (ret != ESP_OK)
    return ret;
  rawBuffer[0] = (int16_t)(buffer[1] << 8) | (buffer[0]);
  rawBuffer[1] = (int16_t)(buffer[3] << 8) | (buffer[2]);
  rawBuffer[2] = (int16_t)(buffer[5] << 8) | (buffer[4]);
  return ret;
}

esp_err_t QMI8658::GetGyroscope(float &x, float &y, float &z)
{
  if (!mGyroEnabled)
  {
    return false;
  }
  int16_t raw[3];
  esp_err_t ret = GetGyroRaw(raw);
  if (ret != ESP_OK)
    return ret;
  x = raw[0] * mGyroScales;
  y = raw[1] * mGyroScales;
  z = raw[2] * mGyroScales;
  return ret;
}

bool QMI8658::GetDataReady(esp_err_t *aErr)
{
  if ((mIrqEnableMask & 0x03) && (mIrqPin1 != GPIO_NUM_NC))
  {
    if (gpio_get_level(mIrqPin1))
    {
      return false;
    }
  }

  switch (sampleMode)
  {
  case SYNC_MODE:
    return GetRegisterBit(QMI8658_REG_STATUS_INT, 1, aErr);
  case ASYNC_MODE:
    // TODO: When Accel and Gyro are configured with different rates, this will always be false
    if (mAccelEnabled & mGyroEnabled)
    {
      return ReadRegister8(QMI8658_REG_STATUS0, aErr) & 0x03;
    }
    else if (mGyroEnabled)
    {
      return ReadRegister8(QMI8658_REG_STATUS0, aErr) & 0x02;
    }
    else if (mAccelEnabled)
    {
      return ReadRegister8(QMI8658_REG_STATUS0, aErr) & 0x01;
    }
    break;
  default:
    break;
  }
  return false;
}

esp_err_t QMI8658::EnableSyncSampleMode()
{
  sampleMode = SYNC_MODE;
  return SetRegisterBit(QMI8658_REG_CTRL7, 7);
}

esp_err_t QMI8658::DisableSyncSampleMode()
{
  sampleMode = ASYNC_MODE;
  return ClrRegisterBit(QMI8658_REG_CTRL7, 7);
}

esp_err_t QMI8658::EnableLockingMechanism()
{
  esp_err_t ret = EnableSyncSampleMode();
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL1_L, 0x01);
  if (ret != ESP_OK)
    return ret;
  return WriteCommand(CTRL_CMD_AHB_CLOCK_GATING);
}

esp_err_t QMI8658::DisableLockingMechanism()
{
  esp_err_t ret = DisableSyncSampleMode();
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL1_L, 0x00);
  if (ret != ESP_OK)
    return ret;
  return WriteCommand(CTRL_CMD_AHB_CLOCK_GATING);
}

esp_err_t QMI8658::PowerDown()
{
  esp_err_t ret = DisableAccelerometer();
  if (ret != ESP_OK)
    return ret;
  ret = DisableGyroscope();
  if (ret != ESP_OK)
    return ret;
  return SetRegisterBit(QMI8658_REG_CTRL1, 1);
}

esp_err_t QMI8658::PowerOn()
{
  return ClrRegisterBit(QMI8658_REG_CTRL1, 1);
}

uint8_t QMI8658::GetStatusRegister(esp_err_t *aErr)
{
  return ReadRegister8(QMI8658_REG_STATUS1, aErr);
}

esp_err_t QMI8658::ConfigActivityInterruptMap(IntPin pin)
{
  return pin == INTERRUPT_PIN_1 ? SetRegisterBit(QMI8658_REG_CTRL8, 6)
                                : ClrRegisterBit(QMI8658_REG_CTRL8, 6);
}

/**
 * @brief  readFromFifo
 * @note   Read the data in the FIFO buffer. configFIFO should be called before use.
 * @retval Returns the size of the element read
 */
uint16_t QMI8658::ReadFromFifo(esp_err_t *aErr)
{
  uint8_t status[2];
  uint16_t fifo_bytes = 0;

  if ((mIrqPin1 != GPIO_NUM_NC) && mFifoInterrupt)
  {
    /*
     * Once the corresponds INT pin is configured to the push-pull mode, the FIFO watermark interrupt can be seen on the
     * corresponds INT pin. It will keep high level as long as the FIFO filled level is equal to or higher than the watermark, will
     * drop to low level as long as the FIFO filled level is lower than the configured FIFO watermark after reading out by host
     * and FIFO_RD_MODE is cleared.
     */
    if (gpio_get_level(mIrqPin1) == 0)
    {
      return false;
    }
  }

  // 1.Got FIFO watermark interrupt by INT pin or polling the FIFO_STATUS register (FIFO_WTM and/or FIFO_FULL).
  esp_err_t ret;
  uint8_t val = ReadRegister8(QMI8658_REG_FIFO_STATUS, &ret);
  if (ret != ESP_OK)
    return ret;

  ESP_LOGI(TAG, "FIFO status:0x%x", val);

  if (!(val & _BV(4)))
  {
    ESP_LOGI(TAG, "FIFO is Empty");
    return 0;
  }
  if (val & _BV(5))
  {
    ESP_LOGI(TAG, "FIFO Overflow condition has happened (data dropping happened)");
    // return 0;
  }
  if (val & _BV(6))
  {
    ESP_LOGI(TAG, "FIFO Water Mark Level Hit");
  }
  if (val & _BV(7))
  {
    ESP_LOGI(TAG, "FIFO is Full");
  }

  // 2.Read the FIFO_SMPL_CNT and FIFO_STATUS registers, to calculate the level of FIFO content data, refer to 8.4 FIFO Sample Count.
  ret = ReadRegister(QMI8658_REG_FIFO_COUNT, status, 2);
  if (ret != ESP_OK)
    return ret;

  // FIFO_Sample_Count (in byte) = 2 * (fifo_smpl_cnt_msb[1:0] * 256 + fifo_smpl_cnt_lsb[7:0])
  fifo_bytes = 2 * (((status[1] & 0x03)) << 8 | status[0]);

  ESP_LOGI(TAG, "reg fifo_bytes:%d ", fifo_bytes);

  // Samples 16  * 6 * 2  = 192
  // Samples 32  * 6 * 2  = 384
  // Samples 64  * 6 * 2  = 768
  // Samples 128 * 6 * 2  = 1536

  // 3.Send CTRL_CMD_REQ_FIFO (0x05) by CTRL9 command, to enable FIFO read mode. Refer to CTRL_CMD_REQ_FIFO for details.
  ret = WriteCommand(CTRL_CMD_REQ_FIFO);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Request FIFO failed!");
    return ret;
  }
  // 4.Read from the FIFO_DATA register per FIFO_Sample_Count.
  ret = ReadRegister(QMI8658_REG_FIFO_DATA, mFifoBuffer, fifo_bytes);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Request FIFO data failed !");
    return 0;
  }

  // 5.Disable the FIFO Read Mode by setting FIFO_CTRL.FIFO_rd_mode to 0. New data will be filled into FIFO afterwards.
  ret = WriteRegister8(QMI8658_REG_FIFO_CTRL, mFifoMode);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Clear FIFO flag failed!");
    return 0;
  }

  return fifo_bytes;
}

/**
 * @brief configPedometer
 * @note The calculation of the Pedometer Detection is based on the accelerometer ODR defined by CTRL2.aODR,
 *      refer to Table 22 for details.
 * @param  ped_sample_cnt: Indicates the count of sample batch/window for calculation
 * @param  ped_fix_peak2peak:Indicates the threshold of the valid peak-to-peak detection
 *                          E.g., 0x00CC means 200mg
 * @param  ped_fix_peak:Indicates the threshold of the peak detection comparing to average
 *                      E.g., 0x0066 means 100mg
 * @param  ped_time_up:Indicates the maximum duration (timeout window) for a step.
 *                     Reset counting calculation if no peaks detected within this duration.
 *                    E.g., 80 means 1.6s @ ODR = 50Hz
 * @param  ped_time_low:Indicates the minimum duration for a step.
 *                     The peaks detected within this duration (quiet time) is ignored.
 *                    E.g., 12 means 0.25s @ ODR = 50Hz
 * @param  ped_time_cnt_entry:Indicates the minimum continuous steps to start the valid step counting.
 *                           If the continuously detected steps is lower than this count and timeout,
 *                           the steps will not be take into account;
 *                           if yes, the detected steps will all be taken into account and
 *                           counting is started to count every following step before timeout.
 *                           This is useful to screen out the fake steps detected by non-step vibrations
 *                           The timeout duration is defined by ped_time_up.
 *                           E.g., 10 means 10 steps entry count
 * @param  ped_fix_precision:0 is recommended
 * @param  ped_sig_count: The amount of steps when to update the pedometer output registers.
 *                      E.g., ped_sig_count = 4, every 4 valid steps is detected, update the registers once (added by 4).
 * @retval
 */
esp_err_t QMI8658::ConfigPedometer(uint16_t ped_sample_cnt, uint16_t ped_fix_peak2peak, uint16_t ped_fix_peak,
                                   uint16_t ped_time_up, uint8_t ped_time_low, uint8_t ped_time_cnt_entry, uint8_t ped_fix_precision,
                                   uint8_t ped_sig_count)
{
  esp_err_t ret;
  // The Pedometer can only work in Non-SyncSample mode
  ret = DisableSyncSampleMode();
  if (ret != ESP_OK)
    return ret;

  bool enGyro = IsEnableGyroscope();
  bool enAccel = IsEnableAccelerometer();

  if (enGyro)
  {
    ret = DisableGyroscope();
    if (ret != ESP_OK)
      return ret;
  }

  if (enAccel)
  {
    ret = DisableAccelerometer();
    if (ret != ESP_OK)
      return ret;
  }

  ret = WriteRegister8(QMI8658_REG_CAL1_L, ped_sample_cnt & 0xFF);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL1_H, (ped_sample_cnt >> 8) & 0xFF);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL2_L, ped_fix_peak2peak & 0xFF);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL2_H, (ped_fix_peak2peak >> 8) & 0xFF);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL3_L, ped_fix_peak & 0xFF);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL3_H, (ped_fix_peak >> 8) & 0xFF);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL4_H, 0x01);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL4_L, 0x02);
  if (ret != ESP_OK)
    return ret;

  ret = WriteCommand(CTRL_CMD_CONFIGURE_PEDOMETER);
  if (ret != ESP_OK)
    return ret;

  ret = WriteRegister8(QMI8658_REG_CAL1_L, ped_time_up & 0xFF);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL1_H, (ped_time_up >> 8) & 0xFF);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL2_L, ped_time_low);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL2_H, ped_time_cnt_entry);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL3_L, ped_fix_precision);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL3_H, ped_sig_count);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL4_H, 0x02);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL4_L, 0x02);
  if (ret != ESP_OK)
    return ret;

  ret = WriteCommand(CTRL_CMD_CONFIGURE_PEDOMETER);
  if (ret != ESP_OK)
    return ret;

  if (enGyro)
  {
    ret = EnableGyroscope();
    if (ret != ESP_OK)
      return ret;
  }

  if (enAccel)
  {
    ret = EnableAccelerometer();
    if (ret != ESP_OK)
      return ret;
  }
  return ESP_OK;
}

uint32_t QMI8658::GetPedometerCounter(esp_err_t *aErr)
{
  esp_err_t ret;
  uint8_t buffer[3];
  ret = ReadRegister(QMI8658_REG_STEP_CNT_LOW, buffer, 3);
  if (ret != ESP_OK)
  {
    if (aErr != NULL)
      *aErr = ret;
    return 0;
  }
  return (uint32_t)(((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[1] << 8) | buffer[0]);
}

esp_err_t QMI8658::ClearPedometerCounter()
{
  return WriteCommand(CTRL_CMD_RESET_PEDOMETER);
}

// The Pedometer can only work in Non-SyncSample mode
esp_err_t QMI8658::EnablePedometer(IntPin pin)
{
  if (!mAccelEnabled)
    return ESP_ERR_INVALID_ARG;
  esp_err_t ret;
  switch (pin)
  {
  case INTERRUPT_PIN_1:
  case INTERRUPT_PIN_2:
    ret = ConfigActivityInterruptMap(pin);
    if (ret != ESP_OK)
      return ret;
    ret = EnableINT(pin);
    if (ret != ESP_OK)
      return ret;
    break;
  default:
    break;
  }
  return SetRegisterBit(QMI8658_REG_CTRL8, 4);
}

esp_err_t QMI8658::DisablePedometer()
{
  if (!mAccelEnabled)
    return ESP_ERR_INVALID_ARG;
  return ClrRegisterBit(QMI8658_REG_CTRL8, 4);
}

/**
 * @brief   configTap
 * @note    The calculation of the Tap Detection is based on the accelerometer ODR defined by CTRL2.aODR, refer to Table 22 for details.
 * @param  priority: Priority definition between the x, y, z axes of acceleration. Only
                     Priority[2:0] bits are used.
                     The axis that output the first peak of Linear Acceleration in a valid
                     Tap detection, will be consider as the Tap axis. However, there is
                     possibility that two or three of the axes shows same Linear
                     Acceleration at exactly same time when reach (or be higher than)
                     the PeakMagThr. In this case, the defined priority is used to judge
                     and select the axis as Tap axis.
 * @param  peakWindow:Defines the maximum duration (in sample) for a valid peak. In a
                    valid peak, the linear acceleration should reach or be higher than
                    the PeakMagThr and should return to quiet (no significant
                    movement) within UDMThr, at the end of PeakWindow. E.g., 20 @500Hz ODR
 * @param  tapWindow:Defines the minimum quiet time before the second Tap happen.
                    After the first Tap is detected, there should be no significant
                    movement (defined by UDMThr) during the TapWindow. The valid
                    second tap should be detected after TapWindow and before
                    DTapWindow. E.g., 50 @500Hz ODR
 * @param  dTapWindow:Defines the maximum time for a valid second Tap for Double Tap,
                    count start from the first peak of the valid first Tap.  E.g., 250 @500Hz ODR
 * @param  alpha:Defines the ratio for calculation the average of the acceleration.
                The bigger of Alpha, the bigger weight of the latest data.  E.g., 0.0625
 * @param  gamma:Defines the ratio for calculating the average of the movement
                magnitude. The bigger of Gamma, the bigger weight of the latest data. E.g., 0.25
 * @param  peakMagThr:Threshold for peak detection.  E.g, 0.8g2
 * @param  UDMThr:Undefined Motion threshold. This defines the threshold of the
                Linear Acceleration for quiet status. E.g., 0.4g2
 * @retval
 */
esp_err_t QMI8658::ConfigTap(uint8_t priority, uint8_t peakWindow, uint16_t tapWindow, uint16_t dTapWindow,
                             float alpha, float gamma, float peakMagThr, float UDMThr)
{
  esp_err_t ret;
  // The Tap detection can only work in Non-SyncSample mode
  ret = DisableSyncSampleMode();
  if (ret != ESP_OK)
    return ret;

  bool enGyro = IsEnableGyroscope();
  bool enAccel = IsEnableAccelerometer();

  if (enGyro)
  {
    ret = DisableGyroscope();
    if (ret != ESP_OK)
      return ret;
  }

  if (enAccel)
  {
    ret = DisableAccelerometer();
    if (ret != ESP_OK)
      return ret;
  }
  ret = WriteRegister8(QMI8658_REG_CAL1_L, peakWindow);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL1_H, priority);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL2_L, tapWindow & 0xFF);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL2_H, (tapWindow >> 8) & 0xFF);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL3_L, dTapWindow & 0xFF);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL3_H, (dTapWindow >> 8) & 0xFF);
  if (ret != ESP_OK)
    return ret;
  // writeRegister(QMI8658_REG_CAL4_L, 0x02);
  ret = WriteRegister8(QMI8658_REG_CAL4_H, 0x01);
  if (ret != ESP_OK)
    return ret;

  ret = WriteCommand(CTRL_CMD_CONFIGURE_TAP);
  if (ret != ESP_OK)
    return ret;

  // 1-byte unsigned,7-bits fraction
  uint8_t alphaHex = (uint8_t)(alpha * 128);
  ret = WriteRegister8(QMI8658_REG_CAL1_L, alphaHex);
  if (ret != ESP_OK)
    return ret;

  // 1-byte unsigned,7-bits fraction
  uint8_t gammaHex = (uint8_t)(gamma * 128);
  ret = WriteRegister8(QMI8658_REG_CAL1_H, gammaHex);
  if (ret != ESP_OK)
    return ret;

  const double g = 9.81;             // Earth's gravitational acceleration m/s^2
  double resolution = 0.001 * g * g; // Calculation resolution  0.001g^2

  double acceleration_square = peakMagThr * g * g;               // Calculate the square of the acceleration
  uint16_t value = (uint16_t)(acceleration_square / resolution); // Calculates the value of a 2-byte unsigned integer

  ret = WriteRegister8(QMI8658_REG_CAL2_L, value & 0xff);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL2_H, value >> 8);
  if (ret != ESP_OK)
    return ret;

  acceleration_square = UDMThr * g * g;                 // Calculate the square of the acceleration
  value = (uint16_t)(acceleration_square / resolution); // Calculates the value of a 2-byte unsigned integer

  ret = WriteRegister8(QMI8658_REG_CAL3_L, value & 0xff);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL3_H, value >> 8);
  if (ret != ESP_OK)
    return ret;
  // writeRegister(QMI8658_REG_CAL4_L, 0x02);
  ret = WriteRegister8(QMI8658_REG_CAL4_H, 0x02);
  if (ret != ESP_OK)
    return ret;

  ret = WriteCommand(CTRL_CMD_CONFIGURE_TAP);
  if (ret != ESP_OK)
    return ret;

  if (enGyro)
  {
    ret = EnableGyroscope();
    if (ret != ESP_OK)
      return ret;
  }

  if (enAccel)
  {
    ret = EnableAccelerometer();
    if (ret != ESP_OK)
      return ret;
  }

  return ESP_OK;
}

esp_err_t QMI8658::EnableTap(IntPin pin)
{
  esp_err_t ret;
  if (!mAccelEnabled)
    return ESP_ERR_INVALID_ARG;
  switch (pin)
  {
  case INTERRUPT_PIN_1:
  case INTERRUPT_PIN_2:
    ret = ConfigActivityInterruptMap(pin);
    if (ret != ESP_OK)
      return ret;
    ret = EnableINT(pin);
    if (ret != ESP_OK)
      return ret;
    break;
  default:
    break;
  }
  return SetRegisterBit(QMI8658_REG_CTRL8, 0);
}

esp_err_t QMI8658::DisableTap()
{
  return ClrRegisterBit(QMI8658_REG_CTRL8, 0);
}

QMI8658::TapEvent QMI8658::GetTapStatus(esp_err_t *aErr)
{
  uint8_t val = ReadRegister8(QMI8658_REG_TAP_STATUS, aErr);
  if (val & _BV(7))
  {
    ESP_LOGI(TAG, "Tap was detected on the negative direction of the Tap axis");
  }
  else
  {
    ESP_LOGI(TAG, "Tap was detected on the positive direction of the Tap axis");
  }
  uint8_t t = (val >> 4) & 0x03;
  switch (t)
  {
  case 0:
    ESP_LOGI(TAG, "No Tap was detected");
    break;
  case 1:
    ESP_LOGI(TAG, "Tap was detected on X axis");
    break;
  case 2:
    ESP_LOGI(TAG, "Tap was detected on Y axis");
    break;
  case 3:
    ESP_LOGI(TAG, "Tap was detected on Z axis");
    break;
  default:
    break;
  }
  t = val & 0x03;
  switch (t)
  {
  case 0:
    ESP_LOGI(TAG, "No Tap was detected");
    return INVALID_TAP;
  case 1:
    ESP_LOGI(TAG, "Single-Tap was detected");
    return SINGLE_TAP;
  case 2:
    ESP_LOGI(TAG, "Double-Tap was detected");
    return DOUBLE_TAP;
  default:
    break;
  }
  return INVALID_TAP;
}

uint8_t QMI8658::MgToBytes(float mg)
{
  float g = mg / 1000.0;               // Convert to grams
  int units = (int)round(g / 0.03125); // Convert grams to units of specified(1/32) resolution
  return (units & 0x1F) << 3;          // Shift the 5 decimal places to the left by 3 places, because there are only 3 integer places
}

esp_err_t QMI8658::ConfigMotion(
    //* See enum MotionCtrl
    uint8_t modeCtrl,
    //* Define the slope threshold of the x-axis for arbitrary motion detection
    float AnyMotionXThr,
    //* Define the slope threshold of the y-axis for arbitrary motion detection
    float AnyMotionYThr,
    //* Define the slope threshold of the z-axis for arbitrary motion detection
    float AnyMotionZThr,
    //* Defines the minimum number of consecutive samples (duration) that the absolute
    //* of the slope of the enabled axis/axes data should keep higher than the threshold
    uint8_t AnyMotionWindow,
    //* Defines the slope threshold of the x-axis for no motion detection
    float NoMotionXThr,
    //* Defines the slope threshold of the y-axis for no motion detection
    float NoMotionYThr,
    //* Defines the slope threshold of the z-axis for no motion detection
    float NoMotionZThr,
    //* Defines the minimum number of consecutive samples (duration) that the absolute
    //* of the slope of the enabled axis/axes data should keep lower than the threshold
    uint8_t NoMotionWindow,
    //* Defines the wait window (idle time) starts from the first Any-Motion event until
    //* starting to detecting another Any-Motion event form confirmation
    uint16_t SigMotionWaitWindow,
    //* Defines the maximum duration for detecting the other Any-Motion
    //* event to confirm Significant-Motion, starts from the first Any -Motion event
    uint16_t SigMotionConfirmWindow)
{
  // Only work in Non-SyncSample mode
  esp_err_t ret = DisableSyncSampleMode();
  if (ret != ESP_OK)
    return ret;

  bool enGyro = IsEnableGyroscope();
  bool enAccel = IsEnableAccelerometer();

  if (enGyro)
  {
    ret = DisableGyroscope();
    if (ret != ESP_OK)
      return ret;
  }

  if (enAccel)
  {
    ret = DisableAccelerometer();
    if (ret != ESP_OK)
      return ret;
  }

  ret = WriteRegister8(QMI8658_REG_CAL1_L, MgToBytes(AnyMotionXThr));
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL1_H, MgToBytes(AnyMotionYThr));
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL2_L, MgToBytes(AnyMotionZThr));
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL2_H, MgToBytes(NoMotionXThr));
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL3_L, MgToBytes(NoMotionYThr));
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL3_H, MgToBytes(NoMotionZThr));
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL4_L, modeCtrl);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL4_H, 0x01);
  if (ret != ESP_OK)
    return ret;

  ret = WriteCommand(CTRL_CMD_CONFIGURE_MOTION);
  if (ret != ESP_OK)
    return ret;

  ret = WriteRegister8(QMI8658_REG_CAL1_L, AnyMotionWindow);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL1_H, NoMotionWindow);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL2_L, SigMotionWaitWindow & 0xff);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL2_H, SigMotionWaitWindow >> 8);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL3_L, SigMotionConfirmWindow & 0xff);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister8(QMI8658_REG_CAL3_H, SigMotionConfirmWindow >> 8);
  if (ret != ESP_OK)
    return ret;
  // writeRegister(QMI8658_REG_CAL4_L, 0x02);
  ret = WriteRegister8(QMI8658_REG_CAL4_H, 0x02);
  if (ret != ESP_OK)
    return ret;

  ret = WriteCommand(CTRL_CMD_CONFIGURE_MOTION);
  if (ret != ESP_OK)
    return ret;

  if (enGyro)
  {
    ret = EnableGyroscope();
    if (ret != ESP_OK)
      return ret;
  }

  if (enAccel)
  {
    ret = EnableAccelerometer();
    if (ret != ESP_OK)
      return ret;
  }
  return ESP_OK;
}

esp_err_t QMI8658::EnableMotionDetect(IntPin pin)
{
  esp_err_t ret;
  if (!mAccelEnabled)
    return ESP_ERR_INVALID_ARG;
  switch (pin)
  {
  case INTERRUPT_PIN_1:
  case INTERRUPT_PIN_2:
    ret = ConfigActivityInterruptMap(pin);
    if (ret != ESP_OK)
      return ret;
    ret = EnableINT(pin);
    if (ret != ESP_OK)
      return ret;
    break;
  default:
    break;
  }
  ret = SetRegisterBit(QMI8658_REG_CTRL8, 1);
  if (ret != ESP_OK)
    return ret;
  ret = SetRegisterBit(QMI8658_REG_CTRL8, 2);
  if (ret != ESP_OK)
    return ret;
  ret = SetRegisterBit(QMI8658_REG_CTRL8, 3);
  if (ret != ESP_OK)
    return ret;
  return true;
}

esp_err_t QMI8658::DisableMotionDetect()
{
  esp_err_t ret;
  ret = ClrRegisterBit(QMI8658_REG_CTRL8, 1);
  if (ret != ESP_OK)
    return ret;

  ret = ClrRegisterBit(QMI8658_REG_CTRL8, 2);
  if (ret != ESP_OK)
    return ret;
  return ClrRegisterBit(QMI8658_REG_CTRL8, 3);
}

/**
 * @brief  configWakeOnMotion
 * @note   Configuring Wom will reset the sensor, set the function to Wom, and there will be no data output
 * @param  WoMThreshold: Resolution = 1mg ,default 200mg
 * @param  odr: Accelerometer output data rate  ,default low power 128Hz
 * @param  pin: Interrupt Pin( 1 or 2 ) ,default use pin2
 * @param  defaultPinValue: WoM Interrupt Initial Value select: ,default pin high
 *  01 – INT2 (with initial value 0)
 *  11 – INT2 (with initial value 1)
 *  00 – INT1 (with initial value 0)
 *  10 – INT1 (with initial value 1)
 * @param  blankingTime: Interrupt Blanking Time
 *  (in number of accelerometer samples), the
 *  number of consecutive samples that will be ignored after
 *  enabling the WoM, to screen out unwanted fake detection
 * @retval
 */
esp_err_t QMI8658::ConfigWakeOnMotion(uint8_t WoMThreshold, AccelODR odr, IntPin pin, uint8_t defaultPinValue, uint8_t blankingTime)
{
  esp_err_t ret;
  uint8_t val = 0;

  // Reset default value
  ret = Reset();
  if (ret != ESP_OK)
    return ret;

  // Disable sensors
  ret = ClrRegisterBit(QMI8658_REG_CTRL7, 0);
  if (ret != ESP_OK)
    return ret;

  // setAccelRange
  ret = WriteRegister(QMI8658_REG_CTRL2, 0x8F, (ACC_RANGE_8G << 4));
  if (ret != ESP_OK)
    return ret;

  // setAccelOutputDataRate
  ret = WriteRegister(QMI8658_REG_CTRL2, 0xF0, odr);
  if (ret != ESP_OK)
    return ret;

  // set wom
  ret = WriteRegister8(QMI8658_REG_CAL1_L, WoMThreshold);
  if (ret != ESP_OK)
    return ret;

  if (pin == INTERRUPT_PIN_1)
  {
    val = defaultPinValue ? 0x02 : 0x00;
  }
  else if (pin == INTERRUPT_PIN_2)
  {
    val = defaultPinValue ? 0x03 : 0x01;
  }

  val <<= 6;
  val |= (blankingTime & 0x3F);
  ret = WriteRegister8(QMI8658_REG_CAL1_H, val);
  if (ret != ESP_OK)
    return ret;

  ret = WriteCommand(CTRL_CMD_WRITE_WOM_SETTING);
  if (ret != ESP_OK)
    return ret;

  ret = EnableAccelerometer();
  if (ret != ESP_OK)
    return ret;

  return EnableINT(pin);
}

void QMI8658::GetChipUsid(uint8_t *buffer, uint8_t length)
{
  if (length > 6)
  {
    length = 6;
  }
  memcpy(buffer, usid, length);
}

uint32_t QMI8658::GetChipFirmwareVersion()
{
  return mRevisionID;
}

/**
 * @brief update
 * @note  Get the interrupt status and status 0, status 1 of the sensor
 * @retval  Return SensorStatus
 */
uint16_t QMI8658::Update(esp_err_t *aErr)
{
  esp_err_t ret;
  uint16_t result = 0;
  // STATUSINT 0x2D
  // STATUS0 0x2E
  // STATUS1 0x2F
  uint8_t status[3];
  ret = ReadRegister(QMI8658_REG_STATUS_INT, status, 3);
  if (ret != ESP_OK)
    return ret;

  // log_i("STATUSINT:0x%X BIN:", status[0]);
  // log_i("STATUS0:0x%X BIN:", status[1]);
  // log_i("STATUS1:0x%X BIN:", status[2]);
  // log_i("------------------\n");

  // Ctrl9 CmdDone
  // Indicates CTRL9 Command was done, as part of CTRL9 protocol
  // 0: Not Completed
  // 1: Done
  if (status[0] & 0x80)
  {
    result |= STATUS_INT_CTRL9_CMD_DONE;
  }
  // If syncSample (CTRL7.bit7) = 1:
  //      0: Sensor Data is not locked.
  //      1: Sensor Data is locked.
  // If syncSample = 0, this bit shows the same value of INT1 level
  if (status[0] & 0x02)
  {
    result |= STATUS_INT_LOCKED;
  }
  // If syncSample (CTRL7.bit7) = 1:
  //      0: Sensor Data is not available
  //      1: Sensor Data is available for reading
  // If syncSample = 0, this bit shows the same value of INT2 level
  if (status[0] & 0x01)
  {
    result |= STATUS_INT_AVAIL;
    // if (eventGyroDataReady)eventGyroDataReady();
    // if (eventAccelDataReady)eventAccelDataReady();
  }

  // Locking Mechanism Can reading..
  if ((status[0] & 0x03) == 0x03)
  {
    if (eventDataLocking)
      eventDataLocking();
  }

  //=======================================
  // Valid only in asynchronous mode
  if (sampleMode == ASYNC_MODE)
  {
    // Gyroscope new data available
    // 0: No updates since last read.
    // 1: New data available
    if (status[1] & 0x02)
    {
      result |= STATUS0_GYRO_DATA_READY;
      if (eventGyroDataReady)
        eventGyroDataReady();
      mgDataReady = true;
    }
    // Accelerometer new data available
    // 0: No updates since last read.
    // 1: New data available.
    if (status[1] & 0x01)
    {
      result |= STATUS0_ACCEL_DATA_READY;
      if (eventAccelDataReady)
        eventAccelDataReady();
      maDataReady = true;
    }
  }

  //=======================================
  // Significant Motion
  // 0: No Significant-Motion was detected
  // 1: Significant-Motion was detected
  if (status[2] & 0x80)
  {
    result |= STATUS1_SIGNIFICANT_MOTION;
    if (eventSignificantMotion)
      eventSignificantMotion();
  }
  // No Motion
  // 0: No No-Motion was detected
  // 1: No-Motion was detected
  if (status[2] & 0x40)
  {
    result |= STATUS1_NO_MOTION;
    if (eventNoMotionEvent)
      eventNoMotionEvent();
  }
  // Any Motion
  // 0: No Any-Motion was detected
  // 1: Any-Motion was detected
  if (status[2] & 0x20)
  {
    result |= STATUS1_ANY_MOTION;
    if (eventAnyMotionEvent)
      eventAnyMotionEvent();
  }
  // Pedometer
  // 0: No step was detected
  // 1: step was detected
  if (status[2] & 0x10)
  {
    result |= STATUS1_PEDOMETER_MOTION;
    if (eventPedometerEvent)
      eventPedometerEvent();
  }
  // WoM
  // 0: No WoM was detected
  // 1: WoM was detected
  if (status[2] & 0x04)
  {
    result |= STATUS1_WOM_MOTION;
    if (eventWomEvent)
      eventWomEvent();
  }
  // TAP
  // 0: No Tap was detected
  // 1: Tap was detected
  if (status[2] & 0x02)
  {
    result |= STATUS1_TAP_MOTION;
    if (eventTagEvent)
      eventTagEvent();
  }
  return result;
}

void QMI8658::SetWakeupMotionEventCallBack(EventCallBack_t cb)
{
  eventWomEvent = cb;
}

void QMI8658::SetTapEventCallBack(EventCallBack_t cb)
{
  eventTagEvent = cb;
}

void QMI8658::SetPedometerEventCallBack(EventCallBack_t cb)
{
  eventPedometerEvent = cb;
}

void QMI8658::SetNoMotionEventCallBack(EventCallBack_t cb)
{
  eventNoMotionEvent = cb;
}

void QMI8658::SetAnyMotionEventCallBack(EventCallBack_t cb)
{
  eventAnyMotionEvent = cb;
}

void QMI8658::SetSignificantMotionEventCallBack(EventCallBack_t cb)
{
  eventSignificantMotion = cb;
}

void QMI8658::SetGyroDataReadyCallBack(EventCallBack_t cb)
{
  eventGyroDataReady = cb;
}

void QMI8658::SetAccelDataReadyEventCallBack(EventCallBack_t cb)
{
  eventAccelDataReady = cb;
}

void QMI8658::SetDataLockingEventCallBack(EventCallBack_t cb)
{
  eventDataLocking = cb;
}

esp_err_t QMI8658::Calibration(uint16_t *gX_gain, uint16_t *gY_gain, uint16_t *gZ_gain)
{
  esp_err_t ret;
  // 1.Set CTRL7.aEN = 0 and CTRL7.gEN = 0, to disable the accelerometer and gyroscope.
  ret = WriteRegister8(QMI8658_REG_CTRL7, 0x00);
  if (ret != ESP_OK)
    return ret;

  // 2.Issue the CTRL_CMD_ON_DEMAND_CALIBRATION (0xA2) by CTRL9 command.
  ret = WriteCommand(CTRL_CMD_ON_DEMAND_CALIBRATION, 3000);
  if (ret != ESP_OK)
    return ret;

  // 3.And wait about 1.5 seconds for QMI8658A to finish the CTRL9 command.
  vTaskDelay(pdMS_TO_TICKS(1600));

  // 4.Read the COD_STATUS register (0x46) to check the result/status of the COD implementation.
  uint8_t result = ReadRegister8(QMI8658_REG_COD_STATUS, &ret);
  if (ret != ESP_OK)
    return ret;

  // During the process, it is recommended to place the device in quiet, otherwise, the COD might fail and report error.
  if (result & _BV(7))
  {
    ESP_LOGE(TAG, "COD failed for checking low sensitivity limit of X axis of gyroscope");
    return false;
  }
  if (result & _BV(6))
  {
    ESP_LOGE(TAG, "COD failed for checking high sensitivity limit of X axis of gyroscope");
    return false;
  }
  if (result & _BV(5))
  {
    ESP_LOGE(TAG, "COD failed for checking low sensitivity limit of Y axis of gyroscope");
    return false;
  }
  if (result & _BV(4))
  {
    ESP_LOGE(TAG, "COD failed for checking high sensitivity limit of Y axis of gyroscope");
    return false;
  }
  if (result & _BV(3))
  {
    ESP_LOGE(TAG, "Accelerometer checked failed (significant vibration happened during COD)");
    return false;
  }
  if (result & _BV(2))
  {
    ESP_LOGE(TAG, "Gyroscope startup failure happened when COD was called");
    return false;
  }
  if (result & _BV(1))
  {
    ESP_LOGE(TAG, "COD was called while gyroscope was enabled, COD return failure");
    return false;
  }
  if (result & _BV(0))
  {
    ESP_LOGE(TAG, "COD failed; no COD correction applied");
    return false;
  }
  ESP_LOGI(TAG, "All calibrations are completed");

  if (gX_gain && gY_gain && gZ_gain)
  {
    uint8_t rawBuffer[6] = {0};
    ret = ReadRegister(QMI8658_REG_DVX_L, rawBuffer, 6);
    if (ret != ESP_OK)
      return ret;
    *gX_gain = ((uint16_t)rawBuffer[0]) | (uint16_t)(rawBuffer[1] << 8);
    *gY_gain = ((uint16_t)rawBuffer[2]) | (uint16_t)(rawBuffer[3] << 8);
    *gZ_gain = ((uint16_t)rawBuffer[4]) | (uint16_t)(rawBuffer[5] << 8);
  }

  return ESP_OK;
}

esp_err_t QMI8658::WriteCalibration(uint16_t gX_gain, uint16_t gY_gain, uint16_t gZ_gain)
{
  // 1. Disable Accelerometer and Gyroscope by setting CTRL7.aEN = 0 and CTRL7.gEN = 0
  esp_err_t ret = WriteRegister8(QMI8658_REG_CTRL7, 0x00);
  if (ret != ESP_OK)
    return ret;

  uint8_t buffer[] = {

      // 2. write Gyro-X gain (16 bits) to registers CAL1_L and CAL1_H registers (0x0B, 0x0C)
      uint8_t(gX_gain & 0xff),
      uint8_t(gX_gain << 8),
      // 3. write Gyro-Y gain (16 bits) to registers CAL2_L and CAL2_H registers (0x0D, 0x0E)
      uint8_t(gY_gain & 0xff),
      uint8_t(gY_gain >> 8),
      // 4. write Gyro-Z gain (16 bits) to registers CAL3_L and CAL3_H registers (0x0F, 0x10)
      uint8_t(gZ_gain & 0xff),
      uint8_t(gZ_gain >> 8),
  };

  ret = WriteRegister(QMI8658_REG_CAL1_L, buffer, sizeof(buffer));
  if (ret != ESP_OK)
    return ret;

  // 5. Write 0xAA to CTRL9 and follow CTRL9 protocol
  return WriteCommand(CTRL_CMD_APPLY_GYRO_GAINS, 3000);
}

esp_err_t QMI8658::SelfTestAccel()
{
  esp_err_t ret;
  // 1- Disable the sensors (CTRL7 = 0x00).
  ret = WriteRegister8(QMI8658_REG_CTRL7, 0x00);
  if (ret != ESP_OK)
    return ret;

  // 2- Set proper accelerometer ODR (CTRL2.aODR) and bit CTRL2.aST (bit7) to 1 to trigger the Self-Test.
  ret = WriteRegister(QMI8658_REG_CTRL2, 0xF0, ACC_ODR_1000Hz | 0x80);
  if (ret != ESP_OK)
    return ret;

  // 3- Wait for QMI8658A to drive INT2 to High, if INT2 is enabled (CTRL1.bit4 = 1), or STATUSINT.bit0 is set to 1.
  int retry = 50;
  int dataReady = 0x00;
  while (dataReady != 0x01)
  {
    uint8_t reg_var = ReadRegister8(QMI8658_REG_STATUS_INT, &ret);
    if (ret != ESP_OK)
      return ret;
    ESP_LOGI(TAG, "reg_var : %x", reg_var);
    dataReady = reg_var & 0x01;
    // dataReady = readRegister(QMI8658_REG_STATUS_INT) & 0x01;
    vTaskDelay(pdMS_TO_TICKS(20));
    if (--retry <= 0)
    {
      ESP_LOGE(TAG, "No response.");
      return ESP_ERR_TIMEOUT;
    }
  }

  ESP_LOGI(TAG, "Data is ready for reading....");

  // 4- Set CTRL2.aST(bit7) to 0, to clear STATUSINT1.bit0 and/or INT2.
  ret = ClrRegisterBit(QMI8658_REG_CTRL2, 7);
  if (ret != ESP_OK)
    return ret;

  // 5- Check for QMI8658A drives INT2 back to Low, and sets STATUSINT1.bit0 to 0.
  retry = 50;
  while (dataReady == 0x01)
  {
    uint8_t reg_var = ReadRegister8(QMI8658_REG_STATUS_INT, &ret);
    if (ret != ESP_OK)
      return ret;
    ESP_LOGI(TAG, "reg_var : %x", reg_var);
    dataReady = (reg_var & 0x01);
    // dataReady = !(readRegister(QMI8658_REG_STATUS_INT) & 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));
    if (--retry <= 0)
    {
      ESP_LOGE(TAG, "No response.");
      return ESP_ERR_TIMEOUT;
    }
  }

  /*
      6- Read the Accel Self-Test result:
          X channel: dVX_L and dVX_H (registers 0x51 and 0x52)
          Y channel: dVY_L and dVY_H (registers 0x53 and 0x54)
          Z channel: dVZ_L and dVZ_H (registers 0x55 and 0x56)
          The results are 16-bits in format signed U5.11, resolution 0.5mg (1 / 2^11 g).
  */
  uint8_t rawBuffer[6];

  ret = ReadRegister(QMI8658_REG_DVX_L, rawBuffer, 6);
  if (ret != ESP_OK)
    return ret;

  int16_t dVX = (int16_t)(rawBuffer[0]) | (int16_t)(((int16_t)rawBuffer[1]) << 8);
  int16_t dVY = (int16_t)(rawBuffer[2]) | (int16_t)(((int16_t)rawBuffer[3]) << 8);
  int16_t dVZ = (int16_t)(rawBuffer[4]) | (int16_t)(((int16_t)rawBuffer[5]) << 8);

  // To convert to mg, considering the U5.11 format, we need to divide by (2^11) to get the actual mg value
  float dVX_mg = dVX * 0.5; // 0.5mg is the smallest unit of this format
  float dVY_mg = dVY * 0.5;
  float dVZ_mg = dVZ * 0.5;

  ESP_LOGI(TAG, "\n\tdVX_mg:%05.11f \n\tdVY_mg:%05.11f \n\tdVZ_mg:%05.11f", dVX_mg, dVY_mg, dVZ_mg);
  // If the absolute results of all three axes are higher than 200mg, the accelerometer can be considered functional.
  // Otherwise, the accelerometer cannot be considered functional.
  if (abs(dVX_mg) > 200 && abs(dVY_mg) > 200 && abs(dVZ_mg) > 200)
  {
    ESP_LOGI(TAG, "Accelerometer is working properly.");
  }
  else
  {
    ESP_LOGI(TAG, "Accelerometer is not working properly.");
    return ESP_ERR_INVALID_RESPONSE;
  }
  return true;
}

esp_err_t QMI8658::SelfTestGyro()
{
  esp_err_t ret;
  // 1- Disable the sensors (CTRL7 = 0x00).
  ret = WriteRegister8(QMI8658_REG_CTRL7, 0x00);
  if (ret != ESP_OK)
    return ret;

  // 2- Set the bit gST to 1. (CTRL3.bit7 = 1’b1).
  ret = SetRegisterBit(QMI8658_REG_CTRL3, 7);
  if (ret != ESP_OK)
    return ret;

  // 3- Wait for QMI8658A to drive INT2 to High, if INT2 is enabled, or STATUS_INT.bit0 is set to 1.
  int retry = 50;
  int dataReady = 0x00;
  while (dataReady != 0x01)
  {
    dataReady = ReadRegister8(QMI8658_REG_STATUS_INT, &ret) & 0x01;
    if (ret != ESP_OK)
      return ret;
    vTaskDelay(pdMS_TO_TICKS(20));
    if (--retry <= 0)
    {
      ESP_LOGE(TAG, "No response.");
      return ESP_ERR_TIMEOUT;
    }
  }

  ESP_LOGI(TAG, "Data is ready for reading....");

  // 4- Set CTRL3.aST(bit7) to 0, to clear STATUS_INT1.bit0 and/or INT2.
  ret = ClrRegisterBit(QMI8658_REG_CTRL3, 7);
  if (ret != ESP_OK)
    return ret;

  // 5- Check for QMI8658A drives INT2 back to Low, or sets STATUSINT1.bit0 to 0.
  retry = 50;
  while (dataReady != 0x00)
  {
    dataReady = !(ReadRegister8(QMI8658_REG_STATUS_INT, &ret) & 0x01);
    if (ret != ESP_OK)
      return ret;
    vTaskDelay(pdMS_TO_TICKS(20));
    if (--retry <= 0)
    {
      ESP_LOGE(TAG, "No response.");
      return ESP_ERR_TIMEOUT;
    }
  }

  /*
      6- Read the Gyro Self-Test result:
          X channel: dVX_L and dVX_H (registers 0x51 and 0x52)
          Y channel: dVY_L and dVY_H (registers 0x53 and 0x54)
          Z channel: dVZ_L and dVZ_H (registers 0x55 and 0x56)
          Read the 16 bits result in format signed U12.4, resolution is 62.5mdps (1 / 2^4 dps).
  */
  uint8_t rawBuffer[6];
  ret = ReadRegister(QMI8658_REG_DVX_L, rawBuffer, 6);
  if (ret != ESP_OK)
    return ret;

  float dVX = (((int16_t)rawBuffer[0]) << 12) | ((int16_t)(rawBuffer[1]) >> 4);
  float dVY = (((int16_t)rawBuffer[2]) << 12) | ((int16_t)(rawBuffer[3]) >> 4);
  float dVZ = (((int16_t)rawBuffer[4]) << 12) | ((int16_t)(rawBuffer[5]) >> 4);

  dVX *= (1.0 / (1 << 4)); // 62.5 mdps
  dVY *= (1.0 / (1 << 4)); // 62.5 mdps
  dVZ *= (1.0 / (1 << 4)); // 62.5 mdps

  ESP_LOGI(TAG, "\n\tdVX:%12.4f \n\tdVY:%12.4f \n\tdVZ:%12.4f", dVX, dVY, dVZ);

  //  If the absolute results of all three axes are higher than 300dps, the gyroscope can be considered functional.
  // Otherwise, the gyroscope cannot be considered functional.
  if (abs(dVX) > 300 && abs(dVY) > 300 && abs(dVZ) > 300)
  {
    ESP_LOGI(TAG, "Gyro is working properly.");
  }
  else
  {
    ESP_LOGI(TAG, "Gyro is not working properly.");
    return ESP_ERR_INVALID_RESPONSE;
  }

  return ESP_OK;
}

// This offset change is lost when the sensor is power cycled, or the system is reset
// Each delta offset value should contain 16 bits and the format is signed 11.5 (5 fraction bits, unit is 1 / 2^5).
esp_err_t QMI8658::SetAccelOffset(int16_t offset_x, int16_t offset_y, int16_t offset_z)
{

  uint8_t data[6];
  data[0] = offset_x & 0xff;
  data[1] = offset_x >> 8;
  data[2] = offset_y & 0xff;
  data[3] = offset_y >> 8;
  data[4] = offset_z & 0xff;
  data[5] = offset_z >> 8;
  esp_err_t ret;
  ret = WriteRegister(QMI8658_REG_CAL1_L, data, 2);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister(QMI8658_REG_CAL2_L, data + 2, 2);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister(QMI8658_REG_CAL3_L, data + 4, 2);
  if (ret != ESP_OK)
    return ret;
  return WriteCommand(CTRL_CMD_ACCEL_HOST_DELTA_OFFSET);
}

// This offset change is lost when the sensor is power cycled, or the system is reset
// Each delta offset value should contain 16 bits and the format is signed 11.5 (5 fraction bits, unit is 1 / 2^5).
esp_err_t QMI8658::SetGyroOffset(int16_t offset_x, int16_t offset_y, int16_t offset_z)
{
  esp_err_t ret;
  uint8_t data[6];
  data[0] = offset_x & 0xff;
  data[1] = offset_x >> 8;
  data[2] = offset_y & 0xff;
  data[3] = offset_y >> 8;
  data[4] = offset_z & 0xff;
  data[5] = offset_z >> 8;
  ret = WriteRegister(QMI8658_REG_CAL1_L, data, 2);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister(QMI8658_REG_CAL2_L, data + 2, 2);
  if (ret != ESP_OK)
    return ret;
  ret = WriteRegister(QMI8658_REG_CAL3_L, data + 4, 2);
  if (ret != ESP_OK)
    return ret;

  return WriteCommand(CTRL_CMD_GYRO_HOST_DELTA_OFFSET);
}
