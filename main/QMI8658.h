/*
 * Ansteuerung eines QMI8658-Gyrokops.
 * Umgesetzt auf ESP32, I2C-Interface
 */
#ifndef __QMI8658_H__
#define __QMI8658_H__

#include "driver/i2c_master.h"
#include "QMI8658Constants.h"
#include <math.h>
#include <stdio.h>

#ifdef _BV
#undef _BV
#endif
#define _BV(b) (1UL << (uint32_t)(b))
typedef void (*EventCallBack_t)(void);

typedef struct __IMUdata
{
  float x;
  float y;
  float z;
} IMUdata;

class QMI8658
{
private:
  i2c_master_bus_handle_t mBusHandle;
  i2c_master_dev_handle_t mDevHandle;

  float accelScales, gyroScales;
  uint32_t lastTimestamp = 0;
  uint8_t sampleMode = ASYNC_MODE;
  bool __accel_enabled = false;
  bool __gyro_enabled = false;
  uint32_t revisionID;
  uint8_t usid[6];
  bool __gDataReady = false;
  bool __aDataReady = false;
  gpio_num_t __irq = GPIO_NUM_NC;
  uint8_t __irq_enable_mask = false;
  uint8_t __fifo_mode;
  bool __fifo_interrupt = false;

  uint8_t *__fifo_buffer = NULL;

  EventCallBack_t eventWomEvent = NULL;
  EventCallBack_t eventTagEvent = NULL;
  EventCallBack_t eventPedometerEvent = NULL;
  EventCallBack_t eventNoMotionEvent = NULL;
  EventCallBack_t eventAnyMotionEvent = NULL;
  EventCallBack_t eventSignificantMotion = NULL;
  EventCallBack_t eventGyroDataReady = NULL;
  EventCallBack_t eventAccelDataReady = NULL;
  EventCallBack_t eventDataLocking = NULL;

  esp_err_t ReadRegister(uint8_t aStartReg, uint8_t *aReadBuf, uint8_t aSize);
  esp_err_t WriteRegister(uint8_t aStartReg, uint8_t *aWriteBuf, uint8_t aSize);
  esp_err_t WriteRegister(uint8_t reg, uint8_t norVal, uint8_t orVal);
  uint8_t ReadRegister8(uint8_t aAddr, esp_err_t *aErr = NULL);
  esp_err_t WriteRegister8(uint8_t aAddr, uint8_t aWriteByte);
  esp_err_t SetRegisterBit(uint8_t aAddr, uint8_t bit);
  bool GetRegisterBit(uint8_t aAddr, uint8_t bit, esp_err_t *aErr = NULL);
  esp_err_t ClrRegisterBit(uint8_t aAddr, uint8_t bit);
  /// Aktuelle Zeit in Mikrosekunden
  int64_t GetTime_us();
  uint16_t ReadFromFifo(esp_err_t *aErr = NULL);
  uint8_t MgToBytes(float mg);

public:
  enum AccelRange
  {
    ACC_RANGE_2G,
    ACC_RANGE_4G,
    ACC_RANGE_8G,
    ACC_RANGE_16G
  };

  enum GyroRange
  {
    GYR_RANGE_16DPS,
    GYR_RANGE_32DPS,
    GYR_RANGE_64DPS,
    GYR_RANGE_128DPS,
    GYR_RANGE_256DPS,
    GYR_RANGE_512DPS,
    GYR_RANGE_1024DPS,
  };

  // In 6DOF mode (accelerometer and gyroscope are both enabled),
  // the output data rate is derived from the nature frequency of gyroscope
  enum AccelODR
  {
    ACC_ODR_1000Hz = 3,
    ACC_ODR_500Hz,
    ACC_ODR_250Hz,
    ACC_ODR_125Hz,
    ACC_ODR_62_5Hz,
    ACC_ODR_31_25Hz,
    ACC_ODR_LOWPOWER_128Hz = 12, // The accelerometer low power mode is only available when the gyroscope is disabled
    ACC_ODR_LOWPOWER_21Hz,       // The accelerometer low power mode is only available when the gyroscope is disabled
    ACC_ODR_LOWPOWER_11Hz,       // The accelerometer low power mode is only available when the gyroscope is disabled
    ACC_ODR_LOWPOWER_3Hz         // The accelerometer low power mode is only available when the gyroscope is disabled
  };

  enum GyroODR
  {
    GYR_ODR_7174_4Hz,
    GYR_ODR_3587_2Hz,
    GYR_ODR_1793_6Hz,
    GYR_ODR_896_8Hz,
    GYR_ODR_448_4Hz,
    GYR_ODR_224_2Hz,
    GYR_ODR_112_1Hz,
    GYR_ODR_56_05Hz,
    GYR_ODR_28_025Hz
  };

  enum TapEvent
  {
    INVALID_TAP,
    SINGLE_TAP,
    DOUBLE_TAP,
  };

  // Low-Pass Filter.
  enum LpfMode
  {
    LPF_MODE_0, // 2.66% of ODR
    LPF_MODE_1, // 3.63% of ODR
    LPF_MODE_2, // 5.39% of ODR
    LPF_MODE_3, // 13.37% of ODR
    LPF_OFF,    // OFF Low-Pass Filter
  };

  enum MotionEvent
  {
    MOTION_TAP,
    MOTION_ANT_MOTION,
    MOTION_NO_MOTION,
    MOTION_SIGNIFICANT,
    MOTION_PEDOMETER,
  };

  enum IntPin
  {
    INTERRUPT_PIN_1,
    INTERRUPT_PIN_2,
    INTERRUPT_PIN_DISABLE
  };

  enum FIFO_Samples
  {
    FIFO_SAMPLES_16,
    FIFO_SAMPLES_32,
    FIFO_SAMPLES_64,
    FIFO_SAMPLES_128,
  };

  enum FIFO_Mode
  {
    // Configure the FIFO_MODE to ‘Bypass’ (0) mode, will disable the FIFO functionality.
    FIFO_MODE_BYPASS,
    // In ‘FIFO’ mode, once FIFO is full,
    // the data filling will stop and new data will be discarded until host reads out the FIFO data and release the space for new data to be written to.
    FIFO_MODE_FIFO,
    // In ‘Stream’ mode, once FIFO is full,
    // the data filling will continue and the oldest data will be discarded,
    // until host reads out the FIFO data and release the space for new data to be written to
    FIFO_MODE_STREAM,
  };

  enum SampleMode
  {
    SYNC_MODE,  // Synchronous sampling
    ASYNC_MODE, // Asynchronous sampling
  };

  enum CommandTable
  {
    CTRL_CMD_ACK = 0x00,
    CTRL_CMD_RST_FIFO = 0x04,
    CTRL_CMD_REQ_FIFO = 0x05,
    CTRL_CMD_WRITE_WOM_SETTING = 0x08,
    CTRL_CMD_ACCEL_HOST_DELTA_OFFSET = 0x09,
    CTRL_CMD_GYRO_HOST_DELTA_OFFSET = 0x0A,
    CTRL_CMD_CONFIGURE_TAP = 0x0C,
    CTRL_CMD_CONFIGURE_PEDOMETER = 0x0D,
    CTRL_CMD_CONFIGURE_MOTION = 0x0E,
    CTRL_CMD_RESET_PEDOMETER = 0x0F,
    CTRL_CMD_COPY_USID = 0x10,
    CTRL_CMD_SET_RPU = 0x11,
    CTRL_CMD_AHB_CLOCK_GATING = 0x12,
    CTRL_CMD_ON_DEMAND_CALIBRATION = 0xA2,
    CTRL_CMD_APPLY_GYRO_GAINS = 0xAA,
  };

  enum StatusReg
  {
    EVENT_SIGNIFICANT_MOTION = 128,
    EVENT_NO_MOTION = 64,
    EVENT_ANY_MOTION = 32,
    EVENT_PEDOMETER_MOTION = 16,
    EVENT_WOM_MOTION = 4,
    EVENT_TAP_MOTION = 2,
  };

  enum SensorStatus
  {
    STATUS_INT_CTRL9_CMD_DONE = _BV(0),
    STATUS_INT_LOCKED = _BV(1),
    STATUS_INT_AVAIL = _BV(2),
    STATUS0_GYRO_DATA_READY = _BV(3),
    STATUS0_ACCEL_DATA_READY = _BV(4),
    STATUS1_SIGNIFICANT_MOTION = _BV(5),
    STATUS1_NO_MOTION = _BV(6),
    STATUS1_ANY_MOTION = _BV(7),
    STATUS1_PEDOMETER_MOTION = _BV(8),
    STATUS1_WOM_MOTION = _BV(9),
    STATUS1_TAP_MOTION = _BV(10),
  };

  enum TapDetectionPriority
  {
    PRIORITY0, // (X > Y> Z)
    PRIORITY1, // (X > Z > Y)
    PRIORITY2, // (Y > X > Z)
    PRIORITY3, // (Y > Z > X)
    PRIORITY4, // (Z > X > Y)
    PRIORITY5, // (Z > Y > X)
  };

  enum MotionCtrl
  {

    ANY_MOTION_EN_X = _BV(0),
    ANY_MOTION_EN_Y = _BV(1),
    ANY_MOTION_EN_Z = _BV(2),

    // Logic-AND between events of enabled axes for No-Motion detection, Otherwise, logical OR
    ANY_MOTION_LOGIC_AND = _BV(3),

    NO_MOTION_EN_X = _BV(4),
    NO_MOTION_EN_Y = _BV(5),
    NO_MOTION_EN_Z = _BV(6),

    // Logic-AND between events of enabled axes for No-Motion detection , Otherwise, logical OR
    NO_MOTION_LOGIC_OR = _BV(7),
  };

  QMI8658();
  esp_err_t Init(i2c_master_bus_handle_t aBusHandle, uint8_t aI2CAddr, uint32_t aI2CSpeed_Hz, gpio_num_t aIrqPin, bool ReInit = false);
  void Close();
  ~QMI8658(void);
  uint8_t GetChipID(esp_err_t *aErr = NULL);
  uint8_t WhoAmI(esp_err_t *aErr = NULL);
  uint32_t GetTimestamp(esp_err_t *aErr = NULL);
  float GetTemperature_C(esp_err_t *aErr = NULL);
  esp_err_t EnableINT(IntPin pin, bool enable = true);
  uint8_t GetIrqStatus(esp_err_t *aErr);
  esp_err_t EnableDataReadyINT(bool enable = true);
  esp_err_t ConfigAccelerometer(AccelRange range, AccelODR odr, LpfMode lpfOdr = LPF_MODE_0);
  esp_err_t ConfigGyroscope(GyroRange range, GyroODR odr, LpfMode lpfOdr = LPF_MODE_0);
  esp_err_t ConfigFIFO(FIFO_Mode mode, FIFO_Samples samples = FIFO_SAMPLES_16, IntPin pin = INTERRUPT_PIN_DISABLE, uint8_t trigger_samples = 16);
  uint16_t ReadFromFifo(IMUdata *acc, uint16_t accLength, IMUdata *gyro, uint16_t gyrLength);
  esp_err_t Reset(bool waitResult = true);
  esp_err_t WriteCommand(CommandTable cmd, uint32_t wait_ms = 1000);
  esp_err_t EnableAccelerometer();
  esp_err_t DisableAccelerometer();
  bool IsEnableAccelerometer();
  bool IsEnableGyroscope();
  esp_err_t EnableGyroscope();
  esp_err_t DisableGyroscope();
  esp_err_t GetAccelRaw(int16_t *rawBuffer);
  esp_err_t GetAccelerometer(float &x, float &y, float &z);
  float GetAccelerometerScales();
  float GetGyroscopeScales();
  esp_err_t GetGyroRaw(int16_t *rawBuffer);
  esp_err_t GetGyroscope(float &x, float &y, float &z);
  bool GetDataReady(esp_err_t *aErr = NULL);
  esp_err_t PowerDown();
  esp_err_t PowerOn();
  uint8_t GetStatusRegister(esp_err_t *aErr = NULL);
  esp_err_t ConfigActivityInterruptMap(IntPin pin);
  esp_err_t EnableSyncSampleMode();
  esp_err_t DisableSyncSampleMode();
  esp_err_t EnableLockingMechanism();
  esp_err_t DisableLockingMechanism();
  esp_err_t ConfigPedometer(uint16_t ped_sample_cnt, uint16_t ped_fix_peak2peak, uint16_t ped_fix_peak,
                            uint16_t ped_time_up, uint8_t ped_time_low = 0x14, uint8_t ped_time_cnt_entry = 0x0A, uint8_t ped_fix_precision = 0x00,
                            uint8_t ped_sig_count = 0x04);
  uint32_t GetPedometerCounter(esp_err_t *aErr);
  esp_err_t ClearPedometerCounter();
  esp_err_t EnablePedometer(IntPin pin = INTERRUPT_PIN_DISABLE);
  esp_err_t DisablePedometer();
  esp_err_t ConfigTap(uint8_t priority, uint8_t peakWindow, uint16_t tapWindow, uint16_t dTapWindow,
                      float alpha, float gamma, float peakMagThr, float UDMThr);
  esp_err_t EnableTap(IntPin pin = INTERRUPT_PIN_DISABLE);
  esp_err_t DisableTap();
  TapEvent GetTapStatus(esp_err_t *aErr = NULL);
  esp_err_t ConfigMotion(uint8_t modeCtrl, float AnyMotionXThr, float AnyMotionYThr, float AnyMotionZThr,
                         uint8_t AnyMotionWindow, float NoMotionXThr, float NoMotionYThr, float NoMotionZThr,
                         uint8_t NoMotionWindow, uint16_t SigMotionWaitWindow, uint16_t SigMotionConfirmWindow);
  esp_err_t EnableMotionDetect(IntPin pin = INTERRUPT_PIN_DISABLE);
  esp_err_t DisableMotionDetect();
  esp_err_t ConfigWakeOnMotion(uint8_t WoMThreshold = 200,
                               AccelODR odr = ACC_ODR_LOWPOWER_128Hz,
                               IntPin pin = INTERRUPT_PIN_2,
                               uint8_t defaultPinValue = 1,
                               uint8_t blankingTime = 0x20);
  void GetChipUsid(uint8_t *buffer, uint8_t length);
  uint32_t GetChipFirmwareVersion();
  uint16_t Update(esp_err_t *aErr);
  void SetWakeupMotionEventCallBack(EventCallBack_t cb);
  void SetTapEventCallBack(EventCallBack_t cb);
  void SetPedometerEventCallBack(EventCallBack_t cb);
  void SetNoMotionEventCallBack(EventCallBack_t cb);
  void SetAnyMotionEventCallBack(EventCallBack_t cb);
  void SetSignificantMotionEventCallBack(EventCallBack_t cb);
  void SetGyroDataReadyCallBack(EventCallBack_t cb);
  void SetAccelDataReadyEventCallBack(EventCallBack_t cb);
  void SetDataLockingEventCallBack(EventCallBack_t cb);
  esp_err_t Calibration(uint16_t *gX_gain = NULL, uint16_t *gY_gain = NULL, uint16_t *gZ_gain = NULL);
  esp_err_t WriteCalibration(uint16_t gX_gain, uint16_t gY_gain, uint16_t gZ_gain);
  esp_err_t SelfTestAccel();
  esp_err_t SelfTestGyro();
  esp_err_t SetAccelOffset(int16_t offset_x, int16_t offset_y, int16_t offset_z);
  esp_err_t SetGyroOffset(int16_t offset_x, int16_t offset_y, int16_t offset_z);
};

#endif
