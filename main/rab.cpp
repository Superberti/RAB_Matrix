// Waveshare ESP32-S3 LED-Matrix (8x8)-Board
// https://www.waveshare.com/wiki/ESP32-S3-Matrix
// mit QMI8658C Gyroskop

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include <string.h>
#include "driver/gpio.h"
#include <driver/rtc_io.h>
#include "esp_random.h"
#include <esp_sleep.h>
#include "esp_check.h"
#include <memory>
#include "esp_timer.h"
#include "rab.h"
#include <led_strip.h>
#include "QMI8658.h"
#include "fonts_8x8.h"

#define FONT8X8 font8x8_ic8x8u

/*
 * Pindefinitionen:
 * WS2812-Datenausgang: 14
 *
 */

#define BLINK_GPIO GPIO_NUM_14
#define TAG "RAB_MATRIX"
#define NUM_PIXEL 64
#define NUM_PIXEL_X 8
#define NUM_PIXEL_Y 8
#define I2C_FREQ_HZ 400000
#define PIN_SDA_BUS0 GPIO_NUM_11
#define PIN_SCL_BUS0 GPIO_NUM_12
#define QMI_INT1 GPIO_NUM_10
#define QMI_INT2 GPIO_NUM_13
#define QME8658_ADDRESS 0x6b // 107

const uint8_t ROTATION_0 = 0;   // wires at the left
const uint8_t ROTATION_90 = 1;  // wires at the top
const uint8_t ROTATION_180 = 2; // wires at the right
const uint8_t ROTATION_270 = 3; // wires at the bottom
led_strip_handle_t led_strip;

extern "C"
{
  void app_main()
  {
    // gpio_set_level(BOARD_LED, 0);
    // esp_log_level_set("*", ESP_LOG_ERROR);
    app_main_cpp();
  }
}

void app_main_cpp()
{
  // vTaskDelay(pdMS_TO_TICKS(1000));
  LEDMatrix m;
  m.Run();
}

LEDMatrix::LEDMatrix()
{
  i2c_bus_h_0 = NULL;
  Qmi = new QMI8658();
}

void LEDMatrix::Run()
{
  esp_err_t ret;

  // Initialisierung GPIO
  ESP_LOGI(TAG, "GPIO init...");
  InitGPIO();

  // Initialisierung RGB-Matrix
  ESP_LOGI(TAG, "Starte RAB MATRIX");
  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = {};
  strip_config.strip_gpio_num = BLINK_GPIO;             // The GPIO that connected to the LED strip's data line
  strip_config.max_leds = NUM_PIXEL;                    // The number of LEDs in the strip,
  strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB; // Pixel format of your LED strip
  strip_config.led_model = LED_MODEL_WS2812;            // LED strip model
  strip_config.flags.invert_out = false;                // whether to invert the output signal (useful when your hardware has a level inverter)

  led_strip_rmt_config_t rmt_config = {};
  rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;    // different clock source can lead to different power consumption
  rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz
  rmt_config.flags.with_dma = true;            // whether to enable the DMA feature

  ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
  if (ret != ESP_OK)
  {
    error("Fehler beim Init des RMT-Devices: %d.", ret);
  }

  bool QMI_OK = false;
  // Initialisierung I2C
  ESP_LOGI(TAG, "I2C init...");
  ret = InitI2C(I2C_NUM_0, PIN_SDA_BUS0, PIN_SCL_BUS0, &i2c_bus_h_0);
  if (ret != ESP_OK)
  {
    error("Fehler beim InitI2C(0): %d.", ret);
  }

  // Init Gyroskop
  ret = Qmi->Init(i2c_bus_h_0, QME8658_ADDRESS, I2C_FREQ_HZ, QMI_INT1, QMI_INT2);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Fehler beim Initialisieren des QME8658: %d.", ret);
  }
  else
    QMI_OK = true;
  if (QMI_OK)
  {
    ESP_LOGI(TAG, "QMI setup...");
    if (Qmi->SelfTestAccel() == ESP_OK)
    {
      ESP_LOGI(TAG, "Accelerometer self-test successful");
    }
    else
    {
      ESP_LOGE(TAG, "Accelerometer self-test failed!");
    }

    if (Qmi->SelfTestGyro() == ESP_OK)
    {
      ESP_LOGI(TAG, "Gyroscope self-test successful");
    }
    else
    {
      ESP_LOGE(TAG, "Gyroscope self-test failed!");
    }

    ret = Qmi->ConfigAccelerometer(
        /*
         * ACC_RANGE_2G
         * ACC_RANGE_4G
         * ACC_RANGE_8G
         * ACC_RANGE_16G
         * */
        QMI8658::ACC_RANGE_4G,
        /*
         * ACC_ODR_1000H
         * ACC_ODR_500Hz
         * ACC_ODR_250Hz
         * ACC_ODR_125Hz
         * ACC_ODR_62_5Hz
         * ACC_ODR_31_25Hz
         * ACC_ODR_LOWPOWER_128Hz
         * ACC_ODR_LOWPOWER_21Hz
         * ACC_ODR_LOWPOWER_11Hz
         * ACC_ODR_LOWPOWER_3H
         * */
        QMI8658::ACC_ODR_1000Hz,
        /*
         *  LPF_MODE_0     //2.66% of ODR
         *  LPF_MODE_1     //3.63% of ODR
         *  LPF_MODE_2     //5.39% of ODR
         *  LPF_MODE_3     //13.37% of ODR
         *  LPF_OFF        // OFF Low-Pass Fitter
         * */
        QMI8658::LPF_MODE_0);
    if (ret != ESP_OK)
      ESP_LOGI(TAG, "ConfigAccelerometer failed with error code: %d", ret);

    ret = Qmi->ConfigGyroscope(
        /*
         * GYR_RANGE_16DPS
         * GYR_RANGE_32DPS
         * GYR_RANGE_64DPS
         * GYR_RANGE_128DPS
         * GYR_RANGE_256DPS
         * GYR_RANGE_512DPS
         * GYR_RANGE_1024DPS
         * */
        QMI8658::GYR_RANGE_64DPS,
        /*
         * GYR_ODR_7174_4Hz
         * GYR_ODR_3587_2Hz
         * GYR_ODR_1793_6Hz
         * GYR_ODR_896_8Hz
         * GYR_ODR_448_4Hz
         * GYR_ODR_224_2Hz
         * GYR_ODR_112_1Hz
         * GYR_ODR_56_05Hz
         * GYR_ODR_28_025H
         * */
        QMI8658::GYR_ODR_896_8Hz,
        /*
         *  LPF_MODE_0     //2.66% of ODR
         *  LPF_MODE_1     //3.63% of ODR
         *  LPF_MODE_2     //5.39% of ODR
         *  LPF_MODE_3     //13.37% of ODR
         *  LPF_OFF        // OFF Low-Pass Fitter
         * */
        QMI8658::LPF_MODE_3);
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "ConfigGyroscope failed with error code: %d", ret);
    /*
     * If both the accelerometer and gyroscope sensors are turned on at the same time,
     * the output frequency will be based on the gyroscope output frequency.
     * The example configuration is 896.8HZ output frequency,
     * so the acceleration output frequency is also limited to 896.8HZ
     * */
    ret = Qmi->EnableGyroscope();
    if (ret != ESP_OK)
      ESP_LOGI(TAG, "EnableGyroscope failed with error code: %d", ret);
    ret = Qmi->EnableAccelerometer();
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "EnableAccelerometer failed with error code: %d", ret);
  }

  IMUdata Accel;
  IMUdata Gyro;
  ESP_LOGI(TAG, "Hauptschleife...");
  const char *Anzeige = "Hallo, RAB *2025*";
  int CharIndex = 0;
  for (;;)
  {
    /*
    for (int i = 0; i < NUM_PIXEL; i++)
    {
      if (i > 0)
        led_strip_set_pixel(led_strip, i - 1, 0, 0, 0);
      else
        led_strip_set_pixel(led_strip, NUM_PIXEL - 1, 0, 0, 0);
      led_strip_set_pixel(led_strip, i, 50, 50, 50);
      led_strip_refresh(led_strip);
      vTaskDelay(pdMS_TO_TICKS(10));
    }*/
    if (Anzeige[CharIndex] == 0)
      CharIndex = 0;
    RenderChar(Anzeige[CharIndex], 30, 30, 30);
    CharIndex++;
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (QMI_OK && Qmi->GetDataReady())
    {
      ret = Qmi->GetAccelerometer(Accel.x, Accel.y, Accel.z);
      if (ret == ESP_OK)
      {
        ESP_LOGI(TAG, "ACCEL:  %f  %f  %f\r\n", Accel.x, Accel.y, Accel.z);
      }
      else
        ESP_LOGE(TAG, "GetAccelerometer failed with error code: %d", ret);

      ret = Qmi->GetGyroscope(Gyro.x, Gyro.y, Gyro.z);
      if (ret == ESP_OK)
      {
        ESP_LOGI(TAG, "GYRO:  %f  %f  %f", Gyro.x, Gyro.y, Gyro.z);
      }
      else
        ESP_LOGE(TAG, "GetGyroscope failed with error code: %d", ret);
      ESP_LOGI(TAG, "\t\t>      %lu   %.2f ℃", Qmi->GetTimestamp(), Qmi->GetTemperature_C());
    }

  }
}

LEDMatrix::~LEDMatrix()
{
  delete Qmi;
}

esp_err_t LEDMatrix::InitGPIO()
{
  // Outputs
  /*
  const uint64_t OutputBitMask = (1ULL << BOARD_LED);

  gpio_config_t ConfigOutput = {};
  ConfigOutput.pin_bit_mask = OutputBitMask;
  ConfigOutput.mode = GPIO_MODE_INPUT_OUTPUT; // damit man auch den aktuell eingestellten Wert zurücklesen kann!
  ConfigOutput.pull_up_en = GPIO_PULLUP_DISABLE;
  ConfigOutput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigOutput.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&ConfigOutput), TAG, "Fehler bei Init output GPIO");
*/
  // Inputs

  const uint64_t InputBitMask = (1ULL << QMI_INT1) | (1ULL << QMI_INT2);
  gpio_config_t ConfigInput = {};
  ConfigInput.pin_bit_mask = InputBitMask;
  ConfigInput.mode = GPIO_MODE_INPUT;
  ConfigInput.pull_up_en = GPIO_PULLUP_ENABLE;
  ConfigInput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigInput.intr_type = GPIO_INTR_POSEDGE;
  ESP_RETURN_ON_ERROR(gpio_config(&ConfigInput), TAG, "Fehler bei Init input GPIO");

  return ESP_OK;
}

esp_err_t LEDMatrix::InitI2C(i2c_port_t aPort, gpio_num_t aSDA_Pin, gpio_num_t aSCL_Pin, i2c_master_bus_handle_t *aBusHandle)
{
  if (aPort > 1)
    return ESP_ERR_INVALID_ARG;

  i2c_master_bus_config_t conf={};
  conf.i2c_port = aPort;
  conf.sda_io_num = aSDA_Pin;
  conf.scl_io_num = aSCL_Pin;
  conf.clk_source = I2C_CLK_SRC_DEFAULT;
  conf.glitch_ignore_cnt = 7;
  conf.intr_priority = 0;
  conf.trans_queue_depth = 0;
  conf.flags.enable_internal_pullup = 1;
  return i2c_new_master_bus(&conf, aBusHandle);
}

#define MAX_VPBUFLEN 256
char vprintf_buffer[MAX_VPBUFLEN];

void LEDMatrix::error(const char *format, ...)
{
  va_list myargs;
  va_start(myargs, format);

  vsnprintf(vprintf_buffer, MAX_VPBUFLEN, format, myargs);
  ESP_LOGE(TAG, "%s", vprintf_buffer);
  va_end(myargs);
  int toggle = 0;
  for (;;)
  {
    toggle = 1 - toggle;
    // gpio_set_level(BOARD_LED, toggle);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

const uint8_t *LEDMatrix::getImage(uint8_t ch)
{
  if (ch < 128)
  {
    return FONT8X8[ch];
  }
  if (ch >= 128 && ch < 160)
  {
    // miscellaneous
    ch -= 128;
    if (ch < sizeof(font8x8_misc) / 8)
    {
      return font8x8_misc[ch];
    }
    return FONT8X8[0]; // default
  }
  return FONT8X8[ch - 160 + 128];
}

uint8_t LEDMatrix::reverseBits(uint8_t b)
{
  uint8_t inv = 0;
  for (int j = 0; j < 8; ++j)
  {
    inv <<= 1;
    if (b & 0x1)
      inv |= 1;
    b >>= 1;
  }
  return inv;
}

void LEDMatrix::rotateChar90(const uint8_t *image, uint8_t newb[8])
{
  for (int i = 0; i < 8; ++i)
  {
    newb[i] = 0;
    for (int j = 0; j < 8; ++j)
    {
      uint8_t b = image[j];
      newb[i] |= (b & (1 << i)) ? 1 << (7 - j) : 0;
    }
  }
}

void LEDMatrix::rotateChar270(const uint8_t *image, uint8_t newb[8])
{
  for (int i = 0; i < 8; ++i)
  {
    newb[i] = 0;
    for (int j = 0; j < 8; ++j)
    {
      uint8_t b = image[j];
      newb[i] |= (b & (1 << (7 - i))) ? 1 << j : 0;
    }
  }
}

void LEDMatrix::RenderChar(char aChar, uint8_t r, uint8_t g, uint8_t b)
{
  led_strip_clear(led_strip);
  const uint8_t *CharBitmap = getImage(aChar);
  for (uint8_t x = 0; x < 8; x++)
  {
    for (uint8_t y = 0; y < 8; y++)
    {
      if (((const uint8_t *)CharBitmap)[x] & (1 << (7 - y)))
        SetMatrixPixel(7 - x, 7 - y, r, g, b);
    }
  }
  led_strip_refresh(led_strip);
}

void LEDMatrix::SetMatrixPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b)
{
  if (x >= 8 || y >= 8)
    return;
  uint32_t LEDIndex = x * 8 + y;
  led_strip_set_pixel(led_strip, LEDIndex, r, g, b);
}