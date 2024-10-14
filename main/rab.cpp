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

/*
 * Pindefinitionen:
 * WS2812-Datenausgang: 14
 *
 */

#define BLINK_GPIO GPIO_NUM_14
#define TAG "RAB_MATRIX"
#define NUM_PIXEL 64
#define I2C_FREQ_HZ 400000
#define PIN_SDA_BUS0 GPIO_NUM_11
#define PIN_SCL_BUS0 GPIO_NUM_12
#define QMI_INT1 GPIO_NUM_10
#define QMI_INT2 GPIO_NUM_13
#define QME8658_ADDRESS 0x6a // 106

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
  // Log auf eigene Funktion umbiegen
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

  // Initialisierung I2C
  ESP_LOGI(TAG, "I2C init...");
  ret = InitI2C(I2C_NUM_0, PIN_SDA_BUS0, PIN_SCL_BUS0, &i2c_bus_h_0);
  if (ret != ESP_OK)
  {
    error("Fehler beim InitI2C(0): %d.", ret);
  }

  // Init Gyroskop
  ret = Qmi->Init(i2c_bus_h_0, QME8658_ADDRESS, I2C_FREQ_HZ, QMI_INT1);
  if (ret != ESP_OK)
  {
    error("Fehler beim Initialisieren des QME8658: %d.", ret);
  }

  for (;;)
  {
    for (int i = 0; i < NUM_PIXEL; i++)
    {
      if (i > 0)
        led_strip_set_pixel(led_strip, i - 1, 0, 0, 0);
      else
        led_strip_set_pixel(led_strip, NUM_PIXEL - 1, 0, 0, 0);
      led_strip_set_pixel(led_strip, i, 50, 50, 50);
      led_strip_refresh(led_strip);
      vTaskDelay(pdMS_TO_TICKS(10));
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
  
  const uint64_t InputBitMask = (1ULL << QMI_INT1) |  (1ULL << QMI_INT2);
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

  i2c_master_bus_config_t conf;
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
