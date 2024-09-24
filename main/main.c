#include <stdio.h>
#include <led_strip.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BLINK_GPIO 14

led_strip_handle_t led_strip;

#define TAG "RAB_MATRIX"
#define NUM_PIXEL 64

void app_main(void)
{
  ESP_LOGI(TAG, "Starte RAB MATRIX");
  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = {
      .strip_gpio_num = BLINK_GPIO,             // The GPIO that connected to the LED strip's data line
      .max_leds = NUM_PIXEL,                    // The number of LEDs in the strip,
      .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
      .led_model = LED_MODEL_WS2812,            // LED strip model
      .flags.invert_out = false,                // whether to invert the output signal (useful when your hardware has a level inverter)
  };

  led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
      .rmt_channel = 0,
#else
      .clk_src = RMT_CLK_SRC_DEFAULT,    // different clock source can lead to different power consumption
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
      .flags.with_dma = true,            // whether to enable the DMA feature
#endif
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

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
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}
