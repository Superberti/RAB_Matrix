#ifndef RAB_H
#define RAB_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "time.h"
#include <string>
#include <vector>

void app_main_cpp();
void error(const char *format, ...);

// Forwards
class QMI8658;

class LEDMatrix
{
protected:
  i2c_master_bus_handle_t i2c_bus_h_0;

  QMI8658 *Qmi;
  esp_err_t InitI2C(i2c_port_t aPort, gpio_num_t aSDA_Pin, gpio_num_t aSCL_Pin, i2c_master_bus_handle_t *aBusHandle);
  esp_err_t InitGPIO();
  void error(const char *format, ...);
  /// @brief Zeichnet einen Buchstaben auf der 8x8 LED-Matrix
  /// @param aChar Zeichen
  /// @param r Rotanteil
  /// @param g Grünanteil
  /// @param b Blauanteil
  void RenderChar(char aChar, uint8_t r, uint8_t g, uint8_t b);
  const uint8_t* getImage(uint8_t ch);
  uint8_t reverseBits(uint8_t b);
  void rotateChar90(const uint8_t *image, uint8_t newb[8]);
  void rotateChar270(const uint8_t *image, uint8_t newb[8]);
  void SetMatrixPixel(uint8_t x, uint8_t y,uint8_t r, uint8_t g, uint8_t b);
public:
  LEDMatrix();
  ~LEDMatrix();
  void Run();
};

#endif