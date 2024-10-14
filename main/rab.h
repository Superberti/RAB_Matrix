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

public:
  LEDMatrix();
  ~LEDMatrix();
  void Run();
};

#endif