#ifndef RAB_H
#define RAB_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "time.h"
#include <string>
#include <vector>
#include "Effects.h"

void app_main_cpp();
void error(const char *format, ...);

// Forwards
class QMI8658;

class LEDMatrix
{
protected:
  i2c_master_bus_handle_t i2c_bus_h_0;
  Rainbower* MyRainbower;
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
  void RenderChar(uint8_t aChar[8], uint8_t r, uint8_t g, uint8_t b);
  uint8_t reverseBits(uint8_t b);
  void rotateChar90(const uint8_t *image, uint8_t newb[8]);
  void rotateChar270(const uint8_t *image, uint8_t newb[8]);
  void SetMatrixPixel(uint8_t x, uint8_t y,uint8_t r, uint8_t g, uint8_t b, bool aSet);
public:
  LEDMatrix();
  ~LEDMatrix();
  void Run();
  static const uint8_t* getImage(uint8_t ch);
};

/// @brief Klasse zur Speicherung einer Text-Bitmap mit einem 8x8-Font. Also pro Zeichen 8 Byte. Für ein 8x8 LED-Array
class TextBitmap
{
  protected:
    /// @brief Anzuzeigender Text
    std::string mText;
    /// @brief Text-Bitmap
    uint8_t* mTextBitmap;
    /// @brief Breite der Bitmap (=Anzahl Zeichen mText*8)
    const uint8_t mWidth;
    /// @brief Höhe der Bitmap, immer 8
    const uint8_t mHeight=8;
    /// @brief Anzahl bytes einer Zeile der Bitmap
    const uint8_t mStride;
    /// @brief Pixel der Bitmap abfragen
    /// @param x 
    /// @param y 
    /// @return 
    bool GetPixel(uint8_t x, uint8_t y);
    /// @brief Pixel der Bitmap setzen
    /// @param x 
    /// @param y 
    /// @param aSet 
    /// @return 
    void SetPixel(uint8_t x, uint8_t y, bool aSet);
  public:
    /// @brief 
    /// @param aText 
    TextBitmap(const std::string & aText);
    ~TextBitmap();
    /// @brief 8x8 Pixel-Ausschnitt aus der Text-Bitmap zurückliefern
    /// @param aLeft Linker Anfang des Ausschnitts (kann auch negativ sein!)
    /// @param aTop Oberer Anfang des Ausschnitts (kann auch negativ sein!)
    /// @param aFillZero true: Außerhalb liegende Pixel werden mit 0 (false) zurückgegeben, ansonsten wird ein wrap-around gemacht
    /// @param aClipBuffer 8x8 Teilstück
    void GetClip(int16_t aLeft, int16_t aTop, bool aFillWhite, uint8_t aClipBuffer[8]);
    uint8_t Width(){return mWidth;}
    uint8_t Height(){return mHeight;}
    uint8_t Stride(){return mStride;}
};

#endif