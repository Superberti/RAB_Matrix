#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <vector>
#include <list>

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

uint32_t millis();

typedef union {
  struct __attribute__ ((packed)) {
    uint8_t b, g, r, w;  // Little-endian ordered
  };
  uint32_t raw32;
} pixelColor_t;

inline pixelColor_t pixelFromRGB(uint8_t r, uint8_t g, uint8_t b)
{
  pixelColor_t v;
  v.r = r;
  v.g = g;
  v.b = b;
  v.w = 0;
  return v;
}

inline pixelColor_t pixelFromRGBhex(uint8_t r, uint8_t g, uint8_t b)
{
  pixelColor_t v;
  v.r = r;
  v.g = g;
  v.b = b;
  v.w = 0;
  return v;
}

inline pixelColor_t pixelFromRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
  pixelColor_t v;
  v.r = r;
  v.g = g;
  v.b = b;
  v.w = w;
  return v;
}

inline pixelColor_t pixelFromRGBWhex(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
  // The value is of the form 0xWWRRGGBB
  pixelColor_t v;
  v.r = r;
  v.g = g;
  v.b = b;
  v.w = w;
  return v;
}

#ifdef __cplusplus
}
#endif


class LEDParticle
{
protected:
  pixelColor_t mStarColor;
  uint32_t mBirthTime;
  uint32_t mLastUpdate;
  double mVelocity;
  double mPosition;

public:
  LEDParticle(pixelColor_t aStarColor, double aPos, double aMaxSpeed);
  double Age();
  void Update();
  static pixelColor_t AdjustByUniformFactor(pixelColor_t aColor, double aAdjFactor);
  pixelColor_t GetStarColor() { return mStarColor; };
  double GetPosition() { return mPosition; };
};

class LEDEffectBase
{
protected:
  uint32_t mStartTime;
  pixelColor_t mEffectColor;
  pixelColor_t *mLEDColors;
  uint32_t mCounter;
  uint32_t mNumLEDs;
  uint32_t elapsed();
  int GetRandInt(int aMax);
  int GetRandInt(int aMin, int aMax);
  double GetRandDouble();

public:
  pixelColor_t GetLEDColors();
  uint32_t NumLEDs();

  LEDEffectBase(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor);
  ~LEDEffectBase();

  virtual uint32_t DrawNext() = 0;
  int GetLEDNum(int aCounter);
  static pixelColor_t Rainbow(double progress);
  static pixelColor_t Multiply(pixelColor_t aColor, double aFactor);
  static pixelColor_t FadeColor(pixelColor_t aColor, uint8_t fadeValue);
  void SetPixelColor(uint32_t aPixelNum, uint8_t r, uint8_t g, uint8_t b);
  void SetPixelColor(uint32_t aPixelNum, pixelColor_t aColor);
  void SetAllColors(pixelColor_t aColor);
  void SetAllColors(uint8_t r, uint8_t g, uint8_t b);
};

// ----------------------------------------------------------------------------

class Rainbower
{
protected:
  double mStretchFactor = 5;
  double mDimmDivisor = 1;
  uint8_t stepVal1 = 0;
  double r, g, b;

public:
  Rainbower(pixelColor_t aStartColor, double aStretchFactor = 5, double aDimmDivisor = 1);
  pixelColor_t DrawNext();
};

// ----------------------------------------------------------------------------

class LE_Scanner : LEDEffectBase
{
protected:
  int mNumLEDsToFade;
  double rb = 0.0;

public:
  LE_Scanner(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor, uint32_t aNumLEDsToFade);
  uint32_t DrawNext();
};

// ----------------------------------------------------------------------------

class LE_RainbowWaves : LEDEffectBase
{
protected:
  std::vector<Rainbower> StartRainbowers;

public:
  LE_RainbowWaves(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor);
  uint32_t DrawNext();
  ~LE_RainbowWaves();
};

// ----------------------------------------------------------------------------

class LE_RainbowWaves2 : LEDEffectBase
{
protected:
  double inc = 0.01;
  double *prog_arr = NULL;
  bool vz = true;

public:
  LE_RainbowWaves2(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor);
  uint32_t DrawNext();
  ~LE_RainbowWaves2();
};

// ----------------------------------------------------------------------------

class LE_Fireworks : LEDEffectBase
{
protected:
  const static uint32_t mColorChoices[];
  const double MaxSpeed = 37.0;              // Max velocity
  const double NewParticleProbability = 0.1; // Odds of new particle
  const double ParticlePreignitonTime = 0.0; // How long to "wink"
  const double ParticleIgnition = 0.2;       // How long to "flash"
  const double ParticleHoldTime = 0.0;       // Main lifecycle time
  const double ParticleFadeTime = 2.0;       // Fade out time
  double ParticleSize = 0.0;                 // Size of the particle
  std::list<LEDParticle> mParticles;

public:
  LE_Fireworks(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor);
  uint32_t DrawNext();
  void Render();
  void FillSolid(pixelColor_t c);
  void SetPixels(double pos, int width, pixelColor_t c);
};

// ----------------------------------------------------------------------------

class LE_RunningLights : LEDEffectBase
{
protected:
public:
  LE_RunningLights(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor);
  uint32_t DrawNext();
};

// ----------------------------------------------------------------------------

class LE_Fire : LEDEffectBase
{
protected:
  uint8_t *heat = NULL;
  int mCooling = 55;
  int mSparking = 120;
  int mSpeedDelay = 25;

public:
  LE_Fire(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor);
  uint32_t DrawNext();
  void Init(int Cooling, int Sparking, int SpeedDelay);
  void SetPixelHeatColor(int Pixel, uint8_t temperature);
  ~LE_Fire();
};

// ----------------------------------------------------------------------------

class LE_BouncingColoredBalls : LEDEffectBase
{
protected:
  const double Gravity = -9.81;
  const int BallCount = 12;
  int StartHeight = 1;
  double *Height = NULL;
  double ImpactVelocityStart;
  double *ImpactVelocity = NULL;
  double *TimeSinceLastBounce = NULL;
  int *Position = NULL;
  uint32_t *ClockTimeSinceLastBounce = NULL;
  double *Dampening = NULL;
  pixelColor_t *BallColors;
  int *Offset = NULL;

public:
  LE_BouncingColoredBalls(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor);
  uint32_t DrawNext();
  ~LE_BouncingColoredBalls();
};

// ----------------------------------------------------------------------------

class LE_Meteor : LEDEffectBase
{
protected:
  uint8_t mMeteorSize = 20;
  uint8_t mMeteorTrailDecay = 64;
  bool mMeteorRandomDecay = true;
  int mStartIndex;
  int mTTL;
  int mFramesAfterTTL = 50;
  pixelColor_t mFadeColor;
  bool mDirection = true;

public:
  LE_Meteor(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor);
  uint32_t DrawNext();
};

// ----------------------------------------------------------------------------

class LE_RandomTwinkle : LEDEffectBase
{
protected:
  struct star
  {
    int pos;
    uint8_t fade;
  };
  int mNumStars = 40;
  star* mStars=NULL;

public:
  LE_RandomTwinkle(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor);
  ~LE_RandomTwinkle();
  uint32_t DrawNext();
};