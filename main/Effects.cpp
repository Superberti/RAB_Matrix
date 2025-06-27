#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "Effects.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


uint32_t IRAM_ATTR millis() 
{ 
  return xTaskGetTickCount() * portTICK_PERIOD_MS; 
}


LEDParticle::LEDParticle(pixelColor_t aStarColor, double aPos, double aMaxSpeed)
{
  srand(millis());
  mStarColor = aStarColor;
  mPosition = aPos;
  mVelocity = ((double)rand() / (double)RAND_MAX) * aMaxSpeed * 2 - aMaxSpeed;
  mBirthTime = millis();
}

double LEDParticle::Age()
{
  return (millis() - mBirthTime) / 1000.0;
}

void LEDParticle::Update()
{
  // As the particle ages we actively fade its color and slow its speed
  double deltaTime = (millis() - mLastUpdate) / 1000.0;
  mPosition += mVelocity * deltaTime;
  mLastUpdate = millis();
  mVelocity -= 2 * mVelocity * deltaTime;
  mStarColor = AdjustByUniformFactor(mStarColor, ((double)rand() / (double)RAND_MAX) * 0.1);
}

pixelColor_t LEDParticle::AdjustByUniformFactor(pixelColor_t aColor, double aAdjFactor)
{
  aColor.r = aColor.r * (1.0 - aAdjFactor);
  aColor.g = aColor.g * (1.0 - aAdjFactor);
  aColor.b = aColor.b * (1.0 - aAdjFactor);
  return aColor;
}

// ----------------------------------------------------------------------------

LEDEffectBase::LEDEffectBase(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor)
{
  mStartTime = millis();
  mEffectColor = aEffectColor;
  mNumLEDs = aNumLEDs;
  mLEDColors = aPixels;
  mCounter = 0;
  srand(millis());
}

int LEDEffectBase::GetRandInt(int aMax)
{
  return rand() % aMax;
}

int LEDEffectBase::GetRandInt(int aMin, int aMax)
{
  return rand() % aMax + aMin;
}

double LEDEffectBase::GetRandDouble()
{
  return ((double)rand() / (double)RAND_MAX);
}

uint32_t LEDEffectBase::elapsed()
{
  return millis() - mStartTime;
}

int LEDEffectBase::GetLEDNum(int aCounter)
{
  int ModCount = aCounter % mNumLEDs;
  return ModCount >= 0 ? ModCount : ModCount + mNumLEDs;
}

pixelColor_t LEDEffectBase::Rainbow(double progress)
{
  double div = (fabs(fmod(progress, 1)) * 6);
  int ascending = (int)((fmod(div, 1)) * 255);
  int descending = 255 - ascending;

  switch ((int)div)
  {
  case 0:
    return pixelFromRGB(255, ascending, 0);
  case 1:
    return pixelFromRGB(descending, 255, 0);
  case 2:
    return pixelFromRGB(0, 255, ascending);
  case 3:
    return pixelFromRGB(0, descending, 255);
  case 4:
    return pixelFromRGB(ascending, 0, 255);
  default: // case 5:
    return pixelFromRGB(255, 0, descending);
  }
}

pixelColor_t LEDEffectBase::Multiply(pixelColor_t aColor, double aFactor)
{
  pixelColor_t iStartColor = aColor;
  iStartColor.r = (iStartColor.r * fabs(aFactor));
  iStartColor.g = (iStartColor.g * fabs(aFactor));
  iStartColor.b = (iStartColor.b * fabs(aFactor));
  return iStartColor;
}

pixelColor_t LEDEffectBase::FadeColor(pixelColor_t aColor, uint8_t fadeValue)
{
  uint8_t r, g, b;
  r = aColor.r;
  g = aColor.g;
  b = aColor.b;

  r = ((r <= 10) ? 0 : r - (r * fadeValue / 256));
  g = ((g <= 10) ? 0 : g - (g * fadeValue / 256));
  b = ((b <= 10) ? 0 : b - (b * fadeValue / 256));

  pixelColor_t FadeColor;
  FadeColor.r = r;
  FadeColor.g = g;
  FadeColor.b = b;
  return FadeColor;
}

void LEDEffectBase::SetPixelColor(uint32_t aPixelNum, uint8_t r, uint8_t g, uint8_t b)
{
  if (aPixelNum > mNumLEDs - 1)
    return;
  mLEDColors[aPixelNum].r = r;
  mLEDColors[aPixelNum].g = g;
  mLEDColors[aPixelNum].b = b;
}

void LEDEffectBase::SetPixelColor(uint32_t aPixelNum, pixelColor_t aColor)
{
  if (aPixelNum > mNumLEDs - 1)
    return;
  mLEDColors[aPixelNum] = aColor;
}

void LEDEffectBase::SetAllColors(pixelColor_t aColor)
{
  for (int i = 0; i < mNumLEDs; i++)
    mLEDColors[i] = aColor;
}

void LEDEffectBase::SetAllColors(uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = 0; i < mNumLEDs; i++)
  {
    mLEDColors[i].r = r;
    mLEDColors[i].g = g;
    mLEDColors[i].b = b;
  }
}

// ----------------------------------------------------------------------------

Rainbower::Rainbower(pixelColor_t aStartColor, double aStretchFactor, double aDimmDivisor)
{
  r = aStartColor.r;
  g = aStartColor.g;
  b = aStartColor.b;
  mStretchFactor = aStretchFactor;
  mDimmDivisor = aDimmDivisor;
}

pixelColor_t Rainbower::DrawNext()
{
  pixelColor_t CurrentColor = pixelFromRGB(fmin(255.0, fmax(0.0, r)) / mDimmDivisor + 0.5, fmin(255.0, fmax(0.0, g)) / mDimmDivisor + 0.5, fmin(255.0, fmax(0.0, b)) / mDimmDivisor + 0.5);
  switch (stepVal1)
  {
  case 0:
    g += mStretchFactor;
    if (g >= 255.0)
      stepVal1++;
    break;
  case 1:
    r -= mStretchFactor;
    if (r <= 0.0)
      stepVal1++;
    break;
  case 2:
    b += mStretchFactor;
    if (b >= 255.0)
      stepVal1++;
    break;
  case 3:
    g -= mStretchFactor;
    if (g <= 0.0)
      stepVal1++;
    break;
  case 4:
    r += mStretchFactor;
    if (r >= 255.0)
      stepVal1++;
    break;
  case 5:
    b -= mStretchFactor;
    if (b <= 0.0)
      stepVal1 = 0;
    break;
  }
  return CurrentColor;
}

// ----------------------------------------------------------------------------

LE_Scanner::LE_Scanner(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor, uint32_t aNumLEDsToFade)
    : LEDEffectBase(aNumLEDs, aPixels, aEffectColor)
{
  mNumLEDsToFade = aNumLEDsToFade;
}

uint32_t LE_Scanner::DrawNext()
{
  int count = mCounter % mNumLEDs;
  if (mCounter % (mNumLEDs / 4) == 0)
  {
    mEffectColor = Rainbow(rb);
    rb += 0.05;
  }
  for (int i = 0; i < mNumLEDs; i++)
  {
    mLEDColors[i].raw32 = 0;
  }
  // x vorher, x nachher faden...
  int fadeout = mNumLEDsToFade;
  double f = -fadeout;
  for (int i = count - fadeout; i < count + fadeout; i++)
  {
    int LedNum = GetLEDNum(i);
    mLEDColors[LedNum].r = mEffectColor.r - fabs(f) / (double)fadeout * mEffectColor.r;
    mLEDColors[LedNum].g = mEffectColor.g - fabs(f) / (double)fadeout * mEffectColor.g;
    mLEDColors[LedNum].b = mEffectColor.b - fabs(f) / (double)fadeout * mEffectColor.b;
    f++;
  }
  mCounter++;
  return 10; // 10 ms Delay
}

// ----------------------------------------------------------------------------

LE_RainbowWaves::LE_RainbowWaves(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor)
    : LEDEffectBase(aNumLEDs, aPixels, aEffectColor)
{
  pixelColor_t StartColor = pixelFromRGB(255, 0, 0);
  Rainbower StartRainbower(StartColor, 10);
  for (int i = 0; i < mNumLEDs; i++)
  {
    StartRainbowers.push_back(Rainbower(StartRainbower.DrawNext()));
  }
}

uint32_t LE_RainbowWaves::DrawNext()
{
  for (int i = 0; i < mNumLEDs; i++)
  {
    mLEDColors[i] = StartRainbowers[i].DrawNext();
  }
  return 25;
}

// ----------------------------------------------------------------------------

LE_RainbowWaves2::LE_RainbowWaves2(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor)
    : LEDEffectBase(aNumLEDs, aPixels, aEffectColor)
{
  prog_arr = new double[mNumLEDs];
  double curr = 0;
  for (int i = 0; i < mNumLEDs; i++)
  {
    prog_arr[i] = curr;
    curr += inc;
  }
}

uint32_t LE_RainbowWaves2::DrawNext()
{
  mCounter++;
  if (mCounter > 500)
  {
    vz = !vz;
    mCounter = 0;
  }
  for (int i = 0; i < mNumLEDs; i++)
  {
    mLEDColors[i] = LEDEffectBase::Rainbow(prog_arr[i]);
    prog_arr[i] += (sin(i / 360.0 * 2 * 3.141592) + cos(i / 360.0 * 0.5 * 3.141592)) / 100.0 * (vz ? 1.0 : -1.0);
  }
  return 25;
}

LE_RainbowWaves2::~LE_RainbowWaves2()
{
  delete[] prog_arr;
}

// ----------------------------------------------------------------------------

const uint32_t LE_Fireworks::mColorChoices[] = {
    0xFFFF00, 0xFF7F00, 0xFF0000,
    0x7FFF00, 0x7F7F00, 0x7F0000,
    0x00FF00, 0x007F00, //0x000000,
    0x00FFFF, 0x00FF7F, 0x00FF00,
    0x007FFF, 0x007F7F, 0x007F00,
    0x0000FF, 0x00007F, //0x000000,
    0xFF00FF, 0x7F00FF, 0x0000FF,
    0xFF007F, 0x7F007F, 0x00007F,
    0xFF0000, 0x7F0000, //0x000000,
};

LE_Fireworks::LE_Fireworks(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor)
    : LEDEffectBase(aNumLEDs, aPixels, aEffectColor)
{
}

uint32_t LE_Fireworks::DrawNext()
{
  Render();
  return 25;
}

void LE_Fireworks::Render()
{
  // Randomly create some new stars this frame; the number we create is tied
  // to the size of the display so that the display size can change and
  // the "effect density" will stay the same

  const int min_width = 10;
  const int max_width = 50;
  const int max_speed_multiplier = 3;

  for (int iPass = 0; iPass < mNumLEDs / max_width; iPass++)
  {
    if (GetRandDouble() < NewParticleProbability && mParticles.size() == 0)
    {
      // Pick a random color and location.
      // If you don't have FastLED palettes, all you need to do
      // here is generate a random color.
      uint32_t mColorChoicesLength = sizeof(mColorChoices) / sizeof(uint32_t);
      int iStartPos = GetRandInt(mNumLEDs);
      pixelColor_t color;
      uint32_t ci = mColorChoices[GetRandInt(mColorChoicesLength)];
      color = pixelFromRGB(ci >> 16, (ci >> 8) & 0xff, ci & 0xff);
      int c = (int)(GetRandDouble() * (max_width - min_width) + min_width);
      double speed_multiplier = GetRandDouble() * MaxSpeed * max_speed_multiplier;

      for (int i = 1; i < c; i++)
      {
        LEDParticle particle(color, iStartPos, GetRandDouble() * speed_multiplier);
        mParticles.push_back(particle);
      }
    }
  }

  // In the degenerate case of particles not aging out for some reason,
  // we need to set a pseudo-realistic upper bound, and the very number of
  // possible pixels seems like a reasonable one

  while (mParticles.size() > mNumLEDs)
  {
    mParticles.pop_front();
  }

  pixelColor_t black;
  black.raw32 = 0;
  FillSolid(black);

  //foreach (var star in mParticles)
  for (std::list<LEDParticle>::iterator itStar = mParticles.begin(); itStar != mParticles.end(); ++itStar)
  {
    itStar->Update();

    pixelColor_t c = itStar->GetStarColor();
    double fade = 0.0;

    // If the star is brand new, it flashes white briefly.
    // Otherwise it just fades over time.
    if (itStar->Age() > ParticlePreignitonTime && itStar->Age() < (ParticleIgnition + ParticlePreignitonTime))
    {
      c = pixelFromRGB(0x3F, 0x3F, 0x3F);
      //c = Color.FromRgb(0xFF, 0xFF, 0xFF);
    }
    else
    {
      // Figure out how much to fade and shrink the star based on
      // its age relative to its lifetime
      double age = itStar->Age();
      if (age < ParticlePreignitonTime)
      {
        fade = 1.0 - (age / ParticlePreignitonTime);
      }
      else
      {
        age -= ParticlePreignitonTime;

        if (age < ParticleHoldTime + ParticleIgnition)
          fade = 0.0; // Just born
        else if (age > ParticleHoldTime + ParticleIgnition + ParticleFadeTime)
          fade = 1.0; // Black hole, all faded out
        else
        {
          age -= ParticleHoldTime + ParticleIgnition;
          fade = age / ParticleFadeTime; // Fading star
        }
      }
      if (c.r > 0 || c.g > 0 || c.b > 0)
      {
        c = LEDParticle::AdjustByUniformFactor(c, fade);
      }
    }
    ParticleSize = (1 - fade) * 5.0;

    // Because (the original) supports antialiasing and partial pixels, this takes a
    // non-integer number of pixels to draw.  But if you just made it
    // plot 'ParticleSize' pixels in int form, you'd be 99% of the way there

    SetPixels(itStar->GetPosition(), (int)ParticleSize, c);
  }

  // Remove any particles who have completed their lifespan
  while (mParticles.size() != 0 && mParticles.front().Age() > ParticleHoldTime + ParticleIgnition + ParticleFadeTime)
  {
    mParticles.pop_front();
  }
}

void LE_Fireworks::FillSolid(pixelColor_t c)
{
  for (int i = 0; i < mNumLEDs; i++)
    mLEDColors[i] = c;
}

void LE_Fireworks::SetPixels(double pos, int width, pixelColor_t c)
{
  for (int i = -width / 2; i <= width / 2; i++)
  {
    if (pos + i >= 0 && pos + i < mNumLEDs)
    {
      mLEDColors[(int)(pos + i)] = c;
    }
  }
}

// ----------------------------------------------------------------------------

LE_RunningLights::LE_RunningLights(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor)
    : LEDEffectBase(aNumLEDs, aPixels, aEffectColor)
{
}

uint32_t LE_RunningLights::DrawNext()
{
  mCounter++;
  mEffectColor = Rainbow((mCounter % mNumLEDs) / double(mNumLEDs));
  for (int i = 0; i < mNumLEDs; i++)
  {

    double l = (sin((i + mCounter) / 5.0) * 127.0 + 128.0) / 255.0;
    mLEDColors[i].r = l * mEffectColor.r;
    mLEDColors[i].g = l * mEffectColor.g;
    mLEDColors[i].b = l * mEffectColor.b;
  }
  return 25;
}

// ----------------------------------------------------------------------------

LE_Fire::LE_Fire(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor)
    : LEDEffectBase(aNumLEDs, aPixels, aEffectColor)
{
  heat = new uint8_t[mNumLEDs];
}

LE_Fire::~LE_Fire()
{
  delete[] heat;
}

void LE_Fire::Init(int Cooling, int Sparking, int SpeedDelay)
{
  mCooling = Cooling;
  mSparking = Sparking;
  mSpeedDelay = SpeedDelay;
}

uint32_t LE_Fire::DrawNext()
{
  int cooldown;
  int LEDsToUse = mNumLEDs;

  // Step 1.  Cool down every cell a little
  for (int cor = 0; cor < 4; cor++)
  {
    int offset = cor * LEDsToUse;
    for (int i = 0; i < LEDsToUse; i++)
    {
      cooldown = GetRandInt(0, ((mCooling * 10) / LEDsToUse) + 2);

      if (cooldown > heat[i + offset])
      {
        heat[i + offset] = 0;
      }
      else
      {
        heat[i + offset] = heat[i + offset] - cooldown;
      }
    }

    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for (int k = LEDsToUse - 1; k >= 2; k--)
    {
      heat[k + offset] = (heat[k - 1 + offset] + heat[k - 2 + offset] + heat[k - 2 + offset]) / 3;
    }

    // Step 3.  Randomly ignite new 'sparks' near the bottom
    if (GetRandInt(255) < mSparking)
    {
      int y = GetRandInt(7);
      heat[y + offset] = heat[y + offset] + GetRandInt(160, 255);
      //heat[y] = random(160,255);
    }

    // Step 4.  Convert heat to LED colors
    for (int j = 0; j < LEDsToUse; j++)
    {
      SetPixelHeatColor(j + offset, heat[j + offset]);
    }
  }
  return mSpeedDelay;
}

void LE_Fire::SetPixelHeatColor(int Pixel, uint8_t temperature)
{
  // Scale 'heat' down from 0-255 to 0-191
  uint8_t t192 = ((temperature / 255.0) * 191) + 0.5;

  // calculate ramp up from
  uint8_t heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2;                 // scale up to 0..252

  // figure out which third of the spectrum we're in:
  if (t192 > 0x80)
  {
    // hottest
    SetPixelColor(Pixel, 255, 255, heatramp);
  }
  else if (t192 > 0x40)
  {
    // middle
    SetPixelColor(Pixel, 255, heatramp, 0);
  }
  else
  {
    // coolest
    SetPixelColor(Pixel, heatramp, 0, 0);
  }
}

// ----------------------------------------------------------------------------

LE_BouncingColoredBalls::LE_BouncingColoredBalls(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor)
    : LEDEffectBase(aNumLEDs, aPixels, aEffectColor)
{
  Height = new double[BallCount];
  ImpactVelocityStart = sqrt(-2 * Gravity * StartHeight);
  ImpactVelocity = new double[BallCount];
  TimeSinceLastBounce = new double[BallCount];
  Position = new int[BallCount];
  ClockTimeSinceLastBounce = new uint32_t[BallCount];
  Dampening = new double[BallCount];
  BallColors = new pixelColor_t[BallCount];
  int LedsPerLine = mNumLEDs / 4;
  Offset = new int[BallCount];
  for (int i = 0; i < BallCount; i++)
  {
    ClockTimeSinceLastBounce[i] = millis();
    Height[i] = StartHeight;
    Position[i] = 0;
    ImpactVelocity[i] = ImpactVelocityStart;
    TimeSinceLastBounce[i] = 0;
    Dampening[i] = 0.90 - (double)i / (BallCount * BallCount);
    BallColors[i] = Rainbow(GetRandDouble());
    Offset[i] = (i / 3) * LedsPerLine;
  }
}

LE_BouncingColoredBalls::~LE_BouncingColoredBalls()
{
  delete[] Height;
  delete[] ImpactVelocity;
  delete[] TimeSinceLastBounce;
  delete[] Position;
  delete[] ClockTimeSinceLastBounce;
  delete[] Dampening;
  delete[] BallColors;
  delete[] Offset;
}

uint32_t LE_BouncingColoredBalls::DrawNext()
{
  SetAllColors(0, 0, 0);

  for (int i = 0; i < BallCount; i++)
  {
    TimeSinceLastBounce[i] = millis() - ClockTimeSinceLastBounce[i];
    Height[i] = 0.5 * Gravity * pow(TimeSinceLastBounce[i] / 5000, 2.0) + ImpactVelocity[i] * TimeSinceLastBounce[i] / 5000;

    if (Height[i] < 0)
    {
      Height[i] = 0;
      ImpactVelocity[i] = Dampening[i] * ImpactVelocity[i];
      ClockTimeSinceLastBounce[i] = millis();

      if (ImpactVelocity[i] < 0.01)
      {
        ImpactVelocity[i] = ImpactVelocityStart;
        BallColors[i] = Rainbow(GetRandDouble());
      }
    }
    Position[i] = (int)((Height[i] * (mNumLEDs / 4 - 1) / StartHeight) + 0.5) + Offset[i];
    SetPixelColor(Position[i], BallColors[i]);
  }
  return 10;
}

// ----------------------------------------------------------------------------

LE_Meteor::LE_Meteor(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor)
    : LEDEffectBase(aNumLEDs, aPixels, aEffectColor)
{
  mDirection = (bool)GetRandInt(2);
  mMeteorSize = GetRandInt(5, 35);
  SetAllColors(pixelFromRGB(0, 0, 0));
  mStartIndex = GetRandInt(mNumLEDs);
  mTTL = GetRandInt(mNumLEDs / 4) + 2 * mMeteorSize;
  mEffectColor = Rainbow(GetRandDouble());
  mFadeColor = mEffectColor;
}

uint32_t LE_Meteor::DrawNext()
{
  int i = mCounter % (mTTL + mFramesAfterTTL);
  if (i == 0)
    mCounter = 0;

  // fade brightness all LEDs one step
  for (int j = 0; j < mNumLEDs; j++)
  {
    if ((!mMeteorRandomDecay) || (GetRandInt(10) > 5))
    {
      SetPixelColor(j, FadeColor(mLEDColors[j], mMeteorTrailDecay));
    }
  }

  // draw meteor
  if (mCounter >= mTTL)
    mFadeColor = FadeColor(mFadeColor, mMeteorTrailDecay / 4);
  for (int j = 0; j < mMeteorSize; j++)
  {
    int index = mDirection ? i - j + mStartIndex : mNumLEDs - (i - j + mStartIndex);
    SetPixelColor(GetLEDNum(index), mFadeColor);
  }

  mCounter++;
  return 10;
}

// ----------------------------------------------------------------------------

LE_RandomTwinkle::LE_RandomTwinkle(uint32_t aNumLEDs, pixelColor_t *aPixels, pixelColor_t aEffectColor)
    : LEDEffectBase(aNumLEDs, aPixels, aEffectColor)
{
  SetAllColors(pixelFromRGB(0, 0, 0));
  mStars = new star[mNumStars];
  for (int i = 0; i < mNumStars; i++)
  {
    int rnd_led = GetRandInt(mNumLEDs);
    mLEDColors[rnd_led] = Rainbow(GetRandDouble());
    mStars[i].pos = rnd_led;
    mStars[i].fade = GetRandInt(5, 20);
  }
}

uint32_t LE_RandomTwinkle::DrawNext()
{
  mCounter++;
  pixelColor_t Temp;
  for (int i = 0; i < mNumStars; i++)
  {
    Temp = mLEDColors[mStars[i].pos];
    mLEDColors[mStars[i].pos] = FadeColor(mLEDColors[mStars[i].pos], mStars[i].fade);
    if (mLEDColors[mStars[i].pos].raw32 == Temp.raw32)
    {
      // Farbe ändert sich nicht mehr weiter...
      mLEDColors[mStars[i].pos] = pixelFromRGB(0, 0, 0);
      int rnd_led = GetRandInt(mNumLEDs);
      mLEDColors[rnd_led] = Rainbow(GetRandDouble());
      mStars[i].pos = rnd_led;
      mStars[i].fade = GetRandInt(5, 20);
    }
  }
  return 50;
}

LE_RandomTwinkle::~LE_RandomTwinkle()
{
  delete[] mStars;
}