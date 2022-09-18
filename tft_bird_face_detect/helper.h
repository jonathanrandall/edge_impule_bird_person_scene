#ifndef HELPER_H
#define HELPER_H

#include <Adafruit_GFX.h>

class aFrameBuffer : public Adafruit_GFX {
  public:
    uint8_t *buffer = NULL;
    uint32_t size;
    int w;
    int h;

    aFrameBuffer(int16_t ww, int16_t hh): Adafruit_GFX(ww, hh)
    {
      w = ww;
      h = hh;
      size = h*w;
      
    }

    void setBuffer(uint8_t *b)
    {
      buffer = b;
    }

    void drawPixel(int16_t x, int16_t y, uint16_t color)
    {
      if (x<0 || x >= w || y <0 || y >=h)
        return;
      buffer[x +y*w] = color;
    }

    uint8_t get(int16_t x, int16_t y)
    {
      if (x<0 || x >= w || y <0 || y >=h)
        return 0;
      return buffer[x +y*w];
    }
};

#endif
