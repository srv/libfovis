#ifndef __DRAW_H__
#define __DRAW_H__

#include <stdint.h>
#include <vector>

struct DrawColor {
  DrawColor(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct DrawImage {
  DrawImage(int w, int h);

  std::vector<uint8_t> data;
  int width;
  int height;
  int stride;
};

void draw_line_rgb(int x0, int y0, int x1, int y1,
    DrawColor color, DrawImage* img);

void draw_box_rgb(int x0, int y0, int x1, int y1,
    DrawColor color, DrawImage* img);

void draw_gray_img_rgb(const uint8_t* gray_data,
    int gray_width, int gray_height, int gray_stride,
    int dest_x0, int dest_y0,
    DrawImage* rgb_img);

#endif
