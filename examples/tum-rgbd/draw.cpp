#include <cstdlib>

#include "draw.hpp"

// draw a single point
static inline void
draw_point_rgb(int x, int y, DrawColor color, DrawImage* img)
{
  if(x>=0 && x<img->width && y>=0 && y<img->height) {
    uint8_t *pix = &img->data[y*img->stride + x*3];
    pix[0] = color.r;
    pix[1] = color.g;
    pix[2] = color.b;
  }
}

DrawImage::DrawImage(int w, int h) :
  data(w*h*3),
  width(w),
  height(h),
  stride(width*3)
{}

void
draw_hline_rgb(int x0, int y0, int x1, DrawColor color, DrawImage* img)
{
  if(y0 < 0 || y0 >= img->height)
    return;

  if(x0 > x1)
  {
    int t = x0;
    x0 = x1;
    x1 = t;
  }

  if(x0 < 0)
    x0 = 0;
  else if (x0 >= img->width)
    return;

  if(x1 < 0)
    return;
  else if(x1 >= img->width)
    x1 = img->width-1;

  uint8_t* pix = &img->data[y0*img->stride + x0*3];
  while(x0 <= x1) {
    pix[0] = color.r;
    pix[1] = color.g;
    pix[2] = color.b;
    x0++;
    pix += 3;
  }
}

void
draw_line_rgb(int x0, int y0, int x1, int y1,
        DrawColor color, DrawImage* img)
{
  int xs;
  int ys;

  int dx = x1-x0;
  if (dx < 0)
  {
    dx = -dx;
    xs = -1;
  }
  else
    xs = 1;

  int dy = y1-y0;
  if (dy < 0)
  {
    dy = -dy;
    ys = -1;
  }
  else
    ys = 1;

  int n = (dx > dy) ? dx : dy;

  if (dx == 0) {
    for (int i = 0; i < dy; i++) {
      draw_point_rgb(x0, y0, color, img);
      y0 += ys;
    }
  } else if (dy == 0) {
    draw_hline_rgb(x0, y0, x1, color, img);
  } else if (dx > dy) {
    n = dx;
    dy += dy;
    int e = dy - dx;
    dx += dx;

    for(int i = 0; i < n; i++) {
      draw_point_rgb(x0, y0, color, img);
      if (e >= 0) {
        y0 += ys;
        e -= dx;
      }
      e += dy;
      x0 += xs;
    }
  } else {
    n = dy;
    dx += dx;
    int e = dx - dy;
    dy += dy;

    for(int i = 0; i < n; i++) {
      draw_point_rgb(x0, y0, color, img);
      if (e >= 0) {
        x0 += xs;
        e -= dy;
      }
      e += dx;
      y0 += ys;
    }
  }
}

void draw_box_rgb(int x0, int y0, int x1, int y1,
    DrawColor color, DrawImage* img)
{
  for(int y=y0; y<=y1; y++)
    draw_hline_rgb(x0, y, x1, color, img);
}

void draw_gray_img_rgb(const uint8_t* gray_img,
    int gray_width, int gray_height, int gray_stride,
    int dest_x0, int dest_y0,
    DrawImage* rgb_img)
{
  for(int gray_row=0; gray_row<gray_height; gray_row++) {
    uint8_t* rgb_pixel = &rgb_img->data[(gray_row + dest_y0) * rgb_img->stride + dest_x0 * 3];
    const uint8_t* gray_pixel = &gray_img[gray_row * gray_stride];
    for(int gray_col=0; gray_col<gray_width; gray_col++) {
      rgb_pixel[0] = *gray_pixel;
      rgb_pixel[1] = *gray_pixel;
      rgb_pixel[2] = *gray_pixel;
      rgb_pixel += 3;
      gray_pixel++;
    }
  }
}
