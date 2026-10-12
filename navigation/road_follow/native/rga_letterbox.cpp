#include <cstddef>
#include <im2d.h>
#include <im2d_buffer.h>
#include <im2d_single.h>
#include <rga.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>

extern "C" int road_rga_letterbox(const uint8_t* bgr, int width, int height,
                                   uint8_t* rgb640, int* resized_w, int* resized_h,
                                   int* left, int* top) {
    if (!bgr || !rgb640 || width <= 0 || height <= 0) return -1;
    const double ratio = std::min(640.0 / height, 640.0 / width);
    const int rw = static_cast<int>(width * ratio + 0.5);
    const int rh = static_cast<int>(height * ratio + 0.5);
    const int x = (640 - rw) / 2;
    const int y = (640 - rh) / 2;
    std::memset(rgb640, 114, 640 * 640 * 3);
    auto src = wrapbuffer_virtualaddr(const_cast<uint8_t*>(bgr), width, height,
                                      RK_FORMAT_BGR_888);
    auto tmp = wrapbuffer_virtualaddr(rgb640, 640, 640, RK_FORMAT_RGB_888);
    auto dst = wrapbuffer_virtualaddr(rgb640 + (y * 640 + x) * 3, rw, rh,
                                      rw, rh, RK_FORMAT_RGB_888);
    if (imresize(src, dst, 0, 0, INTER_LINEAR) < 0) return -2;
    // RGA can convert BGR to RGB while resizing; the destination is the centered patch.
    if (imcvtcolor(wrapbuffer_virtualaddr(rgb640 + (y * 640 + x) * 3, rw, rh,
                                          rw, rh, RK_FORMAT_BGR_888),
                   dst, RK_FORMAT_BGR_888, RK_FORMAT_RGB_888) < 0) return -3;
    *resized_w = rw; *resized_h = rh; *left = x; *top = y;
    return 0;
}
