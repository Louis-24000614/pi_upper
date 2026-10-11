#include <cstddef>
#include <im2d.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace {
thread_local std::string error;
void require(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
struct Buffer {
    int fd = -1;
    size_t size = 0;
    void* ptr = MAP_FAILED;
    rga_buffer_handle_t handle = 0;
    void allocate(size_t bytes) {
        int heap = open("/dev/dma_heap/system-uncached-dma32", O_RDWR | O_CLOEXEC);
        require(heap >= 0, "DMA heap open: " + std::string(strerror(errno)));
        dma_heap_allocation_data allocation{};
        allocation.len = (bytes + 4095) & ~size_t(4095);
        allocation.fd_flags = O_RDWR | O_CLOEXEC;
        int result = ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &allocation);
        close(heap);
        require(result == 0, "DMA alloc: " + std::string(strerror(errno)));
        fd = allocation.fd; size = allocation.len;
        ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        require(ptr != MAP_FAILED, "DMA mmap failed");
        handle = importbuffer_fd(fd, static_cast<int>(size));
        require(handle != 0, "RGA importbuffer_fd failed");
    }
    void sync(uint64_t flags) {
        dma_buf_sync sync{flags};
        require(ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) == 0, "DMA sync failed");
    }
    ~Buffer() {
        if (handle) releasebuffer_handle(handle);
        if (ptr != MAP_FAILED) munmap(ptr, size);
        if (fd >= 0) close(fd);
    }
};
struct Session {
    Buffer input, output;
    int width, height;
    int external_fd = -1;
    rga_buffer_handle_t external_handle = 0;
    ~Session() { if (external_handle) releasebuffer_handle(external_handle); }
};
}
extern "C" {
const char* road_rga_error() { return error.c_str(); }
void* road_rga_create(int width, int height) {
    auto* s = new Session;
    try {
        require(width > 0 && height > 0 && width % 16 == 0, "input width must be 16-aligned");
        s->width = width; s->height = height;
        s->input.allocate(static_cast<size_t>(width) * height * 3);
        s->output.allocate(640 * 640 * 3);
        return s;
    } catch (const std::exception& e) { error = e.what(); delete s; return nullptr; }
}
int road_rga_run(void* handle, const uint8_t* bgr, uint8_t* rgb,
                 int rw, int rh, int left, int top) {
    auto* s = static_cast<Session*>(handle);
    try {
        require(rw > 0 && rh > 0 && left >= 0 && top >= 0 && left + rw <= 640 && top + rh <= 640,
                "invalid letterbox ROI");
        s->input.sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);
        std::memcpy(s->input.ptr, bgr, static_cast<size_t>(s->width) * s->height * 3);
        s->input.sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
        s->output.sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);
        std::memset(s->output.ptr, 114, 640 * 640 * 3);
        s->output.sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
        auto src = wrapbuffer_handle(s->input.handle, s->width, s->height, RK_FORMAT_BGR_888);
        auto dst = wrapbuffer_handle(s->output.handle, 640, 640, RK_FORMAT_RGB_888);
        im_rect srect{0, 0, s->width, s->height}, drect{left, top, rw, rh};
        auto code = imcheck(src, dst, srect, drect);
        require(code == IM_STATUS_NOERROR, "RGA check: " + std::string(imStrError(code)));
        code = improcess(src, dst, {}, srect, drect, {}, -1, nullptr, nullptr, IM_SYNC);
        require(code == IM_STATUS_SUCCESS, "RGA process: " + std::string(imStrError(code)));
        s->output.sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
        std::memcpy(rgb, s->output.ptr, 640 * 640 * 3);
        s->output.sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
        return 0;
    } catch (const std::exception& e) { error = e.what(); return -1; }
}
void road_rga_destroy(void* handle) { delete static_cast<Session*>(handle); }
int road_rga_run_shared(void* handle, const uint8_t* bgr, void* output,
                        int output_fd, unsigned output_size, unsigned stride,
                        int rw, int rh, int left, int top) {
    auto* s = static_cast<Session*>(handle);
    try {
        require(output && output_fd >= 0 && stride >= 640 && output_size >= stride * 640 * 3,
                "invalid RKNN shared input memory");
        require(rw > 0 && rh > 0 && left >= 0 && top >= 0 && left + rw <= 640 && top + rh <= 640,
                "invalid shared ROI");
        if (output_fd != s->external_fd) {
            if (s->external_handle) releasebuffer_handle(s->external_handle);
            s->external_handle = importbuffer_fd(output_fd, static_cast<int>(output_size));
            require(s->external_handle != 0, "RGA import RKNN FD failed");
            s->external_fd = output_fd;
        }
        s->input.sync(DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);
        std::memcpy(s->input.ptr, bgr, static_cast<size_t>(s->width) * s->height * 3);
        s->input.sync(DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
        dma_buf_sync begin{DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE};
        require(ioctl(output_fd, DMA_BUF_IOCTL_SYNC, &begin) == 0, "shared CPU write begin sync failed");
        std::memset(output, 114, output_size);
        dma_buf_sync end{DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE};
        require(ioctl(output_fd, DMA_BUF_IOCTL_SYNC, &end) == 0, "shared CPU write end sync failed");
        auto src = wrapbuffer_handle(s->input.handle, s->width, s->height, RK_FORMAT_BGR_888);
        auto dst = wrapbuffer_handle(s->external_handle, 640, 640, RK_FORMAT_RGB_888, stride, 640);
        im_rect srect{0, 0, s->width, s->height}, drect{left, top, rw, rh};
        auto code = imcheck(src, dst, srect, drect);
        require(code == IM_STATUS_NOERROR, "shared RGA check: " + std::string(imStrError(code)));
        code = improcess(src, dst, {}, srect, drect, {}, -1, nullptr, nullptr, IM_SYNC);
        require(code == IM_STATUS_SUCCESS, "shared RGA process: " + std::string(imStrError(code)));
        begin.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
        end.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
        require(ioctl(output_fd, DMA_BUF_IOCTL_SYNC, &begin) == 0, "shared CPU read begin sync failed");
        require(ioctl(output_fd, DMA_BUF_IOCTL_SYNC, &end) == 0, "shared CPU read end sync failed");
        return 0;
    } catch (const std::exception& e) { error = e.what(); return -1; }
}
}
