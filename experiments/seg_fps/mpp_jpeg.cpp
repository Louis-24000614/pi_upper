#include <rk_mpi.h>
#include <mpp_buffer.h>
#include <mpp_packet.h>
#include <mpp_frame.h>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>

namespace {
thread_local std::string error;
using Clock = std::chrono::steady_clock;
double ms(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
void check(MPP_RET ret, const char* stage) {
    if (ret != MPP_OK) throw std::runtime_error(std::string(stage) + " MPP=" + std::to_string(ret));
}
void sync_buffer(MppBuffer buffer, uint64_t flags) {
    dma_buf_sync sync{flags};
    if (ioctl(mpp_buffer_get_fd(buffer), DMA_BUF_IOCTL_SYNC, &sync) < 0)
        throw std::runtime_error("MPP buffer cache sync failed");
}
struct Decoder {
    MppCtx ctx = nullptr;
    MppApi* api = nullptr;
    MppBufferGroup group = nullptr;
    MppBuffer input = nullptr, output = nullptr;
    MppPacket packet = nullptr;
    MppFrame frame = nullptr;
    ~Decoder() {
        if (ctx) mpp_destroy(ctx);
        if (packet) mpp_packet_deinit(&packet);
        if (frame) mpp_frame_deinit(&frame);
        if (input) mpp_buffer_put(input);
        if (output) mpp_buffer_put(output);
        if (group) mpp_buffer_group_put(group);
    }
};
}
extern "C" {
const char* road_mpp_error() { return error.c_str(); }
void* road_mpp_create(int width, int height, int rgb_output) {
    auto* s = new Decoder;
    try {
        if (width <= 0 || height <= 0 || width > 4096 || height > 4096)
            throw std::runtime_error("invalid JPEG dimensions");
        check(mpp_create(&s->ctx, &s->api), "create");
        check(mpp_init(s->ctx, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG), "init MJPEG");
        MppFrameFormat format = rgb_output ? MPP_FMT_BGR888 : MPP_FMT_YUV420SP;
        check(s->api->control(s->ctx, MPP_DEC_SET_OUTPUT_FORMAT, &format), "output format");
        check(mpp_buffer_group_get_internal(&s->group, MPP_BUFFER_TYPE_DRM), "buffer group");
        check(mpp_buffer_get(s->group, &s->input, 4 * 1024 * 1024), "packet buffer");
        int ws = (width + 15) & ~15, hs = (height + 15) & ~15;
        check(mpp_buffer_get(s->group, &s->output, static_cast<size_t>(ws) * hs * 4), "frame buffer");
        check(mpp_packet_init_with_buffer(&s->packet, s->input), "packet init");
        check(mpp_frame_init(&s->frame), "frame init");
        mpp_frame_set_width(s->frame, width); mpp_frame_set_height(s->frame, height);
        mpp_frame_set_hor_stride(s->frame, ws);
        mpp_frame_set_ver_stride(s->frame, hs);
        mpp_frame_set_fmt(s->frame, format);
        mpp_frame_set_buffer(s->frame, s->output);
        return s;
    } catch (const std::exception& e) { error = e.what(); delete s; return nullptr; }
}
int road_mpp_decode(void* handle, const unsigned char* jpeg, size_t bytes,
                    unsigned char* destination, size_t capacity, int* info, double* timings) {
    auto* s = static_cast<Decoder*>(handle);
    MppTask task = nullptr;
    bool output_dequeued = false;
    try {
        if (bytes < 4 || bytes > mpp_buffer_get_size(s->input) || jpeg[0] != 0xff || jpeg[1] != 0xd8)
            throw std::runtime_error("invalid original JPEG packet");
        auto start = Clock::now();
        sync_buffer(s->input, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);
        std::memcpy(mpp_buffer_get_ptr(s->input), jpeg, bytes);
        sync_buffer(s->input, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
        mpp_packet_set_pos(s->packet, mpp_buffer_get_ptr(s->input));
        mpp_packet_set_length(s->packet, bytes);
        mpp_frame_set_eos(s->frame, 0);
        timings[0] = ms(start); start = Clock::now();
        // MJPEG uses the advanced task interface with an explicitly owned output frame.
        check(s->api->poll(s->ctx, MPP_PORT_INPUT, static_cast<MppPollType>(2000)), "poll input");
        check(s->api->dequeue(s->ctx, MPP_PORT_INPUT, &task), "dequeue input");
        if (!task) throw std::runtime_error("empty input task");
        check(mpp_task_meta_set_packet(task, KEY_INPUT_PACKET, s->packet), "input packet");
        check(mpp_task_meta_set_frame(task, KEY_OUTPUT_FRAME, s->frame), "output frame");
        check(s->api->enqueue(s->ctx, MPP_PORT_INPUT, task), "enqueue input");
        task = nullptr;
        check(s->api->poll(s->ctx, MPP_PORT_OUTPUT, static_cast<MppPollType>(2000)), "poll output");
        check(s->api->dequeue(s->ctx, MPP_PORT_OUTPUT, &task), "dequeue output");
        output_dequeued = true;
        MppFrame frame = nullptr;
        check(mpp_task_meta_get_frame(task, KEY_OUTPUT_FRAME, &frame), "get output frame");
        if (!frame) throw std::runtime_error("empty JPEG output frame");
        info[0] = mpp_frame_get_width(frame); info[1] = mpp_frame_get_height(frame);
        info[2] = mpp_frame_get_hor_stride(frame); info[3] = mpp_frame_get_ver_stride(frame);
        info[4] = mpp_frame_get_fmt(frame); info[5] = mpp_frame_get_errinfo(frame);
        info[6] = mpp_frame_get_color_range(frame);
        if (info[5] || mpp_frame_get_discard(frame)) throw std::runtime_error("JPEG frame error/discard");
        MppBuffer buffer = mpp_frame_get_buffer(frame);
        if (!buffer) throw std::runtime_error("missing JPEG output buffer");
        size_t copy_bytes = static_cast<size_t>(info[2]) * info[3];
        if (info[4] == MPP_FMT_YUV420SP) copy_bytes = copy_bytes * 3 / 2;
        else if (info[4] == MPP_FMT_BGR888) copy_bytes *= 3;
        else throw std::runtime_error("unexpected JPEG output format " + std::to_string(info[4]));
        if (copy_bytes > capacity || copy_bytes > mpp_buffer_get_size(buffer)) throw std::runtime_error("JPEG frame bounds exceeded");
        timings[1] = ms(start); start = Clock::now();
        sync_buffer(buffer, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
        std::memcpy(destination, mpp_buffer_get_ptr(buffer), copy_bytes);
        sync_buffer(buffer, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
        info[7] = copy_bytes;
        check(s->api->enqueue(s->ctx, MPP_PORT_OUTPUT, task), "return output task");
        task = nullptr; output_dequeued = false;
        timings[2] = ms(start);
        return 0;
    } catch (const std::exception& e) {
        if (output_dequeued && task) s->api->enqueue(s->ctx, MPP_PORT_OUTPUT, task);
        error = e.what(); return -1;
    }
}
void road_mpp_destroy(void* handle) { delete static_cast<Decoder*>(handle); }
}
